#include <Wire.h>
#include "RTClib.h"
#include <Servo.h>
#include <SD.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <ArduinoHttpClient.h>

RTC_DS1307 rtc;
volatile bool sqwInterrupt = false;

const char* ssid = "";
const char* password = "";

const char* serverName = "script.google.com";
const int port = 443;
const char* path = "/macros/s/AKfycbxLk0AUvrZxNG8ZYEUGZZpyWSjIkLb0AvH-lx9giByP0r3ZlGCannJT1hCHVZ-aXqYEpw/exec";

// reader pins
const int rfidTxPin = 1; 
const int rfidRstPin = 7;

Servo myServo;

const int LEDPin = 15;  // Using a different pin for LED, adjust as needed
const int servoPin = 9;
const int sqwPin = 4; 
const int chipSelect = 17;  // CS pin for SD card reader on RP2040, adjust if needed

const int SERVO_OPEN_POS = 150;
const int SERVO_CLOSED_POS = 50;

int pos = SERVO_CLOSED_POS;
unsigned long lastRFIDScanTime = 0;
unsigned long mealStartTime = 0;
const unsigned long doorOpenTimeout = 15000;
unsigned long rfidResetInterval = 5000;
unsigned long lastRFIDResetTime = 0;
int state = 0;

// Interrupt Service Routine for the SQW pin
void sqwISR() {
  sqwInterrupt = true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println("Serial Monitor Initialized");

  pinMode(LEDPin, OUTPUT);
  pinMode(rfidRstPin, OUTPUT);
  digitalWrite(rfidRstPin, HIGH);

  Wire.begin();

  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  rtc.writeSqwPinMode(DS1307_SquareWave1HZ);

  pinMode(sqwPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(sqwPin), sqwISR, FALLING);

  Serial1.setRX(rfidTxPin);
  Serial1.setTX(NC);  // We don't need TX for the RFID reader
  Serial1.begin(9600);
  Serial.println("RFID Reader Initialized");

  myServo.attach(servoPin);
  myServo.write(0);

  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }
  Serial.println("SD card initialized.");

  File logFile = SD.open("meals.csv", FILE_WRITE);
  if (logFile) {
    logFile.close();
    Serial.println("Log file created or exists.");
  } else {
    Serial.println("Error creating log file.");
  }

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi..");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");

  Serial.println("Setup complete");
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastRFIDResetTime >= rfidResetInterval) {
    resetRFIDModule();
    lastRFIDResetTime = currentMillis;
    Serial.println("Resetting RFID module");
  }

  if (Serial1.available()) {
    String rfidData = "";
    while (Serial1.available()) {
      char c = Serial1.read();
      rfidData += c;
    }

    Serial.print("RFID Reader Reading: ");
    Serial.println(rfidData);

    if (rfidData.length() > 1) {
      Serial.println("Data available from RFID module");
      Serial.print("RFID Tag Detected: ");
      Serial.println(rfidData);

      lastRFIDScanTime = millis();

      if (pos == SERVO_CLOSED_POS && state == 0) {
        openFeeder();
        mealStartTime = lastRFIDScanTime;
      }
    }
  }
  if (pos == SERVO_OPEN_POS && state == 1) {
    myServo.detach();
    rfidResetInterval = 500;
    waitToCloseFeeder();
  }
}

void openFeeder() {
  if (pos != SERVO_OPEN_POS && state == 0) {
    Serial.println("Feeder Activated");
    for (int i = SERVO_CLOSED_POS; i < SERVO_OPEN_POS + 1; ++i) {
      myServo.write(i);
      delay(7);
    }
    pos = SERVO_OPEN_POS;
    digitalWrite(LEDPin, HIGH);
  }
  lastRFIDScanTime = millis();
  state = 1;
}

void closeFeeder() {
  rfidResetInterval = 5000;

  if (pos != SERVO_CLOSED_POS && state == 1) {
    Serial.println("Closing feeder due to inactivity");
    myServo.attach(servoPin);
    for (int i = SERVO_OPEN_POS; i > SERVO_CLOSED_POS + 1; --i) {
      myServo.write(i);
      delay(7);
    }
    pos = SERVO_CLOSED_POS;
    state = 0;
    digitalWrite(LEDPin, LOW);
    myServo.detach();
    logMealData(mealStartTime, millis());
    myServo.attach(servoPin);
  }
}

void waitToCloseFeeder() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastRFIDScanTime >= doorOpenTimeout) {
    closeFeeder();
  }
}

void resetRFIDModule() {
  digitalWrite(rfidRstPin, LOW);
  delay(50);
  digitalWrite(rfidRstPin, HIGH);
  delay(50);
  Serial.println("RFID Reader Reset");
}

void logMealData(unsigned long startTime, unsigned long endTime) {
  Serial.println("Attempting to write to SD card");

  DateTime now = rtc.now();
  Serial.print("Current RTC Time: ");
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.println(now.second(), DEC);

  if (now.year() < 2000 || now.year() > 2100 || now.month() < 1 || now.month() > 12 || now.day() < 1 || now.day() > 31 || now.hour() > 23 || now.minute() > 59 || now.second() > 59) {
    Serial.println("Error: Invalid RTC time read!");
    return;
  }

  // Calculate elapsed time
  unsigned long elapsedStartMillis = millis() - startTime;
  unsigned long elapsedEndMillis = millis() - endTime;

  Serial.print("Elapsed start milliseconds: ");
  Serial.println(elapsedStartMillis);
  Serial.print("Elapsed end milliseconds: ");
  Serial.println(elapsedEndMillis);

  // Calculate the start and end times based on the current RTC time minus the elapsed time
  DateTime start = now - TimeSpan(elapsedStartMillis / 1000);
  DateTime end = now - TimeSpan(elapsedEndMillis / 1000);

  Serial.print("Start Time after calculation: ");
  Serial.print(start.year(), DEC);
  Serial.print('/');
  Serial.print(start.month(), DEC);
  Serial.print('/');
  Serial.print(start.day(), DEC);
  Serial.print(" ");
  Serial.print(start.hour(), DEC);
  Serial.print(':');
  Serial.print(start.minute(), DEC);
  Serial.print(':');
  Serial.println(start.second(), DEC);

  Serial.print("End Time after calculation: ");
  Serial.print(end.year(), DEC);
  Serial.print('/');
  Serial.print(end.month(), DEC);
  Serial.print('/');
  Serial.print(end.day(), DEC);
  Serial.print(" ");
  Serial.print(end.hour(), DEC);
  Serial.print(':');
  Serial.print(end.minute(), DEC);
  Serial.print(':');
  Serial.println(end.second(), DEC);

  // Create timestamp strings
  String startTimestamp = String(start.year()) + "/" + String(start.month()) + "/" + String(start.day()) + " " +
                          String(start.hour()) + ":" + String(start.minute()) + ":" + String(start.second());
  Serial.println("Start Time: " + startTimestamp);

  String endTimestamp = String(end.year()) + "/" + String(end.month()) + "/" + String(end.day()) + " " +
                        String(end.hour()) + ":" + String(end.minute()) + ":" + String(end.second());
  Serial.println("End Time: " + endTimestamp);

  String logEntry = "Meal Start: " + startTimestamp + ", Meal End: " + endTimestamp;
  Serial.println("Log Entry: " + logEntry);

  // Attempt to open the log file
  File logFile = SD.open("meals.csv", FILE_WRITE);
  if (logFile) {
    Serial.println("File opened successfully.");
    logFile.println(logEntry);
    logFile.close();
    Serial.println("Logged to SD: " + logEntry);
  } else {
    Serial.println("Error opening log file.");
  }



  if (WiFi.status() == WL_CONNECTED) { // Check Wi-Fi connection status
    // Construct the full URL with parameters
    String fullPath = String(path) + "?" + startTimestamp + "&" + endTimestamp + "&" + numScans + "&" + rfidData;

    // Debug: Starting HTTP GET request
    Serial.println("Starting connection to server...");
    Serial.print("Request URL: ");
    Serial.println(fullPath);

    // Make a HTTP GET request
    client.get(fullPath.c_str());

    int statusCode = client.responseStatusCode();
    String response = client.responseBody();

    Serial.print("Status code: ");
    Serial.println(statusCode);

    Serial.print("Response: ");
    Serial.println(response);

    // Free resources
    client.stop();
  } else {
    Serial.println("WiFi Disconnected");
  }



