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

// RFID reader pins
const int rfidTxPin = 1;  // GPIO pin connected to RFID TX
const int rfidRstPin = 7; // GPIO pin connected to RFID RST

Servo myServo1;
Servo myServo2;

const int LEDPin = LED_BUILTIN;  // Using built-in LED
const int servo1Pin = 9;
const int servo2Pin = 10;  // Adjust this pin as needed
const int sqwPin = 4; 
const int chipSelect = 8;  // CS pin for SD card reader on RP2040, adjust if needed

const int SERVO_OPEN_POS = 150;
const int SERVO_CLOSED_POS = 50;

int pos1 = SERVO_CLOSED_POS;
int pos2 = SERVO_CLOSED_POS;
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

  // Initialize UART1 for RFID reader
  Serial1.begin(9600);
  Serial.println("RFID Reader Initialized");

  myServo1.attach(servo1Pin);
  myServo2.attach(servo2Pin);
  myServo1.write(SERVO_CLOSED_POS);
  myServo2.write(SERVO_CLOSED_POS);

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
  }

  if (Serial1.available()) {
    String rfidData = "";
    unsigned long startTime = millis();

    // Read data for a maximum of 100ms or until no more data is available
    while (millis() - startTime < 100) {
      if (Serial1.available()) {
        char c = Serial1.read();
        rfidData += c;
        startTime = millis(); // Reset the timer if we've received data
      }
    }

    if (rfidData.length() > 0) {
      Serial.println("Data available from RFID module");
      Serial.print("RFID Tag Detected: ");
      Serial.println(rfidData);

      // Blink the LED to indicate a successful read
      digitalWrite(LEDPin, HIGH);
      delay(200);
      digitalWrite(LEDPin, LOW);

      lastRFIDScanTime = millis();

      if ((pos1 == SERVO_CLOSED_POS || pos2 == SERVO_CLOSED_POS) && state == 0) {
        openFeeder();
        mealStartTime = lastRFIDScanTime;
      }
    }
  }

  if ((pos1 == SERVO_OPEN_POS || pos2 == SERVO_OPEN_POS) && state == 1) {
    myServo1.detach();
    myServo2.detach();
    rfidResetInterval = 500;
    waitToCloseFeeder();
  }
}

void openFeeder() {
  if ((pos1 != SERVO_OPEN_POS || pos2 != SERVO_OPEN_POS) && state == 0) {
    Serial.println("Feeder Activated");
    for (int i = SERVO_CLOSED_POS; i < SERVO_OPEN_POS + 1; ++i) {
      myServo1.write(i);
      myServo2.write(i);
      delay(7);
    }
    pos1 = SERVO_OPEN_POS;
    pos2 = SERVO_OPEN_POS;
    digitalWrite(LEDPin, HIGH);
  }
  lastRFIDScanTime = millis();
  state = 1;
}

void closeFeeder() {
  rfidResetInterval = 5000;

  if ((pos1 != SERVO_CLOSED_POS || pos2 != SERVO_CLOSED_POS) && state == 1) {
    Serial.println("Closing feeder due to inactivity");
    myServo1.attach(servo1Pin);
    myServo2.attach(servo2Pin);
    for (int i = SERVO_OPEN_POS; i > SERVO_CLOSED_POS + 1; --i) {
      myServo1.write(i);
      myServo2.write(i);
      delay(7);
    }
    pos1 = SERVO_CLOSED_POS;
    pos2 = SERVO_CLOSED_POS;
    state = 0;
    digitalWrite(LEDPin, LOW);
    myServo1.detach();
    myServo2.detach();
    logMealData(mealStartTime, millis());
    myServo1.attach(servo1Pin);
    myServo2.attach(servo2Pin);
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

  // Get the current RTC time
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
    String fullPath = String(path) + "?" + startTimestamp + "&" + endTimestamp;

    // Debug: Starting HTTP GET request
    Serial.println("Starting connection to server...");
    Serial.print("Request URL: ");
    Serial.println(fullPath);

    // Create WiFiClient object
    WiFiClient wifi;
    HttpClient client = HttpClient(wifi, serverName, port);

    client.get(fullPath.c_str());

    int statusCode = client.responseStatusCode();
    String response = client.responseBody();

    Serial.print("Status code: ");
    Serial.println(statusCode);

    Serial.print("Response: ");
    Serial.println(response);

    client.stop();
  } else {
    Serial.println("WiFi Disconnected");
  }
}