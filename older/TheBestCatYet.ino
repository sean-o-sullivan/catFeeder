#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ESP32Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

const char* ssid = "";
const char* password = "";

const char* serverName = "https://script.google.com/macros/s/AKfycbyOmO3fYNebgT6COppmaw9mCtrjy8Lxqp3J1xm9vCngsqNl23Lx_VhE1Ai8ER00gcmQgQ/exec";

const int rfidTxPin = 3; 
const int rfidRstPin = 4;  
const int buttonPin = 2;   
const int ldrPin = 34;     

Servo myServo1;
Servo myServo2;

const int LEDPin = 2;      
const int servo1Pin = 13;  
const int servo2Pin = 14;  
const int chipSelect = 5;  

const int SERVO1_OPEN_POS = 0;
const int SERVO1_CLOSED_POS = 90;

const int SERVO2_OPEN_POS = 90;
const int SERVO2_CLOSED_POS = 0;

int pos1 = SERVO1_CLOSED_POS;
int pos2 = SERVO2_CLOSED_POS;

unsigned long lastRFIDScanTime = 0;
unsigned long mealStartTime = 0;
unsigned long lastUpdateTime = 0;
const unsigned long doorOpenTimeout = 13000;
unsigned long rfidResetInterval = 5000; // Adjusted to 5000 ms for normal state
unsigned long lastRFIDResetTime = 0;
int state = 0;
bool feederRefilled = false;
int logID = 0;
String tagID = "button"; // Initialize tagID as "button" for button press

WiFiClient wifi;
HTTPClient httpClient;

const unsigned long debounceDelay = 50; 
const unsigned long pressDuration = 500; 
unsigned long lastDebounceTime = 0;
unsigned long buttonPressTime = 0; 
int lastButtonState = HIGH; 
int buttonState = HIGH; 
bool buttonHandled = false; 

Adafruit_BME680 bme; // create BME680 object

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println("Serial Monitor Initialized");

  pinMode(LEDPin, OUTPUT);
  pinMode(rfidRstPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP); // Initialize the push button pin with an internal pull-up resistor
  pinMode(ldrPin, INPUT); // Initialize the LDR pin as input
  digitalWrite(rfidRstPin, HIGH);
  Wire.begin(26, 27);  // Initialize I2C communication

  Serial2.begin(9600, SERIAL_8N1, rfidTxPin, -1);  // Initialize Serial2 for RFID reader
  Serial.println("RFID Reader Initialized");

  myServo1.attach(servo1Pin);
  myServo2.attach(servo2Pin);
  myServo1.write(SERVO1_CLOSED_POS);
  myServo2.write(SERVO2_CLOSED_POS);

  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320°C for 150 ms

  SPI.begin(18, 19, 23, chipSelect); // SCK, MISO, MOSI, SS
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }
  Serial.println("SD card initialized.");

  // Check if the log file exists, if not, create it and add headers
  if (!SD.exists("/meals.csv")) {
    File logFile = SD.open("/meals.csv", FILE_WRITE);
    if (logFile) {
      logFile.println("LogID,TagID,Duration,Light Level,Temperature,Humidity,Pressure,Gas Resistance,Refilled");
      logFile.close();
      Serial.println("Log file created with headers.");
    } else {
      Serial.println("Failed to create new log file.");
      while (1);
    }
  }

  // Read the highest logID from the existing file
  logID = getHighestLogID();

  connectToWiFi();
  lastUpdateTime = millis();
  Serial.println("Setup complete");
}

void loop() {
  unsigned long currentMillis = millis();

  if (state == 1 && (currentMillis - lastRFIDResetTime >= 500)) { // 500ms reset interval when feeder is open
    resetRFIDModule();
    lastRFIDResetTime = currentMillis;
  } else if (currentMillis - lastRFIDResetTime >= rfidResetInterval) {
    resetRFIDModule();
    lastRFIDResetTime = currentMillis;
  }

  // Check Wi-Fi connection status and reconnect if disconnected
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
  }

  // Read the state of the push button
  int reading = digitalRead(buttonPin);

  // Check if the button state has changed
  if (reading != lastButtonState) {
    lastDebounceTime = millis(); // reset the debounce timer
    buttonHandled = false; // reset the button handled flag when state changes
    if (reading == LOW) { // button is pressed
      buttonPressTime = millis(); // record the time the button was pressed
    }
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // if the button state has changed
    if (reading != buttonState) {
      buttonState = reading;
    }

    // if the button is still pressed and the press duration has passed
    if (buttonState == LOW && (millis() - buttonPressTime) >= pressDuration && !buttonHandled) {
      Serial.println("Push button pressed");
      feederRefilled = true;
      lastRFIDScanTime = millis();
      tagID = "button"; // Set tagID to "button" when the button is pressed
      if ((pos1 == SERVO1_CLOSED_POS || pos2 == SERVO2_CLOSED_POS) && state == 0) {
        openFeeder();
        mealStartTime = lastRFIDScanTime;
        lastUpdateTime = millis(); // Initialize last update time when feeder opens
      }
      buttonHandled = true; // mark button as handled
    }
  }

  lastButtonState = reading; // save the reading for the next loop

  if (Serial2.available()) {
    String rfidData = "";
    unsigned long startTime = millis();

    // Read data for a maximum of 100ms or until no more data is available
    while (millis() - startTime < 100) {
      if (Serial2.available()) {
        char c = Serial2.read();
        rfidData += c;
        startTime = millis(); // Reset the timer if we've received data
      }
    }

    if (rfidData.length() > 1) {
      Serial.println("Data available from RFID module");
      Serial.print("RFID Tag Detected: ");
      Serial.println(rfidData);

      // Blink the LED to indicate a successful read
      digitalWrite(LEDPin, HIGH);
      delay(200);
      digitalWrite(LEDPin, LOW);

      lastRFIDScanTime = millis();
      tagID = sanitizeTagID(rfidData); // Set tagID to the sanitized RFID tag

      if ((pos1 == SERVO1_CLOSED_POS || pos2 == SERVO2_CLOSED_POS) && state == 0) {
        openFeeder();
        mealStartTime = lastRFIDScanTime;
        lastUpdateTime = millis(); // Initialize last update time when feeder opens
      }
    }
  }

  if ((pos1 == SERVO1_OPEN_POS || pos2 == SERVO2_OPEN_POS) && state == 1) {
    waitToCloseFeeder();
  }
}

void openFeeder() {
  if ((pos1 != SERVO1_OPEN_POS || pos2 != SERVO2_OPEN_POS) && state == 0) {
    Serial.println("Feeder Activated");
    myServo1.attach(servo1Pin);
    myServo2.attach(servo2Pin);
    for (int i = 0; i <= 90; ++i) {
      myServo1.write(SERVO1_CLOSED_POS - i);
      myServo2.write(SERVO2_CLOSED_POS + i);
      delay(15);
    }
    pos1 = SERVO1_OPEN_POS;
    pos2 = SERVO2_OPEN_POS;
    digitalWrite(LEDPin, HIGH);
    myServo1.detach();
    myServo2.detach();
  }
  lastRFIDScanTime = millis();
  state = 1;
  rfidResetInterval = 500; // Increase reset frequency when feeder is open
}

void closeFeeder() {
  rfidResetInterval = 5000; // Restore reset interval

  if ((pos1 != SERVO1_CLOSED_POS || pos2 != SERVO2_CLOSED_POS) && state == 1) {
    Serial.println("Closing feeder due to inactivity");
    myServo1.attach(servo1Pin);
    myServo2.attach(servo2Pin);
    for (int i = 0; i <= 90; ++i) {
      myServo1.write(SERVO1_OPEN_POS + i);
      myServo2.write(SERVO2_OPEN_POS - i);
      delay(15);
    }
    pos1 = SERVO1_CLOSED_POS;
    pos2 = SERVO2_CLOSED_POS;
    state = 0;
    digitalWrite(LEDPin, LOW);
    myServo1.detach();
    myServo2.detach();
    logMealData(mealStartTime, millis(), feederRefilled, tagID);
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

String sanitizeTagID(String tagID) {
  String cleanTagID = "";
  for (char c : tagID) {
    if (isPrintable(c) && c != ',' && c != '\n' && c != '\r') {
      cleanTagID += c;
    }
  }
  return cleanTagID;
}

int getHighestLogID() {
  File logFile = SD.open("/meals.csv");
  if (!logFile) {
    Serial.println("Error opening log file to read log ID.");
    return 0;
  }

  int maxLogID = 0;
  while (logFile.available()) {
    String line = logFile.readStringUntil('\n');
    if (line.length() > 0) {
      int firstComma = line.indexOf(',');
      if (firstComma != -1) {
        int logID = line.substring(0, firstComma).toInt();
        if (logID > maxLogID) {
          maxLogID = logID;
        }
      }
    }
  }
  logFile.close();
  return maxLogID;
}

void logMealData(unsigned long startTime, unsigned long endTime, bool refilled, String tagID) {
  Serial.println("Attempting to write to SD card");

  // Calculate elapsed time
  float mealDuration = (endTime - startTime) / 1000.0; // in seconds

  // Read LDR value
  int ldrValue = analogRead(ldrPin);

  //float lightLevel = map(ldrValue, 0, 1023, 0, 100); // Map the LDR value to a percentage
  float lightLevel = ldrValue; // Map the LDR value to a percentage

  // Read BME680 values
  if (!bme.performReading()) {
    Serial.println("Failed to perform reading from BME680 sensor!");
    return;
  }
  float temperature = bme.temperature;
  float humidity = bme.humidity;
  float pressure = bme.pressure / 100.0; // convert to hPa
  float gasResistance = bme.gas_resistance / 1000.0; // convert to kOhms

  // Increment log ID
  logID++;

  // Create log entry in CSV format
  String logEntry = String(logID) + "," + tagID + "," + String(mealDuration, 2) + "," + String(lightLevel) + "," + String(temperature) + "," + 
                    String(humidity) + "," + String(pressure) + "," + String(gasResistance) + "," + 
                    String(refilled ? "true" : "false");
  Serial.println("Log Entry: " + logEntry);

  // Attempt to open the log file
  File logFile = SD.open("/meals.csv", FILE_APPEND);
  if (logFile) {
    Serial.println("File opened successfully.");
    logFile.println(logEntry);
    logFile.close();
    Serial.println("Logged to SD: " + logEntry);
  } else {
    Serial.println("Error opening log file.");

    // Create a new file if it does not exist
    logFile = SD.open("/meals.csv", FILE_WRITE);
    if (logFile) {
      Serial.println("New file created successfully.");
      logFile.println("LogID,TagID,Duration,Light Level,Temperature,Humidity,Pressure,Gas Resistance,Refilled"); // Add header
      logFile.println(logEntry);
      logFile.close();
      Serial.println("Logged to SD: " + logEntry);
    } else {
      Serial.println("Failed to create new file.");
    }
  }

  if (WiFi.status() == WL_CONNECTED) { // Check Wi-Fi connection status
    // Construct the full URL with parameters
    String fullPath = String(serverName) + "?logID=" + String(logID);
    fullPath += "&tagID=" + tagID;
    fullPath += "&duration=" + String(mealDuration, 2);
    fullPath += "&lightLevel=" + String(lightLevel);
    fullPath += "&temp=" + String(temperature);
    fullPath += "&humidity=" + String(humidity);
    fullPath += "&pressure=" + String(pressure);
    fullPath += "&gasRes=" + String(gasResistance);
    fullPath += "&refilled=" + String(refilled ? "true" : "false");

    // Debug: Starting HTTP GET request
    Serial.println("Starting connection to server...");
    Serial.print("Request URL: ");
    Serial.println(fullPath);

    // Make a HTTP GET request
    httpClient.begin(fullPath);
    int httpCode = httpClient.GET();

    if (httpCode > 0) {
      String response = httpClient.getString();
      Serial.print("HTTP Code: ");
      Serial.println(httpCode);
      Serial.print("Response: ");
      Serial.println(response);
    } else {
      Serial.print("Error on HTTP request: ");
      Serial.println(httpCode);
    }

    httpClient.end();
  } else {
    Serial.println("WiFi Disconnected");
  }
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi..");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");
}
