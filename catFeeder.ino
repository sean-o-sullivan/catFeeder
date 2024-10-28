#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ESP32Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>


const char* ssid = "my_home_wifi";
const char* password = "private";

const char* serverName = "https://script.google.com/macros/s/AKfycbyOmO3fYNebgT6COppmaw9mCtrjy8Lxqp3J1xm9vCngsqNl23Lx_VhE1Ai8ER00gcmQgQ/exec";

// rfid reader pins
const int rfidTxPin = 3;  
const int rfidRstPin = 4; 
const int buttonPin = 15;   
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
unsigned long rfidResetInterval = 5000; // adjusted to 5000 ms for normal state
unsigned long lastRFIDResetTime = 0;
int state = 0;
bool feederRefilled = false;
int logID = 0;
String tagID = "button";

WiFiClient wifi;
HTTPClient httpClient;

const unsigned long debounceDelay = 50; 
const unsigned long pressDuration = 500;
unsigned long lastDebounceTime = 0; 
unsigned long buttonPressTime = 0;
int lastButtonState = HIGH; // the previous state of the button
int buttonState = HIGH; 
bool buttonHandled = false; // flag to track if button press has been handled

Adafruit_BME680 bme; 

const unsigned long wifiReconnectInterval = 30 * 60 * 1000; // 30 minutes
unsigned long lastWifiReconnectAttempt = 0;
bool wifiConnected = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println("Serial Monitor Initialized");

  pinMode(LEDPin, OUTPUT);
  pinMode(rfidRstPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ldrPin, INPUT);
  digitalWrite(rfidRstPin, HIGH);
  Wire.begin(26, 27);

  Serial2.begin(9600, SERIAL_8N1, rfidTxPin, -1);
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
  bme.setGasHeater(320, 150);

  SPI.begin(18, 19, 23, chipSelect);
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }
  Serial.println("SD card initialized.");

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

  logID = getHighestLogID();

  // disable auto-reconnection
  WiFi.setAutoReconnect(false);

  // Try to connect to WiFi, but don't block if it fails
  connectToWiFi(1);  // Retry 5 times

  lastUpdateTime = millis();
  lastWifiReconnectAttempt = millis();
  Serial.println("Setup complete");
}

void loop() {
  unsigned long currentMillis = millis();

  // Periodic Wi-Fi reconnection attempts
  if (!wifiConnected && currentMillis - lastWifiReconnectAttempt >= wifiReconnectInterval) {
    Serial.println("Attempting periodic Wi-Fi reconnection...");
    connectToWiFi(5); // Retry 5 times
    lastWifiReconnectAttempt = currentMillis;
  }

  if (state == 1 && (currentMillis - lastRFIDResetTime >= 500)) {
    resetRFIDModule();
    lastRFIDResetTime = currentMillis;
  } else if (currentMillis - lastRFIDResetTime >= rfidResetInterval) {
    resetRFIDModule();
    lastRFIDResetTime = currentMillis;
  }

  int reading = digitalRead(buttonPin);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
    buttonHandled = false;
    if (reading == LOW) {
      buttonPressTime = millis();
    }
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
    }

    if (buttonState == LOW && (millis() - buttonPressTime) >= pressDuration && !buttonHandled) {
      Serial.println("Push button pressed");
      feederRefilled = true;
      lastRFIDScanTime = millis();
      tagID = "button";
      if ((pos1 == SERVO1_CLOSED_POS || pos2 == SERVO2_CLOSED_POS) && state == 0) {
        openFeeder();
        mealStartTime = lastRFIDScanTime;
        lastUpdateTime = millis();
      }
      buttonHandled = true;
    }
  }

  lastButtonState = reading;

  if (Serial2.available()) {
    String rfidData = "";
    unsigned long startTime = millis();

    while (millis() - startTime < 100) {
      if (Serial2.available()) {
        char c = Serial2.read();
        rfidData += c;
        startTime = millis();
      }
    }

    if (rfidData.length() > 1) {
      Serial.println("Data available from RFID module");
      Serial.print("RFID Tag Detected: ");
      Serial.println(rfidData);

      digitalWrite(LEDPin, HIGH);
      delay(200);
      digitalWrite(LEDPin, LOW);

      lastRFIDScanTime = millis();
      tagID = sanitizeTagID(rfidData);

      if ((pos1 == SERVO1_CLOSED_POS || pos2 == SERVO2_CLOSED_POS) && state == 0) {
        openFeeder();
        mealStartTime = lastRFIDScanTime;
        lastUpdateTime = millis();
      }
    }
  }

  if ((pos1 == SERVO1_OPEN_POS || pos2 == SERVO2_OPEN_POS) && state == 1) {
    waitToCloseFeeder();
  }

  static unsigned long lastOfflineUpload = 0;
  if (wifiConnected && currentMillis - lastOfflineUpload >= 3600000) { // every hour
    uploadOfflineData();
    lastOfflineUpload = currentMillis;
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
  rfidResetInterval = 500;
}

void closeFeeder() {
  rfidResetInterval = 5000;

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
    feederRefilled = false; // reset refilled status after logging data
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

  float mealDuration = (endTime - startTime) / 1000.0;
  int ldrValue = analogRead(ldrPin);
  float lightLevel = ldrValue;

  if (!bme.performReading()) {
    Serial.println("Failed to perform reading from BME680 sensor!");
    return;
  }
  float temperature = bme.temperature;
  float humidity = bme.humidity;
  float pressure = bme.pressure / 100.0;
  float gasResistance = bme.gas_resistance / 1000.0;

  logID++;

  String logEntry = String(logID) + "," + tagID + "," + String(mealDuration, 2) + "," + String(lightLevel) + "," + String(temperature) + "," +
                    String(humidity) + "," + String(pressure) + "," + String(gasResistance) + "," +
                    String(refilled ? "true" : "false");
  Serial.println("Log Entry: " + logEntry);

  File logFile = SD.open("/meals.csv", FILE_APPEND);
  if (logFile) {
    Serial.println("File opened successfully.");
    logFile.println(logEntry);
    logFile.close();
    Serial.println("Logged to SD: " + logEntry);
  } else {
    Serial.println("Error opening log file.");

    logFile = SD.open("/meals.csv", FILE_WRITE);
    if (logFile) {
      Serial.println("New file created successfully.");
      logFile.println("LogID,TagID,Duration,Light Level,Temperature,Humidity,Pressure,Gas Resistance,Refilled");
      logFile.println(logEntry);
      logFile.close();
      Serial.println("Logged to SD: " + logEntry);
    } else {
      Serial.println("Failed to create new file.");
    }
  }

  if (wifiConnected) {
    String fullPath = String(serverName) + "?logID=" + String(logID);
    fullPath += "&tagID=" + tagID;
    fullPath += "&duration=" + String(mealDuration, 2);
    fullPath += "&lightLevel=" + String(lightLevel);
    fullPath += "&temp=" + String(temperature);
    fullPath += "&humidity=" + String(humidity);
    fullPath += "&pressure=" + String(pressure);
    fullPath += "&gasRes=" + String(gasResistance);
    fullPath += "&refilled=" + String(refilled ? "true" : "false");

    Serial.println("Starting connection to server...");
    Serial.print("Request URL: ");
    Serial.println(fullPath);

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
      Serial.println("Storing data locally due to HTTP error.");
      storeOfflineData(logEntry);
    }

    httpClient.end();
  } else {
    Serial.println("WiFi not connected. Storing data for later upload.");
    storeOfflineData(logEntry);
  }
}

void connectToWiFi(int maxRetries) {
  int retries = 0;
  const unsigned long wifiTimeout = 10000; // 10 seconds timeout per attempt

  // ensure that any previous connection attempts are terminated
  WiFi.disconnect(true);

  while (WiFi.status() != WL_CONNECTED && retries < maxRetries) {
    Serial.print("Attempting to connect to WiFi (Attempt ");
    Serial.print(retries + 1);
    Serial.println(")...");

    WiFi.begin(ssid, password);

    unsigned long startAttemptTime = millis();

    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < wifiTimeout) {
      delay(1000);
      Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nConnected to WiFi");
      wifiConnected = true;
      break;
    } else {
      Serial.println("\nFailed to connect to WiFi. Retrying...");
      retries++;
      // Ensure disconnection before next attempt
      WiFi.disconnect(true);
      delay(5000); // Delay before retrying
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Unable to connect to WiFi after retries. Running in offline mode.");
    wifiConnected = false;
  }
}

void storeOfflineData(String logEntry) {
  File offlineFile = SD.open("/offlineData.csv", FILE_APPEND);
  if (offlineFile) {
    offlineFile.println(logEntry);
    offlineFile.close();
    Serial.println("Stored offline data: " + logEntry);
  } else {
    Serial.println("Failed to open offline data file.");
  }
}

void uploadOfflineData() {
  if (wifiConnected) {
    File offlineFile = SD.open("/offlineData.csv");
    if (offlineFile) {
      Serial.println("Attempting to upload offline data...");
      while (offlineFile.available()) {
        String logEntry = offlineFile.readStringUntil('\n');

        // Parse the logEntry to extract data
        String parsedURL = parseLogEntryToURL(logEntry);
        if (parsedURL == "") {
          Serial.println("Failed to parse log entry. Skipping...");
          continue;
        }

        String fullPath = serverName + parsedURL;

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
          Serial.println("Will retry uploading this entry later.");
          break; // Exit the loop to retry later
        }

        httpClient.end();
      }
      offlineFile.close();

      if (!offlineFile.available()) {
        SD.remove("/offlineData.csv");
        Serial.println("Offline data uploaded and file deleted.");
      }
    } else {
      Serial.println("No offline data to upload.");
    }
  } else {
    Serial.println("WiFi not connected, cannot upload offline data.");
  }
}

String parseLogEntryToURL(String logEntry) {
  // Split the logEntry by commas
  int indices[9];
  int idx = -1;
  for (int i = 0; i < 9; i++) {
    idx = logEntry.indexOf(',', idx + 1);
    if (idx == -1) {
      return ""; // Parsing error
    }
    indices[i] = idx;
  }

  // extract each field
  String logID = logEntry.substring(0, indices[0]);
  String tagID = logEntry.substring(indices[0] + 1, indices[1]);
  String duration = logEntry.substring(indices[1] + 1, indices[2]);
  String lightLevel = logEntry.substring(indices[2] + 1, indices[3]);
  String temp = logEntry.substring(indices[3] + 1, indices[4]);
  String humidity = logEntry.substring(indices[4] + 1, indices[5]);
  String pressure = logEntry.substring(indices[5] + 1, indices[6]);
  String gasRes = logEntry.substring(indices[6] + 1, indices[7]);
  String refilled = logEntry.substring(indices[7] + 1);

  // URL parameters
  String url = "?logID=" + logID + "&tagID=" + tagID + "&duration=" + duration + "&lightLevel=" + lightLevel +
               "&temp=" + temp + "&humidity=" + humidity + "&pressure=" + pressure + "&gasRes=" + gasRes +
               "&refilled=" + refilled;

  return url;
}
