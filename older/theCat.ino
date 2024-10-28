#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <ArduinoHttpClient.h>
#include <Servo.h>
#include <Arduino_LSM6DSOX.h> 
#include <PDM.h> 
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

const char* ssid = "";
const char* password = "";

const char* serverName = "script.google.com";
const int port = 443;
const char* path = "/macros/s/AKfycbxLk0AUvrZxNG8ZYEUGZZpyWSjIkLb0AvH-lx9giByP0r3ZlGCannJT1hCHVZ-aXqYEpw/exec";

// reader pins
const int rfidTxPin = 1;  
const int rfidRstPin = 7; 
const int buttonPin = 2;  
const int ldrPin = A0;   

Servo myServo1;
Servo myServo2;

const int LEDPin = LED_BUILTIN;  
const int servo1Pin = 9;
const int servo2Pin = 10;  
const int chipSelect = 8;  // CS pin for SD card reader on RP2040,

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

WiFiSSLClient wifi;
HttpClient client = HttpClient(wifi, serverName, port);

float displacementX = 0, displacementY = 0, displacementZ = 0;
float velocityX = 0, velocityY = 0, velocityZ = 0;
float lastAccelX = 0, lastAccelY = 0, lastAccelZ = 0;
unsigned long lastUpdateTime = 0;
bool feederRefilled = false;

const unsigned long debounceDelay = 50;
const unsigned long pressDuration = 500; 
unsigned long lastDebounceTime = 0;
unsigned long buttonPressTime = 0; 
int lastButtonState = HIGH; 
int buttonState = HIGH; 
bool buttonHandled = false; 

Adafruit_BME680 bme; // create a BME680 object

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println("Serial Monitor Initialized");

  pinMode(LEDPin, OUTPUT);
  pinMode(rfidRstPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP); // Initialize push button pin with an internal pull-up resistor
  pinMode(ldrPin, INPUT);
  digitalWrite(rfidRstPin, HIGH);
  Wire.begin();  // I2C communication

  Serial1.begin(9600);
  Serial.println("RFID Reader Initialized");

  myServo1.attach(servo1Pin);
  myServo2.attach(servo2Pin);
  myServo1.write(SERVO_CLOSED_POS);
  myServo2.write(SERVO_CLOSED_POS);


  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

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

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to initialize PDM!");
    while (1);
  }


  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  lastUpdateTime = millis();
  Serial.println("Setup complete");
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastRFIDResetTime >= rfidResetInterval) {
    resetRFIDModule();
    lastRFIDResetTime = currentMillis;
  }

  // Read the state of the push button
  int reading = digitalRead(buttonPin);

  // Check if the button state has changed
  if (reading != lastButtonState) {
    lastDebounceTime = millis(); 
    buttonHandled = false; 
    if (reading == LOW) {
      buttonPressTime = millis(); 
    }
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // if the button state has changed
    if (reading != buttonState) {
      buttonState = reading;
    }

    if (buttonState == LOW && (millis() - buttonPressTime) >= pressDuration && !buttonHandled) {
      Serial.println("Push button pressed");
      feederRefilled = true;
      lastRFIDScanTime = millis();
      if ((pos1 == SERVO_CLOSED_POS || pos2 == SERVO_CLOSED_POS) && state == 0) {
        openFeeder();
        mealStartTime = lastRFIDScanTime;
        lastUpdateTime = millis();
      }
      buttonHandled = true; // got to mark button as handled
    }
  }

  lastButtonState = reading; // save the reading for the next loop

  if (Serial1.available()) {
    String rfidData = "";
    unsigned long startTime = millis();

    while (millis() - startTime < 100) {
      if (Serial1.available()) {
        char c = Serial1.read();
        rfidData += c;
        startTime = millis(); 
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

      if ((pos1 == SERVO_CLOSED_POS || pos2 == SERVO_CLOSED_POS) && state == 0) {
        openFeeder();
        mealStartTime = lastRFIDScanTime;
        lastUpdateTime = millis(); // Initialize last update time when feeder opens
      }
    }
  }

  if ((pos1 == SERVO_OPEN_POS || pos2 == SERVO_OPEN_POS) && state == 1) {
    myServo1.detach();
    myServo2.detach();
    rfidResetInterval = 500;
    calculateDisplacement();
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
    logMealData(mealStartTime, millis(), feederRefilled);
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

void calculateDisplacement() {
  float accelX, accelY, accelZ;
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastUpdateTime) / 1000.0; // Convert to seconds

  if (IMU.readAcceleration(accelX, accelY, accelZ)) {
    velocityX += ((accelX + lastAccelX) / 2.0) * deltaTime;
    velocityY += ((accelY + lastAccelY) / 2.0) * deltaTime;
    velocityZ += ((accelZ + lastAccelZ) / 2.0) * deltaTime;

    displacementX += velocityX * deltaTime;
    displacementY += velocityY * deltaTime;
    displacementZ += velocityZ * deltaTime;
    lastAccelX = accelX;
    lastAccelY = accelY;
    lastAccelZ = accelZ;

    lastUpdateTime = currentTime;
  }
}

void logMealData(unsigned long startTime, unsigned long endTime, bool refilled) {
  Serial.println("Attempting to write to SD card");

  float mealDuration = (endTime - startTime) / 1000.0; // in seconds

  int ldrValue = analogRead(ldrPin);
  float lightLevel = map(ldrValue, 0, 1023, 0, 100); // Map the LDR value to a percentage

  // ME680 values
  if (!bme.performReading()) {
    Serial.println("Failed to perform reading from BME680 sensor!");
    return;
  }
  float temperature = bme.temperature;
  float humidity = bme.humidity;
  float pressure = bme.pressure / 100.0; // convert to hPa
  float gasResistance = bme.gas_resistance / 1000.0; // convert to kOhms

  // log entry
  String logEntry = "Meal Duration: " + String(mealDuration, 2) + "s";
  logEntry += ", Displacement: (" + String(displacementX) + ", " + String(displacementY) + ", " + String(displacementZ) + ")";
  logEntry += ", Light Level: " + String(lightLevel) + "%";
  logEntry += ", Temperature: " + String(temperature) + "°C";
  logEntry += ", Humidity: " + String(humidity) + "%";
  logEntry += ", Pressure: " + String(pressure) + "hPa";
  logEntry += ", Gas Resistance: " + String(gasResistance) + "kOhms";
  logEntry += ", Refilled: " + String(refilled ? "true" : "false");
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
    String fullPath = String(path) + "?duration=" + String(mealDuration, 2);
    fullPath += "&dispX=" + String(displacementX) + "&dispY=" + String(displacementY) + "&dispZ=" + String(displacementZ);
    fullPath += "&lightLevel=" + String(lightLevel);
    fullPath += "&temp=" + String(temperature);
    fullPath += "&humidity=" + String(humidity);
    fullPath += "&pressure=" + String(pressure);
    fullPath += "&gasRes=" + String(gasResistance);
    fullPath += "&refilled=" + String(refilled ? "true" : "false");

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

    if (statusCode == 302) {
      String redirectUrl;
      Serial.println("Headers:");
      while (client.headerAvailable()) {
        String headerName = client.readHeaderName();
        String headerValue = client.readHeaderValue();
        Serial.print(headerName);
        Serial.print(": ");
        Serial.println(headerValue);
        if (headerName == "Location") {
          redirectUrl = headerValue;
        }
      }
      if (redirectUrl.length() > 0) {
        Serial.print("Redirecting to: ");
        Serial.println(redirectUrl);
        client.get(redirectUrl.c_str());

        statusCode = client.responseStatusCode();
        response = client.responseBody();

        Serial.print("Status code after redirect: ");
        Serial.println(statusCode);

        Serial.print("Response after redirect: ");
        Serial.println(response);
      } else {
        // Attempt to extract redirect URL from response body
        int startIndex = response.indexOf("href=\"") + 6;
        int endIndex = response.indexOf("\">here", startIndex);
        if (startIndex > 6 && endIndex > startIndex) {
          redirectUrl = response.substring(startIndex, endIndex);
          Serial.print("Extracted redirect URL: ");
          Serial.println(redirectUrl);
          client.get(redirectUrl.c_str());

          statusCode = client.responseStatusCode();
          response = client.responseBody();

          Serial.print("Status code after redirect: ");
          Serial.println(statusCode);

          Serial.print("Response after redirect: ");
          Serial.println(response);
        } else {
          Serial.println("Error: Redirect URL not found in response body");
        }
      }
    }

    if (statusCode != 200) {
      Serial.println("Error: Failed to connect to server or invalid response");
    }

    // Free resources
    client.stop();
  } else {
    Serial.println("WiFi Disconnected");
  }

  displacementX = 0;
  displacementY = 0;
  displacementZ = 0;
  velocityX = 0;
  velocityY = 0;
  velocityZ = 0;
}
