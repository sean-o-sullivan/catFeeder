#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <ArduinoHttpClient.h>

const char* ssid = "";
const char* password = "";

const char* serverName = "script.google.com";
const int port = 443;
const char* path = "/macros/s/AKfycbxLk0AUvrZxNG8ZYEUGZZpyWSjIkLb0AvH-lx9giByP0r3ZlGCannJT1hCHVZ-aXqYEpw/exec";

const int chipSelect = 8; 

WiFiSSLClient wifi;
HttpClient client = HttpClient(wifi, serverName, port);

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println("Serial Monitor Initialized");

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
  // Example data for logging
  unsigned long startTime = millis();
  delay(1000);  // Simulate some delay
  unsigned long endTime = millis();

  logMealData(startTime, endTime);

  delay(10000);  
}

void logMealData(unsigned long startTime, unsigned long endTime) {
  Serial.println("Attempting to write to SD card");

  unsigned long elapsedMillis = endTime - startTime;

  Serial.print("Elapsed milliseconds: ");
  Serial.println(elapsedMillis);

  String startTimestamp = String(startTime / 1000);
  String endTimestamp = String(endTime / 1000);

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

  if (WiFi.status() == WL_CONNECTED) { 

    String fullPath = String(path) + "?start=" + startTimestamp + "&end=" + endTimestamp;

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
}
