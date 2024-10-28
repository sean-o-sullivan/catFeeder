const int rstPin = 7;
const int LEDPin = LED_BUILTIN;

unsigned long lastRFIDResetTime = 0;
const unsigned long rfidResetInterval = 5000; 

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println("Serial Monitor Initialized");

  pinMode(LEDPin, OUTPUT);

  Serial1.begin(9600);
  Serial.println("RFID Reader Initialized");

  pinMode(rstPin, OUTPUT);
  digitalWrite(rstPin, HIGH); 

  Serial.println("Setup complete");
}

void loop() {
  unsigned long currentMillis = millis();

  // Periodically reset the RFID reader
  if (currentMillis - lastRFIDResetTime >= rfidResetInterval) {
    resetRFIDModule();
    lastRFIDResetTime = currentMillis;
  }

  // Check for RFID data
  if (Serial1.available()) {
    String rfidData = "";
    unsigned long startTime = millis();
    
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

      digitalWrite(LEDPin, HIGH);
      delay(200);
      digitalWrite(LEDPin, LOW);
    }
  }
}

void resetRFIDModule() {
  digitalWrite(rstPin, LOW);
  delay(50);
  digitalWrite(rstPin, HIGH);
  delay(50);
  Serial.println("RFID Reader Reset");
}