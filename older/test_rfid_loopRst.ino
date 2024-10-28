#include <SoftwareSerial.h>

SoftwareSerial rfidSerial(2, 3); 

const int rstPin = 5;

void setup() {
  Serial.begin(9600);
  while (!Serial) { ; }
  Serial.println("Serial Monitor Initialized");

  rfidSerial.begin(9600);
  Serial.println("RFID Reader Initialized");

  pinMode(rstPin, OUTPUT);
  digitalWrite(rstPin, HIGH); // Set RST pin to HIGH initially

  Serial.println("Setup complete");
}

void loop() {
  if (rfidSerial.available()) {
    String rfidData = "";
    while (rfidSerial.available()) {
      char c = rfidSerial.read();
      rfidData += c;
    }

    if (rfidData.length() > 0) { // Check if the string is not empty
      Serial.println("Data available from RFID module");
      Serial.print("RFID Tag Detected: ");
      Serial.println(rfidData);

      resetRFIDModule();
    }
  }
  delay(100); // Add a short delay to avoid flooding the serial monitor
}

void resetRFIDModule() {
  digitalWrite(rstPin, LOW);
  delay(100);
  digitalWrite(rstPin, HIGH);
  delay(100);
  Serial.println("RFID Reader Reset");
}
