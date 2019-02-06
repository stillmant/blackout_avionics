#include <TinyGPS++.h>
#include <HardwareSerial.h>

// GPS
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);

#define TASK_SERIAL_RATE 100
uint32_t nextSerialTaskTs = 0;

void setup() {
  Serial.begin(115200);
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);

}

void loop() {
  delay(2000);
  
  while (SerialGPS.available() > 0) {
        gps.encode(SerialGPS.read());
  }

  if (nextSerialTaskTs < millis()) {
    Serial.println("Xx---------- GPS ----------xX");
    Serial.print("time ");
    Serial.print(gps.time.hour());Serial.print(":"); // Hour (0-23) (u8)
    Serial.print(gps.time.minute());Serial.print(":"); // Minute (0-59) (u8)
    Serial.println(gps.time.second()); // Second (0-59) (u8)
    Serial.println("<------->Coordinates<------->");
    Serial.print("LAT: ");  Serial.print(gps.location.lat(), 6);
    Serial.print("  LONG: "); Serial.print(gps.location.lng(), 6);
    Serial.print("  ALT: ");  Serial.println(gps.altitude.meters());
    Serial.print("Satellites: ");  Serial.println(gps.satellites.value());
    Serial.println();

    nextSerialTaskTs = millis() + TASK_SERIAL_RATE;
    }
}
