/*Includes------------------------------------------------------------*/
#include "sensors.h"

#include <Arduino.h>
#include <HardwareSerial.h>

#include "TinyGPS++.h"
#include "Adafruit_BMP280.h"
#include "MPU9250_asukiaaa.h"

/*Definitions----------------------------------------------------------*/
#define TASK_SERIAL_RATE 100 // for GPS
/*Set up---------------------------------------------------------------*/

// GPS
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);

// GY-91 
MPU9250 mpu9250;
Adafruit_BMP280 bmp; // I2C
HardwareSerial SerialMPU(2);
HardwareSerial SerialBMP(2);
/*Variables------------------------------------------------------------*/
uint32_t nextSerialTaskTs = 0; // for GPS
/*Functions------------------------------------------------------------*/
/*
 * @brief  Initializes all the sensors
 * @return void
 */
void initSensors(){
    SerialGPS.begin(9600, SERIAL_8N1, 16, 17); // GPS on pins 16 & 17
    
    SerialMPU.begin(115200, SERIAL_8N1, 21, 22);
    SerialBMP.begin(115200, SERIAL_8N1, 21, 22);

    #ifdef _ESP32_HAL_I2C_H_
        // for esp32
        // If not default values, check
        Wire.begin(SDA_PIN, SCL_PIN); //sda, scl
    #else
        Wire.begin();
    #endif

    // BMP280
    Serial.println(F("BMP280 test"));
    if (!bmp.begin()) {
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
    }

    // MPU9250
    mpu9250.setWire(&Wire); 
    mpu9250.beginAccel();
    mpu9250.beginMag();

    return;
}

/**
  * @brief  Polls all the sensors
  * @param  double lat - latitude
  * @param  double lon - longitude
  * @return None
  */
void pollSensors(double *lat, double *lon, double *gpsAlt, double *gpsSats, float barData[], float accelData[], float magData[])
{
    // Get GPS latitude and longitude
    while (SerialGPS.available() > 0) {
        gps.encode(SerialGPS.read());
    }

    // if (nextSerialTaskTs < millis()) { //// not sure if this condition is necessary 
    *lat = gps.location.lat();
    *lon = gps.location.lng();
    *gpsAlt = gps.altitude.meters();
    *gpsSats = gps.satellites.value();
    // may need delay (nextSerialTaskTs = millis() + TASK_SERIAL_RATE;)
    //}
    // end GPS

    // IMU
    
    // BMP280
    barData[0] = bmp.readPressure(); 
    barData[1] = bmp.readAltitude(SEA_PRESSURE);

    // MPU950
    mpu9250.accelUpdate();
    accelData[0] = mpu9250.accelX();
    accelData[1] = mpu9250.accelY();
    accelData[2] = mpu9250.accelZ();
    accelData[3] = mpu9250.accelSqrt();

    mpu9250.magUpdate();
    magData[0] = mpu9250.magX();
    magData[1] = mpu9250.magY();
    magData[2] = mpu9250.magZ();
    magData[3] = mpu9250.magHorizDirection();

}
