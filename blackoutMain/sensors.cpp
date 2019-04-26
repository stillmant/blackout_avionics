/*Includes------------------------------------------------------------*/
#include "sensors.h"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>

/*Set up------------------------------------------------------------*/
// GPS
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);
#define TASK_SERIAL_RATE 100

/*Variables------------------------------------------------------------*/
uint32_t nextSerialTaskTs = 0; // for GPS
/*Functions------------------------------------------------------------*/
/*
 * @brief  Initializes all the sensors
 * @return void
 */

void initSensors()
{
    SerialGPS.begin(9600, SERIAL_8N1, 16, 17); // GPS on pins 16 & 17
    return;
}

/**
  * @brief  Polls all the sensors
  * @param  double lat - latitude
  * @param  double lon - longitude
  * @return None
  */
void pollSensors(double *lat, double *lon)
{
    // Get GPS latitude and longitude
    while (SerialGPS.available() > 0) {
        gps.encode(SerialGPS.read());
    }

    // if (nextSerialTaskTs < millis()) { //// not sure if this condition is necessary 
    *lat = gps.location.lat();
    *lon = gps.location.lng();
    // may need delay (nextSerialTaskTs = millis() + TASK_SERIAL_RATE;)
    //}
    // end GPS

    // IMU
    
    // ACCEL

}