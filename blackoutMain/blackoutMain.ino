
/*---Includes-----------------------------------------------------------*/
#include "statemachine.h"
#include "sensors.h"
#include "calculations.h"
#include "deployment.h"
/*---Variables----------------------------------------------------------*/
static States state = STANDBY;
static float alt, prev_alt, delta_alt, pressure, groundPressure, groundAlt;

// GPS
static double lat, lon, gpsAlt, gpsSats, distToTrgt;

// GY-91
static float barData[2];
static float accelData[4];
static float magData[4];
static int photo_resistor;

/*---Functions----------------------------------------------------------*/
void setup() {
  Serial.begin(115200);
  initSensors(); // Initialize sensors
  initDeployment(); // Inititalize deployment event pins
}

void loop() {
  static unsigned long timestamp;
  static unsigned long old_time = 0; //ms
  static unsigned long new_time = 0; //ms
  unsigned long delta_time;
  static uint16_t time_interval = NOMINAL_POLLING_TIME_INTERVAL; //ms


  if(Serial.read() == 'a'){
    //deploy arms
    deployRotors();
  }
  if(Serial.read() == 'd'){
    //deploy chute
    deployChute();
  }
  if(Serial.read() == 'r'){
    releaseChute();
  }



  if (state == LANDED){
    time_interval = LANDED_POLLING_TIME_INTERVAL;
  } else {
     time_interval = NOMINAL_POLLING_TIME_INTERVAL;
  }


  new_time = millis();
  if ((new_time - old_time) >= time_interval) {
    delta_time = new_time - old_time;
    old_time = new_time;

    pollSensors(&lat, &lon, &gpsAlt, &gpsSats, barData, accelData, magData, &photo_resistor);
    crunchNumbers(barData, accelData, magData, &pressure, &groundPressure, &prev_alt, &alt, &delta_alt, &lat, &lon, &distToTrgt, &delta_time);
    stateMachine(&alt, &delta_alt, &pressure, &groundPressure, &groundAlt, &distToTrgt, &state, &photo_resistor);
    }

}
