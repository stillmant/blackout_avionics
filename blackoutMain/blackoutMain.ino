
/*---Includes-----------------------------------------------------------*/
#include "esp32-hal-ledc.h"
#include "statemachine.h"
#include "sensors.h"
#include "calculations.h"
#include "deployment.h"
#include "flight.h"
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

// pressure arrays
static float pressure_set[PRESSURE_AVG_SET_SIZE];
static float ground_pressure_set[GROUND_PRESSURE_AVG_SET_SIZE];

/*---Functions----------------------------------------------------------*/
void setup() {
  Serial.begin(115200);
  initSensors(); // Initialize sensors
  initDeployment(); // Inititalize deployment event pins

  for (int i = 0; i < GROUND_PRESSURE_AVG_SET_SIZE; i++){
    pollSensors(&lat, &lon, &gpsAlt, &gpsSats, barData, accelData, magData, &photo_resistor);
    addToPressureSet(ground_pressure_set, barData[0], GROUND_PRESSURE_AVG_SET_SIZE);
    delay(50);
  }

  groundPressure = calculateGroundPressureAverage(ground_pressure_set, GROUND_PRESSURE_AVG_SET_SIZE);

  for (int i = 0; i < PRESSURE_AVG_SET_SIZE; i++) // for moving average
  {
      pressure_set[i] = groundPressure;
  }
}

void loop() {
  static unsigned long timestamp;
  static unsigned long old_time = 0; //ms
  static unsigned long new_time = 0; //ms
  unsigned long delta_time;
  static uint16_t time_interval = NOMINAL_POLLING_TIME_INTERVAL; //ms
  double PIDout;

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
    crunchNumbers(barData, accelData, magData, &pressure, &groundPressure, &prev_alt, &alt, &delta_alt, &delta_time, pressure_set);
    stateMachine(&alt, &delta_alt, &pressure, &groundPressure, &groundAlt, &distToTrgt, &state, &photo_resistor, accelData, ground_pressure_set);

    if (state == ALTHOLD){
      PIDout = runPIDhold(&alt);
      ledcWrite(3, PIDout);
    }
    else if (state == LANDING){
      PIDout = runPIDland(&alt);
      ledcWrite(3, PIDout);
    }
    else if (state == LANDED){
      ledcWrite(5, COUNT_LOW);
    }
  }
  Serial.print("barometer: ");
  Serial.println(barData[0]);
  Serial.print("temp: ");
  Serial.println(barData[1]);
  Serial.print("Ground Altitude: ");
  Serial.println(groundAlt);
  Serial.print("Altitude: ");
  Serial.println(alt);
  Serial.print("Photoresistor: ");
  Serial.println(photo_resistor);
  Serial.print("Accel amg: ");
  Serial.println(accelData[3]);


}
