
/*---Includes-----------------------------------------------------------*/
#include "esp32-hal-ledc.h"
#include "statemachine.h"
#include "sensors.h"
#include "calculations.h"
#include "deployment.h"
#include "flight.h"
/*---Variables----------------------------------------------------------*/
static States state = ALTHOLD;
static float alt, prev_alt, delta_alt, pressure, groundPressure, groundAlt;


uint8_t roll = 0;
uint8_t pitch = 15;
uint8_t yaw = 2;
uint8_t throt = 4;

uint8_t arm = 5;
uint8_t manual = 18;
#define buttonPin 34         //CHANGE TO RIGHT PIN
int buttonState = 0;

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

  pinMode(buttonPin, INPUT);

  ledcAttachPin(roll, 1);
  ledcAttachPin(pitch, 2);
  ledcAttachPin(yaw, 3);
  ledcAttachPin(throt, 4);

  ledcAttachPin(arm, 5);
  ledcAttachPin(manual, 6);

  // ledcSetup(channel, Hz, 16-bit resolution)
  ledcSetup(1, 50, 16);
  ledcSetup(2, 50, 16);
  ledcSetup(3, 50, 16);
  ledcSetup(4, 50, 16);
  ledcSetup(5, 50, 16);
  ledcSetup(6, 50, 16);

  ledcWrite(5,3222);   //FOR ARMING - set THROT to 999 (0%), set AUX1 to 999 (0%)
  ledcWrite(3,3222);

  ledcWrite(1,COUNT_MID);
  ledcWrite(2,COUNT_MID);
  ledcWrite(4,COUNT_MID);
  ledcWrite(6,COUNT_MID);

  delay(5000);          //Wait 5 seconds for FC startup

  ledcWrite(5,6540);   //ARMING - set AUX1 permantly at 2001 (100%) to stay armed

  delay(500);
}

void loop() {
  static unsigned long timestamp;
  static unsigned long old_time = 0; //ms
  static unsigned long new_time = 0; //ms
  unsigned long delta_time;
  static uint16_t time_interval = NOMINAL_POLLING_TIME_INTERVAL; //ms
  double PIDout;
  static int buttonState = 0;
  static bool motor = false, delay_done = false, reset = true;

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

    buttonState = analogRead(buttonPin);
    if (buttonState <= 3800 ){
      ledcWrite(5,COUNT_LOW);
      ledcWrite(3, COUNT_LOW);
      ledcWrite(1,COUNT_MID);
      ledcWrite(2,COUNT_MID);
      ledcWrite(4,COUNT_MID);
      motor = false;
      delay_done = false;
      reset = true;
      Serial.println("OFF");
    }
    else{
      // ledcWrite(5,6540);
      motor = true;
      Serial.println("ARMED");
    }

    if (motor == true){
      if (delay_done == false){
        ledcWrite(5,6540);
        delay(1500);
        delay_done = true;
      }
      if (reset == true){
        PIDout = runPIDhold(&alt, true);
        reset = false;
      }

    }

    if (state == ALTHOLD && motor == true){
      Serial.println("ALT HOLD");
      PIDout = runPIDhold(&alt, false);
      ledcWrite(3, PIDout);
    }
    else if (state == LANDING && motor == true){
      Serial.println("LANDING");
      PIDout = runPIDland(&alt);
      ledcWrite(3, PIDout);
    }
    else if (state == LANDED && motor == true){
      Serial.println("LANDED");
      ledcWrite(5, COUNT_LOW);
    }
  }

}
