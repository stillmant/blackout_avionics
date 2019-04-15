
/*---Includes-----------------------------------------------------------*/
#include "statemachine.h"

/*---Variables----------------------------------------------------------*/



/*---Functions----------------------------------------------------------*/
void setup() {
  // Initialize sensors

}

void loop() {
  static unsigned long timestamp;
  static unsigned long old_time = 0; //ms
  static unsigned long new_time = 0; //ms
  unsigned long delta_time;
  static uint16_t time_interval = NOMINAL_POLLING_TIME_INTERVAL; //ms
  
  if (state == LANDED){
    time_interval = LANDED_POLLING_TIME_INTERVAL;
  } else {
    time_interval = NOMINAL_POLLING_TIME_INTERVAL;
  }


  new_time = millis();
  if ((new_time - old_time) >= time_interval) {
    delta_time = new_time - old_time;
    old_time = new_time;

    pollSensors();
    calculateValues();
    stateMachine();
    }

}
