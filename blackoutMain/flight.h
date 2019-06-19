#ifndef FLIGHT_H
#define FLIGHT_H

// Polling times
#define LANDED_POLLING_TIME_INTERVAL 5000 //ms
#define NOMINAL_POLLING_TIME_INTERVAL 50  //ms

#define LOW_PID_OUT -3000 //-40000  //Throttle
#define HIGH_PID_OUT 3000 //40000   //Throttle
#define LOW_PID_OUT_P -1000         //Pitch
#define HIGH_PID_OUT_P 1000         //Pitch
#define LOW_PID_OUT_R -1000         //Roll
#define HIGH_PID_OUT_R 1000         //Roll
#define LOW_PID_OUT_Y -30000         //Yaw
#define HIGH_PID_OUT_Y 30000         //Yaw

#define COUNT_LOW 3222     //999 = (0%)
#define COUNT_MID 4870    //1500 = (50%)
#define COUNT_HIGH 6520   //2001 = (100%)

//PID CONTROLLER VALUES for HOVER
double kp = 50;
double ki = 0.2;
double kd = 3;

/* FUNCTIONS ---------------------------------------*/
double computePID(double inp, double setPoint);

#endif
