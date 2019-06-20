#ifndef FLIGHT_H
#define FLIGHT_H

// Polling times
// #define LANDED_POLLING_TIME_INTERVAL 5000 //ms
// #define NOMINAL_POLLING_TIME_INTERVAL 50  //ms

#define LOW_PID_OUT -1000 //-40000  //Throttle
#define HIGH_PID_OUT 1000 //40000   //Throttle

#define MAX_DIFF 2.5
#define MIN_DIFF 15

#define COUNT_LOW 3222     //999 = (0%)
#define COUNT_PID_LOW 5550 // ~ 1850 midpoint
#define COUNT_MID 4868    //1500 = (50%)
#define COUNT_HIGH 6520   //2001 = (100%)

#define DESCENT_SLOW_PER_SEC 0.5
#define DESCENT_FAST_PER_SEC 1
#define UPDATE_RATE_PER_SEC 25

#define setPointHover 250
#define SLOW_THRESHOLD 150

/* FUNCTIONS ---------------------------------------*/
double runPIDhold(float *altitude, bool reset_PID);
double runPIDland(float * altitude);
double pdBang(double output);
double computePID(double inp, double setPoint, bool reset_PID);

#endif
