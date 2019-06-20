#ifndef FLIGHT_H
#define FLIGHT_H

// Polling times
#define LANDED_POLLING_TIME_INTERVAL 5000 //ms
#define NOMINAL_POLLING_TIME_INTERVAL 50  //ms

#define LOW_PID_OUT -2500 //-40000  //Throttle
#define HIGH_PID_OUT 2500 //40000   //Throttle

#define COUNT_LOW 3222     //999 = (0%)
#define COUNT_PID_LOW 5332 //~ 1400
#define COUNT_MID 4870    //1500 = (50%)
#define COUNT_HIGH 6520   //2001 = (100%)

#define DESCENT_RATE_PER_SEC 0.5
#define UPDATE_RATE_PER_SEC 20
#define setPointHover 1.5

/* FUNCTIONS ---------------------------------------*/
double runPIDhold(float *altitude, bool reset);
double runPIDland(float * altitude);
double computePID(double inp, double setPoint, bool reset);

#endif
