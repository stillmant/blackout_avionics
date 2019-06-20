/*Includes------------------------------------------------------------*/
#include "statemachine.h"
#include "deployment.h"
#include "flight.h"

#include <math.h>
#include <Arduino.h>

//PID CONTROLLER VALUES for HOVER
double kp = 200;
double ki = 0;
double kd = 1000;

double runPIDhold(float *altitude, bool reset_PID){
    double input = *altitude;
    double output = computePID(input, setPointHover, reset_PID);
    output = constrain(output, LOW_PID_OUT, HIGH_PID_OUT);
    output = map(output, LOW_PID_OUT, HIGH_PID_OUT, COUNT_PID_LOW, COUNT_HIGH);
    return output;
}

double runPIDland(float * altitude){
    static double setPointLand = setPointHover;
    double input = *altitude;

    if (input <= SLOW_THRESHOLD)
        setPointLand = (setPointLand - (DESCENT_SLOW_PER_SEC/UPDATE_RATE_PER_SEC));
    else
        setPointLand = (setPointLand - (DESCENT_FAST_PER_SEC/UPDATE_RATE_PER_SEC));

    double output = computePID(input, setPointLand, false);
    double out_return = pdBang(output);
    return out_return;
}

double pdBang(double output){
    static double max_threshold = MAX_DIFF * kp;
    static double min_threshold = MIN_DIFF * kp;
    double mapped;

    if (output >= 0){
        mapped = constrain(output, - max_threshold, max_threshold);
        mapped = map(mapped, - max_threshold, max_threshold, COUNT_PID_LOW, COUNT_HIGH);
    }

    else{
        mapped = constrain(output, - min_threshold, min_threshold);
        mapped = map(mapped, - min_threshold, min_threshold, COUNT_PID_LOW, COUNT_HIGH);
    }

    return mapped;
}

double computePID(double inp, double setPoint, bool reset_PID){
    static double elapsedTime, error = 0, cumError = 0, rateError = 0, lastError = 0;
    unsigned long currentTime;
    static unsigned long previousTime = millis() / 10;

    if (reset_PID){
        cumError = 0;
        rateError = 0;
        lastError = 0;
    }
    currentTime = millis() / 10;                //get current time
    elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation

    error = setPoint - inp;                                // determine error
    cumError += error * elapsedTime;                // compute integral
    rateError = (error - lastError)/elapsedTime;   // compute derivative
    double out = kp*error + ki*cumError - kd*rateError;                //PID output

    lastError = error;                                //remember current error
    previousTime = currentTime;                        //remember current time

    return out;                                        //have function return the PID output
}
