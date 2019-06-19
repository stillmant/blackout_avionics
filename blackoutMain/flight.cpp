/*Includes------------------------------------------------------------*/
#include "statemachine.h"
#include "deployment.h"
#include "flight.h"

#include <math.h>
#include <Arduino.h>

//PID CONTROLLER VALUES for HOVER
double kp = 50;
double ki = 0.2;
double kd = 3;

double runPIDhold(float *altitude){
        double input = *altitude;
        double output = computePID(input, setPointHover);
        output = constrain(output, LOW_PID_OUT, HIGH_PID_OUT);
        output = map(output, LOW_PID_OUT, HIGH_PID_OUT, COUNT_LOW, COUNT_MID);
        return output;
}

double runPIDland(float * altitude){
        static double setPointLand = setPointHover;
        double input = *altitude;
        setPointLand = (setPointLand - (DESCENT_RATE_PER_SEC/UPDATE_RATE_PER_SEC));
        double output = computePID(input, setPointLand);
        output = constrain(output, LOW_PID_OUT, HIGH_PID_OUT);
        output = map(output, LOW_PID_OUT, HIGH_PID_OUT, COUNT_LOW, COUNT_MID);
        return output;
}

double computePID(double inp, double setPoint){
        static double elapsedTime, error = 0, cumError = 0, rateError = 0, lastError = 0;
        unsigned long currentTime;
        static unsigned long previousTime = millis() / 10;

        currentTime = millis() / 10;                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation

        error = setPoint - inp;                                // determine error
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative
        double out = kp*error + ki*cumError + kd*rateError;                //PID output

        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time

        return out;                                        //have function return the PID output
}