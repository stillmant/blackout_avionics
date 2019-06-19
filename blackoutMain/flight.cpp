/*Includes------------------------------------------------------------*/
#include "statemachine.h"
#include "deployment.h"

#include <math.h>
#include <Arduino.h>

void runPIDhold(){

}

void runPIDland(){

}

double computePID(double inp, double setPoint){
        static double elapsedTime, error = 0, cumError = 0, rateError = 0, lastError = 0;
        unsigned long currentTime;
        static unsigned long previousTime;

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
