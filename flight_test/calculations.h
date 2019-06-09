#ifndef CALCULATIONS_H
#define CALCULATIONS_H

/*Includes------------------------------------------------------------*/
#include "sensors.h"

/*Definitions---------------------------------------------------------*/
#define MILLISECONDS 1000
#define GROUND_PRESSURE 101959.05
#define PRESSURE_AVG_SET_SIZE   15

// UBC Fountain Coordinates
#define TRGT_LAT 49.264796   
#define TRGT_LON -123.252810

/*Functions------------------------------------------------------------*/
void crunchNumbers(float*, float*, float*, float*, float*, float*, float*, float*, double*, double*, double*, unsigned long*, float*);
void addToPressureSet(float*, float);
float calculatePressureAverage(float*);
double distanceToTarget(double, double);
#endif
