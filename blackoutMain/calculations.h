#ifndef CALCULATIONS_H
#define CALCULATIONS_H

/*Includes------------------------------------------------------------*/
#include "sensors.h"

/*Definitions---------------------------------------------------------*/
#define MILLISECONDS 1000
// UBC Fountain Coordinates
#define TRGT_LAT 49.264796   
#define TRGT_LON -123.252810

/*Functions------------------------------------------------------------*/
void crunchNumbers(float*, float*, float*, float*, float*, float*, float*, float*, double*, double*, double*, unsigned long*);

double distanceToTarget(double, double);
#endif
