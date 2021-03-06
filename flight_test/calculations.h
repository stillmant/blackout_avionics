#ifndef CALCULATIONS_H
#define CALCULATIONS_H

/*Includes------------------------------------------------------------*/
#include "sensors.h"

/*Definitions---------------------------------------------------------*/
#define MILLISECONDS 1000
// #define GROUND_PRESSURE 101859.34
#define GROUND_PRESSURE 1013.25
#define PRESSURE_AVG_SET_SIZE   5
#define GROUND_PRESSURE_AVG_SET_SIZE   20
#define GPS_AVG_SET_SIZE   5

// UBC Fountain Coordinates
#define TRGT_LAT 49.2603553
#define TRGT_LON -123.2557735 //Middle of grass outside LMRS

/*Functions------------------------------------------------------------*/
void crunchNumbers(float*, float*, float*, float*, float*, float*, float*, float*, double*, double*, double*, unsigned long*, float*);
void addToPressureSet(float*, float);
void addToPressureSet(float*, float, int);
float calculatePressureAverage(float*);
float calculateGroundPressureAverage(float*, int);
double distanceToTarget(double, double);
// void addToGPSSet(double*, double);
// double calculateAvgGPSvalue(float*);
double latDiff(double);
double lonDiff(double);
double targetBearing(double, double);
#endif
