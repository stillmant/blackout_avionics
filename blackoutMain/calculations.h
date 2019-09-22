#ifndef CALCULATIONS_H
#define CALCULATIONS_H

/*Includes------------------------------------------------------------*/
#include "sensors.h"

/*Definitions---------------------------------------------------------*/
#define MILLISECONDS 1000

#define PRESSURE_AVG_SET_SIZE   20
#define GROUND_PRESSURE_AVG_SET_SIZE   20

// UBC Fountain Coordinates
#define TRGT_LAT 49.264796
#define TRGT_LON -123.252810

/*Functions------------------------------------------------------------*/
void crunchNumbers(float*, float*, float*, float*, float*, float*, float*, float*, unsigned long*, float*);
void addToPressureSet(float* average_set, float data);
void addToPressureSet(float* average_set, float data, int set_size);
float calculatePressureAverage(float* average_set);
float calculateGroundPressureAverage(float* average_set, int set_size);
#endif
