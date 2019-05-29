/*Includes------------------------------------------------------------*/
#include "calculations.h"
#include <math.h>
#include <Arduino.h>

/*Definitions----------------------------------------------------------*/

/*Variables------------------------------------------------------------*/

/*Functions------------------------------------------------------------*/
/*
 * @brief  Does all calculations
 * @return void
 */
void crunchNumbers(float barData[], float accelData[], float magData[], 
                  float *pressure, float *groundPressure, float *prev_alt, float *alt, 
                  float *delta_alt, double *lat, double *lon, float *distToTrgt, unsigned long *delta_time)
{
    *pressure = barData[0];
    *prev_alt = *alt;
    *alt = 44330.0 * (1 - powf(*pressure / *groundPressure, 1 / 5.255));
    *delta_alt = (*alt - *prev_alt) * MILLISECONDS / *delta_time;
}

void distanceToTarget(double *lat, double *lon, float *distToTrgt) {
    // math here...
}