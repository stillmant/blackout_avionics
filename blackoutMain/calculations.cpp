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
                  float *delta_alt, double *lat, double *lon, double *distToTrgt, unsigned long *delta_time)
{
    *pressure = barData[0];
    *prev_alt = *alt;
    *alt = 44330.0 * (1 - powf(*pressure / *groundPressure, 1 / 5.255));
    *delta_alt = (*alt - *prev_alt) * MILLISECONDS / *delta_time;

    *distToTrgt = distanceToTarget(*lat, *lon);
}

/*
 * @brief  Calculates distance between current lat lon & target lat lon in meters
 * @return double
 */
double distanceToTarget(double lat, double lon) {
    double p = 0.017453292519943;    // PI / 180
    double a = 0.5 - cos((TRGT_LAT - lat) * p)/2 + 
            cos(lat * p) * cos(TRGT_LAT * p) * 
            (1 - cos((TRGT_LON - lon) * p))/2;
            
    return (12742 * asin(sqrt(a))) * 1000; // in meters
}
