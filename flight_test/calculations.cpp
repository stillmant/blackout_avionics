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
                  float *delta_alt, double *lat, double *lon, double *distToTrgt, unsigned long *delta_time, float *pressure_set){

    static float average_pressure;
    addToPressureSet(pressure_set, barData[0]);
    average_pressure = calculatePressureAverage(pressure_set);

    /*
    static double avg_lat;
    static double avg_lon;
    addToGPSSet(lat_set, *lat);
    addToGPSSet(lon_set, *lon);
    avg_lat = calculateAvgGPSvalue(lat_set);
    avg_lon = calculateAvgGPSvalue(lon_set);
    */

    // *pressure = barData[0];
    *prev_alt = *alt;
    *alt = 44330.0 * (1 - powf(average_pressure / *groundPressure, 1 / 5.255));
    *delta_alt = (*alt - *prev_alt) * MILLISECONDS / *delta_time;

    *distToTrgt = distanceToTarget(*lat, *lon);
}

void addToPressureSet(float* average_set, float data){
    static int i = 0;
    average_set[i] = data;
    if(i >= PRESSURE_AVG_SET_SIZE - 1)
        i = 0;
    else
        i++;
}

// for ground pressure 
void addToPressureSet(float* average_set, float data, int set_size){
    static int i = 0;
    average_set[i] = data;
    if(i >= set_size - 1)
        i = 0;
    else
        i++;
}
/*
void addToGPSSet(double* gps_set, double data){
  static int i = 0;
  gps_set[i] = data;
  if(i >= set_size - 1)
        i = 0;
    else
        i++;
}*/

float calculatePressureAverage(float* average_set){
    float sum = 0;
    for(int i = 0; i < PRESSURE_AVG_SET_SIZE; i++){
        sum = sum + average_set[i];
    }
    return sum / PRESSURE_AVG_SET_SIZE;
}

float calculateGroundPressureAverage(float* average_set, int set_size){
    float sum = 0;
    for(int i = 0; i < GROUND_PRESSURE_AVG_SET_SIZE; i++){
        sum = sum + average_set[i];
    }
    return sum / GROUND_PRESSURE_AVG_SET_SIZE;
}

/*
double calculateAvgGPSvalue(float* gps_set){
  double sum = 0;
  for (int i = 0; i < GPS_AVG_SET_SIZE; i++){
    sum = sum + gps_set[i];
  }
  return sum/ GPS_AVG_SET_SIZE;
}*/

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

double latDiff(double lat) {
  return TRGT_LAT - lat;
}

/*
 * @brief  Calculates bearing to target. Output in degrees!!
 * @return double
 */
double targetBearing(double lat, double lon) {
    double y = sin(TRGT_LON - lon) * cos(TRGT_LAT);
    double x = (cos(lat) * sin(TRGT_LAT)) - (sin(lat) * cos(TRGT_LAT) * cos(TRGT_LON - lon));

    return atan2(y, x)* (180.0 / 3.141592653589793); // in degrees
}
