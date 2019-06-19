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
                  float *delta_alt, unsigned long *delta_time, float pressure_set[]){

    addToPressureSet(pressure_set, barData[0]);
    *pressure = calculatePressureAverage(pressure_set);

    *prev_alt = *alt;
    *alt = 44330.0 * (1 - powf(*pressure / *groundPressure, 1 / 5.255));
    *delta_alt = (*alt - *prev_alt) * MILLISECONDS / *delta_time;
}

void addToPressureSet(float average_set[], float data){
    static int i = 0;
    average_set[i] = data;
    if(i >= PRESSURE_AVG_SET_SIZE - 1)
        i = 0;
    else
        i++;
}

// for ground pressure
void addToPressureSet(float average_set[], float data, int set_size){
    static int i = 0;
    average_set[i] = data;
    if(i >= set_size - 1)
        i = 0;
    else
        i++;
}

float calculatePressureAverage(float average_set[]){
    float sum = 0;
    for(int i = 0; i < PRESSURE_AVG_SET_SIZE; i++){
        sum = sum + average_set[i];
    }
    return sum / PRESSURE_AVG_SET_SIZE;
}

float calculateGroundPressureAverage(float average_set[], int set_size){
    float sum = 0;
    for(int i = 0; i < GROUND_PRESSURE_AVG_SET_SIZE; i++){
        sum = sum + average_set[i];
    }
    return sum / GROUND_PRESSURE_AVG_SET_SIZE;
}