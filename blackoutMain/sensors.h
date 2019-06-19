#ifndef SENSORS_H
#define SENSORS_H

/*Includes------------------------------------------------------------*/
#include "statemachine.h"

/*Definitions---------------------------------------------------------*/
#define BMP_SCK  13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS   10

#ifndef _ESP32_HAL_I2C_H_
#define SDA_PIN 21
#define SCL_PIN 22
//dont know this pin yet, pls update
#define photo_resistor_pin 27
#endif

/*Variables------------------------------------------------------------*/

/*Functions------------------------------------------------------------*/
void initSensors();

void pollSensors(double*, double*, double*, double*, float*, float*, float*, int*);
void checkPhotoRes();
#endif
