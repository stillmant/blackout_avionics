#ifndef SENSORS_H
#define SENSORS_H

/*Includes------------------------------------------------------------*/
#include "statemachine.h"

/*Constants------------------------------------------------------------*/
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 21
#define SCL_PIN 22
#endif

/*Variables------------------------------------------------------------*/

/*Functions------------------------------------------------------------*/
void initSensors();

void pollSensors(double*, double*, float*, float*, float*);
#endif
