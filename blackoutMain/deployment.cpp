#include "deployment.h"
#include "sensors.h"
#include "flight.h"

#include <Arduino.h>
#include <HardwareSerial.h>

uint8_t roll = 0;
uint8_t pitch = 15;
uint8_t yaw = 2;
uint8_t throt = 4;

uint8_t arm = 5;
uint8_t manual = 18;

void initChannels(){

    // FLIGHT CONTROLLER SETUP
    ledcAttachPin(roll, 1);
    ledcAttachPin(pitch, 2);
    ledcAttachPin(yaw, 3);
    ledcAttachPin(throt, 4);

    ledcAttachPin(arm, 5);
    ledcAttachPin(manual, 6);

    // ledcSetup(channel, Hz, 16-bit resolution)
    ledcSetup(1, 50, 16);
    ledcSetup(2, 50, 16);
    ledcSetup(3, 50, 16);
    ledcSetup(4, 50, 16);
    ledcSetup(5, 50, 16);
    ledcSetup(6, 50, 16);
}

void initFC(){
    ledcWrite(5,3222);   //DISARMED CONTROLLER
    ledcWrite(3,3222);

    ledcWrite(1,COUNT_MID);
    ledcWrite(2,COUNT_MID);
    ledcWrite(4,COUNT_MID);
    ledcWrite(6,COUNT_MID);
}

//inititalize pins to output, must be called in setup()
void initDeployment(){
    pinMode(ARMS_IGNITER_PIN, OUTPUT);
    pinMode(CHUTE_CUT_PIN, OUTPUT);
    pinMode(CHUTE_DEPLOY_PIN, OUTPUT);

    //pwm setup for deployment
    ledcSetup(DEPLOY_CHANNEL, FREQUENCY, RESOLUTION);
    ledcAttachPin(CHUTE_DEPLOY_PIN, DEPLOY_CHANNEL);

    ledcSetup(RELEASE_CHANNEL, FREQUENCY, RESOLUTION);
    ledcAttachPin(CHUTE_CUT_PIN, RELEASE_CHANNEL);
}

void deployChute(){
    ledcWrite(DEPLOY_CHANNEL, PWM_STRENGTH);
    delay(PWM_DELAY);
    ledcWrite(DEPLOY_CHANNEL, 0);
}

void deployRotors(){
    digitalWrite(ARMS_IGNITER_PIN, HIGH);
    delay(IGNITER_DELAY);
    digitalWrite(ARMS_IGNITER_PIN, LOW);
}

void releaseChute(){
    ledcWrite(RELEASE_CHANNEL, PWM_STRENGTH);
    delay(PWM_DELAY);
    ledcWrite(RELEASE_CHANNEL, 0);
}