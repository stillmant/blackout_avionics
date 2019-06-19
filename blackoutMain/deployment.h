#ifndef DEPLOYMENT_H
#define DEPLOYMENT_H

#define ARMS_IGNITER_PIN 13
#define CHUTE_DEPLOY_PIN 12
#define CHUTE_CUT_PIN 14

#define IGNITER_DELAY 15
#define FREQUENCY 980 // maybe 980
#define RESOLUTION 8
#define PWM_DELAY 1000
#define PWM_STRENGTH 20 //out of 255

#define DEPLOY_CHANNEL 0
#define RELEASE_CHANNEL 1

#define LOW_PID_OUT -3000 //-40000  //Throttle
#define HIGH_PID_OUT 3000 //40000   //Throttle
#define LOW_PID_OUT_P -1000         //Pitch
#define HIGH_PID_OUT_P 1000         //Pitch
#define LOW_PID_OUT_R -1000         //Roll
#define HIGH_PID_OUT_R 1000         //Roll
#define LOW_PID_OUT_Y -30000         //Yaw
#define HIGH_PID_OUT_Y 30000         //Yaw

#define COUNT_LOW 3222     //999 = (0%)
#define COUNT_MID 4870    //1500 = (50%)
#define COUNT_HIGH 6520   //2001 = (100%)

void initDeployment();
void deployChute();
void deployRotors();
void releaseChute();

#endif

