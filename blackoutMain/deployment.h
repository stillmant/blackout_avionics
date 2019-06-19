#ifndef DEPLOYMENT
#define DEPLOYMENT

#define ARMS_IGNITER_PIN 13
#define CHUTE_DEPLOY_PIN 12
#define CHUTE_CUT_PIN 14

#define IGNITER_DELAY 15
#define FREQUENCY 980 // maybe 980
#define RESOLUTION 8
#define PWM_DELAY 15
#define PWM_STRENGTH 10 //out of 255


#define DEPLOY_CHANNEL 0
#define RELEASE_CHANNEL 1

void initDeployment();
void deployChute();
void deployRotors();
void releaseChute();

#endif

