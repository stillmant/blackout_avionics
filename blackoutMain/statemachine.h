#ifndef STATEMACHINE_H
#define STATEMACHINE_H

/*---Constants-------*/
// State switch checks
#define LAUNCH_CHECKS 40
#define APOGEE_CHECKS 20
#define PHOTO_CHECKS 10
#define DEPLOYMENT_CHECKS 2
#define CHUTE_RELEASE_CHECKS 5
#define LAND_CHECKS 5
#define PHOTO_RESISTOR_THRESHOLD 369

// mach thresholds
#define MACH_THRESHOLD 225 //in meters per second
#define MACH_LOCK_THRESHOLD 220 //in meters per second
#define MACH_CHECKS 5
#define MACH_LOCK_CHECKS 25

// Various thresholds/ event altitudes
#define LAUNCH_THRESHOLD 100 // meters above ground
#define CHUTE_RELEASE_ALT 300 // meters above ground TODO: run the numbers, see if 1 sec fall is ok
#define HOVER_COUNT_THRESHOLD 25
#define HOVER_SLACK 2.5

// Timing delays
#define SEPARATION_DELAY 1750 // ms (Free fall away from rocket for 3 sec before chute deploy)
#define CHUTE_DROP_DELAY 1250 // ms (Free fall for 1 sec after chute relase)
#define CHUTE_FLIGHT_DELAY 60000 // ms
#define ARM_MOTOR_DELAY 62000
#define ARM_MOTOR_DELAY_2 64000

#define ALT_TIMEOUT 300000
#define HOLD_DELAY 1000

#define SEA_PRESSURE 101325	// pascals

// Polling times
#define LANDED_POLLING_TIME_INTERVAL 5000 //ms
#define NOMINAL_POLLING_TIME_INTERVAL 10  //ms
#define PID_UPDATE_TIME_INTERVAL 40

#define GROUND_ALTITUDE_THRESHOLD 50
#define GROUND_FORCE_LAND_THRESHOLD 3.5
#define LAND_VELOCITY_THRESHOLD 3
#define LAND_CHECKS 5
#define LANDING_TIME_INTERVAL 10000

/*---Variables-------*/
enum States {
	STANDBY,		// 0
	ASCENT,			// 1
	MACH_LOCK,		// 2
	DESCENT,		// 3
	CHUTE_DELAY,	// 4
	UNDERCHUTE,		// 5
	ALTHOLD,		// 6
	LANDING,		// 7 <-- Might need a FINAL state (lower altitued a bit) between 6 & 7
	LANDED 			// 8
};

/*---Functions-------*/
void changeState(States *curr_state, States new_state);
void stateMachine(float*, float*, float*, float*, float*, double*, States*, int*, float*, float*);
#endif
