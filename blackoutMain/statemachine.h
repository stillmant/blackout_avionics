#ifndef STATEMACHINE_H
#define STATEMACHINE_H

/*---Constants-------*/
// State switch checks
#define LAUNCH_CHECKS 4
#define APOGEE_CHECKS 5
#define DEPLOYMENT_CHECKS 10
#define CHUTE_RELEASE_CHECKS 3
#define LAND_CHECKS 5

// Various thresholds/ event altitudes
#define LAUNCH_THRESHOLD 50 // meters above ground
#define CHUTE_RELEASE_ALT 550 // meters above ground TODO: run the numbers, see if 1 sec fall is ok

// Timing delays
#define SEPARATION_DELAY 3000 // ms (Free fall away from rocket for 3 sec before chute deploy)
#define CHUTE_DROP_DELAY 1000 // ms (Free fall for 1 sec after chute relase)

#define SEA_PRESSURE 1013.25

// Polling times
#define LANDED_POLLING_TIME_INTERVAL 5000 //ms
#define NOMINAL_POLLING_TIME_INTERVAL 50  //ms

/*---Variables-------*/
enum States {
	STANDBY,		// 0
	ASCENT,			// 1
	DESCENT,		// 2
	UNDERCHUTE,		// 3
	ALTHOLD,		// 4
	FLIGHT,			// 5
	APPROACH,		// 6
	LANDING,		// 7 <-- Might need a FINAL state (lower altitued a bit) between 6 & 7
	LANDED 			// 8
};

/*---Functions-------*/
void changeState(States *curr_state, States new_state);
void stateMachine(float*, float*, float*, float*, float*, double*, States*);
#endif
