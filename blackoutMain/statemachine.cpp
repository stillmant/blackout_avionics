/*
 * Reference: UBC Rocket Avionics 2018/2019 code
 * Description: State machine for UBC Rocket BLACKOUT Payload Vehicle's states
 */

/*Includes------------------------------------------------------------*/
#include "statemachine.h"

#include <math.h>
#include <Arduino.h>

/*---Functions-------*/
/* void switchState(States*, States){}
 * @brief  Switches state machine state.
 * @param  States *curr_state - current state machine state.
 * @param  State new_state - state machine state to switch to.
 * @return void.
 */
void switchState(States *curr_state, States new_state){
	*curr_state = new_state;
}

/* void stateMachine(float*, float*, float*, float*, float*, States*){}
 * @brief  The statemachine of BLACKOUT Payload Vehicle.
 * @param  float *alt - current altitude
 * @param  float *delta_alt - change in altitude
 * @param  float *pressure - barometer pressure
 * @param  float *groundPressure - barometer pressure at launch pad
 * @param  float *groundAlt - ground altitude (taken during STANDBY, updated while rocket is on pad)
 * @param  float *distToTrgt - current distance to landing pad (target)
 * @param  States *state - current state
 * @return void.
 */
void stateMachine(float *alt, float *delta_alt, float *pressure, float *groundPressure, float *groundAlt, double *distToTrgt, States *state) {
	static int launch_count = 0, apogee_count = 0, deploy_count = 0, release_count = 0;
	static unsigned long delay_start;
	static int mach_lock_count = 0, mach_count = 0, base_alt_counter = 0;

	switch (*state) {
		case STANDBY:
			if (*alt > LAUNCH_THRESHOLD) {
				launch_count++;
				if (launch_count >= LAUNCH_CHECKS) {
					switchState(state, ASCENT);
					launch_count = 0;
				}
			} else {
				launch_count = 0;
				base_alt_counter++;
				if(base_alt_counter >= 20){
					*groundPressure = *pressure;
					*groundAlt = 44330.0 * (1 - powf(*groundPressure / SEA_PRESSURE, 1 / 5.255));
					base_alt_counter = 0;
				}
			}
			break;

		// check for apogee
		case ASCENT:
		    if (*delta_altitude > MACH_THRESHOLD) {
                mach_count++;
                if (mach_count >= MACH_CHECKS) {
                    switchState(state, MACH_LOCK);
                    mach_count = 0;
                }
            }
            else {
                mach_count = 0;
            }

			// CHECK FOR PHOTORESISTOR AS APOGEE INSTEAD


			// if (*delta_alt <= 0) {
			// 	apogee_count++;
			// 	if (apogee_count >= APOGEE_CHECKS) {
			// 		switchState(state, DESCENT);
          	// 		delay_start = millis();
			// 		apogee_count = 0;
			// 	}
			// } else {
			// 	apogee_count = 0;
			// }
			break;

		case MACH_LOCK:
			if ((*delta_altitude < MACH_LOCK_THRESHOLD) && (*delta_altitude < *prev_delta_altitude) ) {
                mach_lock_count++;
                if (mach_lock_count >= MACH_LOCK_CHECKS) {
                    switchState(state, ASCENT);
                    mach_lock_count = 0;
                }
            }
            else {
                mach_lock_count = 0;
            }
            break;

		// Deploy drogue chute
		case DESCENT:
			if ((millis() - delay_start) >= SEPARATION_DELAY) {
				deploy_count++;
				if (deploy_count >= DEPLOYMENT_CHECKS) {
					//deployChute(); // <--------------- TODO: Implement & test
					switchState(state, UNDERCHUTE);
					deploy_count = 0;
				}
			} else {
				deploy_count = 0;
			}
			break;

		// deploy rotors, spin up, check stability, fall to chute relese alt
		case UNDERCHUTE:
			//deployRotors();	// <-----------------------TODO: Implement & test (spin up in this function?)
			// TODO: check stability before chute release
			if (*alt <= CHUTE_RELEASE_ALT) {
				release_count++;
				if (release_count >= CHUTE_RELEASE_CHECKS) {
					//relaseChute(); // <--------------- TODO: Implement & test
					delay(CHUTE_DROP_DELAY);
					// might want a command here to fly away from falling chute after release
					switchState(state, ALTHOLD);
					release_count = 0;
				}
			} else {
				release_count = 0;
			}
			break;

    // Alt. Hold will make drone hold alt. but not move, idea being if you lose
    // GPS lock you dont want to keep flying so wait in place for reacquire
    // If GPS doesnt reacquire after some amount of time switch to landing state
		// acquire GPS lock (!!IMPORTANT: calibrate GPS on startup !!)
		case ALTHOLD:
			//holdAlt();
			//getGPS(); // poll GPS? Might do this somewhere else


			// if (goodLock) {
			// 	switchState(state, FLIGHT);
			// }


			break;

		case FLIGHT:
			// if close to tgt -> APPROACH
			// if lose gps for more than 5 sec -> ALTHOLD
			break;
		case APPROACH:
			break;
		case LANDING:
			break;
		case LANDED:
			break;
	}
}
