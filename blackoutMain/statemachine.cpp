/*
 * Reference: UBC Rocket Avionics 2018/2019 code
 * Description: State machine for UBC Rocket BLACKOUT Payload Vehicle's states
 */

/*Includes------------------------------------------------------------*/
#include "statemachine.h"
#include "deployment.h"
#include "flight.h"
#include "calculations.h"

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
void stateMachine(float *alt, float *delta_alt, float *pressure, float *groundPressure, float *groundAlt, double *distToTrgt, States *state, int *photo_resistor, float accel_data[], float ground_pressure_set[]) {
	static int launch_count = 0, apogee_count = 0, deploy_count = 0, release_count = 0;
	static int hover_count = 0, photo_count = 0, mach_lock_count = 0, mach_count = 0;
	static unsigned long delay_start, hold_timeout;
	static int base_alt_counter = 0;
	static bool rotors_deployed = false, rotors_armed = false, apogee = false, photo_trigger = false;
	static int chute_drop_time;
	static uint32_t old_time_landed = millis();
	static float old_altitude_landed = *alt;
	static int land_count = 0;
	static float prev_delta_altitude = *delta_alt;

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
				if(base_alt_counter >= 100){
					addToPressureSet(ground_pressure_set, *alt, GROUND_PRESSURE_AVG_SET_SIZE);
					*groundPressure = calculateGroundPressureAverage(ground_pressure_set, GROUND_PRESSURE_AVG_SET_SIZE);
					*groundAlt = 44330.0 * (1 - powf(*groundPressure / SEA_PRESSURE, 1 / 5.255));
					base_alt_counter = 0;
				}
			}
			break;

		// check for apogee
		case ASCENT:
			if (*delta_alt > MACH_THRESHOLD) {
				mach_count++;
				if (mach_count >= MACH_CHECKS) {
					switchState(state, MACH_LOCK);
					mach_count = 0;
				}
			}
			else {
				mach_count = 0;
			}

			if (*delta_alt <= 0) {
                apogee_count ++;
                if (apogee_count >= APOGEE_CHECKS) {
                    apogee = true;
					delay(100);
                    apogee_count = 0;
                }
            }
            else {
                apogee_count = 0;
            }

			if (*photo_resistor >= PHOTO_RESISTOR_THRESHOLD){
			 	photo_count++;
			 	if (photo_count >= PHOTO_CHECKS) {
			 		photo_trigger = true;
			 		photo_count = 0;
				}
			} else {
				photo_count = 0;
			}

			if (photo_trigger == true || apogee == true){
				switchState(state, DESCENT);
				delay_start = millis();
			}
			break;

		case MACH_LOCK:
			if ((*delta_alt < MACH_LOCK_THRESHOLD) && (*delta_alt < prev_delta_altitude) ) {
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
					deployChute();
					switchState(state, CHUTE_DELAY);
					rotors_deployed = false;
					rotors_armed = false;
					deploy_count = 0;
					chute_drop_time = millis();
				}
			} else {
				deploy_count = 0;
			}
			break;

		case CHUTE_DELAY:
			if (((millis() - chute_drop_time) >= CHUTE_FLIGHT_DELAY) && rotors_deployed == false){
				deployRotors();
				rotors_deployed = true;
			}
			if (((millis() - chute_drop_time) >= ARM_MOTOR_DELAY) && rotors_armed == false){
				// Main channels: ROLL = 1, PITCH = 2, YAW = 3, THROTTLE = 4   -> 3 = THROTTLE, 4 = YAW
				ledcWrite(5,3222);   // DISARMED
  				ledcWrite(3,3222);
				ledcWrite(1,COUNT_MID);
				ledcWrite(2,COUNT_MID);
				ledcWrite(4,COUNT_MID);
				rotors_armed = true;
			}
			if (((millis() - chute_drop_time) >= ARM_MOTOR_DELAY_2) && rotors_armed == true){
				ledcWrite(5,6540);
				switchState(state, UNDERCHUTE);
			}
			break;

		// fall to chute release alt
		case UNDERCHUTE:
			// TODO: check stability before chute release
			if (*alt <= CHUTE_RELEASE_ALT) {
				release_count++;

				if (release_count >= CHUTE_RELEASE_CHECKS) {
					releaseChute();
					delay(CHUTE_DROP_DELAY);
					switchState(state, LANDING);
					release_count = 0;
					hold_timeout = millis();
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
			if (*alt <= (setPointHover + HOVER_SLACK) && *alt >= (setPointHover - HOVER_SLACK)){
				hover_count++;

				if (hover_count > HOVER_COUNT_THRESHOLD){
					switchState(state, LANDING);
				}
			}
			else{
				hover_count = 0;
			}

			if ((millis() - hold_timeout) >= ALT_TIMEOUT){
				ledcWrite(3, COUNT_LOW);
				delay(HOLD_DELAY);
				switchState(state, LANDING);
			}
			break;

		case LANDING:
			if(*alt <= GROUND_ALTITUDE_THRESHOLD){
				float delta = *alt - old_altitude_landed;

				if(millis() - old_time_landed >= LANDING_TIME_INTERVAL){
					if(abs(delta) <= LAND_VELOCITY_THRESHOLD){
						land_count++;
					}
					else{
						land_count = 0;
					}
					old_time_landed = millis();
					old_altitude_landed = *alt;
				}

				if(accel_data[3] >= GROUND_FORCE_LAND_THRESHOLD || land_count >= LAND_CHECKS){
					switchState(state, LANDED);
				}
			}
			break;

		case LANDED:
			break;
	}
	prev_delta_altitude = *delta_alt;
}
