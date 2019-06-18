/*
 * Reference
 * https://www.instructables.com/id/Interfacing-Servo-Motor-With-ESP32/
 *
 * https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/dac.html
 * https://randomnerdtutorials.com/esp32-pwm-arduino-ide/
 * https://github.com/espressif/arduino-esp32/issues/4
 * https://youtu.be/SBG7ccW5gpA?t=348
 */

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

#define HOVER_HEIGHT 2 // meters above ground
#define LANDING_VELOCITY 0.3 // m/s
#define PRE_LANDING_HEIGHT 1 // meters
#define FINAL_DESCENT_INCREMENT 0.05  // meters per time_interval2

#define LAND_ACCEL_THRESHOLD 5 // g's

// Timing delays
#define SEPARATION_DELAY 3000 // ms (Free fall away from rocket for 3 sec before chute deploy)
#define CHUTE_DROP_DELAY 1000 // ms (Free fall for 1 sec after chute relase)

#define SEA_PRESSURE 1013.25

#define HOVER_TIME 300000 // ms (for flight testing purposes, time until commence landing function)
#define YAW_CALBRT_TIME_INTERVAL 20000 // ms (time interval for each yaw calibration)

// Polling times
#define LANDED_POLLING_TIME_INTERVAL 5000 //ms
#define NOMINAL_POLLING_TIME_INTERVAL 50  //ms

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

#include "esp32-hal-ledc.h"
/*---Includes-----------------------------------------------------------*/
#include "sensors.h"
#include "calculations.h"
/*---Variables----------------------------------------------------------*/
static float alt, prev_alt, delta_alt, pressure, groundPressure, groundAlt;

// GPS
static double lat, lon, gpsAlt, gpsSats, distToTrgt;

static float pressure_set[PRESSURE_AVG_SET_SIZE];
static float ground_pressure_set[GROUND_PRESSURE_AVG_SET_SIZE];
// static double lat_set[GPS_AVG_SET_SIZE];
// static double lon_set[GPS_AVG_SET_SIZE];

// GY-91
static float barData[2];
static float accelData[4];
static float magData[4];
// the numbers corresponds to pin numbers on ESP
uint8_t roll = 0;
uint8_t pitch = 15;
uint8_t yaw = 2;
uint8_t throt = 4;

uint8_t arm = 5;
uint8_t manual = 18;

const int buttonPin = 27;         //CHANGE TO RIGHT PIN
int buttonState = 0;

//PID CONTROLLER VALUES for HOVER
double kp = 50;
double ki = 0.2;
double kd = 3;
//VALUES for PITCH
double kp_p = 70;
double ki_p = 0.01;
double kd_p = 2;
//VALUES for ROLL
double kp_r = 70;
double ki_r = 0.01;
double kd_r = 2;
//VALUES for YAW
double kp_y = 50;
double ki_y = 0.2;
double kd_y = 2;

unsigned long currentTime, previousTime, currentTime_p, previousTime_p, currentTime_y, previousTime_y, currentTime_r, previousTime_r;
double elapsedTime, elapsedTime_p, elapsedTime_y, elapsedTime_r;
double error, error_p, error_y, error_r;
double lastError, lastError_p, lastError_y, lastError_r;
double input, output, setPointHover, setPointLandHeight, setPointFinalDescent, input_p, output_p, setPointLatDist, input_y, output_y, setPointYawAng, input_r, output_r, setPointLonDist;
double cumError, rateError, cumError_p, rateError_p, cumError_y, rateError_y, cumError_r, rateError_r;
double LatDist, LonDist;
// double currentTime2, previousTime2, elapsedTime2, error2, lastError2, input2, output2, setPoint2, cumError2, rateError2;

double computePID(double, double);
double computePIDpitch(double, double);
double computePIDyaw(double, double);
double computePIDroll(double, double);


// Main channels: ROLL = 1, PITCH = 2, YAW = 3, THROTTLE = 4   -> 3 = THROTTLE, 4 = YAW
void setChanVal(int channel, int val){
  ledcWrite(channel, val);
}

void setup() {
  Serial.begin(115200);
  delay(10);
  initSensors();

  setPointHover = HOVER_HEIGHT;          //Set desired alt for PID (HOVER)
  setPointLandHeight = PRE_LANDING_HEIGHT;     //Set desired velocity for PID (LANDING)
  setPointFinalDescent = HOVER_HEIGHT; //PRE_LANDING_HEIGHT;
  setPointLatDist = 0; //Set desired difference from Trgt lat for PID (pitch)
  setPointLonDist = 0; //Set desired Trgt lon distance for PID (roll)
  setPointYawAng = 0;  //Set desired yaw bearing for PID (0=NORTH)

  //FINDING GROUND_PRESSURE
  for (int i = 0; i < GROUND_PRESSURE_AVG_SET_SIZE; i++){
    pollSensors(&lat, &lon, &gpsAlt, &gpsSats, barData, accelData, magData);
    addToPressureSet(ground_pressure_set, barData[0], GROUND_PRESSURE_AVG_SET_SIZE);
    delay(50);
    Serial.println("Working...");
  }
  groundPressure = calculateGroundPressureAverage(ground_pressure_set, GROUND_PRESSURE_AVG_SET_SIZE);
  Serial.println(groundPressure);


  pinMode(buttonPin, INPUT);

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

  setChanVal(5,3222);   //FOR ARMING - set THROT to 999 (0%), set AUX1 to 999 (0%)
  setChanVal(3,3222);

  setChanVal(1,COUNT_MID);
  setChanVal(2,COUNT_MID);
  setChanVal(4,COUNT_MID);
  setChanVal(6,COUNT_MID);

  delay(5000);          //Wait 5 seconds for FC startup

  setChanVal(5,6540);   //ARMING - set AUX1 permantly at 2001 (100%) to stay armed

  delay(500);

  for (int i = 0; i < PRESSURE_AVG_SET_SIZE; i++) // for moving average
    {
        pressure_set[i] = groundPressure;
    }

//  float sum = 0;
//  for (int i = 0; i < PRESSURE_AVG_SET_SIZE; i++){
//    sum = sum + pressure_set[i];
//  }
//  GROUND_PRESSURE = sum / PRESSURE_AVG_SET_SIZE;
//  Serial.print("Ground pressure: ");
//  Serial.println(GROUND_PRESSURE);
}

void loop() {
  static unsigned long timestamp, timestamp2;
  static unsigned long old_time = 0, old_time2 = 0, old_time_y = 0; //ms
  static unsigned long new_time = 0, new_time2 = 0, new_time_y = 0; //ms
  unsigned long delta_time, delta_time2;
  static uint16_t time_interval_y = YAW_CALBRT_TIME_INTERVAL;
  static uint16_t time_interval2 = 100;
  static uint16_t time_interval = NOMINAL_POLLING_TIME_INTERVAL; //ms

  static int i = COUNT_LOW;
  static int landCOUNTER = 0;
  static int latCOUNTER = 0;
  static int bearingCOUNTER = 0;
  static bool landed = false;
  static bool gps_lock = false;
  static bool lat_arrived = false;
  static bool yaw_calbrt = true;   //SWITCH BACK TO TRUE
  //Serial.println(yaw_calbrt);


//-------POLL SENSORS/CHECK GPS LOCK-----------------------
  new_time = millis();
  if ((new_time - old_time) >= time_interval) {
    delta_time = new_time - old_time;
    old_time = new_time;

    pollSensors(&lat, &lon, &gpsAlt, &gpsSats, barData, accelData, magData);
    crunchNumbers(barData, accelData, magData, &pressure, &groundPressure, &prev_alt, &alt, &delta_alt, &lat, &lon, &distToTrgt, &delta_time, pressure_set);
    distanceToTarget(lat, lon);
    LatDist = latDiff(lat);
    LonDist = lonDiff(lon);

    //UNCOMMENT FOR GPS LOCK
    if (gpsSats >= 2) {
      gps_lock = true;
    }
    else {
      gps_lock = false;
    }

    //Serial.println(distanceToTarget(lat, lon));
    // Serial.println("x: " + String(magData[0]));
    // Serial.println("y: " + String(magData[1]));
    // Serial.println("z: " + String(magData[2]));
    // Serial.println("mag horizontal direction " + String(magData[3])); //////////// <------- need to check what this is. Bearing? What units?
    //Serial.print("Accel: ");
    //Serial.println(accelData[3]);
  }
//-----------------------------------------------------------


//--SETTING YAW CALBRATION STATE-----------------------------NEEDS WORK
  new_time_y = millis();
  //Serial.println(new_time_y - old_time_y);
  if ((new_time_y - old_time_y) >= time_interval_y) {
    
    yaw_calbrt = true;
    
    
  }
//-----------------------------------------------------------


  if (yaw_calbrt == true) {

     new_time2 = millis();
    if ((new_time2 - old_time2) >= time_interval2) {
        delta_time2 = new_time2 - old_time2;
        old_time2 = new_time2;

        buttonState = analogRead(buttonPin);


        if (buttonState <= 3800) {       //FAIL SAFE SWITCH OFF

          setChanVal(5,COUNT_LOW);
          Serial.println("OFFFFFFFFFFFFFFFFFFFFFFF");

        }
        
        else if (landed == false) {

//---------HOVER FUNCTION - USING PID CONTROL---------------
          Serial.println("HOVER/YAW CALBRT");
          input = alt;
          output = computePID(input, setPointHover);
          output = constrain(output, LOW_PID_OUT, HIGH_PID_OUT);
          output = map(output, LOW_PID_OUT, HIGH_PID_OUT, COUNT_LOW, COUNT_MID);

          setChanVal(3,output);
//----------------------------------------------------------


//----------YAW PID----------------------------------------------
          input_y = magData[3];
          Serial.print(" Bearing: ");
          Serial.print(magData[3]);
          output_y = computePIDyaw(input_y, setPointYawAng);
          Serial.print(" PID OUT: ");
          Serial.print(output_y);
          output_y = constrain(output_y, LOW_PID_OUT_Y, HIGH_PID_OUT_Y);
          output_y = map(output_y, LOW_PID_OUT_Y, HIGH_PID_OUT_Y, COUNT_LOW, COUNT_HIGH);
          Serial.print( " OUTPUT: ");
          Serial.println(output_y);

          setChanVal(4, output_y);

          if ((magData[3] >= -15) && (magData[3] <= 15)) {

            bearingCOUNTER++;
            Serial.println(bearingCOUNTER);

          }

          // if (bearingCOUNTER >= 25) {      //STAY IN HOVER/YAW CALBRT
          //   old_time_y = millis();
          //   yaw_calbrt = false;
          // }
        
        }

        else if (landed == true) { // LANDED, keep motors at 0,
        
          setChanVal(5, COUNT_LOW);
          landed = true;

      
        }
        
    
        }

  }

//-----------------------------------------------------------
  if ((gps_lock == true) && (yaw_calbrt == false)) {
  bearingCOUNTER = 0;
  new_time2 = millis();
  
  if ((new_time2 - old_time2) >= time_interval2) {
    delta_time2 = new_time2 - old_time2;
    old_time2 = new_time2;

    buttonState = analogRead(buttonPin);
    //Serial.println(buttonState);
    Serial.print("GPS LOCKED (sats): ");
    Serial.println(gpsSats);
    

    if (buttonState <= 3800) {       //FAIL SAFE SWITCH OFF

      setChanVal(5,COUNT_LOW);
      Serial.println("OFFFFFFFFFFFFFFFFFFFFFFF");

    }
    else if (landed == false) {
      Serial.println("ON");

      if (lat_arrived == false) {      //(millis() <= (HOVER_TIME + 5760)) {

  //---------HOVER FUNCTION - USING PID CONTROL---------------
        Serial.println("HOVER");
        input = alt;
        output = computePID(input, setPointHover);
        //Serial.print(" PID OUT: ");
        //Serial.print(output);
        output = constrain(output, LOW_PID_OUT, HIGH_PID_OUT);
        output = map(output, LOW_PID_OUT, HIGH_PID_OUT, COUNT_LOW, COUNT_MID);
        //Serial.print(" Mapped: ");
        //Serial.print(output);
        //Serial.print(" Alt: ");
        //Serial.println(alt);

        setChanVal(3,output);
  //----------------------------------------------------------

  //---------PITCH FUNCTION - USING PID WITH LATITUDE-----------------

        input_p = LatDist;
        Serial.print(" Lat Diff: ");
        Serial.print(LatDist,10);
        output_p = computePIDpitch(input_p, setPointLatDist);
        Serial.print(" PID OUT: ");
        Serial.print(output_p, 5);
        output_p = constrain(output_p, LOW_PID_OUT_P, HIGH_PID_OUT_P);
        output_p = map(output_p, LOW_PID_OUT_P, HIGH_PID_OUT_P, COUNT_LOW, COUNT_HIGH);
        Serial.print(" Mapped: ");
        Serial.println(output_p);
        setChanVal(2, output_p);

        if ((LatDist >= -0.000010) && (LatDist <= 0.000010)) {
          latCOUNTER++;
        }
        if (latCOUNTER >= 1000) {
            lat_arrived = true;
          }
        
        Serial.print("latCOUNTER: ");
        Serial.println(latCOUNTER);
  //-----------------------------------------------------------------

  //----------ROLL FUNCTION - USNG PID WITH LONGITUDE----------------

        input_r = LonDist;
        output_r = computePIDroll(input_r, setPointLonDist);
        output_r = constrain(output_r, LOW_PID_OUT_R, HIGH_PID_OUT_R);
        output_r = map(output_r, LOW_PID_OUT_R, HIGH_PID_OUT_R, COUNT_LOW, COUNT_HIGH);
        setChanVal(1, output_r);

        // if ((LonDist >= -0.000010) && (LonDist <= 0.000010)) {
        //   lonCOUNTER++;
        // }
        // if (lonCOUNTER >= 1000) {
        //     lon_arrived = true;
        //   }

      }

      else {   //lat_arrived == true
//------------FINAL DECENT - incrementally lower target alt--------------------------
          Serial.println("FINAL DECENT");

          setPointFinalDescent = setPointFinalDescent - FINAL_DESCENT_INCREMENT;

          input = alt;
          output = computePID(input, setPointFinalDescent);
          //Serial.print(" PID OUT: ");
          //Serial.print(output);
          output = constrain(output, LOW_PID_OUT, HIGH_PID_OUT);
          output = map(output, LOW_PID_OUT, HIGH_PID_OUT, COUNT_LOW, COUNT_MID);
          //Serial.print("  Mapped: ");
          //Serial.println(output);

          setChanVal(3,output);

          if ((alt <= 0.5) && (setPointFinalDescent <= -15)){
            setChanVal(5, COUNT_LOW);
            landed = true;
            Serial.println("LANDED, turn off");
          }
//---------------------------------------------------------------------------------
      }
    }

    else if (landed == true) { // LANDED, keep motors at 0,
        
          setChanVal(5, COUNT_LOW);
          landed = true;
      
      }



    }
  }
  


  else if ((gps_lock == false) && (yaw_calbrt == false)) { //no gps_lock, HOVER
    bearingCOUNTER = 0;
    new_time2 = millis();
  if ((new_time2 - old_time2) >= time_interval2) {
    delta_time2 = new_time2 - old_time2;
    old_time2 = new_time2;

    buttonState = analogRead(buttonPin);
    //Serial.println(buttonState);
    Serial.print("NO GPS LOCK (sats): ");
    Serial.println(gpsSats);

    if (buttonState <= 3800) {       //FAIL SAFE SWITCH OFF

      setChanVal(5,COUNT_LOW);
      Serial.println("OFFFFFFFFFFFFFFFFFFFFFFF");

    }
    else if (landed == false) {
      Serial.println("ON");

    if (lat_arrived == false) {
  //---------HOVER FUNCTION - USING PID CONTROL---------------
        Serial.println("HOVER");
        input = alt;
        output = computePID(input, setPointHover);
        //Serial.print(" PID OUT: ");
        //Serial.print(output);
        output = constrain(output, LOW_PID_OUT, HIGH_PID_OUT);
        output = map(output, LOW_PID_OUT, HIGH_PID_OUT, COUNT_LOW, COUNT_MID);
        //Serial.print(" Mapped: ");
        //Serial.print(output);
        //Serial.print(" Alt: ");
        //Serial.println(alt);

        setChanVal(3,output);
  //---------------------------------------------------------
    }

    else {   //lat_arrived == true

      //------------FINAL DECENT - incrementally lower target alt--------------------------
          Serial.println("FINAL DECENT");

          setPointFinalDescent = setPointFinalDescent - FINAL_DESCENT_INCREMENT;

          input = alt;
          output = computePID(input, setPointFinalDescent);
          //Serial.print(" PID OUT: ");
          //Serial.print(output);
          output = constrain(output, LOW_PID_OUT, HIGH_PID_OUT);
          output = map(output, LOW_PID_OUT, HIGH_PID_OUT, COUNT_LOW, COUNT_MID);
          //Serial.print("  Mapped: ");
          //Serial.println(output);

          setChanVal(3,output);

          if ((alt <= 0.5) && (setPointFinalDescent <= -15)){
            setChanVal(5, COUNT_LOW);
            landed = true;
            Serial.println("LANDED, turn off");
          }
      //---------------------------------------------------------------------------------
      
    }

    }

    else if (landed == true) { // LANDED, keep motors at 0,
        
          setChanVal(5, COUNT_LOW);
          landed = true;
      
      }

  }
    
  }


  }


double computePID(double inp, double setPoint){
        currentTime = millis() / 10;                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation

        error = setPoint - inp;                                // determine error
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative
//        Serial.print("error: ");
//        Serial.print(error);
//        Serial.print(" cumError: ");
//        Serial.print(cumError);
//        Serial.print(" rateError: ");
//        Serial.print(rateError);
        double out = kp*error + ki*cumError + kd*rateError;                //PID output

        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time

        return out;                                        //have function return the PID output
}

double computePIDpitch(double inp_p, double setPoint_p){
        currentTime_p = millis() / 10;                //get current time
        elapsedTime_p = (double)(currentTime_p - previousTime_p);        //compute time elapsed from previous computation

        error_p = setPoint_p - inp_p;                                // determine error
        cumError_p += error_p * elapsedTime_p;                // compute integral
        rateError_p = (error_p - lastError_p)/elapsedTime_p;   // compute derivative

        double out_p = (kp_p*error_p + ki_p*cumError_p + kd_p*rateError_p) * 10000;                //PID output

        lastError_p = error_p;                                //remember current error
        previousTime_p = currentTime_p;                        //remember current time

        return out_p;                                        //have function return the PID output
}

double computePIDroll(double inp_r, double setPoint_r){
        currentTime_r = millis() / 10;                //get current time
        elapsedTime_r = (double)(currentTime_r - previousTime_r);        //compute time elapsed from previous computation

        error_r = setPoint_r - inp_r;                                // determine error
        cumError_r += error_r * elapsedTime_r;                // compute integral
        rateError_r = (error_r - lastError_r)/elapsedTime_r;   // compute derivative

        double out_r = (kp_r*error_r + ki_r*cumError_r + kd_r*rateError_r) * 10000;                //PID output

        lastError_r = error_r;                                //remember current error
        previousTime_r = currentTime_r;                        //remember current time

        return out_r;                                        //have function return the PID output
}

double computePIDyaw(double inp_y, double setPoint_y) {
        currentTime_y = millis() / 100;                //get current time
        elapsedTime_y = (double)(currentTime_y - previousTime_y);        //compute time elapsed from previous computation

        error_y = setPoint_y - inp_y;                                // determine error
        cumError_y += error_y * elapsedTime_y;                // compute integral
        rateError_y = (error_y - lastError_y)/elapsedTime_y;   // compute derivative

        Serial.print("error: ");
        Serial.print(error_y);
        Serial.print(" cumError: ");
        Serial.print(cumError_y);
        Serial.print(" rateError: ");
        Serial.print(rateError_y);

        double out_y = kp_y*error_y + ki_y*cumError_y + kd_y*rateError_y;                //PID output

        lastError_y = error_y;                                //remember current error
        previousTime_y = currentTime_y;                        //remember current time

        return out_y;                                        //have function return the PID output
}

// double computePIDLand(double inp2){
//         currentTime2 = millis();                //get current time
//         elapsedTime2 = (double)(currentTime2 - previousTime2);        //compute time elapsed from previous computation

//         error2 = setPoint2 - inp2;                                // determine error
//         cumError2 += error2 * elapsedTime2;                // compute integral
//         rateError2 = (error2 - lastError2)/elapsedTime2;   // compute derivative

//         double out2 = kp2*error2 + ki2*cumError2 + kd2*rateError2;                //PID output

//         lastError2 = error2;                                //remember current error
//         previousTime2 = currentTime2;                        //remember current time

//         return out2;                                        //have function return the PID output
// }
