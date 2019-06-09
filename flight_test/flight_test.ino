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

// Timing delays
#define SEPARATION_DELAY 3000 // ms (Free fall away from rocket for 3 sec before chute deploy)
#define CHUTE_DROP_DELAY 1000 // ms (Free fall for 1 sec after chute relase)

#define SEA_PRESSURE 1013.25

// Polling times
#define LANDED_POLLING_TIME_INTERVAL 5000 //ms
#define NOMINAL_POLLING_TIME_INTERVAL 50  //ms


 #define COUNT_LOW 3222     //999 = (0%)
 #define COUNT_HIGH 4870    //1500 = (50%)

#include "esp32-hal-ledc.h"
/*---Includes-----------------------------------------------------------*/
#include "sensors.h"
#include "calculations.h"
/*---Variables----------------------------------------------------------*/
static float alt, prev_alt, delta_alt, pressure, groundPressure, groundAlt;

// GPS
static double lat, lon, gpsAlt, gpsSats, distToTrgt;

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

const int buttonPin = 27;
int buttonState = 0; 

// Main channels: ROLL = 1, PITCH = 2, YAW = 3, THROTTLE = 4   -> 3 = THROTTLE
void setChanVal(int channel, int val){
  ledcWrite(channel, val);
}

void setup() {
  Serial.begin(115200);
  delay(10);
  initSensors(); 
  
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
  delay(5000);          //Wait 5 seconds for FC startup

  setChanVal(5,6540);   //ARMING - set AUX1 permantly at 2001 (100%) to stay armed
}

void loop() {
  static unsigned long timestamp, timestamp2;
  static unsigned long old_time = 0, old_time2 = 0; //ms
  static unsigned long new_time = 0, new_time2 = 0; //ms
  unsigned long delta_time, delta_time2;
  static uint16_t time_interval2 = 100;
  static uint16_t time_interval = NOMINAL_POLLING_TIME_INTERVAL; //ms

  static int i = COUNT_LOW;

  new_time = millis();
  if ((new_time - old_time) >= time_interval) {
    delta_time = new_time - old_time;
    old_time = new_time;

    pollSensors(&lat, &lon, &gpsAlt, &gpsSats, barData, accelData, magData);
    crunchNumbers(barData, accelData, magData, &pressure, &groundPressure, &prev_alt, &alt, &delta_alt, &lat, &lon, &distToTrgt, &delta_time);
    }

  new_time2 = millis();
  if ((new_time2 - old_time2) >= time_interval2) {
    delta_time2 = new_time2 - old_time2;
    old_time2 = new_time2;
    
    buttonState = analogRead(buttonPin);

    if (buttonState <= 4090) {       //FAIL SAFE SWITCH OFF

      setChanVal(5,3222);
      Serial.println("OFFFFFFFFFFFFFFFFFFFFFFF");
      
    } else {

      if (i < COUNT_HIGH){
      // put for loop contents here
      setChanVal(3,i);
      i = i + 10;
    }
      else{
      i = COUNT_LOW;
    }
      Serial.println("ON");
      
    }
//    Serial.println(barData[0]);
//    Serial.println(pressure);
//    Serial.println(alt);
//    Serial.println(delta_alt);
    
    }



//  for (int i = COUNT_LOW; i < COUNT_HIGH; i=i+10){     //Slowly increases THROT from 0% to 50%
//
//    buttonState = digitalRead(buttonPin);    //Read state of switch (button)
//    
//    if (buttonState == LOW) {       //FAIL SAFE SWITCH
//
//      setChanVal(6,3222);
//      Serial.println("OFF");
//      
//    } else {
//
//      setChanVal(3,i);
//      Serial.println("ON");
//      
//    }
    
//    setChanVal(1,i);
//    setChanVal(2,i);
//    setChanVal(3,i);
//    setChanVal(4,i);

//    setChanVal(5,i);
//    setChanVal(6,i);
    
  }

//  delay(500);
  // ESP = BetaFlight Value
  // 3222 = 999 (0% input)
  // 4870 = 1500 (50% input, exactly)
  // 6520 = 2001 (100%)

  // 0% input
//  setChanVal(1,3222); 
//  setChanVal(2,3222);
//  setChanVal(3,3222);         //Sets THROT back to 0%
//  setChanVal(4,3222);

//  setChanVal(5,3222);
//  setChanVal(6,3222);
//  delay(2000);
  
  // 50% input
//  setChanVal(1,4870); 
//  setChanVal(2,4870);
//  setChanVal(3,4870);
//  setChanVal(4,4870);

//  setChanVal(5,4870);
//  setChanVal(6,4870);    //AUX2 needs to be at 1500 (For Safe flight mode - no flips/roll)
//  delay(5000);
  
  // 100% input
//  setChanVal(1,6520); 
//  setChanVal(2,6520);
//  setChanVal(3,6520);
//  setChanVal(4,6520);

//  setChanVal(5,6520);   //AUX1 needs to stay at 2001 (100%) for ARMING
//  setChanVal(6,6520);
//  delay(2000);