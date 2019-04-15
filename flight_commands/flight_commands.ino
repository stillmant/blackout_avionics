/*
 * Reference
 * https://www.instructables.com/id/Interfacing-Servo-Motor-With-ESP32/
 * 
 * https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/dac.html
 * https://randomnerdtutorials.com/esp32-pwm-arduino-ide/
 * https://github.com/espressif/arduino-esp32/issues/4
 * https://youtu.be/SBG7ccW5gpA?t=348
 */

 #define COUNT_LOW 1640
 #define COUNT_HIGH 8190

#include "esp32-hal-ledc.h"

// the numbers corresponds to pin numbers on ESP
uint8_t roll = 0;
uint8_t pitch = 15;
uint8_t yaw = 2;
uint8_t throt = 4;

uint8_t arm = 5;
uint8_t manual = 18;

// Main channels: ROLL = 1, PITCH = 2, YAW = 3, THROTTLE = 4
void setChanVal(int channel, int val){
  ledcWrite(channel, val);
}

void setup() {
  Serial.begin(115200);
  delay(10);

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

void loop() {
  for (int i = COUNT_LOW; i < COUNT_HIGH; i=i+100){
    setChanVal(1,i);
    setChanVal(2,i);
    setChanVal(3,i);
    setChanVal(4,i);

    setChanVal(5,i);
    setChanVal(6,i);
    delay(50);
  }

  delay(500);
  // ESP = BetaFlight Value
  // 3140 = 974 (0% input)
  // 4940 = 1521 (50% input, roughly)
  // 6540 = 2008 (100%)

  // 0% input
  setChanVal(1,3140); 
  setChanVal(2,3140);
  setChanVal(3,3140);
  setChanVal(4,3140);

  setChanVal(5,3140);
  setChanVal(6,3140);
  delay(2000);
  
  // 50% input
  setChanVal(1,4940); 
  setChanVal(2,4940);
  setChanVal(3,4940);
  setChanVal(4,4940);

  setChanVal(5,4940);
  setChanVal(6,4940);
  delay(2000);
  
  // 100% input
  setChanVal(1,6540); 
  setChanVal(2,6540);
  setChanVal(3,6540);
  setChanVal(4,6540);

  setChanVal(5,6540);
  setChanVal(6,6540);
  delay(2000);
  
  
}
