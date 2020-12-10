#include <Arduino.h>
/*
   SBT0811 + 28BYJ-48 stepper motor
   will turn the motor 360° clockwise and then anti-clokwise
   https://www.youtube.com/watch?v=BBVmeim2uu8
   Connect :
       IN1 -> D8
       IN2 -> D9
       IN3 -> D10
       IN4 -> D11
       VCC -> 5V-12V external source (+)
       GND -> 5V-12V external source (-)
*/

byte sbt0811_in[4] = {8,9,10,11};   // IN1, IN2, IN3, IN4 to arduino pins

int full_steps_count = 4095;    // steps for a full cycle
int step_time        = 1;       // 1ms for one step

int direction    = 1;           // will be either 1 (clockwise) or -1 (anti-clockwise)
int steps_left   = full_steps_count;
int current_step = 0;           // 0 to 7

boolean step_positions[8][4] = {
    { 1,0,0,1 },
    { 1,0,0,0 },
    { 1,1,0,0 },
    { 0,1,0,0 },
    { 0,1,1,0 },
    { 0,0,1,0 },
    { 0,0,1,1 },
    { 0,0,0,1 },
};

void setup() {
    for (byte i = 0; i <= 3; i++) {
        pinMode(sbt0811_in[i], OUTPUT); 
    }
}

void loop() {
     while (steps_left > 0) {
         move_stepper(1);
         steps_left -= 1;
     }
     steps_left = full_steps_count;
     direction  = (direction > 0 ? -1 : 1);

     delay(500);
}

void move_stepper(int step_count) {
    for (int i=0;i<step_count;i++) {
        // step one step in given direction
        current_step += direction;
        if (current_step > 7) { current_step = 0; }
        if (current_step < 0) { current_step = 7; }

        // set digital pins for the current_step
        for (byte i = 0; i <= 3; i++) {
            digitalWrite(sbt0811_in[i], step_positions[current_step][i]); 
        }
        
        delay(step_time);
    }
} 