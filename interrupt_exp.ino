#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"

#define Kp  100
#define Kd  0
#define Ki  0
#define sampleTime  0.005
#define targetAngle 0

MPU6050 mpu;

int16_t accY, accZ, gyroX;
volatile int PID, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
volatile byte count=0;



float motorSpeed;



byte sbt0811_in[4] = {8,9,10,11};   // IN1, IN2, IN3, IN4 to arduino pins

int step_time;       // in microseconds, min 1ms for one step

int direction;           // will be either 1 (clockwise) or -1 (anti-clockwise)

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



void init_PID() {  
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B    
  // set compare match register to set sample time 5ms
  OCR1A = 9999;    
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}


void setup() {
  // set the status LED to output mode 
  pinMode(13, OUTPUT);

  for (byte i = 0; i <= 3; i++) {
    pinMode(sbt0811_in[i], OUTPUT); 
  }


  // initialize the MPU6050 and set offset values
  mpu.initialize();
  //mpu.setYAccelOffset(1593);
  //mpu.setZAccelOffset(963);
  //mpu.setXGyroOffset(40);

  // initialize PID sampling loop
  init_PID();
  Serial.begin(9600);
}


void move_stepper(int step_time, int direction) {
    for (int i=0; i < 1; i++) {
        // do one step in a given direction
        current_step += direction;
        if (current_step > 7) { current_step = 0; }
        if (current_step < 0) { current_step = 7; }

        // set digital pins for the current_step
        for (byte i = 0; i <= 3; i++) {
            digitalWrite(sbt0811_in[i], step_positions[current_step][i]); 
        }
        
        delayMicroseconds(step_time);
    }
} 


void loop() {
  // read acceleration and gyroscope values
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();  
  gyroX = mpu.getRotationX();

  // move motor after constraining the speed

  motorSpeed = abs(constrain(PID, -700, 700))*10;

  //?? need to compensate for the non-linear behaviour of stepper motors? t in YABR project

  step_time = map(motorSpeed, 0, 7000, 15000, 300);
  if (currentAngle > 0) {
    direction = 1;
  }
  else {
    direction = -1;
  }

  move_stepper(step_time, direction);



  //Serial.println(PID);
  

    
}


// The ISR will be called every 5 milliseconds
ISR(TIMER1_COMPA_vect)
{
  // calculate the angle of inclination
  accAngle = atan2(accY, accZ)*RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate*sampleTime;  
  currentAngle = 0.9934*(prevAngle + gyroAngle) + 0.0066*(accAngle);
  
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;  
  errorSum = constrain(errorSum, -300, 300);

  // calculate output from P, I and D values
  PID = Kp*(error) + Ki*(errorSum)*sampleTime - Kd*(currentAngle-prevAngle)/sampleTime;
  prevAngle = currentAngle;

  // toggle the led on pin13 every second
  count++;
  if(count == 200)  {
    count = 0;
    digitalWrite(13, !digitalRead(13));
  }
}
