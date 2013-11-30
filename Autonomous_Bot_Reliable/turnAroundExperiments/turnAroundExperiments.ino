#include <Servo.h> 

const int SERVO_RIGHT_PIN = 5;
const int SERVO_LEFT_PIN = 6;

Servo servoLeft, servoRight;

void setup(){
  Serial.begin(9600);
  servoLeft.attach(SERVO_LEFT_PIN);
  servoRight.attach(SERVO_RIGHT_PIN);
  
  turnLeft(450);
  delay(2000);
  turnHome();  
}

void loop(){
  
}


int leftTurnsMsec = 0;
int rightTurnsMsec = 0;

void turnHome(){
  int turnsDiff = rightTurnsMsec - leftTurnsMsec;
  int iterations = 30 - turnsDiff / 60;
    
  for (int i=0; i<iterations; i++){
    goForward();
    delay(20);
    turnRight(90);
  }
  
  leftTurnsMsec = 0;
  rightTurnsMsec = 0;
}

void goForward(){
  servoLeft.write(180);
  servoRight.write(0);
}

void turnRight(int msec){
  rightTurnsMsec += msec;
  servoLeft.write(180);
  servoRight.write(180);
  delay(msec);
  halt();
}  

void turnLeft(int msec){
  leftTurnsMsec += msec;
  servoLeft.write(0);
  servoRight.write(0);
  delay(msec);
  halt();
}


void halt(){
  servoLeft.write(90);
  servoRight.write(90);
}


