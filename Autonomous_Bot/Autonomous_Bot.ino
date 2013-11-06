#include <Servo.h> 

Servo servoLeft, servoRight;

//Interface
// Don't use this pins: 4, 7, 10, 11, 12, 13
const int pingPin = 9;
const int RED_PIN = 1;
const int GREEN_PIN = 8;
const int BLUE_PIN = 3;
const int SWITCH_PIN = 2;

const int STATE_PRE_START = -1;
const int STATE_SEARCHING = 0;
const int STATE_FOUND = 1;
const int STATE_NOT_FOUND = 2;
const int STATE_DOCKED = 3;
const int STATE_TURNED = 4;
const int STATE_HOME = 5;

int state;
unsigned long start_time, end_time;
unsigned long time_delta = 0;

void setup() {
  state = STATE_SEARCHING;
  Serial.begin(9600);
  servoLeft.attach(6);
  servoRight.attach(5);
  halt();
  
  //Set LED pins
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  
  attachInterrupt(0, switchInterrupt, CHANGE);
}


void switchInterrupt(){
  if (state==STATE_FOUND){
    end_time = millis();
    time_delta = end_time - start_time;
    Serial.print("Time passed: ");
    Serial.println(time_delta);
    Serial.println();
    state = STATE_DOCKED;
    halt();
  }
}

void goForward(){
  servoLeft.write(180);
  servoRight.write(0);
}

void goBackward(){
  servoLeft.write(0);
  servoRight.write(180);
}

void turnRight(){
  servoLeft.write(180);
  servoRight.write(180);
}

void turnLeft(){
  servoLeft.write(0);
  servoRight.write(0);
}

void goRight(){
  servoLeft.write(180);
  servoRight.write(85);
}

void goLeft(){
  servoLeft.write(95);
  servoRight.write(0);
}

void halt(){
  servoLeft.write(90);
  servoRight.write(90);
}

//returns distance to an object in cm, Code was taken here http://arduino.cc/en/Tutorial/Ping?from=Tutorial.UltrasoundSensor
int pingTarget() {
  Serial.print("Ping:");
  Serial.println();
  
  long duration, cm;
  
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);
  cm = microsecondsToCentimeters(duration);

  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
 
  delay(100); //MAKE ME FASTER
  return cm;
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2; 
}

// LED - Search for Target
void setRED_LED() {
  digitalWrite(RED_PIN, HIGH);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(BLUE_PIN, LOW);
}

// LED - Target found
void setGREEN_LED() {
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GREEN_PIN, HIGH);
  digitalWrite(BLUE_PIN, LOW);
}

// LED - Pause
void setBLUE_LED() {
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(BLUE_PIN, HIGH);
}

// LED - Go to Target
void setPURPLE_LED() {
  digitalWrite(RED_PIN, HIGH);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(BLUE_PIN, HIGH);
}


void stepForward(){
  servoLeft.write(180);
  servoRight.write(0);
  delay(2000); //SHOULD BE CALIBRATED
}

void stepBackward(){
  servoLeft.write(0);
  servoRight.write(180);
  delay(2000); //SHOULD BE CALIBRATED
  halt();
}

void stepRight(){
  servoLeft.write(180);
  servoRight.write(85);
  delay(500); //SHOULD BE CALIBRATED
  halt();
}

void stepLeft(){
  servoLeft.write(95);
  servoRight.write(0);
  delay(500); //SHOULD BE CALIBRATED
  halt();
}

void scanningTurnRight(){
  servoLeft.write(93);
  servoRight.write(93);
}

void scanningTurnLeft(){
  servoLeft.write(87);
  servoRight.write(87);
}

void turnStepRight(){
  servoLeft.write(95);
  servoRight.write(95);
  delay(100); //SHOULD BE CALIBRATED
  halt();
}

void turnStepLeft(){
  servoLeft.write(0);
  servoRight.write(0);
  delay(100); //SHOULD BE CALIBRATED
  halt();
}

void goToTarget() {
  Serial.print("Going to target");
  Serial.println();
  setPURPLE_LED();
  goForward();
  delay(5000); //SHOULD BE CALIBRATED
}


void findTarget(){
  Serial.print("Scanning for Target");
  Serial.println();
  
  setRED_LED();
  halt();
  
  int searchStatus = 0; //0 - not found, 1 - found
  int minDistance = 3; //cm SHOULD BE CALIBRATED
  int maxDistance = 50; //cm SHOULD BE CALIBRATED
  int targetDistance = maxDistance;
  int searchLoop = 3;
  while((searchLoop > 0) && (searchStatus == 0)) {
    targetDistance = pingTarget();
    
    if (targetDistance < minDistance) {
      //do something
      Serial.print("Target is too close");
      Serial.println();
    }
    else if(targetDistance < maxDistance) {
      searchStatus = 1;
      Serial.print("Target found");
      Serial.println();
      setGREEN_LED();
      delay(2000); // SHOULD BE DELETED
      goToTarget();
    }
    else {
      searchLoop--; // Check if the target could not be found
      turnStepRight();
      delay(2000); // SHOULD BE CALIBRATED
      Serial.print("Target not found, continue scanning");
      Serial.println();
    }
  }
  
  if (searchStatus == 0) {
    Serial.print("Target not found. Go to pause regime");
    Serial.println();
    setBLUE_LED();
    delay(2000); // SHOULD BE DELETED
  }
}

void findTarget2(){
  Serial.print("Scanning for Target");
  Serial.println();
  
  setRED_LED();
  scanningTurnRight();
  
  int searchStatus = 0; //0 - not found, 1 - found
  int minDistance = 3; //cm SHOULD BE CALIBRATED
  int maxDistance = 50; //cm SHOULD BE CALIBRATED
  int targetDistance = maxDistance;
  const int left = 0, right = 1, loopLength=20;
  int searchLoop = 100;
  int side = right;
  int iteration = 1;
  int currentLoop = loopLength * iteration;
  while (searchLoop > 0 && searchStatus == 0) {
    if (currentLoop > 0){
      currentLoop --;
    }
    else {
      iteration ++;
      currentLoop = loopLength * iteration;
      side = 1 - side;
      if (side==left) {
        scanningTurnLeft();
      } 
      else {
        scanningTurnRight();
      }
    }
    
    targetDistance = pingTarget();
    
    if(targetDistance < maxDistance) {
      searchStatus = 1;
      Serial.print("Target found");
      Serial.println();
      setGREEN_LED();
      state = STATE_FOUND;
    }
    else if (targetDistance < minDistance) {
      //do something
      Serial.print("Target is too close");
      Serial.println();
    }
    searchLoop--;
    delay(10);
  }

  if (searchStatus == 0) {
    Serial.print("Target not found. Go to pause regime");
    Serial.println();
    setBLUE_LED();
    delay(2000); // SHOULD BE DELETED
  }
}

void loop()
{
  if (state == STATE_SEARCHING) {
    findTarget2();
  }
  else if (state == STATE_FOUND){
    start_time = millis();
    goToTarget();
  }
  else if (state == STATE_DOCKED){
    for (int i=0; i<4; i++){
      goForward();
      delay(100);
      scanningTurnLeft();
      delay(970);
    }
    halt();
    state = STATE_TURNED;
  }
  else if (state == STATE_TURNED){
    goForward();
    delay(time_delta);
    halt();
    state = STATE_HOME;
  }
} 
