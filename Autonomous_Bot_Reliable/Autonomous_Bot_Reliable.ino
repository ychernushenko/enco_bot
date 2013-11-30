#include <Servo.h> 
#include <SPI.h> //WIFI
#include <WiFi.h> //WIFI

Servo servoLeft, servoRight;
char ssid[] = "CMU"; //  your network SSID (name) 

int wifiStatus = WL_IDLE_STATUS;
char server[] = "128.237.140.220";
int port = 8000;
String server_str(server);
WiFiClient client;

//Interface
// Don't use this pins: 4, 7, 10, 11, 12, 13
const int SERVO_RIGHT_PIN = 5;
const int SERVO_LEFT_PIN = 6;

const int PING_PIN = 9;
const int SWITCH_PIN = 2;
const int GREEN_PIN = 1;
const int BLUE_PIN = 3;
const int RED_PIN = 8;
const int RIGHT_QTI_PIN = 17;
const int MIDDLE_QTI_PIN = 18;
const int LEFT_QTI_PIN = 19;

const int STATE_PRE_START = -1;
const int STATE_SEARCHING = 0;
const int STATE_FOUND = 1;
const int STATE_NOT_FOUND = 2;
const int STATE_DOCKED = 3;
const int STATE_TURNED = 4;
const int STATE_HOME = 5;

int turnUndock = 0;

// These are the values, when we assume that underneath a QTI sensor is a black surface.
const int BOUNDARY_QTI_THRESHOLD = 1000;
const int MIDDLE_QTI_THRESHOLD = 1000;

int state;
unsigned long startLineTime, endLineTime;
unsigned long time_delta = 0;

// -------------------------------------------------------------------------------------
// Block to define debugging
boolean useSerial = true;

void debug(int message){
  String msg = String(message);
  debug(msg);
}

void debug(char* message){
  String msg = String(message);
  debug(msg);
}

void debug(String message){
  if (useSerial){
    Serial.println(message);
  }
  else {
    client.println(message);
    while (client.available()){
      client.read();
    }
  }
}

// -------------------------------------------------------------------------------------
// Initial ENCO BOT setup
void setup() {
  Serial.begin(9600);
  
  if (!useSerial){
    // check for the presence of the shield:
    if (WiFi.status() == WL_NO_SHIELD) {
      Serial.println("WiFi shield not present"); 
      while(true);
    } 

    // attempt to connect to Wifi network:
    while (wifiStatus != WL_CONNECTED) { 
      Serial.print("Attempting to connect to SSID: ");
      Serial.println(ssid);
      wifiStatus = WiFi.begin(ssid);
    }
  
    if (client.connect(server, port)){
      Serial.println("Connection established");
    } 
    else {
      Serial.println("Connection failed");  
    }
  }
 
  state = STATE_SEARCHING;
  servoLeft.attach(SERVO_LEFT_PIN);
  servoRight.attach(SERVO_RIGHT_PIN);
  halt();

  //Set LED pins
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  attachInterrupt(0, switchInterrupt, CHANGE);
}

// -------------------------------------------------------------------------------------
// Go to the middle, and start scan
int findTargetBaseFirstStep(){
  int TargetFound = 0;
  
  debug("Scanning for Target from Base - Step 1");
  
  goForward(3400);
  
  if(turnUndock>0){
    turnRight(100);
  }
  else if (turnUndock<0){
    turnLeft(330);
  }

  return rescan(700, 25);
}

// -------------------------------------------------------------------------------------
// If target was not found on the first step move forward and rescan
int findTargetBaseSecondStep(){
  int TargetFound = 0;
  
  debug("Scanning for Target from Base - Step 2");
  
  goForward(3100);
  
 if ((state != STATE_DOCKED) && !rescan(650, 25)) {
    if ((state != STATE_DOCKED) && !rescan(1200, 35)) {
          if ((state != STATE_DOCKED) && rescan(2000, 50)){
            TargetFound = 1;
            turnLeft(100);
          }
        }
    else {
      TargetFound = 1;
      turnLeft(100);
    }
  }
  else{
   TargetFound = 1;
   turnLeft(100);
  }

  return TargetFound;
}


// -------------------------------------------------------------------------------------
// Interrupt on press/release the switch
void switchInterrupt(){
  if (state==STATE_FOUND){
    state = STATE_DOCKED;
    halt();
  }
}


int leftTurnsMsec = 0;
int rightTurnsMsec = 0;

void slowScanningTurnLeft(){  
  servoLeft.write(87);
  servoRight.write(87);
}

void turnRight(int msec){
  rightTurnsMsec += msec;
  
  servoLeft.write(180);
  servoRight.write(180);
  delay(msec);
  halt();
  
//  Serial.println(rightTurnsMsec);
//  delay(3000);
}

void turnLeft(int msec){
  leftTurnsMsec += msec;
  servoLeft.write(0);
  servoRight.write(0);
  delay(msec);
  halt();
}

void goForward(int msec){
  servoLeft.write(180);
  servoRight.write(0);
  delay(msec);
  halt();
}

void goForward(){
  servoLeft.write(180);
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

// returns distance to an object in cm, Code was taken here http://arduino.cc/en/Tutorial/Ping?from=Tutorial.UltrasoundSensor
int pingTarget() {
  debug("Ping:");

  long duration, cm;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(PING_PIN, OUTPUT);
  digitalWrite(PING_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(PING_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(PING_PIN, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(PING_PIN, INPUT);
  duration = pulseIn(PING_PIN, HIGH);
  cm = microsecondsToCentimeters(duration);

  debug(cm);

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
void setRedLED() {
  digitalWrite(RED_PIN, HIGH);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(BLUE_PIN, LOW);
}

// LED - Target found, blink - Go to Target
void setGreenLED() {
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GREEN_PIN, HIGH);
  digitalWrite(BLUE_PIN, LOW);
}

// LED - off
void setOffLED() {
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(BLUE_PIN, LOW);
}

// LED - Target not found
void setRedBlinkLED() {
  digitalWrite(RED_PIN, HIGH);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(BLUE_PIN, LOW);
  delay(1000);
  setOffLED();
}

void scanningTurnRight(){
  servoLeft.write(94);
  servoRight.write(94);
}

void scanningTurnLeft(){
  servoLeft.write(87);
  servoRight.write(87);
}

// Going to target if lost, rescan
void goToTarget() {
  setGreenLED();
  int initialDistance = pingTarget();
  debug("Going to target");
  if (initialDistance < 20){
    setOffLED();
    goForward(100);
    setGreenLED();
  }
  else {
    setOffLED();
    goForward(800);
    setGreenLED();
  }
  //delay(30); //SHOULD BE CALIBRATED
  
  if (pingTarget() > initialDistance) {
    halt();
    setRedLED();
    if ((state != STATE_DOCKED) && !rescan(550, 16)){
      if ((state != STATE_DOCKED) && !rescan(1200, 35)) {
        if ((state != STATE_DOCKED) && !rescan(2000, 50)){
          state = STATE_NOT_FOUND;
        }
      }
    }
  }
}

int rescan (int initialTurn, int turnNumber){
  unsigned long rescan_start_time, rescan_end_time, rescan_time;
  
  int TargetFound = 0;
  
  turnRight(initialTurn);

  rescan_start_time = millis();

  int minDistance = 3; //cm SHOULD BE CALIBRATED
  int maxDistance = 60; //cm SHOULD BE CALIBRATED
  int targetDistance = maxDistance + 1;
  
  int turnCounter = turnNumber;

  while ((targetDistance > maxDistance || targetDistance < minDistance) && (turnCounter>0)) {
    if (pingTarget() <= maxDistance){
      halt();
      int targetDistance1, targetDistance2, targetDistance3;
      targetDistance1 = pingTarget();
      delay(20);
      targetDistance2 = pingTarget();
      delay(20);
      targetDistance3 = pingTarget();
      if ((targetDistance1 <= maxDistance) && (targetDistance2 <= maxDistance) && (targetDistance3 <= maxDistance)) {
        targetDistance = (targetDistance1 + targetDistance2 + targetDistance3)/3;
      }
      slowScanningTurnLeft();
    }
    Serial.println(targetDistance);
    delay(20);
    slowScanningTurnLeft();
    turnCounter--;
  }
  
  rescan_end_time = millis();
  rescan_time = rescan_end_time - rescan_start_time;
  leftTurnsMsec += rescan_time * 0.3;
  
  if (targetDistance <= maxDistance && targetDistance >= minDistance){
    TargetFound = 1;
    if(targetDistance < 20) {
      turnLeft(100);
    }
    else{
      turnLeft(80);
    }
  }
  else {
    turnRight(550);
  }
  
//  Serial.println(rescan_time);
//  Serial.println(leftTurnsMsec);
//  delay(3000);

  return TargetFound;
}


// Finding target using sonar
void findTarget(){
  debug("Scanning for Target");

  scanningTurnRight();

  int searchStatus = 0; //0 - not found, 1 - found
  int minDistance = 3; //cm SHOULD BE CALIBRATED
  int maxDistance = 180; //cm SHOULD BE CALIBRATED
  int targetDistance = maxDistance;
  const int left = 0, right = 1, loopLength=15;
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

   if (targetDistance < minDistance) {
      //do something
      debug("Target is too close");
    }
    else if(targetDistance < maxDistance) {
      searchStatus = 1;
      debug("Target found");
      state = STATE_FOUND;
    }
    searchLoop--;
    delay(10);
  }

  if (searchStatus == 0) {
    debug("Target not found. Go to pause regime");
    delay(2000); // SHOULD BE DELETED
  }
}

// Turning 180 degrees back home
void turnHome(){
  
  int turnsDiff = rightTurnsMsec - leftTurnsMsec;
  int iterations = 30 - turnsDiff / 60;
    
    Serial.println(turnsDiff);  
   // delay(50000);
    
  for (int i=0; i<iterations; i++){
    goForward();
    delay(20);
    turnRight(90);
  }
  
  state = STATE_TURNED;
  Serial.println("Turn Finished");
}

void undockTarget(){
  servoLeft.write(0);
  servoRight.write(180);
  delay(500);
  halt();
  turnLeft(1657 + turnUndock);
  leftTurnsMsec = 0;
  rightTurnsMsec = 0;
  state = STATE_SEARCHING;
  Serial.println("Undock Finished");
}

// Return time http://learn.parallax.com/KickStart/555-27401
long RCTime(int sensorIn){
  long duration = 0;
  pinMode(sensorIn, OUTPUT);     // Make pin OUTPUT
  digitalWrite(sensorIn, HIGH);  // Pin HIGH (discharge capacitor)
  delay(1);                      // Wait 1ms
  pinMode(sensorIn, INPUT);      // Make pin INPUT
  digitalWrite(sensorIn, LOW);   // Turn off internal pullups
  while(digitalRead(sensorIn)){  // Wait for pin to go LOW
    duration++;
  }
  return duration;
}

boolean leftWasBlack = false;
boolean rightWasBlack = false;
boolean middleWasBlack = false;
boolean firstLineCross = false;
boolean firstRightCross = false;
boolean firstLeftCross = false;


// Following the black line
void followBlackLine()
{  
  int rightQtiRCTime = RCTime(RIGHT_QTI_PIN);
  int middleQtiRCTime = RCTime(MIDDLE_QTI_PIN);
  int leftQtiRCTime = RCTime(LEFT_QTI_PIN);

  boolean isRightBlack = rightQtiRCTime > BOUNDARY_QTI_THRESHOLD;
  boolean isMiddleBlack = middleQtiRCTime > MIDDLE_QTI_THRESHOLD;
  boolean isLeftBlack = leftQtiRCTime > BOUNDARY_QTI_THRESHOLD;

  //Serial.println("IR");
  //Serial.println(blackLineTurns);
  String message = String(rightQtiRCTime) + "; " + String(middleQtiRCTime) + "; " + String(leftQtiRCTime);
  String message2 = String(isRightBlack) + "; " + String(isMiddleBlack) + "; " + String(isLeftBlack);
  debug(String(leftWasBlack));
  debug(String(rightWasBlack));

  debug(message);
  debug(message2);
  
  unsigned long timeLineDelta;
  endLineTime = millis();
  timeLineDelta = endLineTime - startLineTime;

  if ((isRightBlack && isMiddleBlack && firstLineCross && (timeLineDelta > 2000))
    ||(isLeftBlack && isMiddleBlack && firstLineCross && (timeLineDelta > 2000)))
  {
      debug("Stop");
      halt();
      if(isRightBlack && isMiddleBlack && isLeftBlack) {
        turnUndock = 0;
      }
      else if (firstRightCross){
        turnUndock = -477;
      }
      else if (firstLeftCross){
        turnUndock = 350;
      }
      
      state = STATE_HOME;
      rightWasBlack = false; 
      leftWasBlack = false;
      middleWasBlack = false;
      firstLineCross = false;
      firstRightCross = false;
      firstLeftCross = false;
  }
  else if (isRightBlack && !isLeftBlack)
  {
    if(!firstLineCross){
      firstLineCross = true;
      startLineTime = millis();
      firstRightCross = true;
    }
    debug("Right");  
    rightWasBlack = true;
    servoLeft.write(180);
    servoRight.write(90);
  }
  else if (!isRightBlack && isLeftBlack)
  {
    if(!firstLineCross){
      firstLineCross = true;
      startLineTime = millis();
      firstLeftCross = true;
    }
    debug("Left");
    leftWasBlack = true;
    servoLeft.write(90);
    servoRight.write(0);
  }
  else if (isMiddleBlack && !isRightBlack && !isLeftBlack)
  {
    if(!firstLineCross){
      firstLineCross = true;
      startLineTime = millis();
    }
    debug("Middle");
    middleWasBlack = true;
  }
  else 
  {
    debug("Forward");
    servoLeft.write(98);
    servoRight.write(82);
  }

  delay(50);
}

void loop()
{
  if (state == STATE_SEARCHING) {
    setRedLED();
    
   if(!findTargetBaseFirstStep()){
      if(findTargetBaseSecondStep()){
        state = STATE_FOUND;
      }
      else {
        halt(); // Target not found
        state = STATE_NOT_FOUND;
        setRedBlinkLED();
      }
    }
    else {
        state = STATE_FOUND;
    }
  }
  else if (state == STATE_FOUND){
    goToTarget();
  }
  else if (state == STATE_DOCKED){
    setGreenLED();
    turnHome();
  }
  else if (state == STATE_TURNED){
    Serial.println("Going Home");
    followBlackLine();
  }
  else if (state == STATE_HOME){
    Serial.println("Leave Target at Home");
    undockTarget();
  }
}