#include <Servo.h> 
#include <SPI.h> //WIFI
#include <WiFi.h> //WIFI

Servo servoLeft, servoRight;

char ssid[] = "CMU"; //  your network SSID (name) 
int wifiStatus = WL_IDLE_STATUS;
WiFiServer server(80);

//Interface
// Don't use this pins: 4, 7, 10, 11, 12, 13
const int SERVO_RIGHT_PIN = 5;
const int SERVO_LEFT_PIN = 6;

const int PING_PIN = 9;
const int SWITCH_PIN = 2;
const int GREEN_PIN = 7;
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
boolean hasMinion = false;

// These are the values, when we assume that underneath a QTI sensor is a black surface.
const int BOUNDARY_QTI_THRESHOLD = 1000;
const int MIDDLE_QTI_THRESHOLD = 1000;

int state;
unsigned long startLineTime, endLineTime;
unsigned long timeDockedDelta = 0, dockedTime = 0;

// -------------------------------------------------------------------------------------
// Block to define debugging
void debug(int message){
  String msg = String(message);
  debug(msg);
}

void debug(char* message){
  String msg = String(message);
  debug(msg);
}

void debug(String message){
  Serial.println(message);
}

// -------------------------------------------------------------------------------------
// Initial ENCO BOT setup
void setup() {
  Serial.begin(9600);

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
  
//  IPAddress ip = WiFi.localIP();
//  Serial.print("IP Address: ");
//  Serial.println(ip);
  
  server.begin();
 
  state = STATE_PRE_START;
//  state = STATE_SEARCHING;
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
  
  goForward(3500);
  
  if(turnUndock>0){
    turnRight(90);
  }
  else if (turnUndock<0){
    turnLeft(350);
  }

  return rescan(700, 29);
}

// -------------------------------------------------------------------------------------
// If target was not found on the first step move forward and rescan
int findTargetBaseSecondStep(){
  int TargetFound = 0;
  
  debug("Scanning for Target from Base - Step 2");
  
  goForward(3100);
  
 if ((state != STATE_DOCKED) && !rescan(650, 27)) {
    if ((state != STATE_DOCKED) && !rescan(1000, 40)) {
      state=STATE_NOT_FOUND;
    }
    else {
      TargetFound = 1;
    }
  }
  else{
   TargetFound = 1;
  }

  return TargetFound;
}


// -------------------------------------------------------------------------------------
// Interrupt on press/release the switch
void switchInterrupt(){
  if (state==STATE_FOUND){
    state = STATE_DOCKED;
    dockedTime = millis();
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
  delay(1000);
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
  if (initialDistance < 15){
    setOffLED();
    goForward(100);
    setGreenLED();
  }
  else {
    setOffLED();
    goForward(900);
    setGreenLED();
  }
  //delay(30); //SHOULD BE CALIBRATED
  
  if (pingTarget() > initialDistance) {
    halt();
    setRedLED();
    if ((state != STATE_DOCKED) && !rescan(550, 16)){
      if ((state != STATE_DOCKED) && !rescan(1200, 38)) {
        state = STATE_NOT_FOUND;
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
      turnLeft(120);
    }
    else if (targetDistance < minDistance){
      goForward(600);
    }
    else {
      turnLeft(90);
    }
  }
  else {
    turnRight(initialTurn);
  }
  
  return TargetFound;
}

// Turning 180 degrees back home
void turnHome(){
  
  int turnsDiff = rightTurnsMsec - leftTurnsMsec;
  int iterations = 30 - turnsDiff / 60;
  
  if (!hasMinion){
    iterations = 30;
  }  
  Serial.println(turnsDiff);  
  
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
  timeDockedDelta = endLineTime - dockedTime;

  if ((isRightBlack && isMiddleBlack && firstLineCross && (timeLineDelta > 2000))
    ||(isLeftBlack && isMiddleBlack && firstLineCross && (timeLineDelta > 2000))
    ||(isRightBlack && isMiddleBlack && isLeftBlack && (timeDockedDelta > 8000) ))
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

void waitForCommands(){
  WiFiClient client = server.available();   // listen for incoming clients
  if (client) {                             // if you get a client,
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        if (c == '\n') {                    // if the byte is a newline character
          if (currentLine.length() == 0) {  
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();
            client.print("<a href=\"/S\"> START MISSION </a>");
            client.println();
            break;         
          } 
          else {      // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        }     
        else if (c != '\r') {    // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /S" 
        if (currentLine.endsWith("GET /S")) {
          state = STATE_SEARCHING;
        }
      }
    }
    // close the connection:
    client.stop();
  }
}

void loop()
{
  if (state == STATE_PRE_START) {
    waitForCommands();
  }
  else if (state == STATE_SEARCHING) {
    setRedLED();
    if(!findTargetBaseFirstStep()){
      if(findTargetBaseSecondStep()){
        state = STATE_FOUND;
      }
      else {
        halt(); // Target not found
        state = STATE_NOT_FOUND;
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
    hasMinion = true;
    turnHome();
  }
  else if (state == STATE_TURNED){
    Serial.println("Going Home");
    followBlackLine();
  }
  else if (state == STATE_HOME){
    Serial.println("Leave Target at Home");
    if (hasMinion){
      undockTarget();
    }
    else {
      turnLeft(2157 + turnUndock);
      state = STATE_PRE_START;  
    }
  }
  else if (state = STATE_NOT_FOUND){
    hasMinion = false;
    turnHome();
    setRedBlinkLED();
  }
}
