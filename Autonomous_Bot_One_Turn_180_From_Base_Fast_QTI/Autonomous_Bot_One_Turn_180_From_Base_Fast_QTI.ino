#include <Servo.h> 
#include <SPI.h> //WIFI
#include <WiFi.h> //WIFI

Servo servoLeft, servoRight;

char ssid[] = "CMU"; //  your network SSID (name) 
int status = WL_IDLE_STATUS;

WiFiServer server(23);
WiFiClient client;

//Interface
// Don't use this pins: 4, 7, 10, 11, 12, 13
const int SERVO_RIGHT_PIN = 5;
const int SERVO_LEFT_PIN = 6;

const int PING_PIN = 9;
const int SWITCH_PIN = 2;
const int GREEN_PIN = 15;
const int BLUE_PIN = 3;
const int RED_PIN = 8;
const int RIGHT_QTI_PIN = 17;
const int MIDDLE_QTI_PIN = 18;
const int LEFT_QTI_PIN = 19;

const int STATE_PRE_START = -1;
const int STATE_SEARCHING_FIRST = 0;
const int STATE_FOUND_FIRST = 1;
const int STATE_NOT_FOUND = 2;
const int STATE_DOCKED = 3;
const int STATE_TURNED = 4;
const int STATE_HOME = 5;
const int STATE_FOUND_SECOND = 6;
const int STATE_SEARCHING_SECOND = 7;
const int STATE_RESUME = 8;
const int STATE_LINE_DETERMENING = 10;

const int maxDistance = 75;

boolean hasNewMessage = false;
boolean movingFirstTime = true;

// These are the values, when we assume that underneath a QTI sensor is a black surface.
const int BOUNDARY_QTI_THRESHOLD = 1100;
const int MIDDLE_QTI_THRESHOLD = 1100;

int state;
unsigned long startLineTime, endLineTime;
unsigned long time_delta = 0;
unsigned long timeDockedDelta = 0, dockedTime = 0;

char* prevMessage = "";

void sendMessage(char* message){
  if (client && message != prevMessage){
      server.write(message);
      server.write("\r\n");
      Serial.println(message);
      prevMessage = message;
  }
}

char getMessage(){
  char thisChar = 'z';
  
  if (client.available() > 0) {
      thisChar = client.read();
  } 

  return thisChar;
}


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
  char a[message.length()+1];
  message.toCharArray(a, message.length()+1);
  //sendMessage(a); 
  Serial.println(message);
}

char* stateString(){
  switch (state){
    case STATE_PRE_START: return "Waiting for commands"; break;
    case STATE_SEARCHING_FIRST: return "Searching for a minion"; break;
    case STATE_FOUND_FIRST: return "Minion found"; break;
    case STATE_NOT_FOUND: return "No minion found. Going to pause state"; break;
    case STATE_DOCKED: return "Docked with minion"; break;
    case STATE_TURNED: return "Docked with minion"; break;
    case STATE_HOME: return "Minion delivered"; break;
    case STATE_FOUND_SECOND: return "Minion found"; break;
    case STATE_SEARCHING_SECOND: return "Searching for a minion"; break;
    case STATE_RESUME: return "Resuming search"; break;
  default: return "ERROR!";  
  }
}

// -------------------------------------------------------------------------------------
// Initial ENCO BOT setup
void setup() {
  Serial.begin(9600);

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present"); 
    // don't continue:
    while(true);
  } 
  
  // attempt to connect to Wifi network:
  while ( status != WL_CONNECTED) { 
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid);

    // wait 1 seconds for connection:
    delay(1000);
  } 
  // start the server:
  server.begin();
  // you're connected now, so print out the status:
  printWifiStatus();
  
  setBlueLED();

  // wait for a new client:
  client = server.available();
  
  while (!client) {
    client = server.available();
  }
  
  // clead out the input buffer:
  client.flush();    
  Serial.println("We have a new client");

  state = STATE_PRE_START; 
//  state = STATE_RESUME;
//  state = STATE_SEARCHING_FIRST;
//  state = STATE_TURNED;
//  state = STATE_SEARCHING_FIRST;
  

  servoLeft.attach(SERVO_LEFT_PIN);
  servoRight.attach(SERVO_RIGHT_PIN);
  halt();

  //Set LED pins
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  attachInterrupt(0, switchInterrupt, CHANGE);
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}



int findTargetInOneStep(){
  int TargetFound = 0;
  debug("Scanning for Target from Base in one step");

  return rescan(700, 37); 
}

void goToTarget(){
  setGreenLED();
  int initialDistance = pingTarget();
  debug("Going to target");
  if (initialDistance < 20){
    setOffLED();
    goForward(120);
    setGreenLED();
  }
  else if (initialDistance < maxDistance){
    setOffLED();
    goForward(700);
    setGreenLED();
  }
  
  if (pingTarget() > maxDistance) {
    halt();
    setRedLED();
    if ((state != STATE_DOCKED) && !rescan(300, 14)){
      if ((state != STATE_DOCKED) && !rescan(500, 30)) {
        state = STATE_NOT_FOUND;
        hasNewMessage = true;
      }
    }
  }
}

// Interrupt on press/release the switch
void switchInterrupt(){
  if ((state==STATE_FOUND_FIRST) || (state==STATE_FOUND_SECOND)) {
    state = STATE_DOCKED;
    hasNewMessage = true;
    dockedTime = millis();
    halt();
  }
}

int leftTurnsMsec = 0;
int rightTurnsMsec = 0;

void turnLeft(){  
  servoLeft.write(0);
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

void goBackward(int msec){
  servoLeft.write(0);
  servoRight.write(180);
  delay(msec);
  halt();
}

void halt(){
  servoLeft.write(90);
  servoRight.write(90);
}

// returns distance to an object in cm, Code was taken here http://arduino.cc/en/Tutorial/Ping?from=Tutorial.UltrasoundSensor
int pingTarget() {
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

  delay(5);
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

// Network connected
void setBlueLED() {
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(BLUE_PIN, HIGH);
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

// ------------------------------------------------------------------------------
int rescan (int initialTurn, int turnNumber){
  unsigned long rescan_start_time, rescan_end_time, rescan_time;
  
  int TargetFound = 0;
  
  turnRight(initialTurn);

  rescan_start_time = millis();

  int minDistance = 3; //cm
  int targetDistance = maxDistance + 1;
  turnLeft();
  
  int turnCounter = turnNumber;
  while ((targetDistance > maxDistance || targetDistance < minDistance) && (turnCounter>0)) {
    targetDistance = pingTarget();
    if (targetDistance <= maxDistance){
      halt();
      break;
    }
    delay(20);
    turnCounter--;
  }
  
  rescan_end_time = millis();
  rescan_time = rescan_end_time - rescan_start_time;
  leftTurnsMsec += rescan_time * 0.9;
  
  if (targetDistance <= maxDistance && targetDistance >= minDistance){
    TargetFound = 1;
    if(targetDistance < 20) {
      turnLeft(110);
    }
    else {
      turnLeft(170);
      goForward(1000);
    }
  }
  else if (targetDistance < minDistance){
    goForward(650);
  }
  else {
    turnRight(initialTurn);
  }

  return TargetFound;
}

// ------------------------------------------------------------------------------
// Turning 180 degrees back home
void turnHome(){  
  int idealTurn = 1500;
  int turnsDiff = rightTurnsMsec - leftTurnsMsec;  
  int iterations = (idealTurn - turnsDiff) / 15 ;

  for (int i=0; i<iterations; i++){
    goForward(20);
    turnRight(20);
  }
  
  state = STATE_TURNED;
  hasNewMessage = true;
  debug("Turn Finished"); 
}

// ------------------------------------------------------------------------------
void undockTarget(){
  servoLeft.write(0);
  servoRight.write(180);
  delay(800);
  halt();

  turnLeft(700); 
  
  int rightQtiRCTime = RCTime(RIGHT_QTI_PIN);
  int middleQtiRCTime = RCTime(MIDDLE_QTI_PIN);
  int leftQtiRCTime = RCTime(LEFT_QTI_PIN);

  boolean isRightBlack = rightQtiRCTime > BOUNDARY_QTI_THRESHOLD;
  boolean isMiddleBlack = middleQtiRCTime > MIDDLE_QTI_THRESHOLD;
  boolean isLeftBlack = leftQtiRCTime > BOUNDARY_QTI_THRESHOLD;
  
  unsigned long time1 = millis();
  while (!isRightBlack && !isMiddleBlack && !isLeftBlack){
    if (millis() - time1 > 3000)
    {
      goBackward(300);
    }
    else {
      turnLeft();    
    }

    rightQtiRCTime = RCTime(RIGHT_QTI_PIN);
    middleQtiRCTime = RCTime(MIDDLE_QTI_PIN);
    leftQtiRCTime = RCTime(LEFT_QTI_PIN);

    isRightBlack = rightQtiRCTime > BOUNDARY_QTI_THRESHOLD;
    isMiddleBlack = middleQtiRCTime > MIDDLE_QTI_THRESHOLD;
    isLeftBlack = leftQtiRCTime > BOUNDARY_QTI_THRESHOLD;      
  }
  startLineTime = millis();
  
  turnLeft(70);
  leftTurnsMsec = 0;
  rightTurnsMsec = 0;
  state = STATE_SEARCHING_FIRST;
  hasNewMessage = true;
  debug("Undock Finished");
}

// ------------------------------------------------------------------------------
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

boolean firstLineCross = false;
boolean firstRightCross = false;
boolean firstLeftCross = false;

// ------------------------------------------------------------------------------
// Following the black line
void followBlackLine()
{  
  int rightQtiRCTime = RCTime(RIGHT_QTI_PIN);
  int middleQtiRCTime = RCTime(MIDDLE_QTI_PIN);
  int leftQtiRCTime = RCTime(LEFT_QTI_PIN);

  boolean isRightBlack = rightQtiRCTime > BOUNDARY_QTI_THRESHOLD;
  boolean isMiddleBlack = middleQtiRCTime > MIDDLE_QTI_THRESHOLD;
  boolean isLeftBlack = leftQtiRCTime > BOUNDARY_QTI_THRESHOLD;
  
  unsigned long timeLineDelta;
  endLineTime = millis();
  timeLineDelta = endLineTime - startLineTime;
  timeDockedDelta = endLineTime - dockedTime;

  if ((isRightBlack && isMiddleBlack && firstLineCross && (timeLineDelta > 2000))
    ||(isLeftBlack && isMiddleBlack && firstLineCross && (timeLineDelta > 2000))
    ||(isRightBlack && isMiddleBlack && isLeftBlack && (timeDockedDelta > 7000)))
  { 
    delay(50);
    halt();
    rightQtiRCTime = RCTime(RIGHT_QTI_PIN);
    leftQtiRCTime = RCTime(LEFT_QTI_PIN);
    middleQtiRCTime = RCTime(MIDDLE_QTI_PIN);

    isRightBlack = rightQtiRCTime > BOUNDARY_QTI_THRESHOLD;
    isMiddleBlack = middleQtiRCTime > MIDDLE_QTI_THRESHOLD;
    isLeftBlack = leftQtiRCTime > BOUNDARY_QTI_THRESHOLD;         

    halt();
    
    state = STATE_HOME;
    hasNewMessage = true;
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
    servoLeft.write(90);
    servoRight.write(0);
  }
  else if (isMiddleBlack && !isRightBlack && !isLeftBlack)
  {
    if(!firstLineCross){
      firstLineCross = true;
      startLineTime = millis();
    }
  }
  else 
  {
    servoLeft.write(180);
    servoRight.write(0);
  }
}

// ------------------------------------------------------------------------------
void waitForCommands(){
  char message = getMessage();
  
  switch(message){
    case 'z': break;
    case 'S': 
      Serial.println("Start");
      hasNewMessage = true;
      state = STATE_SEARCHING_FIRST;
      break;
    case 'R': 
      debug("Resume"); 
      leftTurnsMsec = 0;
      rightTurnsMsec = 0;
      hasNewMessage = true;
      state = STATE_RESUME;
      break;
    default: break;
  }
}

int ping = 180;


// ------------------------------------------------------------------------------

int startMovingToScan(){
  if (movingFirstTime){
    movingFirstTime = false;
    goForward(5000);
  }
  else {
    goForward(4000);
  }
  
  return findTargetInOneStep();
}

void startMovingToScan(int msec)
{  
  int rightQtiRCTime = RCTime(RIGHT_QTI_PIN);
  int middleQtiRCTime = RCTime(MIDDLE_QTI_PIN);
  int leftQtiRCTime = RCTime(LEFT_QTI_PIN);

  boolean isRightBlack = rightQtiRCTime > BOUNDARY_QTI_THRESHOLD;
  boolean isMiddleBlack = middleQtiRCTime > MIDDLE_QTI_THRESHOLD;
  boolean isLeftBlack = leftQtiRCTime > BOUNDARY_QTI_THRESHOLD;
  
  unsigned long timeLineDelta;
  endLineTime = millis();
  timeLineDelta = endLineTime - startLineTime;
  
  if (timeLineDelta >= msec){
    state = STATE_LINE_DETERMENING;
  }
  else if (isRightBlack && !isLeftBlack)
  {
    servoLeft.write(180);
    servoRight.write(90);
  }
  else if (!isRightBlack && isLeftBlack)
  {
    servoLeft.write(90);
    servoRight.write(0);
  }
  else
  {
    servoLeft.write(180);
    servoRight.write(0);
  }
}

void goToTheCenter(){  
  turnRight(480);
  goForward(600);
  
  debug(RCTime(RIGHT_QTI_PIN));
  
  if (RCTime(RIGHT_QTI_PIN) > BOUNDARY_QTI_THRESHOLD){
    debug("Right");
    
    turnLeft(1340);
    goForward(2600);
    turnRight(700);
    
    state = STATE_SEARCHING_FIRST;
  }
  else{
    goBackward(600);
    turnLeft(460);
    
    turnLeft(460);
    goForward(600);
    
    if (RCTime(LEFT_QTI_PIN) > BOUNDARY_QTI_THRESHOLD){
      debug("Left");
      
      turnRight(1200);
      goForward(2600);
      turnLeft(800);
      
      state = STATE_SEARCHING_FIRST;
    }
    else{
      goBackward(600);
      turnRight(410);
      
      state = STATE_SEARCHING_FIRST;
      debug("Center");
    }
  }
}


void loop()
{ 
  if (hasNewMessage){
    hasNewMessage = false;
    sendMessage(stateString()); 
  }
  
  if (state == STATE_PRE_START){
    waitForCommands();
  }
  else if (state == STATE_SEARCHING_FIRST) {
    setRedLED();
    
    if (movingFirstTime){
      if(!startMovingToScan()){
        halt(); // Target not found
        state = STATE_NOT_FOUND;
        hasNewMessage = true;
      }
      else {
        state = STATE_FOUND_FIRST;
        hasNewMessage = true;
      }
    }
    else{
      startMovingToScan(4200);
    }
  }
  else if (state == STATE_LINE_DETERMENING){
    goToTheCenter();
    if(!findTargetInOneStep()){
      halt(); // Target not found
      state = STATE_NOT_FOUND;
      hasNewMessage = true;
    }
    else {
      state = STATE_FOUND_FIRST;
      hasNewMessage = true;
    }
  }
  else if (state == STATE_FOUND_FIRST){
    goToTarget();
  }
  else if (state == STATE_DOCKED){
    setGreenLED();
    turnHome();
  }
  else if (state == STATE_TURNED){
    followBlackLine();
  }
  else if (state == STATE_HOME){
    undockTarget();
    state = STATE_SEARCHING_FIRST;    
  }
  else if (state == STATE_NOT_FOUND){
    waitForCommands();
    setRedBlinkLED();
  }
  else if (state == STATE_RESUME) {
    setRedLED();
    if(!findTargetInOneStep()){
      halt(); // Target not found
      state = STATE_NOT_FOUND;
      hasNewMessage = true;
    }
    else {
      state = STATE_FOUND_FIRST;
      hasNewMessage = true;
    }  
  }
}
