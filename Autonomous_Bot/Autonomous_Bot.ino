#include <Servo.h> 
#include <SPI.h> //WIFI
#include <WiFi.h> //WIFI

Servo servoLeft, servoRight;
char ssid[] = "CMU"; //  your network SSID (name) 

int wifiStatus = WL_IDLE_STATUS;
char server[] = "128.237.250.69";
int port = 8000;
String server_str(server);
WiFiClient client;

//Interface
// Don't use this pins: 4, 7, 10, 11, 12, 13
const int SERVO_RIGHT_PIN = 5;
const int SERVO_LEFT_PIN = 6;

const int PING_PIN = 9;
const int SWITCH_PIN = 2;
const int RED_PIN = 1;
const int BLUE_PIN = 3;
const int GREEN_PIN = 8;
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

// These are the values, when we assume that underneath a QTI sensor is a black surface.
const int BOUNDARY_QTI_THRESHOLD = 1000;
const int MIDDLE_QTI_THRESHOLD = 1000;

int state;
unsigned long start_time, end_time;
unsigned long time_delta = 0;

// -------------------------------------------------------------------------------------
// Block to define debugging
boolean useSerial = false;

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
// Interrupt on press/release the switch
void switchInterrupt(){
  if (state==STATE_FOUND){
    state = STATE_DOCKED;
    end_time = millis();
    time_delta = end_time - start_time;
    debug("Time passed:");
    debug(time_delta);
    halt();
  }
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

void scanningTurnRight(){
  servoLeft.write(94);
  servoRight.write(94);
}

void scanningTurnLeft(){
  servoLeft.write(87);
  servoRight.write(87);
}

// Going forward
void goToTarget() {
  debug("Going to target");
  setPURPLE_LED();
  goForward();
  delay(5000); //SHOULD BE CALIBRATED
}

// Going forward v2
void goToTarget2() {
  debug("Going to target");
  setPURPLE_LED();
  goForward();
  int initialDistance = pingTarget();
  int distance = initialDistance;
  debug("Distance to target = ");
  debug(distance);
  int numberOfOuts = 0;
  int oneSideCycle = 10; // SHOULD BE CALIBRATED
  int outCycle = 0;
  boolean goesRight = false;

  while (distance>10) {
    int newDistance = pingTarget();
    debug("New distance to target = ");
    debug(newDistance);
    if (newDistance > distance + 2){
      debug("Out");
      numberOfOuts++;
      if (numberOfOuts>3){
        debug("Out of way");
        // SHOULD BE CALIBRATED
        outCycle ++;
        if (outCycle>oneSideCycle){
          goesRight = !goesRight;
          oneSideCycle *= 2;
        } 
        if (goesRight){
          goRight();
        }
        else {
          goLeft();
        }
      }    
    }
    else {
      if (numberOfOuts > 3){
        delay(500);
      }
      goForward();
      numberOfOuts = 0;
      outCycle = 0;
      distance = newDistance;
    }
  }
}

// Finding target using sonar
void findTarget(){
  debug("Scanning for Target");

  setRED_LED();
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

    if(targetDistance < maxDistance) {
      searchStatus = 1;
      debug("Target found");
      setGREEN_LED();
      state = STATE_FOUND;
    }
    else if (targetDistance < minDistance) {
      //do something
      debug("Target is too close");
    }
    searchLoop--;
    delay(10);
  }

  if (searchStatus == 0) {
    debug("Target not found. Go to pause regime");
    setBLUE_LED();
    delay(2000); // SHOULD BE DELETED
  }
}

// Turning 180 degrees back home
void turnHome(){
  for (int i=0; i<4; i++){
    goForward();
    delay(100);
    scanningTurnLeft();
    delay(970);
  }
  halt();
  state = STATE_TURNED;
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

  if ((isRightBlack && isMiddleBlack && rightWasBlack && leftWasBlack)
    ||(isLeftBlack && isMiddleBlack && rightWasBlack && leftWasBlack))
  {
    debug("Stop");
    servoLeft.write(90);
    servoRight.write(90);
    state = STATE_HOME;
    //rightWasBlack = false; 
    //leftWasBlack = false;
  } 
  else if (isRightBlack && !isLeftBlack)
  {
    debug("Right");  
    rightWasBlack = true;
    servoLeft.write(180);
    servoRight.write(90);
  }
  else if (!isRightBlack && isLeftBlack)
  {
    debug("Left");
    leftWasBlack = true;
    servoLeft.write(90);
    servoRight.write(0);
  }
  else 
  {
    debug("Forward");
    servoLeft.write(180);
    servoRight.write(0);
  }

  delay(50);
}

void loop()
{
  if (state == STATE_SEARCHING) {
    findTarget();
  }
  else if (state == STATE_FOUND){
    start_time = millis();
    //goToTarget();
    goToTarget2();
  }
  else if (state == STATE_DOCKED){
    turnHome();
  }
  else if (state == STATE_TURNED){
    followBlackLine();
  }
}
