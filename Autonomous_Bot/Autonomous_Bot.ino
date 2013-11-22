#include <Servo.h> 
#include <SPI.h> //WIFI
#include <WiFi.h> //WIFI

Servo servoLeft, servoRight;
char ssid[] = "CMU"; //  your network SSID (name) 

int wifiStatus = WL_IDLE_STATUS;
char server[] = "www.google.com";
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
const int BOUNDARY_QTI_THRESHOLD = 6000;
const int MIDDLE_QTI_THRESHOLD = 15000;

int state;
unsigned long start_time, end_time;
unsigned long time_delta = 0;

String debugInfo = "";

boolean useSerial = false;

void debug(char *message){
  String messag_str(server);
  
  if (useSerial) {
    Serial.println(message);
  } 
  else {
    debugInfo += "<br>";
    debugInfo += message;
  }
}  

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
  while (wifiStatus != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:    
    wifiStatus = WiFi.begin(ssid);
  }
  Serial.println("Connected to wifi");
  printWifiStatus();

  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  if (client.connect(server, 80)) {
    Serial.println("connected to server");
    // Make a HTTP request:
    client.println("GET /search?q=arduino HTTP/1.1");
    client.println("Host: "+server_str);
    client.println("Connection: close");
    client.println();
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

// Interrupt on press/release the switch
void switchInterrupt(){
  Serial.println("Interrupt!");
  if (state==STATE_FOUND){
    end_time = millis();
    time_delta = end_time - start_time;
    Serial.print("Time passed: ");
    Serial.println(time_delta);
    Serial.println();
    state = STATE_DOCKED;
    halt();
  }
  else if (state==STATE_HOME){
    scanningTurnLeft();
    delay(4000); // SHOULD BE CALIBRATED
    halt();
    state = STATE_SEARCHING;
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
  Serial.print("Ping:");
  Serial.println();

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

void scanningTurnRight(){
  servoLeft.write(93);
  servoRight.write(93);
}

void scanningTurnLeft(){
  servoLeft.write(87);
  servoRight.write(87);
}

// Going forward
void goToTarget() {
  Serial.print("Going to target");
  Serial.println();
  setPURPLE_LED();
  goForward();
  delay(5000); //SHOULD BE CALIBRATED
}


// Going forward v2
void goToTarget2() {
  Serial.println("Going to target");
  setPURPLE_LED();
  goForward();
  int initialDistance = pingTarget();
  int distance = initialDistance;
  Serial.println("Distance to target = " + distance);
  int numberOfOuts = 0;
  int oneSideCycle = 10; // SHOULD BE CALIBRATED
  int outCycle = 0;
  boolean goesRight = false;

  while (distance>10) {
    int newDistance = pingTarget();
    Serial.println("New distance to target = " + newDistance);
    if (newDistance > distance + 2){
      Serial.println("Out");
      numberOfOuts++;
      if (numberOfOuts>3){

        Serial.println("Out of way");
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
      goForward();
      numberOfOuts = 0;
      outCycle = 0;
      distance = newDistance;
    }
  }

}

// Finding target using sonar
void findTarget(){
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

// Following the black line
void followBlackLine()
{  
  int rightQtiRCTime = RCTime(RIGHT_QTI_PIN);
  int middleQtiRCTime = RCTime(MIDDLE_QTI_PIN);
  int leftQtiRCTime = RCTime(LEFT_QTI_PIN);

  boolean isRightBlack = rightQtiRCTime > BOUNDARY_QTI_THRESHOLD;
  boolean isMiddleBlack = middleQtiRCTime > MIDDLE_QTI_THRESHOLD;
  boolean isLeftBlack = leftQtiRCTime > BOUNDARY_QTI_THRESHOLD;

  Serial.println("IR");
  String message = String(rightQtiRCTime) + "; " + String(middleQtiRCTime) + "; " + String(leftQtiRCTime);
  String message2 = String(isRightBlack) + "; " + String(isMiddleBlack) + "; " + String(isLeftBlack);

  Serial.println(message);
  Serial.println(message2);

  if (isRightBlack && isMiddleBlack && isLeftBlack)
  {
    Serial.println("Stop");
    servoLeft.write(90);
    servoRight.write(90);
    state = STATE_HOME;
  } 
  else if (isRightBlack && !isLeftBlack)
  {
    Serial.println("Right");  
    servoLeft.write(93);
    servoRight.write(90);
  }
  else if (!isRightBlack && isLeftBlack)
  {
    Serial.println("Left");
    servoLeft.write(90);
    servoRight.write(88);
  }
  else 
  {
    Serial.println("Forward");
    servoLeft.write(94);
    servoRight.write(86);
  }

  delay(50);
}

void loop()
{
  // Wifi client copied from http://arduino.cc/en/Tutorial/WiFiWebClient
  // if there are incoming bytes available
  // from the server, read them and print them:
  while (client.available()) {
    char c = client.read();
    Serial.write(c);
  }

  // if the server's disconnected, stop the client:
  if (!client.connected()) {
    Serial.println();
    Serial.println("disconnecting from server.");
    client.stop();

    // do nothing forevermore:
    //while(true);
  }


  if (state == STATE_SEARCHING) {
    findTarget();
  }
  else if (state == STATE_FOUND){
    start_time = millis();
    goToTarget();
    //goToTarget2();
  }
  else if (state == STATE_DOCKED){
    turnHome();
  }
  else if (state == STATE_TURNED){
    followBlackLine();
  }
}


// Copied from http://arduino.cc/en/Tutorial/WiFiWebClient
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

