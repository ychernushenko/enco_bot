#include <WiFi.h>
#include <Servo.h> 
 
Servo servoLeft, servoRight;

int pos = 0;    // variable to store the servo position 
int port = 80;

char ssid[] = "CMU";     // the name of your network
//char ssid[] = "HOME-7AC2";
//char pass[] = "C24C19E132316E52";
//char ssid[] = "Zak";
//char pass[] = "aaaaaaaa";

int status = WL_IDLE_STATUS;     // the Wifi radio's status
WiFiServer server(port);

void setup() {
  Serial.begin(9600);
  while ( status != WL_CONNECTED) { 
//   status = WiFi.begin(ssid, pass);
     status = WiFi.begin(ssid);
  }
  
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  server.begin();
  
  servoLeft.attach(6);
  servoRight.attach(5);
  halt();
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

void loop() 
{ 
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) 
  {                             // if you get a client,
    String currentLine = "";
    while(client.connected())
    {
      if (client.available())
      {
        char c = client.read();
        if (c == '\n')
        {
          if (currentLine == "w") goForward();
          if (currentLine == "s") goBackward();
          if (currentLine == "q") goLeft();
          if (currentLine == "e") goRight();
          if (currentLine == "a") turnLeft();
          if (currentLine == "d") turnRight();
          if (currentLine == "h") halt();
          currentLine = "";
          client.write('o');
        }
        else if (c!= '\r') 
        {
          currentLine +=c;
        }
      }
    }
    delay(1);
    client.stop();
  }
} 
