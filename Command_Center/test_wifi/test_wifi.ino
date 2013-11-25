#include <Servo.h> 
#include <SPI.h> //WIFI
#include <WiFi.h> //WIFI

Servo servoLeft, servoRight;
char ssid[] = "CMU"; //  your network SSID (name) 

int wifiStatus = WL_IDLE_STATUS;
char server[] = "128.237.133.153";
int port = 8000;
String server_str(server);
WiFiClient client;

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
    if (client.connect(server, port)) {
      client.println(message);
    }
  }
}
