 //=================================================================================
//define Blynk variable
#define BLYNK_TEMPLATE_ID "TMPLHqJJ8pIv"
#define BLYNK_DEVICE_NAME "Quickstart Template"
#define BLYNK_AUTH_TOKEN "9ioO0Y3c-CvkRrj3zrRF36DqNzNCEXWQ"
#define BLYNK_PRINT Serial
//=================================================================================
//include wifi lib
#include "WiFi.h"
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
//=================================================================================
//include servo lib
#include <Servo.h>
Servo myservo1;
Servo myservo2;
//=================================================================================
//define pin
#define relayPin01 33  //26
#define relayPin02 32  //25
#define relayPin03 26  //33
#define relayPin04 25  //32
#define SW1 18
#define SW2 19
//=================================================================================
//define state and value to control
bool state_Sw1;
bool state_Sw2;
bool wifiFlag = 0;
int value1;
int value2;
//=================================================================================
//include humid and temp sensor
#include "DFRobot_SHT20.h"
DFRobot_SHT20 sht20(&Wire, SHT20_I2C_ADDR);
//=================================================================================
//rain_sensor
int rain_sensor = 34;
//=================================================================================
char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "iotkid";
char pass[] = "50470108";
//=================================================================================
BlynkTimer timer;
//=================================================================================

BLYNK_WRITE(V0) {
  //=================================================================================
  //Value from blynk to control fan
  value1 = param.asInt();
  digitalWrite(relayPin01, value1);
  //=================================================================================
}

BLYNK_WRITE(V3) {
  //=================================================================================
  //Value from blynk to control light
  value2 = param.asInt();
  digitalWrite(relayPin02, value2);
  //=================================================================================
}

BLYNK_WRITE(V10) {
  //=================================================================================
  //Value from blynk to control Relay4
  int value = param.asInt();
  Serial.println("V10:[" + String(value) + "]");
  if (value == 1) {                  // if(value)
    digitalWrite(relayPin03, HIGH);  // Turn ON, 1
  } else {
    digitalWrite(relayPin03, LOW);  // Turn OFF, 0
  }
  Blynk.virtualWrite(V13, value);
  //=================================================================================
}

BLYNK_WRITE(V12) {
  //=================================================================================
  //value from blynk to control food
  int value = param.asInt();
  Serial.println("V12:[" + String(value) + "]");
  if (value == 1) {                  // if(value)
    digitalWrite(relayPin04, HIGH);  // Turn ON, 1
  } else {
    digitalWrite(relayPin04, LOW);  // Turn OFF, 0
  }
  //=================================================================================
  Blynk.virtualWrite(V14, value);
}

BLYNK_WRITE(V6) {
  //=================================================================================
  //Value from blynk to control servo1
  int value = param.asInt();
  Blynk.virtualWrite(V7, value);
  if (value == 1) myservo1.write(90);
  if (value == 0) myservo1.write(0);
  //=================================================================================
}

BLYNK_WRITE(V8) {
  //=================================================================================
  //Value from blynk to control servo2
  int value = param.asInt();
  Blynk.virtualWrite(V9, value);
  if (value == 0) myservo2.write(95);
  if (value == 1) myservo2.write(0);
  //=================================================================================
}

void myTimerEvent() {
  //=================================================================================
  //Send rain sensor value to blynk server
  int val = analogRead(rain_sensor);
  Blynk.virtualWrite(V11, val);
  //=================================================================================
  //Send humid and temp value to blynk server
  float h = sht20.readHumidity();
  float t = sht20.readTemperature();
  if (isnan(t) || isnan(h)) {  // check if 'is not a number'
    Serial.println("Failed to read temperature");
  } else {
    String sht = "Temp : " + String(t) + " °C, Humd : " + String(h) + " %";
    //Serial.println(sht);
    Blynk.virtualWrite(V4, t);
    Blynk.virtualWrite(V5, h);
  }
  //=================================================================================
}

void with_internet() {
  //=================================================================================.
  //When connect internet  
  if (digitalRead(SW1) == 1) {
    while (digitalRead(SW1) == 1) delay(10);
    value1 = !value1;
    digitalWrite(relayPin01, value1);
    Blynk.virtualWrite(V1, value1);
    Blynk.virtualWrite(V0, value1);
    Serial.println("SW1");
  }
  if (digitalRead(SW2) == 1) {
    while (digitalRead(SW2) == 1) delay(10);
    value2 = !value2;
    digitalWrite(relayPin02, value2);
    Blynk.virtualWrite(V3, value2);
    Serial.println("SW2");
  }
  //=================================================================================
}

void without_internet() {
  //=================================================================================
  //When not connect internet
  if (digitalRead(SW1) == 1) {
    while (digitalRead(SW1) == 1) delay(10);
    value1 = !value1;
    digitalWrite(relayPin01, value1);
    Serial.println("SW1");
  }
  if (digitalRead(SW2) == 1) {
    while (digitalRead(SW2) == 1) delay(10);
    value2 = !value2;
    digitalWrite(relayPin02, value2);
    Serial.println("SW2");
  }
  //=================================================================================
}

BLYNK_CONNECTED() {
  // Request the latest state from the server
  Blynk.syncVirtual(V0);
  Blynk.syncVirtual(V1);
  Blynk.syncVirtual(V3);
}

void setup() {
  //=================================================================================
  // Debug console
  Serial.begin(115200);
  //=================================================================================
  //Setup pin
  pinMode(relayPin01, OUTPUT);
  pinMode(relayPin02, OUTPUT);
  pinMode(relayPin03, OUTPUT);
  pinMode(relayPin04, OUTPUT);
  pinMode(SW1, INPUT_PULLDOWN);
  pinMode(SW2, INPUT_PULLDOWN);
  //=================================================================================
  //Setup SHT20 Sensor
  sht20.initSHT20();
  delay(100);
  Serial.println("Sensor init finish!");
  sht20.checkSHT20();
  //=================================================================================
  //Setup Wifi and Blynk
  WiFi.begin(ssid, pass);
  timer.setInterval(1000L, myTimerEvent);
  timer.setInterval(3000L, checkBlynkStatus);
  Blynk.config(auth);
  //=================================================================================
  myservo1.attach(16);
  myservo2.attach(17);
  //=================================================================================
}

void loop() {
  //=================================================================================
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi Not Connected");
  } else {
    Serial.println("WiFi Connected");
    Blynk.run();
  }
  if (wifiFlag == 0)
    with_internet();
  else
    without_internet();
  //=================================================================================
  timer.run();
}

void checkBlynkStatus() {  // called every 3 seconds by SimpleTimer
  bool isconnected = Blynk.connected();
  if (isconnected == false) {
    wifiFlag = 1;
  }
  if (isconnected == true) {
    if (wifiFlag == 1) {
      wifiFlag = 0;
    }
  }
}