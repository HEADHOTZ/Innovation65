#include <Servo.h>
#define SW 4
bool state;
Servo servo1;

void setup() {
  pinMode(SW, INPUT);
  Serial.begin(115200);
  servo1.attach(16);
}

void loop() {
  servo1.write(180);
  delay(100);
  servo1.write(140);
  delay(100);
  /*delay(100);
  servo1.write(0);
  delay(100);/*
  Serial.println(SW);
  if (!digitalRead(SW)) {
    state = !state;
  }
  if (state == 1) {
    servo1.write(30);
    delay(100);
    servo1.write(0);
    delay(100);
  }
  if (state == 0) servo1.write(0);*/
}
