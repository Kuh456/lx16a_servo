#include <Arduino.h>
#include "include.h"
#include "SerialServo.h"

int speed = 700;
int wait = 3000;
int ServoID = 1;
int pos = 0;
const int de_pin = 5;
void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 17, 18);
  pinMode(de_pin, OUTPUT);
  digitalWrite(de_pin, LOW);
  delay(1000);
}

void loop()
{

  LobotSerialServoSetMode(Serial1, ServoID, 1, speed);
  delay(wait);

  LobotSerialServoSetMode(Serial1, ServoID, 1, 0); /// stop rotation
  delay(1000);
  pos = LobotSerialServoReadPosition(Serial1, ServoID, de_pin);
  Serial.printf("%d\n", pos);
}
