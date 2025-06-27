#include <Arduino.h>
#include "include.h"
#include "SerialServo.h"

int wait = 3000;
int ServoID = 1;
int pos = 0;
void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 17, 18);
  pinMode(DE_PIN, OUTPUT);
  digitalWrite(DE_PIN, LOW);
  delay(1000);
}

void loop()
{
  LobotSerialServoMove(Serial1, ServoID, 0, 1000);
  delay(wait);
  pos = LobotSerialServoReadPosition(Serial1, ServoID);
  Serial.printf("Before: %d\n", pos);
  LobotSerialServoMove(Serial1, ServoID, 1000, 1000);
  delay(wait);
  pos = LobotSerialServoReadPosition(Serial1, ServoID);
  Serial.printf("After: %d\n", pos);
}
