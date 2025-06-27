#include <Arduino.h>
#include "include.h"
#include "SerialServo.h"

int speed = 700;
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

  LobotSerialServoSetMode(Serial1, ServoID, 1, speed);
  delay(wait);

  LobotSerialServoSetMode(Serial1, ServoID, 1, 0); /// stop rotation
  delay(1000);
  pos = LobotSerialServoReadPosition(Serial1, ServoID);
  Serial.printf("%d\n", pos);
}
