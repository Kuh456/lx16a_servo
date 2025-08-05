#ifndef SerialServo
#define SerialServo
#include <Arduino.h>
#include "include.h"

// checksum
byte CheckSum(byte buf[])
{
  byte i;
  uint16_t temp = 0;
  for (i = 2; i < buf[3] + 2; i++)
  {
    temp += buf[i];
  }
  temp = ~temp;
  i = (byte)temp;
  return i;
}

// parse the received data packet information and return
int SerialServoReceiveHandle(HardwareSerial &SerialX, byte *ret)
{
  bool frameStarted = false;
  bool receiveFinished = false;
  byte frameCount = 0;
  byte dataCount = 0;
  byte dataLength = 2;
  byte rxBuf;
  byte recvBuf[32];
  byte i;

  while (SerialX.available())
  {
    rxBuf = SerialX.read();
    delayMicroseconds(100);
    if (!frameStarted)
    {
      if (rxBuf == LOBOT_SERVO_FRAME_HEADER)
      {
        frameCount++;
        if (frameCount == 2)
        {
          frameCount = 0;
          frameStarted = true;
          dataCount = 1;
        }
      }
      else
      {
        frameStarted = false;
        dataCount = 0;
        frameCount = 0;
      }
    }
    if (frameStarted)
    {
      recvBuf[dataCount] = (uint8_t)rxBuf;
      if (dataCount == 3)
      {
        dataLength = recvBuf[dataCount];
        if (dataLength < 3 || dataCount > 7)
        {
          dataLength = 2;
          frameStarted = false;
        }
      }
      dataCount++;
      if (dataCount == dataLength + 3)
      {

        if (CheckSum(recvBuf) == recvBuf[dataCount - 1])
        {

          frameStarted = false;
          memcpy(ret, recvBuf + 4, dataLength);
          return 1;
        }
        return -1;
      }
    }
  }
}
// write servo ID
void SerialServoSetID(HardwareSerial &SerialX, uint8_t oldID, uint8_t newID)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = oldID;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_ID_WRITE;
  buf[5] = newID;
  buf[6] = CheckSum(buf);
  SerialX.write(buf, 7);
}

// servo rotation control
void SerialServoMove(HardwareSerial &SerialX, uint8_t id, int16_t position, uint16_t time)
{
  byte buf[10];
  if (position < 0)
    position = 0;
  if (position > 1000)
    position = 1000;
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_MOVE_TIME_WRITE;
  buf[5] = GET_LOW_BYTE(position);
  buf[6] = GET_HIGH_BYTE(position);
  buf[7] = GET_LOW_BYTE(time);
  buf[8] = GET_HIGH_BYTE(time);
  buf[9] = CheckSum(buf);
  digitalWrite(DE_PIN, HIGH);
  SerialX.write(buf, 10);
}

// read ID
int SerialServoReadID(HardwareSerial &SerialX)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = ID_ALL; // ID_ALL is 254, which means a broadcast will be sent to all servos, which can be used to read information from servos with unknown IDs
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_ID_READ;
  buf[5] = CheckSum(buf);
  SerialX.write(buf, 6);

  while (SerialX.available())
    SerialX.read();

  while (!SerialX.available())
  {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (SerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(0x00, buf[1]);
  else
    ret = -2048;
  return ret;
}
// read servo position
int SerialServoReadPosition(HardwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_POS_READ;
  buf[5] = CheckSum(buf);
  digitalWrite(DE_PIN, HIGH);
  SerialX.write(buf, 6);
  while (SerialX.available())
    SerialX.read();
  SerialX.flush();
  digitalWrite(DE_PIN, LOW);
  while (!SerialX.available())
  {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (SerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -2048;
  return ret;
}

// read deviation
int SerialServoReadDev(HardwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_ANGLE_OFFSET_READ;
  buf[5] = CheckSum(buf);
  SerialX.write(buf, 6);

  while (SerialX.available())
    SerialX.read();
  while (!SerialX.available())
  {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (SerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -2048;
  return ret;
}

// read rotation range
int retL;
int retH;
int SerialServoReadAngleRange(HardwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_ANGLE_LIMIT_READ;
  buf[5] = CheckSum(buf);
  SerialX.write(buf, 6);

  while (SerialX.available())
    SerialX.read();

  while (!SerialX.available())
  {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (SerialServoReceiveHandle(SerialX, buf) > 0)
  {
    retL = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
    retH = (int16_t)BYTE_TO_HW(buf[4], buf[3]);
  }
  else
    ret = -2048;
  return ret;
}
// read voltage
int SerialServoReadVin(HardwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_VIN_READ;
  buf[5] = CheckSum(buf);
  SerialX.write(buf, 6);

  while (SerialX.available())
    SerialX.read();

  while (!SerialX.available())
  {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (SerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -2049;

  return ret;
}

// read voltage range
int vinL;
int vinH;
int SerialServoReadVinLimit(HardwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_VIN_LIMIT_READ;
  buf[5] = CheckSum(buf);
  SerialX.write(buf, 6);

  while (SerialX.available())
    SerialX.read();

  while (!SerialX.available())
  {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (SerialServoReceiveHandle(SerialX, buf) > 0)
  {
    vinL = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
    vinH = (int16_t)BYTE_TO_HW(buf[4], buf[3]);
  }
  else
    ret = -2048;
  return ret;
}

// read temperature alarm threashold
int SerialServoReadTempLimit(HardwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_TEMP_MAX_LIMIT_READ;
  buf[5] = CheckSum(buf);
  SerialX.write(buf, 6);

  while (SerialX.available())
    SerialX.read();

  while (!SerialX.available())
  {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (SerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(0x00, buf[1]);
  else
    ret = -2049;

  return ret;
}

// read temperature
int SerialServoReadTemp(HardwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_TEMP_READ;
  buf[5] = CheckSum(buf);
  SerialX.write(buf, 6);

  while (SerialX.available())
    SerialX.read();

  while (!SerialX.available())
  {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (SerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(0x00, buf[1]);
  else
    ret = -2049;

  return ret;
}

// read servo status
int SerialServoReadLoadOrUnload(HardwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_READ;
  buf[5] = CheckSum(buf);
  SerialX.write(buf, 6);

  while (SerialX.available())
    SerialX.read();

  while (!SerialX.available())
  {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (SerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(0x00, buf[1]);
  else
    ret = -2049;

  return ret;
}
// stop rotation
void SerialServoStopMove(HardwareSerial &SerialX, uint8_t id)
{
  byte buf[6];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_MOVE_STOP;
  buf[5] = CheckSum(buf);
  SerialX.write(buf, 6);
}
// set servo mode
void SerialServoSetMode(HardwareSerial &SerialX, uint8_t id, uint8_t Mode, int16_t Speed)
{
  byte buf[10];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_OR_MOTOR_MODE_WRITE;
  buf[5] = Mode;
  buf[6] = 0;
  buf[7] = GET_LOW_BYTE((uint16_t)Speed);
  buf[8] = GET_HIGH_BYTE((uint16_t)Speed);
  buf[9] = CheckSum(buf);
  digitalWrite(DE_PIN, HIGH);
  SerialX.write(buf, 10);
}

// servo power-on
void SerialServoLoad(HardwareSerial &SerialX, uint8_t id)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 1;
  buf[6] = CheckSum(buf);

  SerialX.write(buf, 7);
}
// servo poweroff
void SerialServoUnload(HardwareSerial &SerialX, uint8_t id)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 0;
  buf[6] = CheckSum(buf);

  SerialX.write(buf, 7);
}
// int ReadAngle(int position)
// {
//   return 
// }
#endif