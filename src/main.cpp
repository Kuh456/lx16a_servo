#include <Arduino.h>
#include <CANCREATE.h>
#include "include.h"
#include "SerialServo.h"
typedef struct _CAN_send
{
  uint32_t id;
  uint8_t data[8];
  int size;
} CAN_send_msg_t;
int pos = 0;
constexpr int Servo_rx = 17;
constexpr int Servo_tx = 18;
static TaskHandle_t x_CAN_receive_task_handle = nullptr;
static QueueHandle_t CAN_send_task_handle;
static QueueHandle_t CAN_send_queue;
CAN_CREATE CAN(true);
void CAN_receive_task(void *pvParameters);
void CAN_send_task(void *pvParameters);
void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, Servo_rx, Servo_tx);
  pinMode(DE_PIN, OUTPUT);
  digitalWrite(DE_PIN, LOW);
  delay(100);
}

void loop()
{
  SerialServoMove(Serial1, ServoID, 0, 1000);
  delay(wait);
  pos = SerialServoReadPosition(Serial1, ServoID);
  Serial.printf("Before: %d\n", pos);
  SerialServoMove(Serial1, ServoID, 1000, 1000);
  delay(wait);
  pos = SerialServoReadPosition(Serial1, ServoID);
  Serial.printf("After: %d\n", pos);
  delay(10);
}
void canReceiveTask(void *pvParameters)
{
  // CAN受信タスク
  while (true)
  {
    can_return_t receive_packet;
    CAN.readWithDetail(&receive_packet, portMAX_DELAY);
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}
void CAN_send_task(void *pvParameters)
{
  CAN_send_msg_t msg;
  while (true)
  {
    if (xQueueReceive(CAN_send_queue, &msg, portMAX_DELAY) == pdTRUE)
    {
      CAN.sendData(msg.id, msg.data, msg.size);
    }
  }
}