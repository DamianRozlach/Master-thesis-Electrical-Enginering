#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <Wire.h>

#define slaveAdressI2c 8
#define M1A 1
#define M1B 2
#define M2A 3
#define M2B 4
#define debug 1

TaskHandle_t taskCommunicationHandler;

SemaphoreHandle_t xDataSemaphore;
SemaphoreHandle_t interruptSemaphore;

bool mode = false;
int LeftTrack = 0;
int RightTrack = 0;
int servo1 = 0;
int servo2 = 0;

// Tasks

void TaskCommunication(void *pvParameters);
void TaskAutonomousControl(void *pvParameters);
void TaskRemoteControl(void *pvParameters);
void TaskRangeSensor(void *pvParameters);
void TaskLineSensors(void *pvParameters);

void setup() 
{
  xTaskCreate(TaskCommunication, // Task function
              "I2C_communication", // Task name
              128, // Stack size
              NULL,
              3, // Priority
              &taskCommunicationHandler ); // TaskHandle
}

void loop() {
  // not used
}

// ----------------------------------------- TASKS ----------------------------

void TaskCommunication(void *pvParameters)
{
   (void) pvParameters;

   Wire.setClock(100000);
   Wire.begin(slaveAdressI2c);                // join i2c bus
   Wire.onReceive(receiveEventHandler);
   if(debug)
   {
      Serial.begin(9600);
      while (!Serial) 
      {
      ; // just wait for Serial connection
      }
    }
    for(;;)
    {
       if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) 
      {
        Serial.println("Notification received");
        while(Wire.available())
        {  
           int x = Wire.read();
           switch (Wire.available()) 
           {
             case 4:
               mode = bool(x);
             break;
             case 3:
               LeftTrack = x;
             break;
             case 2:
               RightTrack = x;
             break;
             case 1:
               servo1 = x;
             break;
             case 0:
               servo2 = x;
             break;
             default:
             ;
             break;
           }
           if(debug){
             Serial.print(Wire.available());
             Serial.print(": ");
             Serial.print(x);
             Serial.print(" ");
           }
        }
        Serial.println();
      } 
    }
   
 }

void receiveEventHandler(int howMany)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(taskCommunicationHandler, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) 
  {
    taskYIELD();
  }
}
