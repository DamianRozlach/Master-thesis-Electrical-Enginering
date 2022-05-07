#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <Wire.h>
#include <Servo.h>
#include <queue.h>

#define debug 1

#define slaveAdressI2c 8
#define M1A 6
#define M1B 9
#define M2A 10
#define M2B 11
#define servo1Pin 3
#define servo2Pin 5
#define leftSensor A0
#define rightSensor A1

#define maxSpeed 255
#define servo1Min 15
#define servo1Max 90
#define servo2Min 0
#define servo2Max 180

struct Data {
  bool mode;
  int LeftTrack;
  int RightTrack;
  int servo1pos;
  int servo2pos;
};

TaskHandle_t taskCommunicationHandler;
TaskHandle_t TaskAutonomousControlHandler;
TaskHandle_t TaskRemoteControlHandler;
TaskHandle_t TaskRangeSensorHandler;
TaskHandle_t TaskLineSensorsHandler;

QueueHandle_t lineQueue;
QueueHandle_t rangeQueue;
QueueHandle_t dataQueue;

SemaphoreHandle_t xDataSemaphore;
SemaphoreHandle_t autonomousSemaphore;

Servo servo1;
Servo servo2;

const int motorPins[] = {M1A,M1B,M2A,M2B};
int sizeArr = sizeof(motorPins) / sizeof(int);

Data dataRec = {false,0,0,0,0};

const TickType_t xDelay = 2500 / portTICK_PERIOD_MS;



/*
bool mode = false;
int LeftTrack = 0;
int RightTrack = 0;
int servo1pos = 0;
int servo2pos = 0;
*/
// Tasks

void TaskCommunication(void *pvParameters);
void TaskAutonomousControl(void *pvParameters);
void TaskRemoteControl(void *pvParameters);
void TaskRangeSensor(void *pvParameters);
void TaskLineSensors(void *pvParameters);

void setup() 
{
  initializeHardware();

  lineQueue =  xQueueCreate(1,sizeof(int)); 
  rangeQueue =  xQueueCreate(1,sizeof(int));
  dataQueue =  xQueueCreate(3,sizeof(struct Data));
  
  xTaskCreate(TaskCommunication, // Task function
              "I2C_communication", // Task name
              128, // Stack size
              NULL,
              3, // Priority
              &taskCommunicationHandler ); // TaskHandle

  xTaskCreate(TaskAutonomousControl, // Task function
              "Autonomous Control", // Task name
              128, // Stack size
              NULL,
              2, // Priority
              &TaskAutonomousControlHandler ); // TaskHandle

  xTaskCreate(TaskRemoteControl, // Task function
              "TaskRemoteControl", // Task name
              128, // Stack size
              NULL,
              0, // Priority
              &TaskRemoteControlHandler ); // TaskHandle

  xTaskCreate(TaskRangeSensor, // Task function
              "TaskRangeSensor", // Task name
              64, // Stack size
              NULL,
              1, // Priority
              &TaskRangeSensorHandler ); // TaskHandle

  xTaskCreate(TaskLineSensors, // Task function
              "TaskLineSensors", // Task name
              64, // Stack size
              NULL,
              1, // Priority
              &TaskLineSensorsHandler ); // TaskHandle

   vTaskSuspend(TaskAutonomousControlHandler);
  
}

void loop() {
  // not used
}

// ----------------------------------------- TASK Communication ----------------------------

void TaskCommunication(void *pvParameters)
{
   (void) pvParameters;
   bool lastState = false;


   //Wire.setClock(100000);
   Wire.begin(slaveAdressI2c);                
   Wire.onReceive(receiveEventHandler);
   if(debug)
   {
      Serial.begin(9600);
      while (!Serial) 
      {
      ; // wait for Serial connection
      }
      Serial.print("i am alive");
    }
    for(;;)
    {
       if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) 
      {
        while(Wire.available())
        {  
           int x = Wire.read();
           switch (Wire.available()) 
           {
             case 4:
               dataRec.mode = bool(x);
             break;
             case 3:
               dataRec.LeftTrack = x - 100;
             break;
             case 2:
               dataRec.RightTrack = x - 100;
             break;
             case 1:
               dataRec.servo1pos = x;
             break;
             case 0:
               dataRec.servo2pos = x;
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
        if(debug){
          Serial.println();
        }
        if(dataRec.mode != lastState){
          if(dataRec.mode){
            vTaskResume( TaskAutonomousControlHandler );
          } else {
            vTaskSuspend(TaskAutonomousControlHandler);
          }
        }
        if(!dataRec.mode){
          xTaskNotifyGive(TaskRemoteControlHandler);
        }
        lastState = dataRec.mode;
      } 
    }
   
 }

// ----------------------------------------- TASK Autonomous Control ----------------------------
void TaskAutonomousControl(void *pvParameters)
{
  (void) pvParameters;
  for(;;){
    //xTaskNotifyGive(TaskLineSensorsHandler);
    //xTaskNotifyGive(TaskRangeSensorHandler);
    vTaskDelay( xDelay );
  }
  
}

// ----------------------------------------- TASK Remote Control ----------------------------
void TaskRemoteControl(void *pvParameters)
{
  (void) pvParameters;

  for(;;){
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    setMotors(dataRec.LeftTrack,dataRec.RightTrack);
    servo1.write(map(dataRec.servo1pos,0,20,servo1Min,servo1Max));
    servo2.write(map(dataRec.servo2pos,0,20,servo2Max,servo2Min));
  } 
}

// ----------------------------------------- TASK Range Sensor ----------------------------
void TaskRangeSensor(void *pvParameters)
{
  (void) pvParameters;
  for(;;){
    vTaskDelay( xDelay );
  }
}

// ----------------------------------------- TASK Line Sensors ----------------------------
void TaskLineSensors(void *pvParameters)
{
  (void) pvParameters;
  int sensorState[]={0,0};
  for(;;){
    //ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
    vTaskDelay( xDelay );
  }
}

// ----------------------------------------- event notification ----------------------------


void receiveEventHandler(int howMany)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(taskCommunicationHandler, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) 
  {
    taskYIELD();
  }
}

// ----------------------------------------- functions ----------------------------

void setMotors(int leftMotor, int rightMotor){
  /*
  if(leftMotor == 0){
    analogWrite(motorPins[2], 0);
    analogWrite(motorPins[3], 0);
  }
  if(rightMotor == 0){
    analogWrite(motorPins[0], 0);
    analogWrite(motorPins[1], 0);
  }
  */
  if(leftMotor >= 0){
    analogWrite(motorPins[2], map(leftMotor,0,100,0,maxSpeed));
    analogWrite(motorPins[3], 0);
  }
  if(leftMotor < 0){
    analogWrite(motorPins[2], 0 );
    analogWrite(motorPins[3], map(leftMotor,0,-100,0,maxSpeed));
  }
  if(rightMotor >= 0){
    analogWrite(motorPins[0], map(rightMotor,0,100,0,maxSpeed));
    analogWrite(motorPins[1], 0);
  }
  if(rightMotor < 0){
    analogWrite(motorPins[0], 0 );
    analogWrite(motorPins[1], map(rightMotor,0,-100,0,maxSpeed));
  }
}

void stopMotors(){
  for(int z = 0; z < sizeArr;z++){
    analogWrite(motorPins[z], 0);
  }
}

void initializeHardware(){
  for(int z = 0; z < sizeArr ;z++){
    pinMode(motorPins[z]  , OUTPUT);
  }

  stopMotors();

  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);

  servo1.write(map(10,0,20,servo1Min,servo1Max));
  servo2.write(map(10,0,20,servo2Min,servo2Max));
}
