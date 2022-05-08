#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <Wire.h>
#include <Servo.h>
#include <queue.h>

#define debug 0

#define slaveAdressI2c 8
#define M1A 6
#define M1B 9
#define M2A 10
#define M2B 11
#define servo1Pin 3
#define servo2Pin 5
#define leftSensor A0
#define rightSensor A1
#define echoPin 12
#define trigPin 13

#define maxSpeed 255
#define servo1Min 15
#define servo1Max 90
#define servo2Min 0
#define servo2Max 180
#define threshold 800

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



const TickType_t xDelay = 45 / portTICK_PERIOD_MS;



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

  lineQueue =  xQueueCreate(1,2*sizeof(int)); 
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
              256, // Stack size
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
              128, // Stack size
              NULL,
              1, // Priority
              &TaskRangeSensorHandler ); // TaskHandle

  xTaskCreate(TaskLineSensors, // Task function
              "TaskLineSensors", // Task name
              128, // Stack size
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

   Data dataRec = {false,0,0,0,0};


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
          xQueueSend(dataQueue,&dataRec, portMAX_DELAY);
        }
        lastState = dataRec.mode;
      } 
    }
   
 }

// ----------------------------------------- TASK Autonomous Control ----------------------------
void TaskAutonomousControl(void *pvParameters)
{
  (void) pvParameters;
  bool isRetreating = 0;
  bool retreatDir =0;
  int lineData[] = {0,0};
  int rangeData = 0;
  for(;;){
    xTaskNotifyGive(TaskLineSensorsHandler);
    xTaskNotifyGive(TaskRangeSensorHandler);
    xQueueReceive(lineQueue,&lineData, portMAX_DELAY);
    xQueueReceive(rangeQueue,&rangeData, portMAX_DELAY);
    if(rangeData < 20 && !isRetreating){
      setMotors(-40,-80);
      isRetreating = true;
    } else if(rangeData>30 && isRetreating){
      isRetreating = false;
      stopMotors();
    } else if(lineData[0]< threshold && lineData[1]< threshold && !isRetreating){
      setMotors(70,70);
    } else if(lineData[0] > threshold && lineData[1]> threshold && !isRetreating){
      setMotors(-50,-70);
    } else if (lineData[0] > threshold && !isRetreating){
      setMotors(-30,30);
    } else if (lineData[1] > threshold && !isRetreating){
      setMotors(30,-30);
    }

    if(debug){
      Serial.print( lineData[0]);
      Serial.print("   ");
      Serial.print( lineData[1]);
      Serial.println();
      Serial.println();
    }
    
    vTaskDelay( xDelay );
  }
  
}

// ----------------------------------------- TASK Remote Control ----------------------------
void TaskRemoteControl(void *pvParameters)
{
  (void) pvParameters;

  Data dataRecRem = {false,0,0,0,0};

  for(;;){
    xQueueReceive(dataQueue,&dataRecRem, portMAX_DELAY);
    setMotors(dataRecRem.LeftTrack,dataRecRem.RightTrack);
    servo1.write(map(dataRecRem.servo1pos,0,20,servo1Min,servo1Max));
    servo2.write(map(dataRecRem.servo2pos,0,20,servo2Max,servo2Min));
  } 
}

// ----------------------------------------- TASK Range Sensor ----------------------------
void TaskRangeSensor(void *pvParameters)
{
  (void) pvParameters;

  long duration;
  int distance;
  for(;;){
    ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = duration * 0.034 / 2; 
    xQueueSend(rangeQueue,&distance, portMAX_DELAY);
  }
}

// ----------------------------------------- TASK Line Sensors ----------------------------
void TaskLineSensors(void *pvParameters)
{
  (void) pvParameters;
  int sensorState[]={0,0};
  for(;;){
    ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
    sensorState[0]=analogRead(leftSensor);
    sensorState[1]=analogRead(rightSensor);
    xQueueSend(lineQueue,&sensorState, portMAX_DELAY);
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

  pinMode(echoPin,INPUT);
  pinMode(trigPin,OUTPUT);

  stopMotors();

  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);

  servo1.write(map(10,0,20,servo1Min,servo1Max));
  servo2.write(map(10,0,20,servo2Min,servo2Max));
}
