#include <Arduino.h>
#include "motorDriver.h"
#include <sensorDriver.h>

void motorDriver( void * parameter);
void sensorDriver( void * parameter);
sclass s;
int motorSpeedL = 0;
int motorSpeedR = 0;

static uint8_t motorChannel1 = 5;   /* PWM Pin for Motor 0 */
static uint8_t motorChannel2 = 18;  /* PWM Pin for Motor 0 */

void setup(){
  Serial.begin(9600);
  delay(1000);

  s.SETUP();


  xTaskCreate(
    motorDriver,          /* Task function. */
    "MotorDriver",        /* String with name of task. */
    1024,              /* Stack size in bytes. */
    NULL,             /* Parameter passed as input of the task */
    2,                /* Priority of the task. */
    NULL
  );            /* Task handle. */

  xTaskCreate(
    sensorDriver,          /* Task function. */
    "sensorDriver",        /* String with name of task. */
    4096,              /* Stack size in bytes. */
    NULL,             /* Parameter passed as input of the task */
    1,                /* Priority of the task. */
    NULL
  );            /* Task handle. */
}

void loop(){
  delay(1000);
}

void motorDriver( void * parameter )
{

  mclass m;

  m.SETUP();

  //example of a task that executes for some time and then is deleted
  while(1)
  {
    Serial.println("Motor Control");
    
    m.set_speed(MotorR,Forward,motorSpeedR);

    m.set_speed(MotorL,Forward,motorSpeedL);
    
    vTaskDelay(100 / portTICK_PERIOD_MS);

  }

  vTaskDelete( NULL );
}

void sensorDriver( void * parameter )
{
  int16_t* arr;

  //example of a task that executes for some time and then is deleted
  while(1)
  {
    arr = s.reading();
    Serial.println("Hello from task " + String(arr[0]));
    bool leftObjectDetected = arr[0] < 200;
    bool centerObjectDetected = arr[1] < 200;
    bool rightObjectDetected = arr[2] < 200;
    if (leftObjectDetected || centerObjectDetected || rightObjectDetected){
      motorSpeedL = 0;
      motorSpeedR = 0;
      continue;
    }

    motorSpeedL = 255;
    motorSpeedR = 255;
 
    vTaskDelay(10/ portTICK_PERIOD_MS);

  }

  vTaskDelete( NULL );
}
