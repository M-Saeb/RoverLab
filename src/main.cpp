#include <Arduino.h>
#include "motorDriver.h"
#include <sensorDriver.h>

void taskOne( void * parameter);
void taskTwo( void * parameter);
void motorDriver( void * parameter);
void sensorDriver( void * parameter);
sclass s;
int motorSpeedL = 0;
int motorSpeedR = 0;
#define LED_BOARD 2 //change here the pin of the board to V2

static uint8_t motorChannel1 = 5;   /* PWM Pin for Motor 0 */
static uint8_t motorChannel2 = 18;  /* PWM Pin for Motor 0 */

void setup(){
  pinMode(LED_BOARD, OUTPUT);
  Serial.begin(9600);
  delay(1000);



  s.SETUP();


  xTaskCreate(
                    motorDriver,          /* Task function. */
                    "MotorDriver",        /* String with name of task. */
                    1024,              /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    2,                /* Priority of the task. */
                    NULL);            /* Task handle. */

  xTaskCreate(
                  sensorDriver,          /* Task function. */
                  "sensorDriver",        /* String with name of task. */
                  4096,              /* Stack size in bytes. */
                  NULL,             /* Parameter passed as input of the task */
                  1,                /* Priority of the task. */
                  NULL);            /* Task handle. */

                    

  // xTaskCreate(
  //                   sensorDriver,          /* Task function. */
  //                   "sensorDriver",        /* String with name of task. */
  //                   1024,              /* Stack size in bytes. */
  //                   NULL,             /* Parameter passed as input of the task */
  //                   1,                /* Priority of the task. */
  //                   NULL);            /* Task handle. */
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
    if (centerObjectDetected){
      motorSpeedL = 0;
      motorSpeedR = 0;
      continue;
    }
    if(leftObjectDetected)
    {
      motorSpeedL = 255;
      motorSpeedR = 0;
      continue;
    }
    if (rightObjectDetected){
      motorSpeedL = 0;
      motorSpeedR = 255;
      continue;
    }

    motorSpeedL = 255;
    motorSpeedR = 255;
 
    vTaskDelay(10/ portTICK_PERIOD_MS);


  }


    vTaskDelete( NULL );
}

void taskOne( void * parameter )
{
    //example of a task that executes for some time and then is deleted
    for( int i = 0; i < 10; i++ )

    {
      // Serial.println("Hello from task 1");
      
      //Switch on the LED
      digitalWrite(LED_BOARD, HIGH); 
      // Pause the task for 1000ms
      //delay(1000); //This delay doesn't give a chance to the other tasks to execute
      vTaskDelay(1000 / portTICK_PERIOD_MS); //this pauses the task, so others can execute
      // Switch off the LED
      digitalWrite(LED_BOARD, LOW); 
      // Pause the task again for 500ms
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    Serial.println("Ending task: 1");
    vTaskDelete( NULL );
}
 
void taskTwo( void * parameter)
{
    //create an endless loop so the task executes forever
    for( ;; )
    {
        // Serial.println("Hello from task: 2");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    Serial.println("Ending task 2"); //should not reach this point but in case...
    vTaskDelete( NULL );
}
