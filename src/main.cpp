#include <Arduino.h>
#include "motorDriver.h"
#include "AWS.h"
#include <sensorDriver.h>
#include <math.h>
#include <esp_task_wdt.h>


myawsclass awsClient;

void motorDriver( void * parameter);
void sensorDriver( void * parameter);
sclass s;
int motorSpeedL = 0;
int motorSpeedR = 0;

static uint8_t motorChannel1 = 5;   /* PWM Pin for Motor 0 */
static uint8_t motorChannel2 = 18;  /* PWM Pin for Motor 0 */



double distance;

int16_t* arr;

int16_t sensorDat[3];

enum State {
  Navigation,
  ObstacleAvoidance,
  Correction
};

State currentState = Navigation;
unsigned long lastObstacleDetectedTime = 0; // Time when the last obstacle was detected
unsigned long lastNavigTrigger = 0; // Time when the last obstacle was detected
const unsigned long recoveryTime = 1000; // Time to spend in recovery state, for example



void setup(){
  Serial.begin(9600);
  awsClient.connectAWS();
  delay(1000);

  esp_task_wdt_init(1, false); 
  esp_task_wdt_add(NULL);
  
  s.SETUP();
  xTaskCreate(
    motorDriver,          /* Task function. */
    "MotorDriver",        /* String with name of task. */
    4096,              /* Stack size in bytes. */
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

double calculateDirectionError() {
    double desiredAngle = atan2(targetY - currentY, targetX - currentX) * 180 / M_PI;  
    double angleError = desiredAngle - currentAngle; 

    int vectory = targetY - currentY;
    int vectorx = targetX - currentX;

    distance = sqrt(vectorx*vectorx + vectory*vectory);

    angleError = fmod(angleError, 360); // Use fmod to handle floating point numbers
    if (angleError > 180) {
        angleError -= 360;
    } else if (angleError < -180) {
        angleError += 360;
    }

    return angleError;
}

void navigateToTarget() {
  double error = calculateDirectionError();  

  bool leftObjectDetected = sensorDat[0] < 150 ;
  bool centerObjectDetected = sensorDat[1] < 150;
  bool rightObjectDetected = sensorDat[2] < 150;

  switch (currentState) {
    case Navigation:

      if (leftObjectDetected || centerObjectDetected || rightObjectDetected) {
        currentState = ObstacleAvoidance;
        lastObstacleDetectedTime = millis();
        break;
      }
      else if(error < -15 || error >15)
      {
        if(lastNavigTrigger - millis() > 300)
        {
          currentState = Correction;
          break;
        }
        
        
      }

      motorSpeedL = 150;
      motorSpeedR = 150;
      


      
      break;

    case ObstacleAvoidance:
      if (centerObjectDetected)
      {
        if(sensorDat[1] < sensorDat[2])
        {
          motorSpeedL = 150;
          motorSpeedR = -150;
        }
        else
        {
          motorSpeedL = -150;
          motorSpeedR = 150;
        }
      }
      else if(leftObjectDetected)
      {
        motorSpeedL = 200;
        motorSpeedR = 0;
      }else if (rightObjectDetected)
      {
        motorSpeedL = 0;
        motorSpeedR = 200;
      }
      
 
      if(!leftObjectDetected && !centerObjectDetected && !rightObjectDetected)
      {
        if(lastObstacleDetectedTime - millis() > 300)
        {
          currentState = Navigation;
          lastNavigTrigger = millis();
        }
      }

      break;

    case Correction:
      // Recovery state logic
      if(error > 5 && distance > 40)
      {
        motorSpeedL = 80;
        motorSpeedR = -80;
      }
      else if(error < -5 && distance > 40)
      {
        motorSpeedL = -80;
        motorSpeedR = 80;
      }
      else
      {
        currentState = Navigation;
        lastNavigTrigger = millis();
      }
      break;
  }

    if(motorSpeedL < -255)
    {
      motorSpeedL = -255;
    }
    else if(motorSpeedL > 255)
    {
      motorSpeedL = 255;
    }

    if(motorSpeedR < -255)
    {
      motorSpeedR = -255;
    }
    else if(motorSpeedR > 255)
    {
      motorSpeedR = 255;
    }
}

void loop(){
  if(awsClient.stayConnected() == true)
  {
    esp_task_wdt_reset();
  }
  delay(10);
  // awsClient.publishMessage(3);
}

void motorDriver( void * parameter )
{
  mclass m;
  m.SETUP();
  //example of a task that executes for some time and then is deleted
  while(1)
  {
    
    navigateToTarget();


    Serial.print(motorSpeedL);
    Serial.print(" "); // Print a space as a separator
    Serial.println(motorSpeedR);

    if(motorSpeedL > 0)
    {
      m.set_speed(MotorL,Forward,motorSpeedL);
    }
    else
    {
      m.set_speed(MotorL,Backward,-motorSpeedL);
    }
    

    if(motorSpeedR > 0)
    {
      m.set_speed(MotorR,Forward,motorSpeedR);
    }
    else
    {
      m.set_speed(MotorR,Backward,-motorSpeedR);
    }

    

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

  vTaskDelete( NULL );
}

void sensorDriver( void * parameter )
{
  
  
  while(1)
  {
    arr = s.reading();

    sensorDat[0] = arr[0];
    sensorDat[1] = arr[1];
    sensorDat[2] = arr[2];
 
    vTaskDelay(10/ portTICK_PERIOD_MS);

  }

  vTaskDelete( NULL );
}
