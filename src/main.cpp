#include <Arduino.h>
#include "motorDriver.h"
#include "AWS.h"
#include <sensorDriver.h>
#include <math.h>
#include <esp_task_wdt.h>

myawsclass awsClient;

void motorDriver(void *parameter);
void sensorDriver(void *parameter);
sclass s;
int motorSpeedL = 0;
int motorSpeedR = 0;

static uint8_t motorChannel1 = 5;  /* PWM Pin for Motor 0 */
static uint8_t motorChannel2 = 18; /* PWM Pin for Motor 0 */

double distance;

int16_t *arr;

int16_t sensorDat[3];

enum State
{
    IDLE,
    Navigation,
    ObstacleAvoidance,
    Correction
};

State currentState = IDLE;
unsigned long lastObstacleDetectedTime = 0; // Time when the last obstacle was detected
unsigned long lastNavigTrigger = 0;         // Time when the last obstacle was detected
const unsigned long recoveryTime = 1000;    // Time to spend in recovery state, for example

void setup()
{
    Serial.begin(9600);
    awsClient.connectAWS();
    delay(1000);

    esp_task_wdt_init(1, false);
    esp_task_wdt_add(NULL);

    s.SETUP();
    xTaskCreate(
        motorDriver,   /* Task function. */
        "MotorDriver", /* String with name of task. */
        4096,          /* Stack size in bytes. */
        NULL,          /* Parameter passed as input of the task */
        2,             /* Priority of the task. */
        NULL);         /* Task handle. */

    xTaskCreate(
        sensorDriver,   /* Task function. */
        "sensorDriver", /* String with name of task. */
        4096,           /* Stack size in bytes. */
        NULL,           /* Parameter passed as input of the task */
        1,              /* Priority of the task. */
        NULL);          /* Task handle. */
}

double calculateDirectionError()
{
    double desiredAngle = atan2(targetY - currentY, targetX - currentX) * 180 / M_PI;
    double angleError = desiredAngle - currentAngle;

    int vectorY = targetY - currentY;
    int vectorX = targetX - currentX;

    distance = sqrt(vectorX * vectorX + vectorY * vectorY);

    angleError = fmod(angleError, 360); // Use fmod to handle floating point numbers
    if (angleError > 180)
    {
        angleError -= 360;
    }
    else if (angleError < -180)
    {
        angleError += 360;
    }

    return angleError;
}

int adj = 0;

void navigateToTarget()
{
    double error = calculateDirectionError();

    bool leftObjectDetected = sensorDat[0] < 150;
    bool centerObjectDetected = sensorDat[1] < 100;
    bool rightObjectDetected = sensorDat[2] < 150;

    switch (currentState)
    {
    case IDLE:
        if (targetMessageReceived == true && roverMessageReceived == true)
        {
            currentState = Correction;
        }
        break;
    case Navigation:
        if (leftObjectDetected || centerObjectDetected || rightObjectDetected)
        {
            if (millis() - lastNavigTrigger > 200)
            {
                currentState = ObstacleAvoidance;
            }
            lastObstacleDetectedTime = millis();
            break;
        }

        if (error < -20 || error > 20)
        {
            if (millis() - lastNavigTrigger > 2000)
            {
                currentState = Correction;
                break;
            }
        }

        adj = 0;

      if((sensorDat[0] < sensorDat[2] ) && (sensorDat[0] < 350 && sensorDat[2] > 350) )
      {
        adj = 40;
      }
      else if((sensorDat[0] > sensorDat[2] ) && (sensorDat[0] > 350 && sensorDat[2] < 350))
      {
        adj = -40;
      }

        motorSpeedL = 150 + adj;
        motorSpeedR = 150 - adj;

        break;

    case ObstacleAvoidance:

        if (centerObjectDetected)
        {
            if (leftObjectDetected && rightObjectDetected)
            {
                motorSpeedL = -150;
                motorSpeedR = -150;
                break;
            }

            if (sensorDat[1] < sensorDat[2])
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
        else if (leftObjectDetected)
        {
            motorSpeedL = 200;
            motorSpeedR = -200;
        }
        else if (rightObjectDetected)
        {
            motorSpeedL = -200;
            motorSpeedR = 200;
        }

        if (!leftObjectDetected && !centerObjectDetected && !rightObjectDetected)
        {

            if (millis() - lastObstacleDetectedTime > 300)
            {
                currentState = Navigation;
                lastNavigTrigger = millis();
            }
        }

        break;

    case Correction:
        // Recovery state logic
        leftObjectDetected = sensorDat[0] < 50;
        centerObjectDetected = sensorDat[1] < 50;
        rightObjectDetected = sensorDat[2] < 50;

        if (leftObjectDetected || centerObjectDetected || rightObjectDetected)
        {

            currentState = ObstacleAvoidance;
            lastObstacleDetectedTime = millis();
            break;
        }

        if (error > 10 && distance > 40)
        {
            motorSpeedL = 100;
            motorSpeedR = -100;
        }
        else if (error < -10 && distance > 40)
        {
            motorSpeedL = -100;
            motorSpeedR = 100;
        }
        else
        {
            currentState = Navigation;
        }
        break;
    }

    if (motorSpeedL < -255)
    {
        motorSpeedL = -255;
    }
    else if (motorSpeedL > 255)
    {
        motorSpeedL = 255;
    }

    if (motorSpeedR < -255)
    {
        motorSpeedR = -255;
    }
    else if (motorSpeedR > 255)
    {
        motorSpeedR = 255;
    }
}

void loop()
{
    if (awsClient.stayConnected() == true)
    {
        esp_task_wdt_reset();
    }
    delay(10);
    // awsClient.publishMessage(3);
}

void motorDriver(void *parameter)
{
    mclass m;
    m.SETUP();

    int finishedCounter = 0;
    // example of a task that executes for some time and then is deleted
    while (1)
    {

        navigateToTarget();

        if (messageFlag == true)
        {

            finishedCounter = 0;
            messageFlag = false;
        }
        else
        {
            if (finishedCounter > 100)
            {
                currentState = IDLE;
                roverMessageReceived = false;
                targetMessageReceived = false;
                motorSpeedL = 0;
                motorSpeedR = 0;
            }
            finishedCounter++;
        }

        Serial.print(motorSpeedL);
        Serial.print(" "); // Print a space as a separator
        Serial.println(motorSpeedR);

        if (motorSpeedL > 0)
        {
            m.set_speed(MotorL, Forward, motorSpeedL);
        }
        else
        {
            m.set_speed(MotorL, Backward, -motorSpeedL);
        }

        if (motorSpeedR > 0)
        {
            m.set_speed(MotorR, Forward, motorSpeedR);
        }
        else
        {
            m.set_speed(MotorR, Backward, -motorSpeedR);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void sensorDriver(void *parameter)
{

    while (1)
    {
        arr = s.reading();

        sensorDat[0] = arr[0];
        sensorDat[1] = arr[1];
        sensorDat[2] = arr[2];

        Serial.print(arr[0]);
        Serial.print(" "); // Print a space as a separator
        Serial.print(arr[1]);
        Serial.print(" "); // Print a space as a separator
        Serial.println(arr[2]);

        vTaskDelay(5 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}
