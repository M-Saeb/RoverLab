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


// Mathematical function to calculate the error and converting it to degrees
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
    // Calculate the error between the vector to the target and rovers attitude
    double error = calculateDirectionError();

    // Thresholds for object detection from each side respectively
    bool leftObjectDetected = sensorDat[0] < 150;
    bool centerObjectDetected = sensorDat[1] < 100;
    bool rightObjectDetected = sensorDat[2] < 150;

    switch (currentState)
    {
    case IDLE:
        // Flags for activating the Rover scenario, these values are only true if the rover is receiving messages from AWS
        if (targetMessageReceived == true && roverMessageReceived == true)
        {
            //State transition to Correction
            currentState = Correction;
        }
        break;
    case Navigation:

        // Check for detected objects
        if (leftObjectDetected || centerObjectDetected || rightObjectDetected)
        {
            // This timer ensures the navigation will run at least 200 milliseconds regardless of sensor value
            if (millis() - lastNavigTrigger > 200)
            {
                //State Transition
                currentState = ObstacleAvoidance;
            }

            // Timer rebase for obstacle detection
            lastObstacleDetectedTime = millis();

            break;
        }

        // Thresholds for error of angle from the vector to the target
        if (error < -20 || error > 20)
        {
            // This timer ensures the navigation after error correction is ran at least 2000 milliseconds
            if (millis() - lastNavigTrigger > 2000)
            {
                currentState = Correction;
                break;
            }
        }

      adj = 0;

      //Obsolete code for additional obstacle avoidance by slight turning radius
      if((sensorDat[0] < sensorDat[2] ) && (sensorDat[0] < 350 && sensorDat[2] > 350) )
      {
        adj = 0;
      }
      else if((sensorDat[0] > sensorDat[2] ) && (sensorDat[0] > 350 && sensorDat[2] < 350))
      {
        adj = -0;
      }

        motorSpeedL = 150 + adj;
        motorSpeedR = 150 - adj;

        break;

    case ObstacleAvoidance:

        //Handling of the object detection
        if (centerObjectDetected)
        {
            // IF 3 of the paths are blocked, go backwards
            if (leftObjectDetected && rightObjectDetected)
            {
                motorSpeedL = -150;
                motorSpeedR = -150;
                break;
            }

            // If left obstacle is closer turn right
            if (sensorDat[1] < sensorDat[2])
            {
                motorSpeedL = 150;
                motorSpeedR = -150;
            }// If right obstacle is closer turn left
            else
            {
                motorSpeedL = -150;
                motorSpeedR = 150;
            }
        }
        else if (leftObjectDetected) // IF only left object is detected turn right
        {
            motorSpeedL = 200;
            motorSpeedR = -200;
        }
        else if (rightObjectDetected) // IF only right object is detected turn right
        {
            motorSpeedL = -200;
            motorSpeedR = 200;
        }

        // Condition to leave the obstacle avoidance state
        if (!leftObjectDetected && !centerObjectDetected && !rightObjectDetected)
        {
            // Ensures that the obstacle avoidance is ran at least 300 milliseconds
            if (millis() - lastObstacleDetectedTime > 300)
            {
                currentState = Navigation;
                lastNavigTrigger = millis();
            }
        }

        break;

    case Correction:
        // Correction state logic
        leftObjectDetected = sensorDat[0] < 50;
        centerObjectDetected = sensorDat[1] < 50;
        rightObjectDetected = sensorDat[2] < 50;

        // Additional obstacle avoidance on correction function, This ensures the rover wont scrape against the obstacles while correcting
        if (leftObjectDetected || centerObjectDetected || rightObjectDetected)
        {

            currentState = ObstacleAvoidance;
            lastObstacleDetectedTime = millis();
            break;
        }

        // Corrects angle error until it is less then degrees when the distance to the target is larger than 40
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

    // Safeguard for the motor values
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
        //Watchdog reset with on every successfull cycle with a message received
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

        // Counter for cycles since the last message is received
        if (messageFlag == true)
        {
            // Reseting the counter when a message is received successfully
            finishedCounter = 0;
            messageFlag = false;
        }
        else
        {
            // IF counter is greater than 100 (1 Second) reset all of the states and stop the motor
            if (finishedCounter > 100)
            {
                currentState = IDLE;
                roverMessageReceived = false;
                targetMessageReceived = false;
                motorSpeedL = 0;
                motorSpeedR = 0;
            }

            // Increment the counter
            finishedCounter++;
        }

        Serial.print(motorSpeedL);
        Serial.print(" "); // Print a space as a separator
        Serial.println(motorSpeedR);


        // Motor control wrapper that adjusts the values correctly, taking the absolute value and deciding on the direction
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
