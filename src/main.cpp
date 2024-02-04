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


double Kp = 0.3, Ki = 0.01, Kd = 0.2;
double integral = 0.0;
double previousError = 0.0;

double distance;

int lookupTable[]  = {
  -255, -253, -251, -249, -247, -245, -243, -241, -239, -237, -235, -233, -231, -229, -227, -225, -223, -221, -219, -217, -215,
  -213, -211, -209, -207, -205, -203, -201, -199, -197, -195, -193, -191, -189, -187, -185, -183, -181, -179, -177, -175, -173,
  -171, -169, -167, -165, -163, -161, -159, -157, -155, -153, -151, -149, -147, -145, -143, -141, -139, -137, -135, -133, -131,
  -129, -127, -125, -123, -121, -119, -117, -115, -113, -111, -109, -107, -105, -103, -101, -99, -97, -95, -93, -91, -89, -87,
  -85, -83, -81, -79, -77, -75, -73, -71, -69, -67, -65, -63, -61, -59, -57, -55, -53, -51, -49, -49, -49, -49, -49, -49, -49,
  -49, -47, -45, -43, -41, -41, -41, -41, -41, -41, -41, -41, -41, -41, -0, -0, -0, -0, 0, 0, 0, 0, 41, 41, 41, 41, 41, 41, 41,
  41, 41, 41, 43, 45, 47, 49, 49, 49, 49, 49, 49, 49, 49, 51, 53, 55, 57, 59, 61, 63, 65, 67, 69, 71, 73, 75, 77, 79, 81, 83,
  85, 87, 89, 91, 93, 95, 97, 99, 101, 103, 105, 107, 109, 111, 113, 115, 117, 119, 121, 123, 125, 127, 129, 131, 133, 135, 137,
  139, 141, 143, 145, 147, 149, 151, 153, 155, 157, 159, 161, 163, 165, 167, 169, 171, 173, 175, 177, 179, 181, 183, 185, 187, 189,
  191, 193, 195, 197, 199, 201, 203, 205, 207, 209, 211, 213, 215, 217, 219, 221, 223, 225, 227, 229, 231, 233, 235, 237, 239, 241,
  243, 245, 247, 249, 251, 253, 255
};




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
    integral += error;
    
    double derivative = error - previousError;
    double adjustment = Kp*error + Ki*integral + Kd*derivative;  // PID formula
    
    // Calculate motor speed adjustments based on PID adjustment
    if(error > 5 && distance > 40)
    {
      motorSpeedL = 60;
      motorSpeedR = -60;
    }
    else if(error < -5 && distance > 40)
    {
      motorSpeedL = -60;
      motorSpeedR = 60;
    }
    else{
      motorSpeedL = 150;
      motorSpeedR = 150;
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
    
    // Update for next iteration
    previousError = error;
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

    Serial.println(lookupTable[127]);

    

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }

  vTaskDelete( NULL );
}

void sensorDriver( void * parameter )
{
  int16_t* arr;
  
  while(1)
  {
    // arr = s.reading();
  
    // bool leftObjectDetected = arr[0] < 200;
    // bool centerObjectDetected = arr[1] < 200;
    // bool rightObjectDetected = arr[2] < 200;

    // if (leftObjectDetected || centerObjectDetected || rightObjectDetected){
    //   motorSpeedL = 0;
    //   motorSpeedR = 0;
    //   continue;
    // }

    // motorSpeedL = 255;
    // motorSpeedR = 255;
 
    vTaskDelay(10/ portTICK_PERIOD_MS);

  }

  vTaskDelete( NULL );
}
