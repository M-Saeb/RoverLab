/**
 * @file
 * 
 * ESP32 AWS Library
 * 
 * Functions to get the crawler coordinates from the Camera over AWS IoT
 * 
 * Authors: Vipul Deshpande, Jaime Burbano
 */

#ifndef aws_h
#define aws_h

#include <MQTTClient.h>

extern int suggestedAngle;
extern int currentAngle;
extern int currentX;
extern int currentY;
extern int targetX;
extern int targetY;

extern bool targetMessageReceived;
extern bool roverMessageReceived;
extern bool messageFlag;

void messageHandler(String, String);
class myawsclass {
  public:
    myawsclass();

    void connectAWS();                            /* Initialize and connect to AWS */
    void publishMessage(int16_t sensorValue);     /* Publish the values of the sensors */
    bool stayConnected();                         /* Maintain the connection */
};

extern myawsclass awsobject;

#endif