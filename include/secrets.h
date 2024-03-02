/**
 * ESP32 secrets file
 * 
 * Contains information required to connect to WiFi and in-turn, AWS IoT
 * 
 * Authors: Vipul Deshpande, Jaime Burbano
 */


/*
  Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
  Permission is hereby granted, free of charge, to any person obtaining a copy of this
  software and associated documentation files (the "Software"), to deal in the Software
  without restriction, including without limitation the rights to use, copy, modify,
  merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
  permit persons to whom the Software is furnished to do so.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#include <pgmspace.h>

#define SECRET
#define THINGNAME "IotDeviceName"

const char WIFI_SSID[] = "wifiSSID";
const char WIFI_PASSWORD[] = "WifiPassword";
const char AWS_IOT_ENDPOINT[] = "MQTTEndpoint";

/* Amazon Root CA 1 */
static const char AWS_CERT_CA[] PROGMEM = R"EOF(
<<PASTE HERE>>
)EOF";

/* Device Certificate */
static const char AWS_CERT_CRT[] PROGMEM = R"KEY(
<<PASTE HERE>>
)KEY";

/* Device Private Key */
static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(
<<PASTE HERE>>
)KEY";