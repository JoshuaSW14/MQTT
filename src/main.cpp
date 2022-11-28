// COMP-10184 – Mohawk College
// MQTT
//
// This program demonstrates getting x, y, z acceleration data from the MPU6050 sensor and sending it to a ThingSpeak
// channel every 5 seconds using the MQTT protocol.
//
// @author  Joshua Symons-Webb
// @id      000812836
//
// I, Joshua Symons-Webb, 000812836 certify that this material is my original work. No
// other person's work has been used without due acknowledgement.
//

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include "secrets.h"

#define MQTT_MAX_PACKET_SIZE 4096 // Max packet size for MQTT
#define DELAY_5 500               // 5 second delay

// Wifi login details
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

// ThingSpeak Channel & MQTT interface configuration
const char *mqttServer = "mqtt3.thingspeak.com";
const uint16_t mqttPort = 1883;
char mqttUserName[] = SECRET_MQTT_USERNAME;
char mqttPass[] = SECRET_MQTT_PASSWORD;
char clientID[] = SECRET_MQTT_CLIENT_ID;
int channelID = ThingSpeak_Channel;

String mqttTopic = "channels/" + String(channelID) + "/publish"; // MQTT topic

WiFiClient client;               // WiFi client
PubSubClient mqttClient(client); // MQTT publish/subscribe client

Adafruit_MPU6050 mpu; // MPU 6050 Configuration

// ***************************************************************************
// function to setup MPU6050.  configures acceleration range, angular rate change
// and filter bandwidth.
void configureMPU6050()
{
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange())
  {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange())
  {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth())
  {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
}

// ****************************************************************************
// Gets MPU data and publishes to the ThingSpeak MQTT Client
void updateThingSpeak(void)
{
  // Get new sensor events with the readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  String mqttData;
  if (mqttClient.connect(clientID, mqttUserName, mqttPass)) // Makes MQTT Connection
  {
    Serial.println("Connection made to MQTT broker!");

    mqttData = "field1=" + String(a.acceleration.x) + "&field2=" + String(a.acceleration.y) + "&field3=" + String(a.acceleration.z);
    Serial.println("  Publish data=" + mqttData);

    // publish data
    if (mqttClient.publish(mqttTopic.c_str(), mqttData.c_str()))
    {
      Serial.println("  Publish was successful!");
    }
    else
    {
      Serial.println("  Publish failed..");
    }

    // disconnect from broker.  if we were also a subscriber, we would not do this.
    mqttClient.disconnect();
  }
  else
  {
    // See https://pubsubclient.knolleary.net/api.html#state for the failure code explanation.
    Serial.print("  Failed with state: ");
    Serial.println(mqttClient.state());
  }
}

// ****************************************************************************
void setup()
{
  Serial.begin(115200); // configure the USB serial monitor

  // Introduction Message
  Serial.println("--- COMP-10184 - MQTT ---");
  Serial.println("Name      : Joshua Symons-Webb");
  Serial.println("Student ID: 000812836 \n\n\n");
  delay(500); // Delay for intro message

  if (!mpu.begin()) // Try to initialize!
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // set up accel/gyro sensor
  configureMPU6050();

  // connect to WiFi
  Serial.printf("\nConnecting to %s ", ssid);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");

  // set MQTT broker info
  mqttClient.setServer(mqttServer, mqttPort);
}

// ****************************************************************************
void loop()
{
  updateThingSpeak(); // try to publish data to ThingSpeak using MQTT protocol

  mqttClient.loop(); // MQTT libraries need this

  delay(DELAY_5); // Wait every 5 seconds
}