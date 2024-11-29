#include <ArduinoMqttClient.h>
#include "Arduino.h"
#include "Arduino_BHY2Host.h"
#include <WiFiNINA.h>

Sensor tempSensor(SENSOR_ID_TEMP);
SensorBSEC bsec(SENSOR_ID_BSEC);
Sensor gas(SENSOR_ID_GAS);
//you need to defind the sensor in the beginning, temperature sensor and BSEC sensor

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = "VodafoneMobileWiFi-72AF05";        // your network SSID (name)
char pass[] = "7353626621";    // your network password (use for WPA, or use as key for WEP)

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = "pf-eveoxy0ua6xhtbdyohag.cedalo.cloud";
int        port     = 1883;
const char topic1[] = "dongyangtest1";
const char topic2[] = "dongyangtest2";


//set interval for sending messages (milliseconds)
const long interval = 5000;
unsigned long previousMillis = 0;

int count = 0;

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  BHY2Host.begin();
  tempSensor.begin();
  gas.begin();

  // attempt to connect to Wifi network:
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }

  Serial.println("You're connected to the network");
  Serial.println();

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();
}

void loop() {
  // call poll() regularly to allow the library to send MQTT keep alive which
  // avoids being disconnected by the broker
  mqttClient.poll();
  BHY2Host.update();
  unsigned long currentMillis = millis();

  //temperature = tempSensor.value();


  if (currentMillis - previousMillis >= interval) {
    // save the last time a message was sent
    previousMillis = currentMillis;

    //Record temperature and gas sensor values
    float Rvalue1 = tempSensor.value();
    float Rvalue2 = gas.value();

    // Serial.print("Sending message to topic1: ");
    // Serial.println(topic1);
    Serial.println(Rvalue1);

    // Serial.print("Sending message to topic2: ");
    // Serial.println(topic2);
    Serial.println(Rvalue2);

    // send message, the Print interface can be used to set the message contents
    mqttClient.beginMessage(topic1);
    mqttClient.print(Rvalue1);
    mqttClient.endMessage();

    mqttClient.beginMessage(topic2);
    mqttClient.print(Rvalue2);
    mqttClient.endMessage();

    Serial.println();
  }
}
