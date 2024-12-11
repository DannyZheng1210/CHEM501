// Include the neccessary library
#include <ArduinoMqttClient.h>
#include "Arduino.h"
#include "Arduino_BHY2Host.h"
#include <WiFiNINA.h>

// Initialize the sensor objects
Sensor tempSensor(SENSOR_ID_TEMP);
SensorBSEC bsec(SENSOR_ID_BSEC);
Sensor gas(SENSOR_ID_GAS);
Sensor humidity(SENSOR_ID_HUM);
Sensor barometer(SENSOR_ID_BARO);

char ssid[] = "VodafoneMobileWiFi-72AF05";        // your network SSID (name)
char pass[] = "7353626621";    // your network password (use for WPA, or use as key for WEP)

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = "pf-eveoxy0ua6xhtbdyohag.cedalo.cloud";
int        port     = 1883;
const char topic1[] = "dongyang_temp";
const char topic2[] = "dongyang_gas";
const char topic3[] = "dongyang_humidity";
const char topic4[] = "dongyang_barometer";


//set interval for sending messages (milliseconds)
const long interval = 5000;
unsigned long previousMillis = 0;

int count = 0;

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  BHY2Host.begin();  // Start the BHY2Host for sensor data reading
  tempSensor.begin();    // Initialize the temperature sensor
  gas.begin();           // Initialize the gas sensor
  humidity.begin();      // Initialize the humidity sensor
  barometer.begin();     // Initialize the barometric pressure sensor

  // attempt to connect to Wifi network:
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }

  // If successfully connected to WiFi
  Serial.println("You're connected to the network");
  Serial.println();

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

 // Attempt to connect to MQTT broker:
  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    while (1);
  }
  // If successfully connected to MQTT broker
  Serial.println("You're connected to the MQTT broker!");
  Serial.println();
}

void loop() {
  // call poll() regularly to allow the library to send MQTT keep alive which
  // avoids being disconnected by the broker
  mqttClient.poll();
  BHY2Host.update();
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time a message was sent
    previousMillis = currentMillis;

    //Record temperature and gas sensor values
    float Rvalue1 = tempSensor.value();
    float Rvalue2 = gas.value();
    float Rvalue3 = humidity.value();
    float Rvalue4 = barometer.value();
    float Rvalue5 = String(bsec.co2_eq());
    float Rvalue6 = String(bsec.accuracy());

    // Print the recorded sensor values to the serial monitor for debugging
    Serial.println("Temperature: " + String(Rvalue1));
    Serial.println("Gas: " + String(Rvalue2));
    Serial.println("Humidity: " + String(Rvalue3));
    Serial.println("Pressure: " + String(Rvalue4));
    Serial.println("BSEC info: " + bsec.toString());
    Serial.println("CO2: " + String(bsec.co2_eq()));
    Serial.println("VOC: " + String(bsec.b_voc_eq()));
    Serial.println("Accuracy: " + String(bsec.accuracy()));
    Serial.println();

    // send message, the Print interface can be used to set the message contents
    mqttClient.beginMessage(topic1);
    mqttClient.print(Rvalue1);
    mqttClient.endMessage();

    mqttClient.beginMessage(topic2);
    mqttClient.print(Rvalue2);
    mqttClient.endMessage();

    mqttClient.beginMessage(topic3);
    mqttClient.print(Rvalue3);
    mqttClient.endMessage();

    mqttClient.beginMessage(topic4);
    mqttClient.print(Rvalue4);
    mqttClient.endMessage();

    Serial.println();
  }
}
