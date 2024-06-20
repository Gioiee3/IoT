#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <string.h>
#include "DHT.h"

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient); 

#define LED1 2
#define LED  22
#define DHTPIN 18
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
const int PWM_pin = 19;  
const int PWM_freq = 5000;
const int PWM_channel = 0;
const int PWM_resolution = 8;
int pwm_cmd = 0, pwm_pre = 0;

DHT dht(DHTPIN, DHTTYPE);

#define MQTT_USER "HUST"
#define MQTT_PASSWORD "HUST"
const char* ssid = "Laptop";
const char* password = "888888888";

char *mqttServer = "broker.hivemq.com";
int mqttPort = 1883;
int interval = 7000;
long now = 0;
long previous_time = 0;
void setupMQTT() {
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(callback);
}

void reconnect() {
  Serial.println("Connecting to MQTT Broker...");
  while (!mqttClient.connected()) {
      Serial.println("Reconnecting to MQTT Broker..");
      String clientId = "ESP32Client-";
      clientId += String(random(0xffff), HEX);
      
      if (mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
        Serial.println("Connected.");
        // subscribe to topic
        mqttClient.subscribe("esp32/PWM_AND_HVAC_2614");
        // mqttClient.subscribe("esp32/message");        
      }      
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  dht.begin();

  ledcSetup(PWM_channel, PWM_freq, PWM_resolution); // PWM setup
  ledcAttachPin(PWM_pin, PWM_channel);
  
  // connect wifi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  } 
  Serial.println("");
  Serial.println("Connected to Wi-Fi");
  setupMQTT(); 
  digitalWrite(LED, HIGH);
  delay(2000);
}

void loop() {
  // reconnect to HiveMQ
  if (!mqttClient.connected()) reconnect();
  mqttClient.loop();
  now = millis();
  
  // period transmit = interal
  if (now - previous_time > interval) 
  {
    previous_time = now;
    // float temp = random(300, 310)/10.0;
    // float humid = random(700, 715)/10.0;  

    //read DHT22 sensor
    float humid = dht.readHumidity();
    float temp = dht.readTemperature(); // Read temperature as Celsius (the default)
    float f = dht.readTemperature(true); // Read temperature as Fahrenheit (isFahrenheit = true)

    if (isnan(humid) || isnan(temp) || isnan(f)) {
      Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }

    // send data to HiveMQ
    char tempString[8];
    dtostrf(temp, 1, 2, tempString);
    Serial.print("Temperature: ");
    Serial.println(tempString);
    mqttClient.publish("esp32/temp_03hust...", tempString);
    Serial.println("Send data to topic - esp32/temp_03hust...");

    char humString[8];
    dtostrf(humid, 1, 2, humString);
    Serial.print("Humidity: ");
    Serial.println(humString);
    mqttClient.publish("esp32/humid_03hust...", humString);
    Serial.println("Send data to topic - esp32/humid_03hust...");
  }
  // PWM LED
  ledcWrite(PWM_channel, 255 - pwm_cmd);
}

void callback(char* topic, byte* message, unsigned int length) 
{
  String buffer = "";
  String data = "(NODE) " + String(topic) + ": ";
  // Serial.print(data);
  //Save buffer
  for (int i = 0; i < length; i++) {
    buffer += (char)message[i];
    // Serial.print((char)message[i]);
  }

  // control relay or save value PWM
  if (buffer == "led_on") 
  {
    digitalWrite(LED, LOW); // LED OFF
    Serial.println("RELAY on !");
  }
  else if (buffer == "led_off") 
      {
        digitalWrite(LED, HIGH);
        Serial.println("RELAY off !");
      }
      else
      {
        pwm_cmd = buffer.toInt();
        Serial.print("PWM LED: ");
        Serial.println(buffer);
      }
}