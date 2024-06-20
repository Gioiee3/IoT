#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <string.h>
#include "DHT.h"
#include "ThingsBoard.h"
#include "WiFiUdp.h"
#include <stdio.h>

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient); // HiveMQ

WiFiClient client;
PubSubClient mqtt(client);

WiFiClient espClient;
ThingsBoard tb(espClient);

int status = WL_IDLE_STATUS;
uint8_t leds_control[] = { 14, 13, 12 };
bool subscribed = false;
int current_led = 0;
const int PWM_pin = 14;  
const int PWM_freq = 5000;
const int PWM_channel = 0;
const int PWM_resolution = 8;
int pwm_cmd = 0, pwm_pre = 0;

float ldr_value = 0 ; 
float mq135_value = 0 ;
int relay_status = 0 ; 
int led_pwm = 0;
float light_sensor = 0, air_sensor = 0;

#define udpClientPort  21001
#define udpServerPort  24001

WiFiUDP udp;
char packet[255];
char string[100];

const char * udpClient = "192.168.137.178"; // IP Client
const char * udpServer = "192.168.137.229"; // IP gateway

IPAddress local_IP(192,168,137,229);
IPAddress gateway(192,168,137,1);
IPAddress subnet(255,255,255,0);
IPAddress primaryDNS(8,8,8,8);
IPAddress secondaryDNS(8,8,4,4);

uint16_t current_messID = 0x0A10;
/*--------------------------TOKEN DEVICE-----------------------------*/
#define sensor 0xFEFDFCFB     // 4 byte token
#define actuator 0xAAABACAD   // 4 byte token

#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))
#define LED 2
#define MQTT_USER     "HUST"
#define MQTT_USERNAME "HUST_iot_3"
#define MQTT_PASSWORD "HUST"
#define MQTT_SERVER   "thingsboard.hust-2slab.org"
#define MQTT_PORT     1883
#define MQTT_NAME     "HUST_iot_3"
#define TOKEN         "smm3qt7ejgp11rbprtau" 

const char* ssid = "Laptop";
const char* password = "888888888";

char *mqttServer = "broker.hivemq.com";
int interval = 6000;
long now = 0;
long previous_time = 0;
int state = 0;
String tem = "";
String hum = "";

//-------------------CoAP----------------------------------------------

#define type_CON    0 
#define type_NONCON 1
#define type_ACK    2
#define type_RESET  3 

#define CoAP_ver    1
#define TKL         4
#define code_Content    (uint8_t)(2<<5 | 5 ) //  = 69 
#define code_PUT    (uint8_t)(0 | 0<<5 | 3 ) //  = 3
#define code_GET    (uint8_t)(0 | 0<<5 | 1 ) //  = 1

unsigned int new_time = 0;
unsigned int old_time = 0;

uint8_t data2send[255];
char data2recei[255];
char payloadreceive[10];
/*------------------------------------------------------------------------------*/
int char_compare(char text1[],char text2[] ,int len){
    int i;
    int check_1 = 1;
    for (i=0;i<len;i++){
        if(text1[i] != text2[i]){
            check_1 = 0;
            break;
        }
    }
    return check_1;
}

float char2float(char *text, int size){
    float   result  =   0   ;
    int phan_nguyen =   1   ;
    float div10     =   10  ;
    int i;
    for( i=4 ; i < size - 1 ; i++){
      if( text[i]==NULL ) break;
            if(text[i] == '.') phan_nguyen  =  0;
            else{
                if(phan_nguyen){
                    result  =  result * 10 + ( (int)text[i] - (int)'0' );
                }
                else{
                    result  +=  ( (int)text[i] - (int)'0' ) / div10 ;
                    div10 *= 10.0 ;
                }
            }
        
    }
    return result;
}

void CoAPpacket_header(/*CoAPpacket * packet*/  uint8_t * packet , uint8_t ver = 1, uint8_t type = 1 ,uint8_t tokenlen = 4 , uint8_t code = 100 , uint16_t messID =  0xFFFF ){


  packet[0] = ver << 6 | type << 4 | tokenlen ;
  packet[1] = code ;
  packet[2] = messID >> 8 ;
  packet[3] =(uint8_t) (0x00FF & messID);
}

void CoAPpacket_token(/*CoAPpacket * packet*/ uint8_t * packet , uint32_t token = 0xFEFDFCFB ){

  packet[4] = token >> 24;
  packet[5] = (uint8_t) ( 0x00FF & (token >> 16) )  ;
  packet[6] = (uint8_t) ( 0x0000FF & (token >> 8))  ;
  packet[7] = (uint8_t) ( 0x000000FF & token  )     ;

}

void CoAPpacket_options(/*CoAPpacket * packet*/ uint8_t * packet , uint32_t options = 0xFEFDFCFB){
  
  packet[8] = options >> 24;
  packet[9] = (uint8_t) ( 0x00FF & (options >> 16) )  ;
  packet[10] = (uint8_t) ( 0x0000FF & (options >> 8))  ;
  packet[11] = (uint8_t) ( 0x000000FF & options  )     ;

}

void CoAPpacket_payloadmarker(/*CoAPpacket * packet*/ uint8_t * packet ){
  packet[12] = (uint8_t) 0xFF;
}

void CoAPpacket_payload(/*CoAPpacket * packet*/ uint8_t * packet,uint8_t  *payload){
  int sizeload = sizeof(payload);
  int i;
  for(i=0;i<10;i++){
    packet[i+13] = payload[i];
  }
}

int CoAP_get_messID(char packet_reive[]){
  int ID = 0 | ( packet_reive[2] << 8 | packet_reive[3]);
  return ID;
}
void CoAP_get_payload(char packet_reive[],char payloadreceive[]){
  int i;
  for(i=0;i<10;i++){
    if(packet_reive[i+13] != NULL){
      payloadreceive[i] = packet_reive[i+13];
    }
    else{
      payloadreceive[i] = NULL;
      break;
    }
  }
}

uint32_t CoAP_get_token(char data2recei[]){
  uint32_t temp = 0;
  temp = (uint32_t)(data2recei[7] | (data2recei[6] << 8) | (data2recei[5]<< 16) |(data2recei[5]<<24));
  return temp;
}
uint8_t CoAP_get_code(char data2recei[]){
  return (uint8_t) data2recei[1];
}

//-------------------------------MQTT----------------------------------------
RPC_Response processSetGpioState(const RPC_Data &data)
{
  char pressString[8];
  state = data;
  Serial.print("State led: ");
  Serial.println(state);
  digitalWrite(LED, state);
  // Send data to HiveMQ, control relay
  if (state)  
  {
    sprintf(pressString,"led_on");
    mqttClient.publish("esp32/PWM_AND_HVAC_2614", pressString);
  }
  else
  {
    sprintf(pressString,"led_off");
    mqttClient.publish("esp32/PWM_AND_HVAC_2614", pressString);
  }  
  return RPC_Response("setGpioStatus", state);
}

RPC_Response processPwmLed(const RPC_Data &data)
{
  char pressString[8];
  pwm_cmd = data;
  return RPC_Response("PWMLed", pwm_cmd);
}

RPC_Response processGetGpioState(const RPC_Data &data)
{
  Serial.println("Received the get GPIO RPC method");
  String respStr = "{";
  int pin = LED;
  Serial.print("Getting LED ");
  Serial.print(pin);
  Serial.print(" state ");
  bool ledState = digitalRead(pin);
  Serial.println(ledState);
  respStr += String("\"" + String(pin) + "\": " + String(ledState?"true":"false") + ", ");
  respStr = respStr.substring(0, respStr.length() - 2);
  respStr += "}";
  return respStr;
}

RPC_Callback callbacks[] = {
  { "setGpioStatus",    processSetGpioState },
  { "getGpioStatus",    processGetGpioState },
  { "PWMLed"       ,    processPwmLed       },
};

void setupMQTT() {
  mqttClient.setServer(mqttServer, MQTT_PORT); // Hivemq Broker
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);      // Thingsboard server
  mqttClient.setCallback(callback);            // callback function
}
// reconnect to HiveMQ
void reconnect() {
  Serial.println("Connecting to MQTT Broker...");
  while (!mqttClient.connected()) {
      Serial.println("Reconnecting to MQTT Broker..");
      String clientId = "ESP32Client-";
      clientId += String(random(0xffff), HEX);
      
      if (mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
        Serial.println("Connected.");
        mqttClient.subscribe("esp32/temp_03hust...");
        mqttClient.subscribe("esp32/humid_03hust...");
      }      
  }
}
// reconnect wifi
void connect_WiFi()
{
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  WiFi.config(local_IP,gateway,subnet,primaryDNS,secondaryDNS); //config for CoAP
  pinMode(LED, OUTPUT);
  // PWM setup
  ledcSetup(PWM_channel, PWM_freq, PWM_resolution);
  ledcAttachPin(PWM_pin, PWM_channel);
  // connect wifi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  } 
  // CoAP server port 
  udp.begin(udpServerPort);
  Serial.println("");
  Serial.println("Connected to Wi-Fi");
  setupMQTT();
}

void loop() {
  // reconnect to HiveMQ
  if (!mqttClient.connected()) reconnect();
  mqttClient.loop();
  now = millis();
  // PWM led
  ledcWrite(PWM_channel, pwm_cmd);

  // send data to HiveMQ for PWM LED
  if (pwm_cmd != pwm_pre)
  {
    char PWM[8];
    sprintf(PWM, "%d", pwm_cmd);
    Serial.print("PWM: ");
    Serial.println(PWM);
    mqttClient.publish("esp32/PWM_AND_HVAC_2614", PWM);
    pwm_pre = pwm_cmd;
  }

  // send data to Thingsboard
  if (now - previous_time > interval) 
  {
    if (WiFi.status() != WL_CONNECTED)  connect_WiFi();
    // Reconnect to Thingsboard
    if (mqtt.connected() == false) 
    {
      Serial.print("MQTT connect to Thingsboard... ");
      if (mqtt.connect(MQTT_NAME, MQTT_USERNAME, MQTT_PASSWORD))  Serial.println("connected Thingsboard"); 
      else Serial.println("failed");
    }
    
    // send data thingsboard with MQTT
    char json[100];
    char Air_Light[100];
    sprintf(Air_Light, "{\"LDR\":%.1f,\"MQ135\":%.1f}", light_sensor, air_sensor);
    mqtt.publish("v1/devices/me/telemetry", Air_Light);
    // delay(10);
    previous_time = now;
    if (mqtt.connected() == false) 
    {
      Serial.print("MQTT connect to Thingsboard... ");
      if (mqtt.connect(MQTT_NAME, MQTT_USERNAME, MQTT_PASSWORD))  Serial.println("connected Thingsboard"); 
      else Serial.println("failed");
    } 
    mqtt.loop();
    String dataJS = "{\"Temp\":" + tem + ",\"Humid\":" + hum + "}";
    dataJS.toCharArray(json,dataJS.length()+1);
    mqtt.publish("v1/devices/me/telemetry", json);  
  }
  if (WiFi.status() != WL_CONNECTED)  connect_WiFi();

  // connect device of thingsboard to use RPC
  if (!tb.connected()) {
    subscribed = false;
    Serial.print("Connecting to: ");
    Serial.print(MQTT_SERVER);
    Serial.print(" with token ");
    Serial.println(TOKEN);
    if (!tb.connect(MQTT_SERVER, TOKEN)) {
      Serial.println("Failed to connect RPC");
      return;
    }
  }

  // subscribing for RPC protocol
  if (!subscribed) {
    Serial.println("Subscribing for RPC... ");
    if (!tb.RPC_Subscribe(callbacks, COUNT_OF(callbacks))) {
      Serial.println("Failed to subscribe for RPC");
      return;
    }
    Serial.println("Subscribe done");
    subscribed = true;
  }  
  tb.loop();
  // if period = 2s, check data receive from CoAP client and save data
  if ((millis() - old_time) > 2000)
  {
    udp.parsePacket();
    udp.read(data2recei,255);
    // Serial.println(data2recei);
    String buffer_udp = "";
    String topic_udp = "";
    for(int i = 13; i < 16; i++)  topic_udp += (char)data2recei[i];
    for(int i = 17; i < 22; i++)  buffer_udp += (char)data2recei[i];    
    float bufferFloat = buffer_udp.toFloat();
    char buf_value[10];
    sprintf(buf_value,"%.2f", bufferFloat);
    if (topic_udp == "LDR") 
    {
      Serial.print("LDR: ");
      Serial.println(buf_value);
      light_sensor = 100 - bufferFloat;
    }
    if (topic_udp == "AIR") 
    {
      Serial.print("MQ135: ");
      Serial.println(buf_value);
      air_sensor = bufferFloat;
    }
    old_time = millis();
  }
}

void callback(char* topic, byte* message, unsigned int length) 
{
  String buffer = "";
  String data_topic = String(topic);
  Serial.print("(GATE) " + data_topic + ": ");
  for (int i = 0; i < length; i++) {
    buffer += (char)message[i];
    Serial.print((char)message[i]);
  }
  Serial.print("\n");

  if (WiFi.status() != WL_CONNECTED)  connect_WiFi();
  // save temperature and humidity received from topic of HiveMQ
  else if (data_topic == "esp32/temp_03hust...")
      {
        tem = buffer;
      }
      else if (data_topic == "esp32/humid_03hust...")        
          {
            hum = buffer;
          }
}