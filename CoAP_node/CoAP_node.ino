/*

COAP CLIENT   =========   NODE SENSOR   ===========   WITH TOKEN : 0xFEFDFCFB

COAP CLIENT   =========   NODE ACTUATOR ===========   WITH TOKEN : 0xAAABACAD

IP : 192.168.1.199

*/
/*--------------------------Include Lib---------------------------------*/

#include "WiFi.h"
#include "WiFiUdp.h"
//#include "coapclient.h"
#include <string.h>
#include <stdio.h>
/*--------------------------Define Constant-----------------------------*/
#define debug(x)  Serial.println("x=",x);
#define type_CON    0 
#define type_NONCON 1
#define type_ACK    2
#define type_RESET  3 

#define CoAP_ver    1
#define TKL         4
#define code_PUT    (uint8_t)(0 | 0<<5 | 3 )
#define code_GET    (uint8_t)(0 | 0<<5 | 1 ) 


#define LEDPWM  14 
#define RELAY   23
#define LDR     34
#define MQ135   35
#define LED     2

int i = 0;
String str = "";
int mq135;
int ldr;
float value_mq135;
float value_ldr;
int current_time = 0;
int relay_status = 0;
int led_pwm = 0;
/*--------------------------TOKEN DEVICE-----------------------------*/
#define sensor 0xFEFDFCFB     // 4 byte token
#define actuator 0xAAABACAD   // 4 byte token
/*--------------------------Config UDP-----------------------------*/

const char * ssid = "Laptop";
const char * password = "888888888";

#define max_buffer_udp 128
WiFiUDP udp;
//IPAddress local_IP(192,168,1,199);
const char * udpClient = "192.168.137.178";
const char * udpServer = "192.168.137.229";

IPAddress local_IP(192,168,137,178);
IPAddress gateway(192,168,137,1);
IPAddress subnet(255,255,255,0);
IPAddress primaryDNS(8,8,8,8);
IPAddress secondaryDNS(8,8,4,4);

#define udpClientPort  21001
#define udpServerPort  24001
uint8_t data_receive[max_buffer_udp-1];
uint8_t data_send[max_buffer_udp-1];

uint16_t messID = 0x0A10 ;
/*--------------------------Define-----------------------------*/
uint8_t data2send[255];
uint8_t payload[10];
char data2recei[255];
char payloadreceive[10];
char packet[255];

/*--------------------------Declare Function-------------------------------*/
float char2float(char *text, int size){
    float   result  =   0   ;
    int phan_nguyen =   1   ;
    float div10     =   10  ;
    int i;
    for( i=0 ; i < size - 1 ; i++){
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

void LDR_packet(uint8_t * payload , float value){
  if (value > 99) value = 99.98 ;
  int value_int = (int) value  ;      // value = "xx.xx"
  int value_float = (int) ((value - value_int)*100) ;
  payload[0] = 'L';
  payload[1] = 'D';
  payload[2] = 'R';
  payload[3] = '\\';
  payload[4] = (uint8_t) ( (int)'0' + (int) value_int / 10) ;
  payload[5] = (uint8_t) ( (int)'0' + (int) value_int % 10) ;
  payload[6] = '\.';
  payload[7] = (uint8_t) ( (int)'0' + (int) value_float / 10) ;
  payload[8] = (uint8_t) ( (int)'0' + (int) value_float % 10) ;
  payload[9] = NULL;
}

void MQ137_packet(uint8_t * payload , float value){
  if (value > 99) value = 99.98 ;
  int value_int = (int) value  ;      // value = "xx.xx"
  int value_float = (int) ((value - value_int)*100) ;
  payload[0] = 'A';
  payload[1] = 'I';
  payload[2] = 'R';
  payload[3] = '\\';
  payload[4] = (uint8_t) ( (int)'0' + (int) value_int / 10) ;
  payload[5] = (uint8_t) ( (int)'0' + (int) value_int % 10) ;
  payload[6] = '\.';
  payload[7] = (uint8_t) ( (int)'0' + (int) value_float / 10) ;
  payload[8] = (uint8_t) ( (int)'0' + (int) value_float % 10) ;
  payload[9] = NULL;
}

uint16_t createMessID(uint16_t * messid){
  *messid +=1;
  return *messid;
}

uint16_t CoAP_PUT(int type_value ){
//type_value : LDR or MQ135;
  CoAPpacket_header(data2send, CoAP_ver, type_CON, TKL, code_PUT, createMessID(&messID));
  CoAPpacket_token(data2send,sensor);
  CoAPpacket_options(data2send);
  CoAPpacket_payloadmarker(data2send);
  if(type_value == LDR ){
    LDR_packet(payload,value_ldr);

    CoAPpacket_payload(data2send,payload);
  }
  else if( type_value == MQ135) {
    MQ137_packet(payload, value_mq135);

    CoAPpacket_payload(data2send, payload);
  }

  udp.beginPacket(udpServer,udpServerPort);

  udp.write(data2send,sizeof(data2send));

  udp.endPacket();

  return messID;
}
uint16_t CoAP_GET(int type_value){
// type_value : RELAY or LEDPWM
  CoAPpacket_header(data2send, CoAP_ver, type_CON, TKL, code_GET, createMessID(&messID));
  CoAPpacket_token(data2send,sensor);
  CoAPpacket_options(data2send);
  CoAPpacket_payloadmarker(data2send);

  if(type_value == RELAY){
    //Serial.println("RELAY");
    payload[0] = 'R';
    payload[1] = 'E';
    payload[2] = 'L';
    payload[3] = 'A';
    payload[4] = 'Y';
    payload[5] = NULL;
    // uint8_t tempchar[] = "RELAY";
    // for(i=0;i<6;i++)payload[i] = (uint8_t) tempchar[i];
    CoAPpacket_payload(data2send,payload);
  }
  else if(type_value == LEDPWM){
    payload[0] = 'L';
    payload[1] = 'E';
    payload[2] = 'D';
    payload[3] = 'P';
    payload[4] = 'W';
    payload[5] = 'M';
    payload[6] = NULL;
    CoAPpacket_payload(data2send,payload);
  }
  
  udp.beginPacket(udpServer, udpServerPort);
  udp.write(data2send,sizeof(data2send));
  udp.endPacket();
  return messID;
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

int ADCread(int pin){
  int adc = -1;
  adc = analogRead(pin);
  if(adc == -1 ) return -1 ;
  else return adc;
}
void initpin(){

  pinMode(LED,OUTPUT);
  pinMode(LEDPWM,OUTPUT);
  pinMode(RELAY,OUTPUT);

  pinMode(LDR,INPUT);
  pinMode(MQ135,INPUT);
  
}

void initpwm(int freq = 5000, int chanel = 0,int pin = 2){
  ledcSetup(chanel,freq,8);
  ledcAttachPin(pin,chanel);
}
void pwm(int dutycycle = 100 ,int chanel = 0){
  ledcWrite(chanel,(int) dutycycle*255 /100.0);
  delay(10);
}

float ADCconvert(float min_value,float max_value, int value)
{
  float temp = -1 ;
  temp = (float) ( (value/4095.0) * (max_value - min_value) + min_value);
  temp = temp*100;
  temp = round(temp) / 100;
  return temp;
}

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
/*--------------------------Setup ---------------------------------------*/
void setup() {
  WiFi.config(local_IP,gateway,subnet,primaryDNS,secondaryDNS);
  Serial.begin(115200);
  Serial.println("Start connecting to  Wifi");
  Serial.print(ssid);Serial.print(":");
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED){
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected. My IP address:");Serial.println(WiFi.localIP());

  udp.begin(udpClientPort);
  Serial.print("Listening on UDP port");
  Serial.println(udpClientPort);

  pinMode(LDR,INPUT);
  pinMode(MQ135,INPUT);

  Serial.println("START");
  //char str[100]; 
 current_time = millis();

 pinMode(LED, OUTPUT);
 digitalWrite(LED, HIGH);
}
/*-------------------------------------------------------------------------------------------------*/
void loop() {
  int time_out;
  char string[255];
  int ID = 0;
  value_ldr = 0;
  value_mq135 = 0;
  // Read LDR sensor
  for (int i = 0; i<10; i++) value_ldr += ADCconvert(10, 100,analogRead(LDR));
  value_ldr = value_ldr/10.0;
  Serial.print("Read Sensor LDR:"); 
  Serial.println(value_ldr);
  
  // Read MQ135 sensor
  for (int i = 0; i<10; i++) value_mq135 += ADCconvert(0, 100, analogRead(MQ135));
  value_mq135 =  value_mq135 / 10.0;
  Serial.print("Read Sensor MQ135:"); Serial.print(value_mq135); Serial.println("%");

  if (WiFi.status() != WL_CONNECTED)  connect_WiFi();
  
  // PUT data with CoAP
  Serial.println("(CoAP) PUT data LDR!");
  ID = CoAP_PUT(LDR);

  udp.parsePacket();
  udp.read(data2recei, 255);
  delay(3000);
  
   // PUT data with CoAP
  Serial.println("(CoAP) PUT data MQ135!");
  ID = CoAP_PUT(MQ135)  ;
  udp.parsePacket();
  udp.read(data2recei,255);
  delay(3000);
}
