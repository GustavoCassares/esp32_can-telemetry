#include <ESP32CAN.h>
#include <CAN_config.h>
#include <WiFi.h>
#include <PubSubClient.h> //Biblioteca para as publicações via mqtt
#include <Arduino.h>



/* the variable name CAN_cfg is fixed, do not change */
CAN_device_t CAN_cfg;

#define WIFISSID "CAVERNA_DOS_OGROS" //Coloque seu SSID de WiFi aqui
#define PASSWORD "WMRG2002" //Coloque seu password de WiFi aqui
#define CLIENT_ID "MR21" //ID do dispositivo (Client Id, também chamado de client name)
#define SERVER "192.168.15.10" //Servidor (broker)
#define PORT 1883
#define USER "PUBLIC"
#define PASS "public"
#define TOPIC "fsae/mr/rx"

WiFiClient ESP32_Telemetry;
PubSubClient client(ESP32_Telemetry);








//Engine variables declaration:
int engineRpm;
float throttlePosition;
float intakeManifoldPressure;
float intakeManifoldTemperature;
float engineCoolantTemperature;
float lambda1;
float lambda2;
float exhaustManifoldTemperature;



bool mqttInit(){
  WiFi.begin(WIFISSID, PASSWORD);
 
  //Loop até que o WiFi esteja conectado
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Establishing connection to WiFi..");
  }
 
  //Exibe no monitor serial
  Serial.println("Connected to network");

//Seta servidor com o broker e a porta:
  client.setServer(SERVER, PORT);
  
  //Conecta no server com o Client id, o password é informado como vazio:
  while(!client.connect(CLIENT_ID,USER,PASS))
  {
      Serial.println("MQTT - Connect error");
      return false;
  }

  Serial.println("MQTT - Connect ok");
  return true;
  

}









void setup() {
  
    Serial.begin(115200);
    Serial.println("Void Setup");


    //Iniatilize Wifi & mqtt server:
  if(!mqttInit())
  {        
   delay(1000);

    Serial.println("Failed!");
    ESP.restart();
  }

  Serial.println("OK");



    /* set CAN pins and baudrate */
    CAN_cfg.speed=CAN_SPEED_500KBPS;
    CAN_cfg.tx_pin_id = GPIO_NUM_5;
    CAN_cfg.rx_pin_id = GPIO_NUM_4;
    /* create a queue for CAN receiving */
    CAN_cfg.rx_queue = xQueueCreate(10,sizeof(CAN_frame_t));
    //initialize CAN Module
    ESP32Can.CANInit();
}

void loop (){
  getData();
  sendData();
}








void getData() {
  
CAN_frame_t __RX_frame;

   
 //receive next CAN frame from queue
if (xQueueReceive(CAN_cfg.rx_queue,&__RX_frame, 3*portTICK_PERIOD_MS)==pdTRUE){
  switch (__RX_frame.MsgID)
  {
case 0x520L:
 {

 printf("Frame from : 0x%08x, DLC %d \n", __RX_frame.MsgID, __RX_frame.FIR.B.DLC);
 printf("D0: 0x%02x, ", __RX_frame.data.u8[0]);
 printf("D1: 0x%02x, ", __RX_frame.data.u8[1]);
 printf("D2: 0x%02x, ", __RX_frame.data.u8[2]);
 printf("D3: 0x%02x, ", __RX_frame.data.u8[3]);
 printf("D4: 0x%02x, ", __RX_frame.data.u8[4]);
 printf("D5: 0x%02x, ", __RX_frame.data.u8[5]);
 printf("D6: 0x%02x, ", __RX_frame.data.u8[6]);
 printf("D7: 0x%02x\n", __RX_frame.data.u8[7]);
 engineRpm=(((__RX_frame.data.u8[1]*256)+__RX_frame.data.u8[0])/6.0);
 throttlePosition=(((__RX_frame.data.u8[3]*256)+__RX_frame.data.u8[2])/10.0);
 intakeManifoldPressure=(((__RX_frame.data.u8[5]*256)+__RX_frame.data.u8[4])/10.0);
 intakeManifoldTemperature=(((__RX_frame.data.u8[7]*256)+__RX_frame.data.u8[6])/10.0);
 Serial.print("engineRpm=");
 Serial.println(engineRpm);
 Serial.print("throttlePosition=");
 Serial.println(throttlePosition);
 Serial.print("intakeManifoldPressure=");
 Serial.println(intakeManifoldPressure);
 Serial.print("intakeManifoldTemperature=");
 Serial.println(intakeManifoldTemperature);
 printf("==============================================================================\n");
 }
 break;
 
 case 0x521L:
{

   printf("Frame from : 0x%08x, DLC %d \n", __RX_frame.MsgID, __RX_frame.FIR.B.DLC);
 printf("D0: 0x%02x, ", __RX_frame.data.u8[0]);
 printf("D1: 0x%02x, ", __RX_frame.data.u8[1]);
 printf("D2: 0x%02x, ", __RX_frame.data.u8[2]);
 printf("D3: 0x%02x, ", __RX_frame.data.u8[3]);
 printf("D4: 0x%02x, ", __RX_frame.data.u8[4]);
 printf("D5: 0x%02x, ", __RX_frame.data.u8[5]);
 printf("D6: 0x%02x, ", __RX_frame.data.u8[6]);
 printf("D7: 0x%02x\n", __RX_frame.data.u8[7]);
 engineCoolantTemperature=(((__RX_frame.data.u8[1]*256)+__RX_frame.data.u8[0])/10.0);
 lambda1=(((__RX_frame.data.u8[3]*256)+__RX_frame.data.u8[2])/1000.000);
 lambda2=(((__RX_frame.data.u8[5]*256)+__RX_frame.data.u8[4])/1000.000);
 exhaustManifoldTemperature=(((__RX_frame.data.u8[7]*256)+__RX_frame.data.u8[6])/10.0);
 Serial.print("engineCoolantTemperature=");
 Serial.println(engineCoolantTemperature);
 Serial.print("lambda1=");
 Serial.println(lambda1,3);
 Serial.print("lambda2=");
 Serial.println(lambda2,3);
 Serial.print("Exhaust Manifold Temperature=");
 Serial.println(exhaustManifoldTemperature);
 printf("==============================================================================\n");
}
break;

default:
Serial.println("NoID");
break;

}
  
}
//Serial.println("No message on CAN BUS");
}







void sendData(){
if(!client.connected())
    reconnect();

    delay(10);

 if(sendValues(engineRpm, throttlePosition, intakeManifoldPressure, intakeManifoldTemperature, engineCoolantTemperature, lambda1, lambda2, exhaustManifoldTemperature))
  {      
    Serial.println("Successfully send data");

  }
  else
  {      
    Serial.println("Failed to send data");

  }    

    
  
  
}












void reconnect(){

//Loop até que o MQTT esteja conectado
  while (!client.connected()) 
  {
    //sinaliza desconexão do mqtt no display
  
    Serial.println("Attempting MQTT connection...");
    
    //Tenta conectar
    if (client.connect(CLIENT_ID)) 
      Serial.println("connected");
    else 
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      //Aguarda 2 segundos antes de retomar
      delay(2000);
    }
  }




}








bool sendValues (int engineRpm,float throttlePosition,float intakeManifoldPressure,float intakeManifoldTemperature,float engineCoolantTemperature,float lambda1,float lambda2,float exhaustManifoldTemperature)
{
char json[250];
sprintf(json,  "{\"engineRpm\":%02.02d, \"throttlePosition\":%02.02f, \"intakeManifoldPressure\":%02.02f, \"intakeManifoldTemperature\":%02.02f, \"engineCoolantTemperature\":%02.02f, \"lambda1\":%02.02f, \"lambda2\":%02.02f, \"exhaustManifoldTemperature\":%02.02f}", engineRpm, throttlePosition, intakeManifoldPressure, intakeManifoldTemperature, engineCoolantTemperature, lambda1, lambda2,exhaustManifoldTemperature); 
  
  if(!client.publish(TOPIC, json))
    return false;
  else
    return true;
}


 
