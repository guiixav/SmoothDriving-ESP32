//imports para processamento em multitask
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_ipc.h>

//imports para comunicação wifi
#include <WiFi.h>
#include <HTTPClient.h>

//imports para funcionamento sensor MPU
#include <Wire.h>
#include <HardwareSerial.h>

//imports para o funcionamento do sensor LM327 via bluetooth serial
#include "BluetoothSerial.h"
#include "ELMduino.h"


TaskHandle_t sensorMPU;
TaskHandle_t sensorObd;
SemaphoreHandle_t Semaphore;

//identificador de viagem
uint32_t randomNumber;

//Acelerometro config
const int MPU_addr=0x68; //Endereço do sensor
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; //Variaveis para pegar os valores medidos


//ELM327 config com bluetooth serial
BluetoothSerial SerialBT;
#define ELM_PORT   SerialBT
#define DEBUG_PORT Serial
ELM327 myELM327;
float tempRPM ;
int32_t vel ;

// WiFi Network Configuration
const char *ssid = "iPhone de Guilherme"; //const char *ssid = "BorbaXavier"; //const char *ssid = "Ian - Ultra";
const char *password = "12022022";//const char *password = "Ian12345";
const char *hostname = "ESP32";
HTTPClient http;

//Helix IP Address
const char *orionAddressPath = "15.228.222.191:1026/v2";

// Device ID (example: urn:ngsi-ld:entity:001)
const char *deviceID = "urn:ngsi-ld:entity:022";


//Variavel que armazena o payload
String data;
//variaveis liga e desliga das sessões do fluxo
bool isOBDon = true;
bool getODBvalues = true;
bool getMPUvalues = true;

void setup() {

  DEBUG_PORT.begin(115200);
  DEBUG_PORT.println("Iniciando");
  randomNumber = esp_random();
  
  Semaphore = xSemaphoreCreateMutex();

  
  setupWiFi();
  // BEGIN MPU SETUP
  if (getMPUvalues == true) {
    Wire.begin(); //Inicia a comunicação I2C
    Wire.beginTransmission(MPU_addr); //Começa a transmissao de dados para o sensor
    Wire.write(0x6B); // registrador PWR_MGMT_1
    Wire.write(0); // Manda 0 e "acorda" o MPU 6050
    Wire.endTransmission(true);
  }
  // END MPU SETUP

  // BEGIN OBDII SETUP
  if (getODBvalues) {
    ELM_PORT.begin("ArduHUD", true);
    if (!ELM_PORT.connect("OBDII"))
    {
        DEBUG_PORT.println("Couldn't connect to OBD scanner - Phase 1");
        while(1);
    }

    if (!myELM327.begin(ELM_PORT, true, 2000))
    {
        Serial.println("Couldn't connect to OBD scanner - Phase 2");
        while (1);
    }

    Serial.println("Connected to ELM327");
    isOBDon = true;
  }
  // END OBDII SETUP

  xTaskCreatePinnedToCore(getMpu,
                          "sensorMPU",
                          10000,
                          NULL,
                          0,
                          &sensorMPU,
                          APP_CPU_NUM);


  xTaskCreatePinnedToCore(getObd,
                          "sensorOBD",
                          10000,
                          NULL,
                          1,
                          &sensorObd,
                          PRO_CPU_NUM);

}

void setupWiFi()
{
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(hostname);
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi ..");
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print('.');
        delay(50);
    }
    Serial.print("Wifi.LocalIP()");
}

void loop()
{  
      vTaskDelete(NULL);
}


void httpRequest(String path, String data)
{
    String result = makeRequest(path, data);
    Serial.println("##[RESULT]## ==> " + result);
}


String makeRequest(String path, String bodyRequest)
{
    String fullAddress = "http://" + String(orionAddressPath) + path;
    http.begin(fullAddress);
    //Serial.println("Orion URI request: " + fullAddress);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Accept", "application/json");
    http.addHeader("fiware-service", "helixiot");
    http.addHeader("fiware-servicepath", "/");
    Serial.println(bodyRequest);
    int httpCode = http.POST(bodyRequest);
    Serial.println("HTTP CODE");
    Serial.println(httpCode);
    return "OK";
   
    http.end();
}

// Update Values in the Helix Sandbox
void orionUpdate(String entityID, String dados)
{
    String pathRequest = "/entities/" + entityID + "/attrs?options=forcedUpdate";
    httpRequest(pathRequest, dados);
}


void getObd(void *arg)
{
  for(;;){
  if (isOBDon) {

   float RPM = myELM327.rpm();
   //float veloc = myELM327.kph(); 
   if (myELM327.nb_rx_state == ELM_SUCCESS)
    {
      Serial.println("OBD CONECTADO");
      tempRPM = (uint32_t)RPM;
      //vel = (uint32_t)veloc;
      
     // Serial.print("VELOCIDADE: "); Serial.println(vel);
      Serial.print("RPM: "); Serial.println(tempRPM);
      delay(1);
    }
    delay(1);
  }
}
}


void getMpu( void * arg)
{
   
  for(;;){ 
   //MPU6050  
  if (getMPUvalues) {
    DEBUG_PORT.println("MPU CONECTADO");
    Wire.beginTransmission(MPU_addr); //Começa a transmissao de dados para o sensor
    Wire.write(0x3B); // registrador dos dados medidos (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14); // faz um "pedido" para ler 14 registradores, que serão os registrados com os dados medidos
    DEBUG_PORT.println("Coletando dados MPU");
    AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    DEBUG_PORT.print("AcX: "); DEBUG_PORT.println(AcX);
    DEBUG_PORT.print("AcY: "); DEBUG_PORT.println(AcY);
    DEBUG_PORT.print("AcZ: "); DEBUG_PORT.println(AcZ);
    DEBUG_PORT.print("Tmp: "); DEBUG_PORT.println(Tmp);
    DEBUG_PORT.print("GyX: "); DEBUG_PORT.println(GyX);
    DEBUG_PORT.print("GyY: "); DEBUG_PORT.println(GyY);
    DEBUG_PORT.print("GyZ: "); DEBUG_PORT.println(GyZ);
    DEBUG_PORT.println("DADOS DE ACELEROMETRO COLETADOS!");
    }
    sendBroker();
    delay(1);
  }
  
}

void sendBroker()
{
  data = "{\"EixoXAcelerometro\": { \"value\": \"" +String(AcX)+ "\", \"type\": \"float\"}," +
         "\"EixoYAcelerometro\": { \"value\": \"" +String(AcY)+ "\", \"type\": \"float\"}," + 
         "\"EixoZAcelerometro\": { \"value\": \"" +String(AcZ)+ "\", \"type\": \"float\"}," +
         "\"EixoXGiroscopio\": { \"value\": \"" +String(GyX)+ "\", \"type\": \"float\"}," +
         "\"EixoYGiroscopio\": { \"value\": \"" +String(GyY)+ "\", \"type\": \"float\"}," +
         "\"EixoZGiroscopio\": { \"value\": \"" +String(GyZ)+ "\", \"type\": \"float\"}," +
         "\"VelocidadeVeiculo\": { \"value\": \"" + String(vel) + "\", \"type\": \"float\"}," +
         "\"RPMveiculo\": { \"value\": \"" +String(tempRPM)+ "\", \"type\": \"float\"},"
         "\"IdViagem\": { \"value\": \"" + randomNumber + "\", \"type\": \"float\"}}";
   orionUpdate( deviceID,  data);
 }

 
