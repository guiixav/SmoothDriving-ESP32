//imports para processamento em multitask
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_ipc.h>

//imports para comunicação wifi
#include <WiFi.h>
#include <HTTPClient.h>

//imports para funcionamento sensor MPU
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include "MPU6050.h"

//imports para funcionamento do GPS
#include <SoftwareSerial.h>
#include <TinyGPS.h>

//imports para o funcionamento do sensor LM327 via bluetooth serial
#include "BluetoothSerial.h"
#include "ELMduino.h"

Adafruit_MPU6050 mpu;
MPU6050 accelgyro;


TinyGPS gps;
SoftwareSerial ss;



//parametros para o instanciamento do multi task
TaskHandle_t sensorMPU;
TaskHandle_t sensorObdRPM;
TaskHandle_t sensorGps;

//identificador de viagem
uint32_t randomNumber;


//variaveis para armazenar os dados do GPS
String gpsDataLat = "";
String gpsDataLong = "";
String gpsDataVel = "";
char buff[10];
float flat, flon;
unsigned long age;


//variaveis para armazenar os dados do Acelerometro
const int MPU_addr=0x68; //Endereço do sensor
float AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; //Variaveis para pegar os valores medidos

//ELM327 config com bluetooth serial
BluetoothSerial SerialBT;
#define ELM_PORT   SerialBT
#define DEBUG_PORT Serial
ELM327 myELM327;
float tempRPM ;
int32_t vel ;

// WiFi Network Configuration
const char *ssid = "iPhone de Guilherme"; 
//const char *ssid = "BorbaXavier"; //const char *ssid = "Ian - Ultra";
const char *password = "12022022";//const char *password = "Ian12345";
const char *hostname = "ESP32";
HTTPClient http;

//Helix IP Address
const char *orionAddressPath = "15.228.222.191:1026/v2";

// Device ID (example: urn:ngsi-ld:entity:001)
const char *deviceID = "urn:ngsi-ld:entity:025";


//Variavel que armazena o payload
String data;
//variaveis liga e desliga das sessões do fluxo
bool isOBDon = false;
bool getODBvalues = false;
bool getMPUvalues = true;

void setup() {

  DEBUG_PORT.begin(115200);
  DEBUG_PORT.println("Iniciando");
  setupAdaFruit();
  setupGPS();
  setupWiFi();
  setupOBD();
  randomNumber = esp_random();

  xTaskCreatePinnedToCore(getMpu,
                          "sensorMPU",
                          10000,
                          NULL,
                          0,
                          &sensorMPU,
                          APP_CPU_NUM);


  xTaskCreatePinnedToCore(getObdRPM,
                          "sensorObdRPM",
                          10000,
                          NULL,
                          1,
                          &sensorObdRPM,
                          PRO_CPU_NUM);

  xTaskCreatePinnedToCore(getGPS,
                         "sensorGps",
                          10000,
                          NULL,
                          2,
                          &sensorGps,
                          APP_CPU_NUM);

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

void setupAdaFruit()
{ 
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
     accelgyro.initialize();
     mpu.begin();
  if (mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  delay(1000);
  Serial.print("inicializou mpu");
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  
}

void setupGPS()
{
  ss.begin(9600, SWSERIAL_8N1, 19, 18, false);
  if(!ss){
    Serial.println("Invalid SoftwareSerial pin configuration, check config"); 
    while (1) { 
      delay (1000);
    }
  }
}

void setupOBD(){

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
}

void loop()
{  
      vTaskDelete(NULL);
}


void httpRequest(String path, String data)
{
    
  Serial.println("Iniciou httpRequest");
    String result = makeRequest(path, data);
    Serial.println("##[RESULT]## ==> " + result);
  Serial.println("Terminou httpRequest");
}


String makeRequest(String path, String bodyRequest) 
{
  Serial.println("Iniciou makeRequest");
    String fullAddress = "http://" + String(orionAddressPath) + path;
    
    http.begin(fullAddress);
    //Serial.println("Orion URI request: " + fullAddress);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Accept", "application/json");
    http.addHeader("fiware-service", "helixiot");
    http.addHeader("fiware-servicepath", "/");
    Serial.println(bodyRequest);
    http.POST(bodyRequest);
    Serial.println("HTTP CODE");
    Serial.println("Requisição enviada");
    
    Serial.println("Terminou makeRequest");
  return "OK";
}

// Atualiza dados no Broker orion
void orionUpdate(String entityID, String dados)
{
  
  Serial.println("Iniciou OrionUpdate");
    String pathRequest = "/entities/" + entityID + "/attrs?options=forcedUpdate";
    httpRequest(pathRequest, dados);
  Serial.println("Terminou OrionUpdate");
}


void getObdRPM(void *arg)
{
  for(;;){
  if (isOBDon) {

   float RPM = myELM327.rpm();
   if (myELM327.nb_rx_state == ELM_SUCCESS)
    {
      Serial.println("OBD CONECTADO");
      if(RPM != 0)
      {
        tempRPM = (uint32_t)RPM;
      }
      Serial.print("RPM: "); Serial.println(tempRPM);
      delay(1);
    }
    delay(1);
  }delay(1);
}delay(1);
}
  
void getMpu( void * arg)
{
   
  for(;;){ 
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    Serial.print("Acceleration X: ");
    AcX = a.acceleration.x;
    Serial.print(AcX);
    Serial.print(", Y: ");
    AcY = a.acceleration.y;
    Serial.print(AcY);
    Serial.print(", Z: ");
    AcZ = a.acceleration.z;
    Serial.print(AcZ);
    Serial.println(" m/s^2");

    Serial.print("Rotation X: ");
    GyX = g.gyro.x;
    Serial.print(GyX);
    Serial.print(", Y: ");
    GyY = g.gyro.y;
    Serial.print(GyY);
    Serial.print(", Z: ");
    GyZ = g.gyro.z;
    Serial.print(GyZ);
    Serial.println(" rad/s");
    sendToBroker();
  }
    
    delay(1);
  }
  


void sendToBroker()
{ 
  Serial.println("Iniciou SendToBroker");
  data = "{\"EixoXAcelerometro\": { \"value\": \"" +String(AcX)+ "\", \"type\": \"float\"}," +
         "\"EixoYAcelerometro\": { \"value\": \"" +String(AcY)+ "\", \"type\": \"float\"}," + 
         "\"EixoZAcelerometro\": { \"value\": \"" +String(AcZ)+ "\", \"type\": \"float\"}," +
         "\"EixoXGiroscopio\": { \"value\": \"" +String(GyX)+ "\", \"type\": \"float\"}," +
         "\"EixoYGiroscopio\": { \"value\": \"" +String(GyY)+ "\", \"type\": \"float\"}," +
         "\"EixoZGiroscopio\": { \"value\": \"" +String(GyZ)+ "\", \"type\": \"float\"}," +
         "\"Latitude\": { \"value\": \"" +gpsDataLat+ "\", \"type\": \"float\"}," +
         "\"Longitude\": { \"value\": \"" + gpsDataLong + "\", \"type\": \"float\"}," +
         "\"Velocidade\": { \"value\": \"" + gpsDataVel + "\", \"type\": \"float\"}," +
         "\"RPMveiculo\": { \"value\": \"" +String(tempRPM)+ "\", \"type\": \"float\"},"
         "\"IdViagem\": { \"value\": \"" + randomNumber + "\", \"type\": \"float\"}}";
   orionUpdate( deviceID,  data);
   Serial.println(data);
   
  Serial.println("Terminou SendToBroker");
 }

 void getGPS(void * arg)
 {
    for(;;){
        gps.f_get_position(&flat, &flon, &age);
        gpsDataLat = floatToString(flat,TinyGPS::GPS_INVALID_F_ANGLE, 10, 6);
        gpsDataLong = floatToString(flon,TinyGPS::GPS_INVALID_F_ANGLE, 11, 6);
        gpsDataVel = gps.f_speed_kmph();
        if(gpsDataLat.indexOf("v") > 0){
      //gpsData = "lat=7.207573&lng=125.395874";
      Serial.println(gpsDataLat + " " + gpsDataLong + " " + gpsDataVel);
  };
  Serial.println(gpsDataLat + " " + gpsDataLong + " " + gpsDataVel);
  smartdelay(1);
    }
 }

 
//Função para conversão de float para string
String floatToString(float val, float invalid, int len, int prec) {
  String out = "";
  if (val == invalid) {
    while (len-- > 1){
      return "inv" ;
    }
  }
  else{
    for (int i = 0; i < 10; i++) {
       dtostrf(val, len, prec, buff);  
       out += buff;
       return out;
    }
  }
}
static void smartdelay(unsigned long ms)
{
  delay(500);
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}