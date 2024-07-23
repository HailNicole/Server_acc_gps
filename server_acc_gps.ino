#include <TinyGPSPlus.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_SPIDevice.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <WiFi.h>   // ESP32

#define WIFI_SSID "ALEXANDER_CNT.FIBRA"
#define WIFI_PASSWORD "nicolemadeline"
#define RXD2 16
#define TXD2 17
#define NMEA 0

char   datoCmd = 0;
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
HardwareSerial neogps(1);

WiFiClient client;
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char* server = "192.168.1.8"; //Ip de la máquina que está corriendo el servidor
const int port = 8000;
const int sendingInternval = 2 * 1000;

void setup() {
  Serial.begin(115200);

  Serial.println("Connecting");

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println("\r\nWiFi connected");


  // Inicialización del sensor MPU6050
  if (!mpu.begin()) {
    Serial.println("No se pudo encontrar MPU6050");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  //Inicialización del GPS NEO-6M
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
}

void loop() {

  // Leer datos del MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax=abs(a.acceleration.x);
  float ay=abs(a.acceleration.y);
  float az=abs(a.acceleration.z);

  /*x = random(0,50); // ALEATORIOS
  y = random(0,100); // ALEATORIOS
  z = random(0,100); // ALEATORIOS*/

    // Detectar cambios bruscos en la aceleración
  if (abs(a.acceleration.x) > 3 || abs(a.acceleration.y) > 3 || abs(a.acceleration.z) > 20) {
    Serial.println(a.acceleration.x);
    Serial.println(a.acceleration.y);
    Serial.println(a.acceleration.z);
    Serial.println("----------------");
    Serial.println(abs(a.acceleration.x));
    Serial.println(abs(a.acceleration.y));
    Serial.println(abs(a.acceleration.z));
    Serial.println("----------------");
    Serial.println("Bache detectado");

    // Obtener datos GPS
    while (neogps.available() > 0) {
      gps.encode(neogps.read());
      Serial.println("Recibiendo datos por serial");
    }

    if (gps.location.isUpdated()) {
      Visualizacion_GPS();
    }

    enviarDatosServer(ax, ay, az);
  }
}

void enviarDatosServer(float ax, float ay, float az){
  if (client.connect(server, port)) {
    // Crear la cadena de solicitud HTTP
    String req_uri = "/update?x=" + String(ax, 2) + "&y=" + String(ay, 2) + "&z=" + String(az, 2);
    client.print("GET " + req_uri + " HTTP/1.1\r\n" + "Host: " + server + "\r\n" + "Connection: close\r\n\r\n");
    Serial.printf("Datos enviados - X: %s, Y: %s, Z: %s\n", String(ax, 2).c_str(), String(ay, 2).c_str(), String(az, 2).c_str());
  } else {
    Serial.println("Error al conectar con el servidor");
  }
  client.stop();
  delay(sendingInternval);
}

void Visualizacion_GPS(){ 
   
  if (gps.location.isValid() ==  1)
  {  
    Serial.print("Lat: ");
    Serial.println(gps.location.lat(),6);
    Serial.print("Lng: ");
    Serial.println(gps.location.lng(),6);  
    Serial.print("Speed: ");
    Serial.println(gps.speed.kmph());    
    Serial.print("SAT:");
    Serial.println(gps.satellites.value());
    Serial.print("ALT:");   
    Serial.println(gps.altitude.meters(), 0);     

    Serial.print("Date: ");
    Serial.print(gps.date.day()); Serial.print("/");
    Serial.print(gps.date.month()); Serial.print("/");
    Serial.println(gps.date.year());

    Serial.print("Hour: ");
    Serial.print(gps.time.hour()); Serial.print(":");
    Serial.print(gps.time.minute()); Serial.print(":");
    Serial.println(gps.time.second());
    Serial.println("---------------------------");
  }
  else
  {
    Serial.println("Sin señal gps");  
  }  
}

//Esta parte del codigo se debe probar...
void validarGPS(){
    // This sketch displays information every time a new sentence is correctly encoded.
  if (NMEA){
    while (neogps.available())
    {
     datoCmd  = (char)neogps.read(); 
     Serial.print (datoCmd );
    }
 } else{
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (neogps.available())
    {
      if (gps.encode(neogps.read()))
      {
        newData = true;         
      }
    }
  }

  if(newData == true){
    newData = false;
    Serial.println(gps.satellites.value());    
    Visualizacion_GPS();
  }
  else{
    
  }  
  
 }  
}