/*****************************************************************************
GetValues.ino
This example writes a setting value to a holding register, reads it to confirm
the value has changed, and then reads several data values from holding registers.
The register numbers in this example happen to be for an S::CAN oxy::lyser.
*****************************************************************************/

// ---------------------------------------------------------------------------
// Include the base required libraries
// ---------------------------------------------------------------------------
#include <Arduino.h>
#include <SensorModbusMaster.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266mDNS.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <PID_v1.h>
#include <FirebaseESP8266.h>
#include <addons/RTDBHelper.h>
// ---------------------------------------------------------------------------
// Set up the sensor specific information
//   ie, pin locations, addresses, calibrations and related settings
// ---------------------------------------------------------------------------
#define MDNS_HOST_NAME "esp8266-webupdate" // сетевое имя 
#define UPDATE_SERVER_PORT 8080
#define DHTPIN 5 
#define DHTTYPE DHT11

DHT_Unified dht(DHTPIN, DHTTYPE);

// Define the sensor's modbus address
byte modbusAddress = 0x01;   // Invertor's SlaveID
long modbusBaudRate = 9600; // 

const char* ssid = "TP-LINK_2.4GHz";
const char* password = "Vtnfkkjj,hf,jnrf2010";

const int DEREPin = 13;  // Direction pin
const int SSRxPin = 4; // Recieve pin for software serial (Rx on RS485 adapter)
const int SSTxPin = 14; // Send pin for software serial (Tx on RS485 adapter)
SoftwareSerial modbusSerial(SSRxPin, SSTxPin);
// Construct the modbus instance
modbusMaster modbus;

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

double input;
double output;
double setPoint = 30;
PID myPID(&input, &output, &setPoint, 2, 5, 1, DIRECT);


#define DATABASE_URL "confident-slice-343416-default-rtdb.firebaseio.com" //<databaseName>.firebaseio.com or <databaseName>.<region>.firebasedatabase.app

/* 3. Define the Firebase Data object */
FirebaseData fbdo;

/* 4, Define the FirebaseAuth data for authentication data */
FirebaseAuth auth;

/* Define the FirebaseConfig data for config data */
FirebaseConfig config;

unsigned long dataMillis = 0;
int count = 0;
// ---------------------------------------------------------------------------
// Main setup function
// ---------------------------------------------------------------------------
void setup()
{
  // Set various pins as needed
  if (DEREPin >= 0)
  {
    pinMode(DEREPin, OUTPUT);
  }
  
  

  // Turn on the "main" serial port for debugging via USB Serial Monitor
  Serial.begin(115200);
  Serial.println("Booting");
  dht.begin();
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  
  MDNS.begin(MDNS_HOST_NAME);
  httpUpdater.setup(&httpServer);
  httpServer.begin();
  MDNS.addService("http", "tcp", UPDATE_SERVER_PORT);

  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));

  modbusSerial.begin(modbusBaudRate); // Uses 8N1 by default

  // Turn on debugging, if desired
  // modbus.setDebugStream(&Serial);

  // Start the modbus instance
  modbus.begin(modbusAddress, modbusSerial, DEREPin);
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  myPID.SetOutputLimits(100, 500);   
  myPID.SetSampleTime(5000); 
  myPID.SetMode(AUTOMATIC);

  config.database_url = DATABASE_URL;

  config.signer.test_mode = true;

  Firebase.reconnectWiFi(true);
  Firebase.begin(&config, &auth);

}

// ---------------------------------------------------------------------------
// Main setup function
// ---------------------------------------------------------------------------
void loop()
{
  // uint16_t result = 0;

  int reg_addr = 2;


  // result = modbus.uint16FromRegister(read_function_code, reg_addr, bigEndian);


  // Serial.println(result);    
  httpServer.handleClient();
  MDNS.update();

  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    Serial.print(F("Temperature test: "));
    Serial.print(event.temperature);
    Serial.println(F("C"));
    input = event.temperature;
  }
  // // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
  }
  
  myPID.Compute();
  modbus.uint16ToRegister(reg_addr, uint16_t(output), bigEndian);

  delay(2000);
}