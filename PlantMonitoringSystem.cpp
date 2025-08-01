// Arduino to Server HTTP Communication 
// Humidity measurement and HTTP transmission
// Motor control via HTTP

//  Libraries
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>
#include <Stepper.h>
#include "WiFiEsp.h"


// Motor Configuration

const int MOTOR_PIN1 = 4;  // IN1 connected to pin 4
const int MOTOR_PIN2 = 5;  // IN2 connected to pin 5
const int MOTOR_PIN3 = 6;  // IN3 connected to pin 6
const int MOTOR_PIN4 = 7;  // IN4 connected to pin 7

// Initialize stepper motor with 2048 steps per revolution
Stepper moistureMotor(2048, MOTOR_PIN1, MOTOR_PIN3, MOTOR_PIN2, MOTOR_PIN4);

const int FULL_ROTATION = 2048;  // 2048 steps = 360 degree rotation


//Humidity Sensor Configuration

const int SENSOR_PIN = A0;       // Analog pin for moisture sensor
const int DRY_VALUE = 250;       // Minimum value for dry condition
const int WET_VALUE = 530;       // Maximum value for wet condition


//WiFi Configuration

unsigned long lastConnectionTime = 0;         // Last server connection time
const unsigned long INTERVAL_DELAY = 5000L;   // Delay between transmissions
const int WIFI_INIT_TIME = 2000;             // WiFi initialization time

String responseBuffer;
bool connectionStatus = false;

// Server configuration
IPAddress serverIp(192, 168, 4, 1);
const int SERVER_PORT = 80;

#ifndef HAVE_HWSERIAL188
#include "SoftwareSerial.h"
SoftwareSerial Serial1(2, 3); // RX, TX
#endif

const char* NETWORK_SSID = "Metacamp";         // WiFi network name
const char* NETWORK_PASS = "Metacamp0501";     // WiFi password
int wifiStatus = WL_IDLE_STATUS;               // WiFi connection status

const char* SERVER_ADDRESS = "192.168.0.72";   // Backend server address

// Initialize WiFi client
WiFiEspClient client;


//Setup Function

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  // Initialize ESP module serial
  Serial1.begin(9600);
  
  // Set motor speed to 13 RPM
  moistureMotor.setSpeed(13);

  // Initialize WiFi module
  WiFi.init(&Serial1);
  
  // Check for WiFi shield presence
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not detected");
    while (true); // Halt if no shield found
  }

  // Attempt WiFi connection
  while (wifiStatus != WL_CONNECTED) {
    Serial.print("Connecting to network: ");
    Serial.println(NETWORK_SSID);
    wifiStatus = WiFi.begin(NETWORK_SSID, NETWORK_PASS);
  }

  Serial.println("Network connection established");
  Serial.println("Initializing server connection...");
}


//Main Loop

void loop() {
  
  //Send Humidity Data (HTTP POST)
  
  int sensorValue = analogRead(SENSOR_PIN);
  Serial.println("Reading sensor data...");
  
  // Convert analog reading to percentage (0-100%)
  int humidityPercent = map(sensorValue, DRY_VALUE, WET_VALUE, 100, 0);
  
  if (client.connect(SERVER_ADDRESS, 3001)) {
    Serial.println("Establishing server connection...");
    
    // Create JSON payload
    String jsonPayload = "";
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& dataObject = jsonBuffer.createObject();
    dataObject["humidity"] = humidityPercent;
    dataObject.printTo(jsonPayload);
    
    Serial.print("JSON payload: ");
    Serial.println(jsonPayload);

    // Send HTTP POST request
    client.println("POST /arduino/humidity/1 HTTP/1.1");
    client.println("Cache-Control: no-cache");
    client.println("Host: 192.168.4.1:80");
    client.println("User-Agent: Arduino");
    client.println("Content-Type: application/json");
    client.print("Content-Length: ");
    client.println(jsonPayload.length());
    client.println();
    client.println(jsonPayload);
    
    lastConnectionTime = millis();
    connectionStatus = true;

    // Read server response
    String response = client.readStringUntil('\r');
    Serial.print("Server response: ");
    Serial.println(response);
  }
  else {
    Serial.println("Connection attempt failed");
    connectionStatus = false;
  }
  
  client.flush();
  client.stop();

  Serial.println("===========================================");

  
  //Check Motor Status (HTTP GET)
  
  if (client.connect(SERVER_ADDRESS, 3001)) {
    Serial.println("Connecting to server...");
 
    // Send HTTP GET request
    client.println("GET /arduino/motor/1 HTTP/1.1");
    client.println("Cache-Control: no-cache");
    client.println("Host: 192.168.4.1:80");
    client.println("User-Agent: Arduino");
    client.println();
    
    lastConnectionTime = millis();
    connectionStatus = true;

    // Read server response
    String motorResponse = client.readStringUntil('\r');
    Serial.print("Motor control response: ");
    Serial.println(motorResponse);

    // Extract status code
    String statusCode = motorResponse.substring(9,12);
    Serial.print("Status code: ");
    Serial.println(statusCode);

    // Activate motor if approved
    if (statusCode == "200") {
      moistureMotor.step(FULL_ROTATION);    // Clockwise rotation
      delay(1000);
      moistureMotor.step(-FULL_ROTATION);   // Counter-clockwise rotation
      delay(1000);
    }
  }
  else {
    Serial.println("Connection attempt failed");
    connectionStatus = false;
  }
  
  client.flush();
  client.stop();

  Serial.println("===========================================");
  
  // Delay before next cycle
  delay(10000);
}