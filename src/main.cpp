#define TINY_GSM_MODEM_SIM7600
// GSM modem configuration
#define SerialMon Serial
#define SerialAT  Serial1

#define SerialMonSpeed 115200
#define SerialATSpeed 9600

#include <TinyGsmClient.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_INA219.h>
#include <Adafruit_TSL2561_U.h>
#include <MPU6050.h>
#include <HTTPClient.h>

float getLightIntensity();
void sendData(float temp, float hum, float light, float current, float voltage);

// 🔧 WiFi Credentials (Replace with your WiFi SSID & Password)
const char* myFi_SSID = "MyFi 12.5cm";
const char* myFi_Password = "igel-am-pfaffenberg-87";

// 🌐 Local Server URL (Your Flask server running on your PC or Raspberry Pi)
const char* serverURL = "http://192.168.1.100:5000/api/data";


TinyGsm modem(SerialAT);    // Create modem instance
TinyGsmClient client(modem); // Create client instance

// 🌡️ Sensor Objects
Adafruit_BME280 bme;
Adafruit_INA219 ina219;
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
MPU6050 mpu;

void setup() {
    SerialMon.begin(SerialMonSpeed);  // Start serial monitor
    SerialAT.begin(SerialATSpeed);    // Start serial AT for GSM modem

    Serial.println("Initializing...");
    
    // Initialize WiFi
    WiFi.begin(myFi_SSID, myFi_Password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("WiFi connected!");

    Wire.begin();

    // Initialize sensors
    if (!bme.begin(0x76)) Serial.println("BME280 not found!");
    if (!ina219.begin()) Serial.println("INA219 not found!");
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 not connected!");
    } else {
        Serial.println("MPU6050 connected!");
    }
    if (!tsl.begin()) Serial.println("TSL2561 not found!");

    tsl.enableAutoRange(true);
    tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
    
    // Initialize the modem
    modem.restart();
    modem.enableGPS(); // Enable GPS (if supported by your modem)
}

void loop() {
    // Check Wi-Fi connection before sending data
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi Disconnected! Reconnecting...");
        WiFi.begin(myFi_SSID, myFi_Password);
        while (WiFi.status() != WL_CONNECTED) {
            delay(1000);
            Serial.print(".");
        }
        Serial.println("Reconnected!");
    }

    // Read sensor data
    float temp = bme.readTemperature();
    float hum = bme.readHumidity();
    float light = getLightIntensity();
    float current = ina219.getCurrent_mA();
    float voltage = ina219.getBusVoltage_V();

    // Send data to the server
    sendData(temp, hum, light, current, voltage);

    delay(60000);  // Send every 60 seconds
}

float getLightIntensity() {
    uint16_t broadband, infrared;
    tsl.getLuminosity(&broadband, &infrared);
    return (float)broadband;
}

void sendData(float temp, float hum, float light, float current, float voltage) {
    HTTPClient http;
    http.begin(serverURL);
    http.addHeader("Content-Type", "application/json");

    String jsonPayload = "{";
    jsonPayload += "\"temp\":" + String(temp) + ",";
    jsonPayload += "\"hum\":" + String(hum) + ",";
    jsonPayload += "\"light\":" + String(light) + ",";
    jsonPayload += "\"current\":" + String(current) + ",";
    jsonPayload += "\"voltage\":" + String(voltage);
    jsonPayload += "}";

    Serial.println("Sending: " + jsonPayload);
    int httpResponseCode = http.POST(jsonPayload);
    Serial.println("Response: " + String(httpResponseCode));

    http.end();
}
