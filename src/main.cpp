#define TINY_GSM_MODEM_SIM7600

// GSM modem configuration
#define SerialMon Serial
#define SerialAT Serial1

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

// Function prototypes
float getLightIntensity();
void sendData(float temp, float hum, float light, float current, float voltage, float latitude, float longitude, float speed, float altitude);
bool getGPSData(float &latitude, float &longitude, float &speed, float &altitude);

// 🔧 WiFi Credentials
const char* myFi_SSID = "MyFi 12.5cm";
const char* myFi_Password = "igel-am-pfaffenberg-87";

// 🌐 Server URL (Render Flask API)
const char* serverURL = "https://gps-ledu.onrender.com/api/data";

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);

// 🌡️ Sensor Objects
Adafruit_BME280 bme;
Adafruit_INA219 ina219;
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
MPU6050 mpu;

void setup() {
    SerialMon.begin(SerialMonSpeed);  // Start serial monitor
    SerialAT.begin(SerialATSpeed);    // Start serial AT for GSM modem

    SerialMon.println("Initializing...");

    // Initialize WiFi
    WiFi.begin(myFi_SSID, myFi_Password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        SerialMon.print(".");
    }
    SerialMon.println("\nWiFi connected!");

    Wire.begin();

    // Initialize sensors
    if (!bme.begin(0x76)) SerialMon.println("BME280 not found!");
    if (!ina219.begin()) SerialMon.println("INA219 not found!");
    
    mpu.initialize();
    if (!mpu.testConnection()) {
        SerialMon.println("MPU6050 not connected!");
    } else {
        SerialMon.println("MPU6050 connected!");
    }

    if (!tsl.begin()) SerialMon.println("TSL2561 not found!");
    tsl.enableAutoRange(true);
    tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);

    // Initialize the modem
    modem.restart();
    modem.enableGPS(); // Enable GPS (if supported by your modem)
}

void loop() {
    // Ensure Wi-Fi connection
    if (WiFi.status() != WL_CONNECTED) {
        SerialMon.println("WiFi Disconnected! Reconnecting...");
        WiFi.begin(myFi_SSID, myFi_Password);
        while (WiFi.status() != WL_CONNECTED) {
            delay(1000);
            SerialMon.print(".");
        }
        SerialMon.println("\nReconnected!");
    }

    // Read sensor data
    float temp = bme.readTemperature();
    float hum = bme.readHumidity();
    float light = getLightIntensity();
    float current = ina219.getCurrent_mA();
    float voltage = ina219.getBusVoltage_V();

    // Read GPS data
    float latitude, longitude, speed, altitude;
    bool gpsFix = getGPSData(latitude, longitude, speed, altitude);
    
    if (gpsFix) {
        SerialMon.println("GPS Fix Acquired!");
    } else {
        SerialMon.println("No GPS fix, sending last known values...");
    }

    // Send data to the server
    sendData(temp, hum, light, current, voltage, latitude, longitude, speed, altitude);

    delay(60000);  // Send every 60 seconds
}

// 📡 Get GPS Data from SIM7600
bool getGPSData(float &latitude, float &longitude, float &speed, float &altitude) {
    modem.sendAT("+CGNSINF");
    if (modem.waitResponse(1000L) == 1) {
        String gpsData = modem.stream.readStringUntil('\n');
        int fixStatus = gpsData.substring(18, 19).toInt();  // Extract fix status

        if (fixStatus == 1) {
            latitude = gpsData.substring(20, 30).toFloat();
            longitude = gpsData.substring(31, 41).toFloat();
            speed = gpsData.substring(42, 47).toFloat();
            altitude = gpsData.substring(48, 53).toFloat();
            return true;
        }
    }
    return false;
}

// 💡 Get Light Intensity
float getLightIntensity() {
    uint16_t broadband, infrared;
    tsl.getLuminosity(&broadband, &infrared);
    return (float)broadband;
}

// 📤 Send Data to Server
void sendData(float temp, float hum, float light, float current, float voltage, float latitude, float longitude, float speed, float altitude) {
    HTTPClient http;
    http.begin(serverURL);
    http.addHeader("Content-Type", "application/json");

    String jsonPayload = "{";
    jsonPayload += "\"temp\":" + String(temp) + ",";
    jsonPayload += "\"hum\":" + String(hum) + ",";
    jsonPayload += "\"light\":" + String(light) + ",";
    jsonPayload += "\"current\":" + String(current) + ",";
    jsonPayload += "\"voltage\":" + String(voltage) + ",";
    jsonPayload += "\"latitude\":" + String(latitude, 6) + ",";
    jsonPayload += "\"longitude\":" + String(longitude, 6) + ",";
    jsonPayload += "\"speed\":" + String(speed) + ",";
    jsonPayload += "\"altitude\":" + String(altitude);
    jsonPayload += "}";

    SerialMon.println("Sending: " + jsonPayload);
    int httpResponseCode = http.POST(jsonPayload);
    SerialMon.println("Response: " + String(httpResponseCode));

    http.end();
}

