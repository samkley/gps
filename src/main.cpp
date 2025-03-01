#define TINY_GSM_MODEM_SIM7600

// GSM modem configuration
#define SerialMon Serial
#define SerialAT Serial1

#define SerialMonSpeed 115200
#define SerialATSpeed 9600

#include <TinyGsmClient.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_INA219.h>
#include <Adafruit_TSL2561_U.h>
#include <MPU6050.h>

// Function prototypes
float getLightIntensity();
void sendData(float temp, float hum, float light, float current, float voltage, float latitude, float longitude, float speed, float altitude);
bool getGPSData(float &latitude, float &longitude, float &speed, float &altitude);

// SIM7600 configuration - UPDATE THESE WITH YOUR CARRIER'S SETTINGS
const char* apn = "internet";  // Replace with your provider's APN
const char* gprsUser = "";     // Leave empty if not needed
const char* gprsPass = "";     // Leave empty if not needed
const char* pin = "0402";      // SIM PIN if needed, or leave empty

// Server configuration - UPDATE WITH YOUR SERVER'S PUBLIC IP/DOMAIN
const char* server = "gps-ledu.onrender.com";  // Render server domain
const int port = 443;                          // HTTPS port

// GSM Modem setup
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);

// Sensor Objects
Adafruit_BME280 bme;
Adafruit_INA219 ina219;
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
MPU6050 mpu;

bool initModem() {
    SerialMon.println("Initializing modem...");
    
    // Restart modem
    modem.restart();
    delay(3000);
    
    // Unlock SIM if needed
    if (strlen(pin) > 0 && modem.getSimStatus() != 3) {
        modem.simUnlock(pin);
    }

    SerialMon.println("Waiting for network...");
    if (!modem.waitForNetwork(60000L)) {
        SerialMon.println("Network registration failed");
        return false;
    }
    
    if (modem.isNetworkConnected()) {
        SerialMon.println("Network connected");
    }

    SerialMon.println("Connecting to GPRS...");
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        SerialMon.println("GPRS connection failed");
        return false;
    }

    if (modem.isGprsConnected()) {
        SerialMon.println("GPRS connected");
    }

    // Enable GPS
    SerialMon.println("Enabling GPS...");
    modem.enableGPS();
    
    return true;
}

void setup() {
    SerialMon.begin(SerialMonSpeed);
    SerialAT.begin(SerialATSpeed);

    SerialMon.println("Initializing...");

    // Initialize modem and mobile data connection
    if (!initModem()) {
        SerialMon.println("Failed to initialize modem. Check SIM card and antenna.");
        delay(10000);
        return;
    }

    // Initialize sensors
    Wire.begin(8, 9);  // I2C setup for sensor communication
    
    if (!bme.begin(0x76)) {
        SerialMon.println("BME280 not found!");
    }
    
    if (!ina219.begin()) {
        SerialMon.println("INA219 not found!");
    }
    
    mpu.initialize();
    if (!mpu.testConnection()) {
        SerialMon.println("MPU6050 not connected!");
    }

    if (!tsl.begin()) {
        SerialMon.println("TSL2561 not found!");
    }
    tsl.enableAutoRange(true);
    tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
}

// Send Data to Server using TCP connection
void sendData(float temp, float hum, float light, float current, float voltage, float latitude, float longitude, float speed, float altitude) {
    // Check if we're still connected
    if (!modem.isGprsConnected()) {
        SerialMon.println("GPRS disconnected. Reconnecting...");
        if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
            SerialMon.println("GPRS reconnection failed");
            return;
        }
    }

    SerialMon.print("Connecting to ");
    SerialMon.print(server);
    SerialMon.print(":");
    SerialMon.println(port);

    if (!client.connect(server, port)) {
        SerialMon.println("Connection failed");
        return;
    }

    // Prepare JSON payload
    String jsonPayload = "{";
    jsonPayload += "\"latitude\":" + String(latitude, 6) + ",";
    jsonPayload += "\"longitude\":" + String(longitude, 6) + ",";
    jsonPayload += "\"speed\":" + String(speed) + ",";
    jsonPayload += "\"altitude\":" + String(altitude) + ",";
    jsonPayload += "\"temp\":" + String(temp) + ",";
    jsonPayload += "\"humidity\":" + String(hum) + ",";
    jsonPayload += "\"light\":" + String(light) + ",";
    jsonPayload += "\"current\":" + String(current) + ",";
    jsonPayload += "\"voltage\":" + String(voltage);
    jsonPayload += "}";

    // Prepare HTTP POST request
    String httpRequest = "POST /receive_data HTTP/1.1\r\n";
    httpRequest += "Host: " + String(server) + "\r\n";
    httpRequest += "Content-Type: application/json\r\n";
    httpRequest += "Content-Length: " + String(jsonPayload.length()) + "\r\n";
    httpRequest += "Connection: close\r\n\r\n";
    httpRequest += jsonPayload;

    // Send the request
    client.print(httpRequest);

    // Wait for server response
    unsigned long timeout = millis();
    while (client.connected() && millis() - timeout < 10000L) {
        while (client.available()) {
            String line = client.readStringUntil('\n');
            if (line.startsWith("HTTP/1.1")) {
                SerialMon.println("Response: " + line);
            }
        }
    }

    client.stop();
    SerialMon.println("Server connection closed");
}

void loop() {
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
        SerialMon.println("Lat: " + String(latitude, 6) + " Lon: " + String(longitude, 6));
    } else {
        SerialMon.println("No GPS fix...");
    }

    // Send data to server
    sendData(temp, hum, light, current, voltage, latitude, longitude, speed, altitude);

    delay(6000);  // Send every 6 seconds
}

// GPS Data Fetching Function
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

// Get Light Intensity
float getLightIntensity() {
    uint16_t broadband, infrared;
    tsl.getLuminosity(&broadband, &infrared);
    return (float)broadband;
}
