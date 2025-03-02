#include <WiFi.h>
#include <HTTPClient.h>
// Uncomment BME280 related libraries
#include <Wire.h>
#include <Adafruit_BME280.h>
// #include <Adafruit_INA219.h>
// #include <MPU6050.h>
#include <WiFiClientSecure.h>  // Add at the top with other includes

// Define I2C pins
#define I2C_SCL 17  // Alternative SCL for ESP32-S3-WROOM-1
#define I2C_SDA 18  // Alternative SDA for ESP32-S3-WROOM-1

// WiFi credentials
const char* ssid = "MyFi 12.5cm";
const char* password = "igel-am-pfaffenberg-87";

// Server configuration
const char* server = "gps-ledu.onrender.com";
const int port = 443;  // HTTPS port

// Define the pins for UART1 (ESP32)
#define MODEM_TX 1  // ESP32 pin 1 to SIM7600 RX
#define MODEM_RX 2  // ESP32 pin 2 to SIM7600 TX
#define MODEM_PWRKEY 4  // ESP32 pin 4 to SIM7600 PWRKEY
#define MODEM_SLEEP 5   // ESP32 pin 5 to SIM7600 SLEEP
#define SerialGPS Serial1

// Function prototypes
void sendDataWiFi(float temp, float hum, float pressure, float current, float voltage, float latitude, float longitude, float speed, float altitude);
bool getGPSData(float &latitude, float &longitude, float &speed, float &altitude);
bool initGPS();
bool reconnectWiFi();
bool testATCommand();

// Uncomment BME280 object
Adafruit_BME280 bme;
// Comment out other sensor objects
// Adafruit_INA219 ina219;
// MPU6050 mpu;

// Sensor status flags
bool bme280_ok = false;
// Comment out other sensor flags
// bool ina219_ok = false;
// bool mpu6050_ok = false;

unsigned long lastWiFiCheck = 0;
const unsigned long WIFI_CHECK_INTERVAL = 30000; // Check WiFi every 30 seconds

void setup() {
    // Initialize USB Serial for debugging
    Serial.begin(115200);
    delay(1000);  // Give serial time to initialize
    Serial.println("\n--- Setup Starting ---");

    // Setup modem power control pins
    pinMode(MODEM_PWRKEY, OUTPUT);
    pinMode(MODEM_SLEEP, OUTPUT);
    digitalWrite(MODEM_SLEEP, HIGH);  // Keep modem awake
    
    // More robust power on sequence for SIM7600
    digitalWrite(MODEM_PWRKEY, HIGH);
    delay(500);  // Increased initial delay
    digitalWrite(MODEM_PWRKEY, LOW);
    delay(1000);  // Wait 1 second
    digitalWrite(MODEM_PWRKEY, HIGH);
    delay(2000);  // Wait 2 seconds
    digitalWrite(MODEM_PWRKEY, LOW);
    delay(5000);  // Give more time to stabilize

    // Initialize UART1 for SIM7600 GPS with hardware flow control
    SerialGPS.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
    delay(3000);  // Give modem more time to initialize

    // Try different baud rates if communication fails
    if (!testATCommand()) {
        Serial.println("Trying different baud rate...");
        SerialGPS.end();
        delay(100);
        SerialGPS.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
        delay(1000);
        if (!testATCommand()) {
            Serial.println("GPS module not responding at any baud rate!");
        }
    }

    // Initialize GPS properly
    if (!initGPS()) {
        Serial.println("GPS initialization failed!");
    }

    // Connect to WiFi
    WiFi.mode(WIFI_STA); // Set WiFi to station mode
    WiFi.setAutoReconnect(true); // Enable auto-reconnect
    if (!reconnectWiFi()) {
        Serial.println("Initial WiFi connection failed!");
    }

    // Initialize I2C for BME280
    Wire.begin(I2C_SDA, I2C_SCL);  // SDA, SCL
    delay(100);  // Give I2C time to initialize
    
    // BME280 initialization
    if (bme.begin(0x76, &Wire)) {
        Serial.println("BME280 initialized successfully!");
        bme280_ok = true;
    } else {
        Serial.println("BME280 not found! Check wiring and I2C address.");
        Serial.println("SDA: " + String(I2C_SDA) + ", SCL: " + String(I2C_SCL));
        bme280_ok = false;
    }
}

bool testATCommand() {
    int retries = 3;
    while (retries > 0) {
        Serial.println("Testing AT command...");
        // Clear any pending data
        while (SerialGPS.available()) {
            SerialGPS.read();
        }
        
        SerialGPS.println("AT");
        delay(1000);
        
        if (SerialGPS.available()) {
            String response = SerialGPS.readString();
            Serial.println("Response: " + response);
            if (response.indexOf("OK") != -1) {
                return true;
            }
        }
        retries--;
        delay(1000);
    }
    return false;
}

bool initGPS() {
    int retries = 0;
    const int MAX_RETRIES = 5;
    
    while (retries < MAX_RETRIES) {
        Serial.println("Initializing GPS... Attempt " + String(retries + 1));
        
        // Clear any pending data
        while (SerialGPS.available()) {
            SerialGPS.read();
        }
        
        // Test AT command with more detailed feedback
        SerialGPS.println("AT");
        delay(1000);
        String response = "";
        if (SerialGPS.available()) {
            response = SerialGPS.readString();
            Serial.println("AT Response: " + response);
        } else {
            Serial.println("No response to AT command");
            retries++;
            continue;
        }

        // More robust initialization sequence
        const char* commands[] = {
            "AT+CFUN=0",    // Disable RF
            "AT+CGNSPWR=0", // Ensure GPS is off
            "AT",           // Test AT again
            "AT+CGNSPWR=1", // Power on GPS
            "AT+CGNSPWR?",  // Verify GPS power status
            "AT+CGNSINF"    // Test GPS info command
        };
        
        bool initSuccess = true;
        for (const char* cmd : commands) {
            Serial.println("Sending: " + String(cmd));
            SerialGPS.println(cmd);
            delay(2000);  // Longer delay between commands
            
            if (SerialGPS.available()) {
                response = SerialGPS.readString();
                Serial.println("Response: " + response);
                if (response.indexOf("ERROR") != -1) {
                    Serial.println("Error on command: " + String(cmd));
                    initSuccess = false;
                    break;
                }
            } else {
                Serial.println("No response to command: " + String(cmd));
                initSuccess = false;
                break;
            }
        }
        
        if (initSuccess) {
            Serial.println("GPS initialized successfully!");
            return true;
        }
        
        retries++;
        delay(2000);  // Longer delay between retries
    }
    
    Serial.println("GPS initialization failed after " + String(MAX_RETRIES) + " attempts");
    return false;
}

bool reconnectWiFi() {
    Serial.print("Connecting to WiFi");
    
    unsigned long startAttemptTime = millis();
    
    WiFi.begin(ssid, password);
    
    // Try for 20 seconds
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 20000) {
        delay(500);
        Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        return true;
    } else {
        Serial.println("\nWiFi connection failed!");
        return false;
    }
}

void loop() {
    // Check WiFi connection periodically
    unsigned long currentMillis = millis();
    if (currentMillis - lastWiFiCheck >= WIFI_CHECK_INTERVAL) {
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi disconnected. Reconnecting...");
            reconnectWiFi();
        }
        lastWiFiCheck = currentMillis;
    }

    // Initialize variables with default values
    float temp = 0.0;
    float hum = 0.0;
    float pressure = 0.0;
    float current = 0.0;
    float voltage = 0.0;
    float latitude = 0.0;
    float longitude = 0.0;
    float speed = 0.0;
    float altitude = 0.0;

    // Read BME280 sensor
    if (bme280_ok) {
        temp = bme.readTemperature();
        hum = bme.readHumidity();
        pressure = bme.readPressure() / 100.0F; // Convert Pa to hPa
        if (isnan(temp) || isnan(hum) || isnan(pressure)) {
            Serial.println("Failed to read from BME280!");
            temp = 0.0;
            hum = 0.0;
            pressure = 0.0;
        } else {
            Serial.print("Temperature: "); Serial.print(temp); Serial.println(" °C");
            Serial.print("Humidity: "); Serial.print(hum); Serial.println(" %");
            Serial.print("Pressure: "); Serial.print(pressure); Serial.println(" hPa");
        }
    }

    // Read GPS data
    bool gpsFix = getGPSData(latitude, longitude, speed, altitude);
    
    if (gpsFix) {
        Serial.println("GPS Fix Acquired!");
        Serial.println("Lat: " + String(latitude, 6) + " Lon: " + String(longitude, 6));
    } else {
        Serial.println("No GPS fix...");
    }

    // Send data over WiFi only if connected
    if (WiFi.status() == WL_CONNECTED) {
        sendDataWiFi(temp, hum, pressure, current, voltage, latitude, longitude, speed, altitude);
    }

    delay(6000);  // Send every 6 seconds
}

// GPS Data Fetching Function
bool getGPSData(float &latitude, float &longitude, float &speed, float &altitude) {
    Serial.println("Requesting GPS data...");
    
    // Clear any pending data
    while (SerialGPS.available()) {
        SerialGPS.read();
    }
    
    // First check if GPS is still powered on
    SerialGPS.println("AT+CGNSPWR?");
    delay(1000);
    if (SerialGPS.available()) {
        String powerStatus = SerialGPS.readString();
        if (powerStatus.indexOf("+CGNSPWR: 1") == -1) {
            Serial.println("GPS is not powered on, reinitializing...");
            if (!initGPS()) {
                return false;
            }
        }
    }
    
    SerialGPS.println("AT+CGNSINF");
    delay(1000); // Wait for response
    
    String gpsData = "";
    unsigned long startTime = millis();
    while (millis() - startTime < 2000) { // Wait up to 2 seconds for response
        if (SerialGPS.available()) {
            gpsData += SerialGPS.readStringUntil('\n');
        }
    }
    
    Serial.println("Raw GPS data: " + gpsData);
    
    // Check if we got a proper response
    if (gpsData.indexOf("+CGNSINF:") == -1) {
        Serial.println("Invalid GPS response");
        return false;
    }
    
    int fixStatus = gpsData.substring(gpsData.indexOf("+CGNSINF: ") + 9, gpsData.indexOf(",", gpsData.indexOf("+CGNSINF: "))).toInt();
    Serial.print("GPS Fix Status: ");
    Serial.println(fixStatus);

    if (fixStatus == 1) {
        // Parse the comma-separated values
        int commaCount = 0;
        int lastCommaIndex = gpsData.indexOf(",");
        
        // Skip to latitude
        for(int i = 0; i < 3; i++) {
            lastCommaIndex = gpsData.indexOf(",", lastCommaIndex + 1);
        }
        
        // Get latitude
        int nextCommaIndex = gpsData.indexOf(",", lastCommaIndex + 1);
        latitude = gpsData.substring(lastCommaIndex + 1, nextCommaIndex).toFloat();
        
        // Get longitude
        lastCommaIndex = nextCommaIndex;
        nextCommaIndex = gpsData.indexOf(",", lastCommaIndex + 1);
        longitude = gpsData.substring(lastCommaIndex + 1, nextCommaIndex).toFloat();
        
        // Get altitude
        lastCommaIndex = nextCommaIndex;
        nextCommaIndex = gpsData.indexOf(",", lastCommaIndex + 1);
        altitude = gpsData.substring(lastCommaIndex + 1, nextCommaIndex).toFloat();
        
        // Get speed
        lastCommaIndex = nextCommaIndex;
        nextCommaIndex = gpsData.indexOf(",", lastCommaIndex + 1);
        speed = gpsData.substring(lastCommaIndex + 1, nextCommaIndex).toFloat();
        
        Serial.println("GPS Data parsed:");
        Serial.print("Lat: "); Serial.println(latitude, 6);
        Serial.print("Lon: "); Serial.println(longitude, 6);
        Serial.print("Speed: "); Serial.println(speed);
        Serial.print("Altitude: "); Serial.println(altitude);
        return true;
    }
    Serial.println("Failed to get GPS fix");
    return false;
}

// Send Data using WiFi
void sendDataWiFi(float temp, float hum, float pressure, float current, float voltage, float latitude, float longitude, float speed, float altitude) {
    WiFiClientSecure *client = new WiFiClientSecure;
    if(client) {
        client->setInsecure();  // Skip certificate validation
        HTTPClient http;
        
        // Build URL with query parameters
        String url = "https://" + String(server) + "/api/update";  // Add /api/update endpoint
        url += "?latitude=" + String(latitude, 6);
        url += "&longitude=" + String(longitude, 6);
        url += "&speed=" + String(speed);
        url += "&altitude=" + String(altitude);
        url += "&temperature=" + String(temp);
        url += "&humidity=" + String(hum);
        url += "&pressure=" + String(pressure);
        url += "&current=" + String(current);
        url += "&voltage=" + String(voltage);
        
        Serial.println("Sending data via WiFi...");
        Serial.println("URL: " + url);

        http.begin(*client, url);
        http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
        
        int httpResponseCode = http.GET();
        
        if (httpResponseCode > 0) {
            String response = http.getString();
            Serial.println("HTTP Response code: " + String(httpResponseCode));
            Serial.println("Response: " + response);
            
            if (httpResponseCode == 307) {
                String newLocation = http.header("Location");
                Serial.println("Redirect to: " + newLocation);
                // Handle redirect
                http.end();
                http.begin(*client, newLocation);
                httpResponseCode = http.GET();
                if (httpResponseCode > 0) {
                    response = http.getString();
                    Serial.println("HTTP Response code after redirect: " + String(httpResponseCode));
                    Serial.println("Response: " + response);
                }
            }
        } else {
            Serial.println("Error on sending GET: " + String(httpResponseCode));
            Serial.println("Error: " + http.errorToString(httpResponseCode));
        }
        
        http.end();
        delete client;
    } else {
        Serial.println("Unable to create client");
    }
}
