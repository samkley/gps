#include <WiFi.h>
#include <HTTPClient.h>
// Uncomment BME280 related libraries
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_TSL2561_U.h>
// #include <Adafruit_INA219.h>
// #include <MPU6050.h>
#include <WiFiClientSecure.h>  // Add at the top with other includes
#include <TinyGSM.h>

// Define I2C pins
#define I2C_SCL 17  // BME280 SCL
#define I2C_SDA 18  // BME280 SDA
#define I2C2_SCL 9  // TSL2561 SCL
#define I2C2_SDA 8  // TSL2561 SDA
#define MODEM_TX 1  // ESP32 pin 1 to SIM7600 RX
#define MODEM_RX 2  // ESP32 pin 2 to SIM7600 TX
#define MODEM_PWRKEY 4  // ESP32 pin 4 to SIM7600 PWRKEY
#define MODEM_SLEEP 5   // ESP32 pin 5 to SIM7600 SLEEP/DTR
#define SerialGPS Serial1

// Create second I2C instance for TSL2561
TwoWire I2C_TWO = TwoWire(1);

// WiFi credentials
const char* ssid = "MyFi 12.5cm";
const char* password = "igel-am-pfaffenberg-87";

// Server configuration
const char* server = "gps-ledu.onrender.com";
const int port = 443;

// Function prototypes
void sendDataWiFi(float temp, float hum, float pressure, float lux, float latitude, float longitude, float speed, float altitude);
bool reconnectWiFi();
bool initGPS();
bool getGPSData(float &latitude, float &longitude, float &speed, float &altitude);

// Sensor objects
Adafruit_BME280 bme;
// Comment out other sensor objects
// Adafruit_INA219 ina219;
// MPU6050 mpu;

// Sensor status flags
bool bme280_ok = false;
bool tsl2561_ok = false;
bool gps_ok = false;
// Comment out other sensor flags
// bool ina219_ok = false;
// bool mpu6050_ok = false;

unsigned long lastWiFiCheck = 0;
const unsigned long WIFI_CHECK_INTERVAL = 30000;

// Sensor objects
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

// Create instances
TinyGsm modem(SerialGPS);
String modemResponse = "";  // Global variable for modem responses

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n--- Setup Starting ---");

    // Setup modem power control pins
    pinMode(MODEM_PWRKEY, OUTPUT);
    pinMode(MODEM_SLEEP, OUTPUT);
    digitalWrite(MODEM_SLEEP, HIGH);  // Keep module awake

    // Power cycle the modem
    Serial.println("Power cycling modem...");
    digitalWrite(MODEM_PWRKEY, HIGH);
    delay(2000);
    digitalWrite(MODEM_PWRKEY, LOW);
    delay(5000);

    // Power on sequence
    Serial.println("Powering on modem...");
    digitalWrite(MODEM_PWRKEY, HIGH);
    delay(1000);
    digitalWrite(MODEM_PWRKEY, LOW);
    delay(10000);  // Wait for module to boot

    // Initialize UART for SIM7600
    SerialGPS.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
    delay(1000);

    // Initialize I2C for BME280
    Wire.begin(I2C_SDA, I2C_SCL);
    delay(100);

    // Initialize second I2C bus for TSL2561
    I2C_TWO.begin(I2C2_SDA, I2C2_SCL);
    delay(100);

    // Initialize BME280
    if (!bme.begin(0x76, &Wire)) {
        Serial.println("Trying alternate BME280 address...");
        if (!bme.begin(0x77, &Wire)) {
            Serial.println("Could not find BME280 sensor!");
            bme280_ok = false;
        } else {
            Serial.println("BME280 found at address 0x77!");
            bme280_ok = true;
        }
    } else {
        Serial.println("BME280 found at address 0x76!");
        bme280_ok = true;
    }

    if (bme280_ok) {
        bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                       Adafruit_BME280::SAMPLING_X2,
                       Adafruit_BME280::SAMPLING_X16,
                       Adafruit_BME280::SAMPLING_X16,
                       Adafruit_BME280::FILTER_X16,
                       Adafruit_BME280::STANDBY_MS_500);
    }

    // Initialize TSL2561
    if(!tsl.begin(&I2C_TWO)) {
        Serial.println("Could not find TSL2561 sensor!");
        tsl2561_ok = false;
    } else {
        Serial.println("TSL2561 sensor found!");
        tsl2561_ok = true;
        tsl.enableAutoRange(true);
        tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);
    }

    // Initialize GPS
    if (!initGPS()) {
        Serial.println("Failed to initialize GPS!");
        gps_ok = false;
    } else {
        Serial.println("GPS initialized successfully!");
        gps_ok = true;
    }

    // Connect to WiFi
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    if (!reconnectWiFi()) {
        Serial.println("Initial WiFi connection failed!");
    }
}

bool initGPS() {
    Serial.println("\n=== GPS Module Initialization ===");
    Serial.println("1. Setting up modem power control pins...");
    
    // Power cycle the modem
    Serial.println("2. Power cycling modem...");
    digitalWrite(MODEM_PWRKEY, HIGH);
    delay(2000);
    digitalWrite(MODEM_PWRKEY, LOW);
    delay(5000);
    Serial.println("   Power cycle complete");
    
    // Power on sequence
    Serial.println("3. Executing power-on sequence...");
    digitalWrite(MODEM_PWRKEY, HIGH);
    delay(1000);
    digitalWrite(MODEM_PWRKEY, LOW);
    delay(10000);  // Wait for module to boot
    Serial.println("   Power-on sequence complete");

    // Initialize modem
    Serial.println("4. Initializing modem...");
    
    // First, try to get modem's attention
    Serial.println("   Sending AT command...");
    modem.sendAT("+CCID");
    if (modem.waitResponse(10000) != 1) {
        Serial.println("   ERROR: No response to AT command!");
        Serial.println("   - Check if modem is powered correctly");
        Serial.println("   - Verify TX/RX connections are not swapped");
        Serial.println("   - Try increasing power-on delay");
        return false;
    }
    Serial.println("   Modem responded to AT command");

    // Get modem info
    Serial.println("   Requesting modem information...");
    modem.sendAT("ATI");
    if (modem.waitResponse(10000, modemResponse) != 1) {
        Serial.println("   Warning: Could not get modem info");
    } else {
        Serial.println("   Modem Info: " + modemResponse);
    }

    // Set echo off
    Serial.println("   Disabling command echo...");
    modem.sendAT("ATE0");
    if (modem.waitResponse(10000) != 1) {
        Serial.println("   Warning: Could not disable echo");
    }

    // Set text mode
    Serial.println("   Setting text mode...");
    modem.sendAT("AT+CMGF=1");
    if (modem.waitResponse(10000) != 1) {
        Serial.println("   Warning: Could not set text mode");
    }

    // Check SIM card status
    Serial.println("   Checking SIM card status...");
    modem.sendAT("+CPIN?");
    if (modem.waitResponse(10000, modemResponse) != 1) {
        Serial.println("   ERROR: Could not check SIM status!");
        return false;
    }
    Serial.println("   SIM Status: " + modemResponse);

    // Check signal quality
    Serial.println("   Checking signal quality...");
    modem.sendAT("+CSQ");
    if (modem.waitResponse(10000, modemResponse) != 1) {
        Serial.println("   Warning: Could not check signal quality");
    } else {
        Serial.println("   Signal Quality: " + modemResponse);
    }

    Serial.println("   Modem initialized successfully");

    // Configure SIM PIN
    

    // Set APN
    Serial.println("7. Setting APN...");
    
    // Check network registration
    Serial.println("   Checking network registration...");
    modem.sendAT("+CREG?");
    if (modem.waitResponse(10000, modemResponse) != 1) {
        Serial.println("   Warning: Failed to check network registration");
    } else {
        Serial.println("   Network registration response: " + modemResponse);
    }
    
    // Wait for network registration
    Serial.println("   Waiting for network registration...");
    bool registered = false;
    for (int i = 0; i < 30; i++) {  // Try for 30 seconds
        modem.sendAT("+CREG?");
        if (modem.waitResponse(10000, modemResponse) == 1) {
            registered = true;
            break;
        }
        delay(1000);
    }
    
    if (registered) {
        Serial.println("   Network registration successful");
    } else {
        Serial.println("   Network registration failed");
    }

    return true;
}

bool reconnectWiFi() {
    unsigned long currentMillis = millis();
    if (currentMillis - lastWiFiCheck < WIFI_CHECK_INTERVAL) {
        return false;
    }
    lastWiFiCheck = currentMillis;
    
    if (WiFi.status() != WL_CONNECTED) {
        Serial.print("Connecting to WiFi...");
        WiFi.begin(ssid, password);
        int i = 0;
        while (WiFi.status() != WL_CONNECTED && i < 30) {  // Retry for 30 seconds
            delay(1000);
            Serial.print(".");
            i++;
        }
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\nConnected to WiFi");
            return true;
        } else {
            Serial.println("\nFailed to connect to WiFi");
            return false;
        }
    } else {
        return true;
    }
}

bool getGPSData(float &latitude, float &longitude, float &speed, float &altitude) {
    if (SerialGPS.available() > 0) {
        String nmea = SerialGPS.readStringUntil('\n');
        if (nmea.startsWith("$GPGGA")) {
            int commaIndex = nmea.indexOf(',');
            if (commaIndex != -1) {
                latitude = nmea.substring(commaIndex + 1, commaIndex + 10).toFloat();
                longitude = nmea.substring(commaIndex + 11, commaIndex + 21).toFloat();
                speed = nmea.substring(commaIndex + 22, commaIndex + 26).toFloat();
                altitude = nmea.substring(commaIndex + 27).toFloat();
                return true;
            }
        }
    }
    return false;
}

void sendDataWiFi(float temp, float hum, float pressure, float lux, float latitude, float longitude, float speed, float altitude) {
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        String url = "https://" + String(server) + ":" + String(port) + "/data?temp=" + String(temp) + "&hum=" + String(hum) + "&pressure=" + String(pressure) + "&lux=" + String(lux) + "&latitude=" + String(latitude) + "&longitude=" + String(longitude) + "&speed=" + String(speed) + "&altitude=" + String(altitude);
        
        Serial.println("Sending data to server: " + url);
        http.begin(url);
        int httpResponseCode = http.GET();
        if (httpResponseCode > 0) {
            Serial.println("Data sent successfully. Response code: " + String(httpResponseCode));
        } else {
            Serial.println("Error sending data: " + String(httpResponseCode));
        }
        http.end();
    } else {
        Serial.println("WiFi not connected!");
    }
}

void loop() {
    if (bme280_ok && tsl2561_ok && gps_ok) {
        float temperature, humidity, pressure, lux, latitude, longitude, speed, altitude;

        // Read BME280 sensor data
        if (bme280_ok) {
            temperature = bme.readTemperature();
            humidity = bme.readHumidity();
            pressure = bme.readPressure() / 100.0F;
        }

        // Read TSL2561 light level data
        if (tsl2561_ok) {
            sensors_event_t event;
            tsl.getEvent(&event);
            lux = event.light;  // Access the light level from the event object
        }

        // Get GPS data
        if (gps_ok && getGPSData(latitude, longitude, speed, altitude)) {
            sendDataWiFi(temperature, humidity, pressure, lux, latitude, longitude, speed, altitude);
        }
    }
    delay(10000); // Adjust delay as needed
}
