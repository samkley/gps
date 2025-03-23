#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_TSL2561_U.h>
#include <WiFiClientSecure.h>  // To use secure HTTPS communication
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
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

// GPS Module
TinyGsm modem(SerialGPS);

// Sensor status flags
bool bme280_ok = false;
bool tsl2561_ok = false;
bool gps_ok = false;

unsigned long lastWiFiCheck = 0;
const unsigned long WIFI_CHECK_INTERVAL = 30000;

// Create instances
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

    // Power cycle the modem
    Serial.println("Power cycling modem...");
    digitalWrite(MODEM_PWRKEY, HIGH);
    delay(2000);
    digitalWrite(MODEM_PWRKEY, LOW);
    delay(5000);
    
    // Power on sequence
    Serial.println("Power-on sequence...");
    digitalWrite(MODEM_PWRKEY, HIGH);
    delay(1000);
    digitalWrite(MODEM_PWRKEY, LOW);
    delay(10000);  // Wait for module to boot

    // Initialize modem
    Serial.println("Initializing modem...");
    modem.sendAT("+CCID");
    if (modem.waitResponse(10000) != 1) {
        Serial.println("ERROR: No response to AT command!");
        return false;
    }

    // Initialize GPS
    Serial.println("Initializing GPS...");
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
        Serial.println("Raw NMEA: " + nmea);  // Print raw NMEA sentence for debugging
        
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
        String url = "https://gps-ledu.onrender.com/api/data";  // Replace with your API URL

        http.begin(url);
        http.addHeader("Content-Type", "application/json");

        // Create JSON payload
        String payload = "{\"temperature\": " + String(temp) +
                         ", \"humidity\": " + String(hum) +
                         ", \"pressure\": " + String(pressure) +
                         ", \"lux\": " + String(lux) +
                         ", \"latitude\": " + String(latitude) +
                         ", \"longitude\": " + String(longitude) +
                         ", \"speed\": " + String(speed) +
                         ", \"altitude\": " + String(altitude) + "}";

        // Send POST request
        int httpCode = http.POST(payload);

        if (httpCode > 0) {
            Serial.println("Data sent successfully");
        } else {
            Serial.println("Error sending data: " + String(httpCode));
        }

        http.end();  // Close connection
    } else {
        Serial.println("WiFi not connected, skipping data send.");
    }
}

void loop() {
    // Collect sensor data
    float temp = 0.0, hum = 0.0, pressure = 0.0, lux = 0.0;
    float latitude = 0.0, longitude = 0.0, speed = 0.0, altitude = 0.0;

    // Read BME280 data
    if (bme280_ok) {
        temp = bme.readTemperature();
        hum = bme.readHumidity();
        pressure = bme.readPressure() / 100.0F;  // Convert to hPa
    }

    // Read TSL2561 luminosity
    if (tsl2561_ok) {
        uint16_t broadband, infrared;
        tsl.getLuminosity(&broadband, &infrared);  // Pass both variables
        lux = broadband;  // You can use broadband or calculate a total luminosity
    }

    // Try to get GPS data if available
    bool gpsDataAvailable = false;
    if (gps_ok) {
        gpsDataAvailable = getGPSData(latitude, longitude, speed, altitude);
    }

    // Send data to the website even if GPS data is missing
    sendDataWiFi(temp, hum, pressure, lux, latitude, longitude, speed, altitude);

    // If GPS data was available, also send it
    if (gpsDataAvailable) {
        Serial.println("GPS Data Sent");
    } else {
        Serial.println("No GPS data available.");
    }

    // Wait before sending next data
    delay(5000);  // Delay between data readings
}
