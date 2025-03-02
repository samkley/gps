#include <WiFi.h>
#include <HTTPClient.h>
// Uncomment BME280 related libraries
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_TSL2561_U.h>
// #include <Adafruit_INA219.h>
// #include <MPU6050.h>
#include <WiFiClientSecure.h>  // Add at the top with other includes

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
    // Clear any pending data
    while (SerialGPS.available()) {
        SerialGPS.read();
    }

    // Basic AT test
    SerialGPS.println("AT");
    delay(1000);
    if (!SerialGPS.find("OK")) {
        Serial.println("AT test failed!");
        return false;
    }

    // Set to full functionality
    SerialGPS.println("AT+CFUN=1");
    delay(3000);

    // Power on GPS
    SerialGPS.println("AT+CGNSPWR=1");
    delay(3000);
    
    // Check GPS power status
    SerialGPS.println("AT+CGNSPWR?");
    delay(1000);
    if (!SerialGPS.find("+CGNSPWR: 1")) {
        Serial.println("GPS power on failed!");
        return false;
    }

    // Enable GPS
    SerialGPS.println("AT+CGPS=1");
    delay(2000);

    return true;
}

bool getGPSData(float &latitude, float &longitude, float &speed, float &altitude) {
    // Clear buffer
    while (SerialGPS.available()) {
        SerialGPS.read();
    }

    // Request GPS data
    SerialGPS.println("AT+CGNSINF");
    delay(1000);

    String response = "";
    unsigned long startTime = millis();
    while (millis() - startTime < 2000) {
        if (SerialGPS.available()) {
            char c = SerialGPS.read();
            response += c;
        }
    }

    // Check if we got a valid response
    if (response.indexOf("+CGNSINF:") == -1) {
        return false;
    }

    // Parse the response
    // Format: +CGNSINF: <GNSS run status>,<Fix status>,<UTC date & Time>,<Latitude>,<Longitude>,<MSL Altitude>,<Speed Over Ground>,...
    int commaIndex = response.indexOf(",");
    if (commaIndex == -1) return false;

    // Get run status and fix status
    int runStatus = response.substring(response.indexOf(": ") + 2, commaIndex).toInt();
    int lastCommaIndex = commaIndex;
    commaIndex = response.indexOf(",", lastCommaIndex + 1);
    int fixStatus = response.substring(lastCommaIndex + 1, commaIndex).toInt();

    if (runStatus == 1 && fixStatus == 1) {
        // Skip UTC time
        lastCommaIndex = commaIndex;
        commaIndex = response.indexOf(",", lastCommaIndex + 1);
        
        // Get latitude
        lastCommaIndex = commaIndex;
        commaIndex = response.indexOf(",", lastCommaIndex + 1);
        latitude = response.substring(lastCommaIndex + 1, commaIndex).toFloat();
        
        // Get longitude
        lastCommaIndex = commaIndex;
        commaIndex = response.indexOf(",", lastCommaIndex + 1);
        longitude = response.substring(lastCommaIndex + 1, commaIndex).toFloat();
        
        // Get altitude
        lastCommaIndex = commaIndex;
        commaIndex = response.indexOf(",", lastCommaIndex + 1);
        altitude = response.substring(lastCommaIndex + 1, commaIndex).toFloat();
        
        // Get speed
        lastCommaIndex = commaIndex;
        commaIndex = response.indexOf(",", lastCommaIndex + 1);
        speed = response.substring(lastCommaIndex + 1, commaIndex).toFloat();

        return (latitude != 0.0 || longitude != 0.0);
    }

    return false;
}

bool reconnectWiFi() {
    Serial.print("Connecting to WiFi");
    unsigned long startAttemptTime = millis();
    
    WiFi.begin(ssid, password);
    
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
    // Check WiFi connection
    if (millis() - lastWiFiCheck >= WIFI_CHECK_INTERVAL) {
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi disconnected. Reconnecting...");
            reconnectWiFi();
        }
        lastWiFiCheck = millis();
    }

    float temp = 0.0;
    float hum = 0.0;
    float pressure = 0.0;
    float lux = 0.0;
    float latitude = 0.0;
    float longitude = 0.0;
    float speed = 0.0;
    float altitude = 0.0;

    // Read BME280
    if (bme280_ok) {
        temp = bme.readTemperature();
        hum = bme.readHumidity();
        pressure = bme.readPressure() / 100.0F;
        
        if (!isnan(temp) && !isnan(hum) && !isnan(pressure)) {
            Serial.printf("Temperature: %.2f °C\n", temp);
            Serial.printf("Humidity: %.2f %%\n", hum);
            Serial.printf("Pressure: %.2f hPa\n", pressure);
        } else {
            Serial.println("Failed to read from BME280!");
            temp = hum = pressure = 0.0;
        }
    }

    // Read TSL2561
    if (tsl2561_ok) {
        sensors_event_t event;
        tsl.getEvent(&event);
        
        if (event.light) {
            lux = event.light;
            Serial.printf("Light: %.2f lux\n", lux);
        } else {
            Serial.println("Failed to read TSL2561!");
            lux = 0.0;
        }
    }

    // Read GPS
    if (gps_ok) {
        if (getGPSData(latitude, longitude, speed, altitude)) {
            Serial.println("GPS Fix obtained:");
            Serial.printf("Latitude: %.6f°\n", latitude);
            Serial.printf("Longitude: %.6f°\n", longitude);
            Serial.printf("Speed: %.2f km/h\n", speed);
            Serial.printf("Altitude: %.1f m\n", altitude);
        } else {
            Serial.println("No GPS fix available");
        }
    }

    // Send data if WiFi is connected
    if (WiFi.status() == WL_CONNECTED) {
        sendDataWiFi(temp, hum, pressure, lux, latitude, longitude, speed, altitude);
    }

    delay(6000);
}

void sendDataWiFi(float temp, float hum, float pressure, float lux, float latitude, float longitude, float speed, float altitude) {
    WiFiClientSecure *client = new WiFiClientSecure;
    if(client) {
        client->setInsecure();
        HTTPClient http;
        
        String url = "https://" + String(server) + "/api/update";
        url += "?temperature=" + String(temp);
        url += "&humidity=" + String(hum);
        url += "&pressure=" + String(pressure);
        url += "&light=" + String(lux);
        url += "&latitude=" + String(latitude, 6);
        url += "&longitude=" + String(longitude, 6);
        url += "&speed=" + String(speed);
        url += "&altitude=" + String(altitude);
        
        Serial.println("\nSending data via WiFi...");
        Serial.println("URL: " + url);

        http.begin(*client, url);
        
        int httpResponseCode = http.GET();
        
        if (httpResponseCode > 0) {
            String response = http.getString();
            Serial.println("HTTP Response code: " + String(httpResponseCode));
            Serial.println("Response: " + response);
        } else {
            Serial.println("Error on sending request: " + String(httpResponseCode));
            Serial.println("Error: " + http.errorToString(httpResponseCode));
            Serial.println("Check if the server is running and accessible");
        }
        
        http.end();
        delete client;
    } else {
        Serial.println("Error creating HTTP client");
    }
}

