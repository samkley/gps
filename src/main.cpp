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
    modem.sendAT("AT");
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
    Serial.println("6. Configuring SIM PIN...");
    modem.sendAT("+CPIN=" + String(8871));
    if (modem.waitResponse(10000) != 1) {
        Serial.println("   ERROR: Failed to set SIM PIN!");
        Serial.println("   - Check if SIM card is properly inserted");
        Serial.println("   - Verify PIN code is correct");
        return false;
    }
    Serial.println("   SIM PIN configured successfully");

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
            if (modemResponse.indexOf("+CREG: 0,1") != -1 || modemResponse.indexOf("+CREG: 0,5") != -1) {
                registered = true;
                Serial.println("   Network registered successfully");
                break;
            }
        }
        delay(1000);
    }
    
    if (!registered) {
        Serial.println("   ERROR: Failed to register on network!");
        Serial.println("   - Check SIM card is properly inserted");
        Serial.println("   - Verify network coverage");
        return false;
    }
    
    // Set new PDP context with APN
    Serial.println("   Setting new PDP context...");
    modem.sendAT("+CGDCONT=1,\"IP\",\"internet\"");
    if (modem.waitResponse(10000) != 1) {
        Serial.println("   ERROR: Failed to set APN!");
        Serial.println("   - Check if APN is correct for your carrier");
        Serial.println("   - Verify network settings");
        return false;
    }
    Serial.println("   APN set to 'internet'");

    // Activate PDP context
    Serial.println("   Activating PDP context...");
    modem.sendAT("+CGACT=1,1");
    if (modem.waitResponse(10000) != 1) {
        Serial.println("   ERROR: Failed to activate PDP context!");
        Serial.println("   - Check network coverage");
        Serial.println("   - Verify SIM card is active");
        return false;
    }
    Serial.println("   PDP context activated successfully");

    // Set modem to full functionality
    Serial.println("8. Setting modem to full functionality...");
    modem.sendAT("+CFUN=1");
    if (modem.waitResponse(10000) != 1) {
        Serial.println("   ERROR: Failed to set modem functionality!");
        Serial.println("   - Module might be in a bad state");
        Serial.println("   - Try power cycling");
        return false;
    }
    Serial.println("   Modem set to full functionality");

    // Power on GPS
    Serial.println("9. Powering on GPS...");
    modem.sendAT("+CGNSPWR=1");
    if (modem.waitResponse(10000) != 1) {
        Serial.println("   ERROR: Failed to power on GPS!");
        Serial.println("   - Check if GPS antenna is connected");
        Serial.println("   - Verify module supports GPS");
        return false;
    }
    Serial.println("   GPS powered on successfully");

    // Enable GPS
    Serial.println("10. Enabling GPS...");
    modem.sendAT("+CGPS=1");
    if (modem.waitResponse(10000) != 1) {
        Serial.println("   ERROR: Failed to enable GPS!");
        Serial.println("   - GPS might already be enabled");
        Serial.println("   - Try disabling and re-enabling");
        return false;
    }
    Serial.println("   GPS enabled successfully");

    Serial.println("=== GPS Initialization Complete ===\n");
    return true;
}

bool getGPSData(float &latitude, float &longitude, float &speed, float &altitude) {
    Serial.println("\n--- Reading GPS Data ---");
    
    // Request GPS data
    Serial.println("1. Requesting GPS information...");
    modem.sendAT("+CGNSINF");
    if (modem.waitResponse(10000, modemResponse) != 1) {
        Serial.println("   ERROR: Failed to get GPS data!");
        Serial.println("   - No response from module");
        Serial.println("   - Check if GPS is still powered on");
        return false;
    }
    Serial.println("   Raw response: " + modemResponse);

    // Parse response
    if (modemResponse.indexOf("+CGNSINF: ") == -1) {
        Serial.println("   ERROR: Invalid response format!");
        Serial.println("   - Expected +CGNSINF: prefix");
        Serial.println("   - Got: " + modemResponse);
        return false;
    }

    modemResponse = modemResponse.substring(modemResponse.indexOf(": ") + 2);
    int commaIndex = 0;
    int nextCommaIndex = 0;

    // Get run status
    Serial.println("2. Parsing GPS status...");
    nextCommaIndex = modemResponse.indexOf(',');
    int runStatus = modemResponse.substring(commaIndex, nextCommaIndex).toInt();
    commaIndex = nextCommaIndex + 1;
    Serial.println("   Run status: " + String(runStatus));

    // Get fix status
    nextCommaIndex = modemResponse.indexOf(',', commaIndex);
    int fixStatus = modemResponse.substring(commaIndex, nextCommaIndex).toInt();
    commaIndex = nextCommaIndex + 1;
    Serial.println("   Fix status: " + String(fixStatus));

    if (runStatus != 1 || fixStatus != 1) {
        Serial.println("   ERROR: No GPS fix!");
        Serial.println("   - Run status should be 1, got: " + String(runStatus));
        Serial.println("   - Fix status should be 1, got: " + String(fixStatus));
        Serial.println("   - Make sure you have a clear view of the sky");
        Serial.println("   - Wait for GPS to acquire satellites");
        return false;
    }
    Serial.println("   GPS has valid fix");

    // Skip UTC time
    Serial.println("3. Parsing GPS data...");
    nextCommaIndex = modemResponse.indexOf(',', commaIndex);
    String utcTime = modemResponse.substring(commaIndex, nextCommaIndex);
    commaIndex = nextCommaIndex + 1;
    Serial.println("   UTC Time: " + utcTime);

    // Get latitude
    nextCommaIndex = modemResponse.indexOf(',', commaIndex);
    latitude = modemResponse.substring(commaIndex, nextCommaIndex).toFloat();
    commaIndex = nextCommaIndex + 1;

    // Get longitude
    nextCommaIndex = modemResponse.indexOf(',', commaIndex);
    longitude = modemResponse.substring(commaIndex, nextCommaIndex).toFloat();
    commaIndex = nextCommaIndex + 1;

    // Get altitude
    nextCommaIndex = modemResponse.indexOf(',', commaIndex);
    altitude = modemResponse.substring(commaIndex, nextCommaIndex).toFloat();
    commaIndex = nextCommaIndex + 1;

    // Get speed
    nextCommaIndex = modemResponse.indexOf(',', commaIndex);
    speed = modemResponse.substring(commaIndex, nextCommaIndex).toFloat();

    // Print GPS data summary
    Serial.println("\n=== GPS Data Summary ===");
    Serial.printf("Latitude:  %.6f°\n", latitude);
    Serial.printf("Longitude: %.6f°\n", longitude);
    Serial.printf("Speed:     %.2f km/h\n", speed);
    Serial.printf("Altitude:  %.1f m\n", altitude);
    Serial.println("UTC Time:  " + utcTime);
    Serial.println("=====================\n");
    // Print GPS data for debugging
    Serial.println("GPS Data:");
    Serial.printf("Latitude: %.6f°\n", latitude);
    Serial.printf("Longitude: %.6f°\n", longitude);
    Serial.printf("Speed: %.2f km/h\n", speed);
    Serial.printf("Altitude: %.1f m\n", altitude);

    return true;
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
            modemResponse = http.getString();
            Serial.println("HTTP Response code: " + String(httpResponseCode));
            Serial.println("Response: " + modemResponse);
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

