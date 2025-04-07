#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_TSL2561_U.h>
#include <HardwareSerial.h>

#define I2C_SCL_BME 17  // BME280 SCL
#define I2C_SDA_BME 18  // BME280 SDA
#define I2C_SCL_TSL 9   // TSL2561 SCL
#define I2C_SDA_TSL 8   // TSL2561 SDA
#define MODEM_TX 1      // ESP32 TX → SIM7600 RX
#define MODEM_RX 2      // ESP32 RX ← SIM7600 TX
#define MODEM_PWRKEY 4  // SIM7600 PWRKEY
#define MODEM_SLEEP 5   // SIM7600 DTR/SLEEP
#define SerialGPS Serial1

bool bme280_ok = false;
bool tsl2561_ok = false;

Adafruit_BME280 bme;
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

HardwareSerial mySerial(1);

// Function prototypes
bool setupHTTP();
String sendATCommand(const String &command, const int timeout);
String convertToDMM(String degreeMinutes);
void parseGPSData(const String &gpsData);
void readSensors();
String adjustTimezone(String hours, int offset);

// Server URL
const char* serverUrl = "https://gps-ledu.onrender.com/receive_data";

// Global variables to store sensor data
float currentTemperature = 0;
float currentHumidity = 0;
float currentPressure = 0;
float currentLight = 0;
float currentLatitude = 0;
float currentLongitude = 0;
float currentSpeed = 0;
float currentAltitude = 0;

bool setupHTTP() {
  Serial.println("Setting up HTTP client...");
  
  // First try to terminate any existing HTTP session
  sendATCommand("AT+HTTPTERM", 2000);
  delay(1000);
  
  // Initialize HTTP service
  String response = sendATCommand("AT+HTTPINIT", 2000);
  if (response.indexOf("OK") == -1) {
    Serial.println("Failed to initialize HTTP!");
    return false;
  }
  Serial.println("HTTP initialized");
  
  // Set HTTP parameters
  response = sendATCommand("AT+HTTPPARA=\"CID\",1", 2000);
  if (response.indexOf("OK") == -1) {
    Serial.println("Failed to set CID!");
    sendATCommand("AT+HTTPTERM", 2000);
    return false;
  }
  
  // Enable redirects
  response = sendATCommand("AT+HTTPPARA=\"REDIR\",1", 2000);
  if (response.indexOf("OK") == -1) {
    Serial.println("Failed to enable redirects!");
    sendATCommand("AT+HTTPTERM", 2000);
    return false;
  }
  
  // Set timeout to 30 seconds
  response = sendATCommand("AT+HTTPPARA=\"TIMEOUT\",30", 2000);
  if (response.indexOf("OK") == -1) {
    Serial.println("Failed to set timeout!");
    sendATCommand("AT+HTTPTERM", 2000);
    return false;
  }
  
  // Set user agent
  response = sendATCommand("AT+HTTPPARA=\"UA\",\"SIM7600\"", 2000);
  if (response.indexOf("OK") == -1) {
    Serial.println("Failed to set user agent!");
    sendATCommand("AT+HTTPTERM", 2000);
    return false;
  }
  
  // Set accept type
  response = sendATCommand("AT+HTTPPARA=\"ACCEPT\",\"application/json\"", 2000);
  if (response.indexOf("OK") == -1) {
    Serial.println("Failed to set accept type!");
    sendATCommand("AT+HTTPTERM", 2000);
    return false;
  }
  
  Serial.println("HTTP setup complete!");
  return true;
}

void setupMobileData() {
  Serial.println("Setting up mobile data connection...");
  
  // Check if SIM is ready
  String response = sendATCommand("AT+CPIN?", 2000);
  if (response.indexOf("+CPIN: READY") == -1) {
    Serial.println("SIM card not ready!");
    return;
  }
  Serial.println("SIM card is ready");
  
  // Wait for network registration
  response = sendATCommand("AT+CREG?", 2000);
  while (response.indexOf("+CREG: 0,1") == -1 && response.indexOf("+CREG: 0,5") == -1) {
    delay(1000);
    response = sendATCommand("AT+CREG?", 2000);
    Serial.println("Waiting for network registration...");
  }
  Serial.println("Network registered!");
  
  // Get network operator info
  response = sendATCommand("AT+COPS?", 2000);
  Serial.println("Network operator: " + response);
  
  // Get signal quality
  response = sendATCommand("AT+CSQ", 2000);
  Serial.println("Signal quality: " + response);
  
  // Set PDP context type to IPv4
  response = sendATCommand("AT+CGDCONT=1,\"IP\",\"internet\",\"0.0.0.0\",0,0", 2000);
  if (response.indexOf("OK") == -1) {
    Serial.println("Failed to set PDP context type!");
    return;
  }
  Serial.println("PDP context type set to IPv4");
  
  // Set APN authentication (if needed)
  response = sendATCommand("AT+CGAUTH=1,1,\"\",\"\"", 2000);
  Serial.println("APN authentication set");
  
  // Activate PDP context
  response = sendATCommand("AT+CGACT=1,1", 2000);
  if (response.indexOf("OK") == -1) {
    Serial.println("Failed to activate PDP context!");
    return;
  }
  Serial.println("PDP context activated");
  
  // Wait for network to be ready
  delay(5000);
  
  // Check network status
  response = sendATCommand("AT+CGATT?", 2000);
  if (response.indexOf("+CGATT: 1") == -1) {
    Serial.println("Network not attached!");
    return;
  }
  Serial.println("Network attached successfully");
  
  // Start TCP/IP service
  response = sendATCommand("AT+NETOPEN", 2000);
  if (response.indexOf("OK") == -1) {
    Serial.println("Failed to start TCP/IP service!");
    return;
  }
  Serial.println("TCP/IP service started");
  
  // Wait for TCP/IP service to be ready
  delay(2000);
  
  // Get IP address using IPADDR command
  response = sendATCommand("AT+IPADDR", 2000);
  if (response.indexOf("ERROR") == -1 && response.length() > 0) {
    response.trim();
    if (response.startsWith("+IPADDR:")) {
      response = response.substring(8);
    }
    Serial.println("IP Address: " + response);
  } else {
    Serial.println("Failed to get IP address!");
    // Try to close and reopen TCP/IP service
    sendATCommand("AT+NETCLOSE", 2000);
    delay(2000);
    response = sendATCommand("AT+NETOPEN", 2000);
    if (response.indexOf("OK") == -1) {
      Serial.println("Failed to restart TCP/IP service!");
      return;
    }
    delay(2000);
    response = sendATCommand("AT+IPADDR", 2000);
    if (response.indexOf("ERROR") == -1 && response.length() > 0) {
      response.trim();
      if (response.startsWith("+IPADDR:")) {
        response = response.substring(8);
      }
      Serial.println("IP Address (retry): " + response);
    } else {
      Serial.println("Failed to get IP address after retry!");
      return;
    }
  }
  
  // Setup HTTP
  if (!setupHTTP()) {
    Serial.println("Failed to setup HTTP!");
    return;
  }
  
  Serial.println("Mobile data connection established!");
}

void sendDataToServer() {
  // Set URL (ensure it starts with https://)
  String url = "https://gps-ledu.onrender.com/receive_data";
  String response = sendATCommand("AT+HTTPPARA=\"URL\",\"" + url + "\"", 2000);
  if (response.indexOf("OK") == -1) {
    Serial.println("Failed to set URL!");
    return;
  }
  Serial.println("URL set successfully");

  // Set content type
  response = sendATCommand("AT+HTTPPARA=\"CONTENT\",\"application/json\"", 2000);
  if (response.indexOf("OK") == -1) {
    Serial.println("Failed to set content type!");
    return;
  }
  Serial.println("Content type set successfully");

  // Create JSON payload
  String jsonPayload = "{";
  jsonPayload += "\"latitude\":" + String(currentLatitude, 6) + ",";
  jsonPayload += "\"longitude\":" + String(currentLongitude, 6) + ",";
  jsonPayload += "\"speed\":" + String(currentSpeed, 1) + ",";
  jsonPayload += "\"altitude\":" + String(currentAltitude, 1) + ",";
  jsonPayload += "\"temperature\":" + String(currentTemperature, 1) + ",";
  jsonPayload += "\"humidity\":" + String(currentHumidity, 1) + ",";
  jsonPayload += "\"light\":" + String(currentLight, 1) + ",";
  jsonPayload += "\"current\":0,";  // Placeholder for current
  jsonPayload += "\"voltage\":0";   // Placeholder for voltage
  jsonPayload += "}";

  // Set data length
  response = sendATCommand("AT+HTTPDATA=" + String(jsonPayload.length()) + ",10000", 2000);
  if (response.indexOf("DOWNLOAD") == -1) {
    Serial.println("Failed to prepare data upload!");
    return;
  }
  Serial.println("Data upload prepared");

  // Send the data
  response = sendATCommand(jsonPayload, 2000);
  if (response.indexOf("OK") == -1) {
    Serial.println("Failed to upload data!");
    return;
  }
  Serial.println("Data uploaded successfully");

  // Send POST request with longer timeout
  response = sendATCommand("AT+HTTPACTION=1", 30000);
  if (response.indexOf("+HTTPACTION: 1,200") != -1) {
    Serial.println("Data sent successfully!");
  } else if (response.indexOf("+HTTPACTION: 1,307") != -1) {
    Serial.println("Server redirected request, checking new URL...");
    // Get the response data to see the redirect URL
    delay(1000); // Wait for response to be ready
    response = sendATCommand("AT+HTTPREAD", 2000);
    if (response.indexOf("ERROR") == -1) {
      Serial.println("Response data: " + response);
      // Try to extract the new URL from the response
      int start = response.indexOf("Location: ");
      if (start != -1) {
        start += 10;
        int end = response.indexOf("\r\n", start);
        if (end != -1) {
          String newUrl = response.substring(start, end);
          Serial.println("Redirect URL: " + newUrl);
          // Try the request again with the new URL
          response = sendATCommand("AT+HTTPPARA=\"URL\",\"" + newUrl + "\"", 2000);
          if (response.indexOf("OK") != -1) {
            response = sendATCommand("AT+HTTPACTION=1", 30000);
            if (response.indexOf("+HTTPACTION: 1,200") != -1) {
              Serial.println("Data sent successfully after redirect!");
            } else {
              Serial.println("Failed to send data after redirect! Response: " + response);
            }
          }
        }
      }
    } else {
      Serial.println("Failed to read response data!");
    }
  } else {
    Serial.println("Failed to send data! Response: " + response);
  }
}

void setup() {
  Serial.begin(115200);
  mySerial.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(1000);

  String response = sendATCommand("AT", 2000);
  Serial.println("SIM7600 Response: ");
  Serial.println(response);

  if (response.indexOf("OK") != -1) {
    Serial.println("SIM7600 is responding!");
  } else {
    Serial.println("No response from SIM7600!");
    return;
  }

  // Setup mobile data connection
  setupMobileData();

  Serial.println(sendATCommand("AT+CGPSINFO?", 2000));
  Serial.println(sendATCommand("AT+CGPSINFO=1", 2000));
  Serial.println(sendATCommand("AT+CGPS?", 2000));
  Serial.println(sendATCommand("AT+CGPS=1", 2000));
  Serial.println(sendATCommand("AT+CBC", 2000));

  Wire.begin(I2C_SDA_BME, I2C_SCL_BME);
  Wire1.begin(I2C_SDA_TSL, I2C_SCL_TSL);

  if (bme.begin(0x76)) {
    bme280_ok = true;
    Serial.println("BME280 initialized successfully.");
  } else {
    Serial.println("Couldn't find BME280 sensor!");
  }

  if (tsl.begin(&Wire1)) {
    tsl2561_ok = true;
    tsl.setGain(TSL2561_GAIN_16X);
    tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);
    Serial.println("TSL2561 initialized successfully.");
  } else {
    Serial.println("Couldn't find TSL2561 sensor!");
  }

  delay(5000); // Wait for GPS lock
}

void loop() {
  String response = sendATCommand("AT+CGPSINFO", 5000);
  Serial.println("GPS Data: ");
  Serial.println(response);
  parseGPSData(response);
  delay(5000);
}

void readSensors() {
  if (bme280_ok) {
    float temperature = bme.readTemperature();
    float humidity = bme.readHumidity();
    float pressure = bme.readPressure() / 100.0F;

    Serial.println("BME280 Sensor Data:");
    Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" °C");
    Serial.print("Humidity: "); Serial.print(humidity); Serial.println(" %");
    Serial.print("Pressure: "); Serial.print(pressure); Serial.println(" hPa");
  }

  if (tsl2561_ok) {
    uint16_t broadband, ir;
    tsl.getLuminosity(&broadband, &ir);
    float lux = tsl.calculateLux(broadband, ir);

    if (lux < 65536) {
      Serial.println("TSL2561 Sensor Data:");
      Serial.print("Lux: "); Serial.println(lux);
    } else {
      Serial.println("Lux reading out of range");
    }
  }
}

String sendATCommand(const String &command, const int timeout) {
  String response = "";
  mySerial.println(command);
  unsigned long time = millis();
  while (millis() - time < timeout) {
    while (mySerial.available()) {
      response += char(mySerial.read());
    }
  }
  return response;
}

String convertToDMM(String degreeMinutes) {
  float degree = degreeMinutes.toFloat();
  int degrees = (int)degree / 100;
  float minutes = degree - (degrees * 100);
  return String(degrees) + "° " + String(minutes, 6) + "'";
}

String adjustTimezone(String hours, int offset) {
  int h = hours.toInt();
  h += offset;
  if (h >= 24) h -= 24;
  if (h < 0) h += 24;
  return (h < 10 ? "0" : "") + String(h);
}

void parseGPSData(const String &gpsData) {
  int startIndex = gpsData.indexOf(":") + 1;
  String data = gpsData.substring(startIndex);

  int latEnd = data.indexOf(",");
  String latitude = data.substring(0, latEnd);
  data = data.substring(latEnd + 1);

  int latDirEnd = data.indexOf(",");
  String latDirection = data.substring(0, latDirEnd);
  data = data.substring(latDirEnd + 1);

  int lonEnd = data.indexOf(",");
  String longitude = data.substring(0, lonEnd);
  data = data.substring(lonEnd + 1);

  int lonDirEnd = data.indexOf(",");
  String lonDirection = data.substring(0, lonDirEnd);
  data = data.substring(lonDirEnd + 1);

  int dateEnd = data.indexOf(",");
  String date = data.substring(0, dateEnd);
  data = data.substring(dateEnd + 1);

  int timeEnd = data.indexOf(",");
  String time = data.substring(0, timeEnd);
  data = data.substring(timeEnd + 1);

  int altitudeEnd = data.indexOf(",");
  String altitude = data.substring(0, altitudeEnd);
  data = data.substring(altitudeEnd + 1);

  int speedEnd = data.indexOf(",");
  String speed = data.substring(0, speedEnd);

  // Check if we have valid GPS data
  if (latitude.length() > 0 && longitude.length() > 0 && 
      latitude != "0" && longitude != "0") {
    
    // Update global variables
    currentLatitude = latitude.toFloat();
    currentLongitude = longitude.toFloat();
    currentSpeed = speed.toFloat();
    currentAltitude = altitude.toFloat();
    
    // Read sensor data
    if (bme280_ok) {
      currentTemperature = bme.readTemperature();
      currentHumidity = bme.readHumidity();
      currentPressure = bme.readPressure() / 100.0F;
    }

    if (tsl2561_ok) {
      uint16_t broadband, ir;
      tsl.getLuminosity(&broadband, &ir);
      currentLight = tsl.calculateLux(broadband, ir);
      if (currentLight >= 65536) {
        currentLight = 0;
      }
    }
    
    // Send data to server
    Serial.println("Sending data to server...");
    sendDataToServer();
  } else {
    Serial.println("No valid GPS data yet");
  }
}


