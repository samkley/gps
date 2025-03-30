#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_TSL2561_U.h>
#include <HardwareSerial.h>

#define I2C_SCL_BME 17  // BME280 SCL
#define I2C_SDA_BME 18  // BME280 SDA
#define I2C_SCL_TSL 9   // TSL2561 SCL
#define I2C_SDA_TSL 8   // TSL2561 SDA
#define MODEM_TX 1  // ESP32 pin 1 to SIM7600 RX
#define MODEM_RX 2  // ESP32 pin 2 to SIM7600 TX
#define MODEM_PWRKEY 4  // ESP32 pin 4 to SIM7600 PWRKEY
#define MODEM_SLEEP 5   // ESP32 pin 5 to SIM7600 SLEEP/DTR
#define SerialGPS Serial1  // Serial for GPS

bool bme280_ok = false;
bool tsl2561_ok = false;
// Create sensor objects
Adafruit_BME280 bme;  // BME280 object for temperature, pressure, and humidity
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

HardwareSerial mySerial(1);  // Use UART1 for SIM7600 (pins 1 and 2)

// Function declarations
String sendATCommand(const String &command, const int timeout);
String convertToDMM(String degreeMinutes);
void parseGPSData(const String &gpsData);
void readSensors();

void setup() {
  Serial.begin(115200);  // Serial monitor output

  // Initialize SIM7600 serial communication
  mySerial.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);

  delay(1000);  // Wait for SIM7600 to power up
  String response = sendATCommand("AT", 2000);  // Send AT command to check if the modem responds
  Serial.println("SIM7600 Response: ");
  Serial.println(response);  // Print the SIM7600 response

  if (response.indexOf("OK") != -1) {
    Serial.println("SIM7600 is responding!");
  } else {
    Serial.println("No response from SIM7600!");
    return;
  }

  // Enable GPS on SIM7600
  response = sendATCommand("AT+CGPS=1", 5000);  // Start GPS
  Serial.println("Start GPS Response: ");
  Serial.println(response);  // Print response to start GPS

  // Initialize I2C for both BME280 and TSL2561
  Wire.begin(I2C_SDA_BME, I2C_SCL_BME);  // Initialize I2C bus for both BME280 and TSL2561
  Wire1.begin(I2C_SDA_TSL, I2C_SCL_TSL);  // Initialize Wire1 for TSL2561
  
  // Initialize sensors once in setup
  if (bme.begin(0x76)) {
    bme280_ok = true;
    Serial.println("BME280 initialized successfully.");
  } else {
    Serial.println("Couldn't find BME280 sensor!");
  }

  if (tsl.begin(&Wire1)) {
    tsl2561_ok = true;
    tsl.setGain(TSL2561_GAIN_16X);  // Set the gain to 16x
    tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  // Set integration time
    Serial.println("TSL2561 initialized successfully.");
  } else {
    Serial.println("Couldn't find TSL2561 sensor!");
  }

  delay(5000); // Give some time for GPS to lock
}

void loop() {
  // Get GPS data from SIM7600 every 5 seconds
  String response = sendATCommand("AT+CGPSINFO", 5000);  // Get GPS info
  Serial.println("GPS Data: ");
  Serial.println(response);  // Print raw GPS data

  // Parse and print the GPS data in a nice format
  parseGPSData(response);

  // Read sensors every 5 seconds
  readSensors();

  delay(5000);  // Wait 5 seconds before sending the next command
}

void readSensors() {
  if (bme280_ok) {
    // Read BME280 sensor (temperature, humidity, pressure)
    float temperature = bme.readTemperature();
    float humidity = bme.readHumidity();
    float pressure = bme.readPressure() / 100.0F;
    
    Serial.println("BME280 Sensor Data:");
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");
    Serial.print("Pressure: ");
    Serial.print(pressure);
    Serial.println(" hPa");
  }

  if (tsl2561_ok) {
    // Read TSL2561 sensor (lux)
    uint16_t broadband, ir;
    tsl.getLuminosity(&broadband, &ir);
    float lux = tsl.calculateLux(broadband, ir);
    
    if (lux < 65536) {  // Check if lux reading is within range
      Serial.println("TSL2561 Sensor Data:");
      Serial.print("Lux: ");
      Serial.println(lux);
    } else {
      Serial.println("Lux reading out of range");
    }
  }
}

String sendATCommand(const String &command, const int timeout) {
  String response = "";
  mySerial.println(command);  // Send command
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
  int degrees = (int)degree / 100;  // Degrees (first two digits)
  float minutes = degree - (degrees * 100);  // Decimal minutes
  return String(degrees) + "° " + String(minutes, 6) + "'";  // DMM format
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

  String day = date.substring(0, 2);
  String month = date.substring(2, 4);
  String year = "20" + date.substring(4, 6);
  String formattedDate = day + "/" + month + "/" + year;

  String hours = time.substring(0, 2);
  String minutes = time.substring(2, 4);
  String seconds = time.substring(4, 6);
  String formattedTime = hours + ":" + minutes + ":" + seconds;

  String formattedLatitude = convertToDMM(latitude);
  String formattedLongitude = convertToDMM(longitude);

  Serial.println("GPS Data:");
  Serial.print("Latitude: ");
  Serial.print(formattedLatitude);
  Serial.print(" ");
  Serial.println(latDirection);

  Serial.print("Longitude: ");
  Serial.print(formattedLongitude);
  Serial.print(" ");
  Serial.println(lonDirection);

  Serial.print("Date: ");
  Serial.println(formattedDate);

  Serial.print("Time: ");
  Serial.println(formattedTime);

  Serial.print("Altitude: ");
  Serial.println(altitude);

  Serial.print("Speed: ");
  Serial.println(speed);
}
