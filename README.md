# GPS Environmental Monitoring System

This project implements an environmental monitoring system using an ESP32 microcontroller with various sensors and a SIM7600 GPS/GSM module. The system collects environmental data and GPS location information, then sends it to a web server.

## Hardware Components

- ESP32 Development Board
- BME280 Temperature/Humidity/Pressure Sensor
- TSL2561 Light Sensor
- SIM7600 GPS/GSM Module

## Pin Configuration

### I2C Pins
- BME280:
  - SCL: GPIO17
  - SDA: GPIO18
- TSL2561:
  - SCL: GPIO9
  - SDA: GPIO8

### SIM7600 Module Pins
- TX: GPIO1 (ESP32 RX)
- RX: GPIO2 (ESP32 TX)
- PWRKEY: GPIO4
- SLEEP/DTR: GPIO5

## Features

- Temperature, humidity, and pressure monitoring using BME280
- Ambient light measurement using TSL2561
- GPS location tracking using SIM7600 module
- WiFi connectivity for data transmission
- Secure HTTPS data upload to server
- Automatic network reconnection
- Detailed debug output via Serial Monitor

## Dependencies

- WiFi.h
- HTTPClient.h
- Wire.h
- Adafruit_BME280.h
- Adafruit_TSL2561_U.h
- WiFiClientSecure.h
- TinyGSM.h

## Setup

1. Install the required libraries using the Arduino Library Manager
2. Configure your WiFi credentials in the code
3. Update the server URL if needed
4. Upload the code to your ESP32
5. Monitor the Serial output for debugging information

## Data Format

The system sends data to the server with the following parameters:
- temperature (°C)
- humidity (%)
- pressure (hPa)
- light (lux)
- latitude (degrees)
- longitude (degrees)
- speed (km/h)
- altitude (m)

## License

This project is open source and available under the MIT License.
