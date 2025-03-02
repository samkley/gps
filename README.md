# GPS Tracker Dashboard

A real-time GPS tracking dashboard that displays location, environmental, and power data from an ESP32 device.

## Features

- Real-time GPS tracking with map display
- Environmental data monitoring (temperature, humidity, pressure)
- Power monitoring (current, voltage)
- Automatic data updates every 5 seconds
- Responsive design for mobile and desktop

## Setup

### Prerequisites

- Node.js (v14 or higher)
- npm (comes with Node.js)

### Installation

1. Clone the repository:
   ```bash
   git clone <your-repo-url>
   cd gps-tracker
   ```

2. Install dependencies:
   ```bash
   cd server
   npm install
   ```

3. Start the server:
   ```bash
   npm start
   ```

The server will start on port 3000 by default. You can change this by setting the `PORT` environment variable.

## ESP32 Configuration

The ESP32 device should send data to the `/api/update` endpoint with the following query parameters:

- `latitude`: GPS latitude (decimal degrees)
- `longitude`: GPS longitude (decimal degrees)
- `speed`: Speed in km/h
- `altitude`: Altitude in meters
- `temperature`: Temperature in °C
- `humidity`: Humidity in %
- `pressure`: Atmospheric pressure in hPa
- `current`: Current in amperes
- `voltage`: Voltage in volts

Example URL:
```
https://your-domain.com/api/update?latitude=51.5074&longitude=-0.1278&speed=0&altitude=10&temperature=25&humidity=60&pressure=1013&current=0.5&voltage=3.3
```

## Deployment

This application can be deployed to Render.com:

1. Create a new Web Service on Render
2. Connect your GitHub repository
3. Use the following settings:
   - Build Command: `cd server && npm install`
   - Start Command: `cd server && npm start`
   - Environment Variable: `PORT=10000`

## License

MIT
