import express from 'express';
import cors from 'cors';
import { fileURLToPath } from 'url';
import { dirname, join } from 'path';

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);

const app = express();
const port = process.env.PORT || 3000;

// Store the latest data in memory
let latestData = {
  location: {
    latitude: 0,
    longitude: 0,
    speed: 0,
    altitude: 0
  },
  environmental: {
    temperature: 0,
    humidity: 0,
    pressure: 0
  },
  power: {
    current: 0,
    voltage: 0
  },
  lastUpdate: null
};

app.use(cors());
app.use(express.json());
app.use(express.static(join(__dirname, '../../public')));

// API endpoint to receive data from ESP32
app.get('/api/update', (req, res) => {
  const { 
    latitude, longitude, speed, altitude,
    temperature, humidity, pressure,
    current, voltage 
  } = req.query;

  // Update the latest data
  latestData = {
    location: {
      latitude: parseFloat(latitude) || 0,
      longitude: parseFloat(longitude) || 0,
      speed: parseFloat(speed) || 0,
      altitude: parseFloat(altitude) || 0
    },
    environmental: {
      temperature: parseFloat(temperature) || 0,
      humidity: parseFloat(humidity) || 0,
      pressure: parseFloat(pressure) || 0
    },
    power: {
      current: parseFloat(current) || 0,
      voltage: parseFloat(voltage) || 0
    },
    lastUpdate: new Date().toISOString()
  };

  res.json({ status: 'success', timestamp: latestData.lastUpdate });
});

// API endpoint to get the latest data
app.get('/api/data', (req, res) => {
  res.json(latestData);
});

// Catch-all route to serve the frontend
app.get('*', (req, res) => {
  res.sendFile(join(__dirname, '../../public/index.html'));
});

app.listen(port, () => {
  console.log(`Server running on port ${port}`);
}); 