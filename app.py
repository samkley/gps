from flask import Flask, request, render_template, jsonify
import os
from datetime import datetime
from flask_sqlalchemy import SQLAlchemy
import logging

app = Flask(__name__)

# Configure SQLAlchemy with PostgreSQL (you'll need to set this environment variable in Render)
app.config['SQLALCHEMY_DATABASE_URI'] = os.getenv('DATABASE_URL', 'sqlite:///gps_data.db')
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False

db = SQLAlchemy(app)

# Database Model
class GPSData(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    timestamp = db.Column(db.DateTime, nullable=False, default=datetime.utcnow)
    latitude = db.Column(db.Float, nullable=False)
    longitude = db.Column(db.Float, nullable=False)
    speed = db.Column(db.Float)
    altitude = db.Column(db.Float)
    temperature = db.Column(db.Float)
    humidity = db.Column(db.Float)
    light = db.Column(db.Float)
    current = db.Column(db.Float)
    voltage = db.Column(db.Float)

    def to_dict(self):
        return {
            'timestamp': self.timestamp.strftime('%Y-%m-%d %H:%M:%S'),
            'latitude': self.latitude,
            'longitude': self.longitude,
            'speed': self.speed,
            'altitude': self.altitude,
            'temperature': self.temperature,
            'humidity': self.humidity,
            'light': self.light,
            'current': self.current,
            'voltage': self.voltage
        }

# Set up logging
logging.basicConfig(level=logging.INFO)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/receive_data', methods=['POST'])
def receive_data():
    try:
        data = request.get_json()
        logging.info(f"Received data: {data}")  # Log the received data

        gps_data = GPSData(
            latitude=data['latitude'],
            longitude=data['longitude'],
            speed=data['speed'],
            altitude=data['altitude'],
            temperature=data['temperature'],
            humidity=data['humidity'],
            light=data['light'],
            current=data['current'],
            voltage=data['voltage']
        )
        
        db.session.add(gps_data)
        db.session.commit()

        return jsonify({'status': 'success'}), 200
    except Exception as e:
        logging.error(f"Error receiving data: {str(e)}")  # Log errors
        return jsonify({'status': 'error', 'message': str(e)}), 400

@app.route('/get_data')
def get_data():
    # Get the last 100 data points
    data_points = GPSData.query.order_by(GPSData.timestamp.desc()).limit(100).all()
    return jsonify([point.to_dict() for point in data_points])

@app.route('/latest_data')
def latest_data():
    latest = GPSData.query.order_by(GPSData.timestamp.desc()).first()
    return jsonify(latest.to_dict() if latest else {})

if __name__ == '__main__':
    with app.app_context():
        db.create_all()
    app.run(host='0.0.0.0', port=int(os.getenv('PORT', 5001)))
