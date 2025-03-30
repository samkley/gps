from flask import Flask, request, render_template
from datetime import datetime
from flask_sqlalchemy import SQLAlchemy
import os

app = Flask(__name__)

# Configure the database URI
app.config['SQLALCHEMY_DATABASE_URI'] = os.getenv('DATABASE_URL', 'sqlite:///gps_data.db')
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
db = SQLAlchemy(app)

# Define the database model
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

@app.route('/')
def index():
    # Fetch the latest data from the database
    latest_data = GPSData.query.order_by(GPSData.timestamp.desc()).first()
    # Fetch the last 100 data points for history
    data_history = GPSData.query.order_by(GPSData.timestamp.desc()).limit(100).all()
    return render_template('index.html', latest_data=latest_data, data_history=data_history)

@app.route('/receive_data', methods=['POST'])
def receive_data():
    data = request.get_json()
    
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
    
    return {'status': 'success'}, 200

if __name__ == '__main__':
    db.create_all()  # Create the database tables if they don't exist
    app.run(host='0.0.0.0', port=5001, debug=True)
