from flask import Flask, request, render_template_string
from datetime import datetime
import json

app = Flask(__name__)

# Store the last 100 data points
data_history = []
MAX_HISTORY = 100

# HTML template for the dashboard
HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <title>GPS Tracker Dashboard</title>
    <meta http-equiv="refresh" content="5">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; }
        .data-container { 
            display: grid;
            grid-template-columns: repeat(auto-fill, minmax(300px, 1fr));
            gap: 20px;
            margin-bottom: 20px;
        }
        .data-card {
            border: 1px solid #ddd;
            padding: 15px;
            border-radius: 8px;
            background-color: #f9f9f9;
        }
        .history {
            border: 1px solid #ddd;
            padding: 15px;
            border-radius: 8px;
            margin-top: 20px;
        }
        table {
            width: 100%;
            border-collapse: collapse;
        }
        th, td {
            border: 1px solid #ddd;
            padding: 8px;
            text-align: left;
        }
        th { background-color: #f2f2f2; }
    </style>
</head>
<body>
    <h1>GPS Tracker Dashboard</h1>
    
    {% if latest_data %}
    <div class="data-container">
        <div class="data-card">
            <h2>Location Data</h2>
            <p>Latitude: {{ latest_data.latitude }}°</p>
            <p>Longitude: {{ latest_data.longitude }}°</p>
            <p>Altitude: {{ latest_data.altitude }} m</p>
            <p>Speed: {{ latest_data.speed }} km/h</p>
        </div>
        
        <div class="data-card">
            <h2>Environmental Data</h2>
            <p>Temperature: {{ latest_data.temp }}°C</p>
            <p>Humidity: {{ latest_data.humidity }}%</p>
            <p>Light: {{ latest_data.light }} lux</p>
        </div>
        
        <div class="data-card">
            <h2>Power Data</h2>
            <p>Current: {{ latest_data.current }} mA</p>
            <p>Voltage: {{ latest_data.voltage }} V</p>
        </div>
    </div>
    {% else %}
    <p>Waiting for data...</p>
    {% endif %}
    
    <div class="history">
        <h2>Data History</h2>
        <table>
            <tr>
                <th>Time</th>
                <th>Location</th>
                <th>Speed</th>
                <th>Temperature</th>
                <th>Humidity</th>
                <th>Light</th>
            </tr>
            {% for entry in data_history %}
            <tr>
                <td>{{ entry.timestamp }}</td>
                <td>{{ entry.data.latitude }}, {{ entry.data.longitude }}</td>
                <td>{{ entry.data.speed }} km/h</td>
                <td>{{ entry.data.temp }}°C</td>
                <td>{{ entry.data.humidity }}%</td>
                <td>{{ entry.data.light }} lux</td>
            </tr>
            {% endfor %}
        </table>
    </div>
</body>
</html>
'''

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE, 
                                latest_data=data_history[-1].get('data') if data_history else None,
                                data_history=data_history)

@app.route('/receive_data', methods=['POST'])
def receive_data():
    data = request.get_json()
    
    # Add timestamp to the data
    entry = {
        'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
        'data': data
    }
    
    # Add to history and maintain max size
    data_history.append(entry)
    if len(data_history) > MAX_HISTORY:
        data_history.pop(0)
    
    return {'status': 'success'}, 200

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001, debug=True)
