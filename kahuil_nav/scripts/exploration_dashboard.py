#!/usr/bin/env python3
"""
Web Dashboard for SLAM Exploration Monitoring
Flask server with WebSocket for real-time visualization.
Ported from rastec — adapted to kahuil global topics.
"""

import rclpy
from rclpy.node import Node
from flask import Flask, render_template_string
from flask_socketio import SocketIO
import threading

from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>Kahuil Explorer Dashboard</title>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <script src="https://cdn.socket.io/4.5.4/socket.io.min.js"></script>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }
        .container {
            max-width: 800px;
            margin: 0 auto;
            background: rgba(255, 255, 255, 0.95);
            border-radius: 20px;
            padding: 30px;
            box-shadow: 0 20px 60px rgba(0,0,0,0.3);
        }
        h1 {
            color: #667eea;
            text-align: center;
            margin-bottom: 30px;
            font-size: 2em;
        }
        .status-card {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 20px;
            border-radius: 15px;
            margin-bottom: 20px;
        }
        .status-card h2 {
            font-size: 1.2em;
            margin-bottom: 10px;
        }
        .status-value {
            font-size: 2em;
            font-weight: bold;
        }
        .sensor-bar {
            background: #f0f0f0;
            border-radius: 10px;
            padding: 15px;
            margin: 10px 0;
        }
        .sensor-label {
            font-weight: bold;
            margin-bottom: 5px;
            color: #333;
        }
        .bar-container {
            background: #ddd;
            border-radius: 5px;
            height: 30px;
            position: relative;
            overflow: hidden;
        }
        .bar-fill {
            height: 100%;
            background: linear-gradient(90deg, #667eea, #764ba2);
            transition: width 0.3s ease;
            display: flex;
            align-items: center;
            justify-content: center;
            color: white;
            font-weight: bold;
        }
        .controls {
            display: flex;
            gap: 10px;
            margin-top: 20px;
        }
        button {
            flex: 1;
            padding: 15px;
            border: none;
            border-radius: 10px;
            font-size: 1em;
            font-weight: bold;
            cursor: pointer;
            transition: transform 0.2s;
        }
        button:hover {
            transform: scale(1.05);
        }
        .btn-pause {
            background: #ffa500;
            color: white;
        }
        .btn-clear {
            background: #ff6b6b;
            color: white;
        }
        .grid {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 15px;
            margin-bottom: 20px;
        }
        .metric {
            background: #f8f9fa;
            padding: 15px;
            border-radius: 10px;
            text-align: center;
        }
        .metric-label {
            color: #666;
            font-size: 0.9em;
        }
        .metric-value {
            font-size: 1.5em;
            font-weight: bold;
            color: #667eea;
            margin-top: 5px;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Kahuil Explorer Dashboard</h1>

        <div class="status-card">
            <h2>Current State</h2>
            <div class="status-value" id="status">Connecting...</div>
        </div>

        <div class="grid">
            <div class="metric">
                <div class="metric-label">Position X</div>
                <div class="metric-value" id="pos-x">0.0</div>
            </div>
            <div class="metric">
                <div class="metric-label">Position Y</div>
                <div class="metric-value" id="pos-y">0.0</div>
            </div>
            <div class="metric">
                <div class="metric-label">Frontiers</div>
                <div class="metric-value" id="frontiers">0</div>
            </div>
            <div class="metric">
                <div class="metric-label">Visited</div>
                <div class="metric-value" id="visited">0</div>
            </div>
        </div>

        <div class="sensor-bar">
            <div class="sensor-label">Front Sensor</div>
            <div class="bar-container">
                <div class="bar-fill" id="bar-front" style="width: 0%">0.0m</div>
            </div>
        </div>

        <div class="sensor-bar">
            <div class="sensor-label">Left Sensor</div>
            <div class="bar-container">
                <div class="bar-fill" id="bar-left" style="width: 0%">0.0m</div>
            </div>
        </div>

        <div class="sensor-bar">
            <div class="sensor-label">Right Sensor</div>
            <div class="bar-container">
                <div class="bar-fill" id="bar-right" style="width: 0%">0.0m</div>
            </div>
        </div>

        <div class="controls">
            <button class="btn-pause" onclick="togglePause()">Pause</button>
            <button class="btn-clear" onclick="clearHistory()">Clear</button>
        </div>
    </div>

    <script>
        const socket = io();

        socket.on('update', function(data) {
            document.getElementById('status').textContent = data.status || 'Exploring';
            document.getElementById('pos-x').textContent = data.pos_x?.toFixed(2) || '0.0';
            document.getElementById('pos-y').textContent = data.pos_y?.toFixed(2) || '0.0';
            document.getElementById('frontiers').textContent = data.frontiers || '0';
            document.getElementById('visited').textContent = data.visited || '0';

            updateBar('front', data.dist_front);
            updateBar('left', data.dist_left);
            updateBar('right', data.dist_right);
        });

        function updateBar(sensor, distance) {
            const maxDist = 5.0;
            const dist = Math.min(distance || 0, maxDist);
            const percent = (dist / maxDist) * 100;
            const bar = document.getElementById('bar-' + sensor);
            bar.style.width = percent + '%';
            bar.textContent = dist.toFixed(2) + 'm';
        }

        function togglePause() {
            socket.emit('toggle_pause');
        }

        function clearHistory() {
            socket.emit('clear_history');
        }
    </script>
</body>
</html>
"""

class DashboardNode(Node):
    def __init__(self):
        super().__init__('exploration_dashboard')

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.min_front = 0.0
        self.min_left = 0.0
        self.min_right = 0.0
        self.frontiers_count = 0
        self.visited_count = 0
        self.status = "Exploring"

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriptions — kahuil global topics
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)

        self.timer = self.create_timer(0.5, self.update_position)

        self.get_logger().info('Dashboard node started')

    def laser_callback(self, msg):
        """Process laser data"""
        if len(msg.ranges) < 10:
            return

        n = len(msg.ranges)
        front_start = n // 3
        front_end = 2 * n // 3

        def safe_min(ranges):
            valid = [r for r in ranges if msg.range_min < r < msg.range_max]
            return min(valid) if valid else 0.0

        self.min_front = safe_min(msg.ranges[front_start:front_end])
        self.min_right = safe_min(msg.ranges[:front_start])
        self.min_left = safe_min(msg.ranges[front_end:])

    def update_position(self):
        """Update robot position"""
        try:
            transform = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            self.robot_x = transform.transform.translation.x
            self.robot_y = transform.transform.translation.y
        except Exception:
            pass

    def get_data(self):
        """Return current data"""
        return {
            'status': self.status,
            'pos_x': self.robot_x,
            'pos_y': self.robot_y,
            'dist_front': self.min_front,
            'dist_left': self.min_left,
            'dist_right': self.min_right,
            'frontiers': self.frontiers_count,
            'visited': self.visited_count
        }

# Flask app
app = Flask(__name__)
app.config['SECRET_KEY'] = 'kahuil_dashboard'
socketio = SocketIO(app, cors_allowed_origins="*")

dashboard_node = None

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@socketio.on('connect')
def handle_connect():
    print('Client connected')

@socketio.on('toggle_pause')
def handle_pause():
    print('Toggle pause requested')

@socketio.on('clear_history')
def handle_clear():
    print('Clear history requested')

def ros_spin():
    """Thread for ROS2"""
    rclpy.spin(dashboard_node)

def emit_updates():
    """Emit periodic updates"""
    while True:
        if dashboard_node:
            data = dashboard_node.get_data()
            socketio.emit('update', data)
        socketio.sleep(0.5)

def main():
    global dashboard_node

    rclpy.init()
    dashboard_node = DashboardNode()

    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    update_thread = threading.Thread(target=emit_updates, daemon=True)
    update_thread.start()

    print('\n' + '='*50)
    print('Dashboard: http://localhost:5000')
    print('='*50 + '\n')

    socketio.run(app, host='0.0.0.0', port=5000, debug=False, allow_unsafe_werkzeug=True)

if __name__ == '__main__':
    main()
