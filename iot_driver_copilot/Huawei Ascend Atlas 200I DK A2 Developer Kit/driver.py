import os
import io
import threading
import time
import base64
from flask import Flask, request, jsonify, Response, send_file
import serial
import serial.tools.list_ports
import yaml

# ENVIRONMENT VARIABLES
HTTP_HOST = os.environ.get("HTTP_HOST", "0.0.0.0")
HTTP_PORT = int(os.environ.get("HTTP_PORT", "8080"))
SERIAL_PORT = os.environ.get("SERIAL_PORT", "/dev/ttyUSB0")
SERIAL_BAUDRATE = int(os.environ.get("SERIAL_BAUDRATE", "115200"))
SERIAL_TIMEOUT = float(os.environ.get("SERIAL_TIMEOUT", "1.0"))
MAP_DIR = os.environ.get("MAP_DIR", "./maps")
FIRMWARE_UPLOAD_DIR = os.environ.get("FIRMWARE_UPLOAD_DIR", "./firmware")

# Ensure directories exist
os.makedirs(MAP_DIR, exist_ok=True)
os.makedirs(FIRMWARE_UPLOAD_DIR, exist_ok=True)

app = Flask(__name__)

# SERIAL CONNECTION MANAGEMENT
serial_lock = threading.Lock()
serial_conn = None

def get_serial_connection():
    global serial_conn
    if serial_conn is None or not serial_conn.is_open:
        serial_conn = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=SERIAL_TIMEOUT)
    return serial_conn

def send_serial_command(command, expect_response=True, response_timeout=2):
    with serial_lock:
        conn = get_serial_connection()
        conn.write((command + '\n').encode('utf-8'))
        if expect_response:
            conn.flush()
            deadline = time.time() + response_timeout
            lines = []
            while time.time() < deadline:
                line = conn.readline()
                if line:
                    lines.append(line.decode('utf-8').strip())
                else:
                    break
            return '\n'.join(lines)
        return None

# LIDAR DATA STREAMING
lidar_streaming = False
lidar_data_buffer = []
lidar_thread = None

def lidar_data_reader():
    global lidar_streaming, lidar_data_buffer
    while lidar_streaming:
        response = send_serial_command("GET_LIDAR_SCAN", expect_response=True, response_timeout=0.2)
        if response:
            lidar_data_buffer.append(response)
            if len(lidar_data_buffer) > 100:
                lidar_data_buffer = lidar_data_buffer[-100:]
        time.sleep(0.1)

@app.route('/lidar/start', methods=['POST'])
def lidar_start():
    global lidar_streaming, lidar_thread
    if not lidar_streaming:
        resp = send_serial_command("LIDAR_START", expect_response=True)
        lidar_streaming = True
        lidar_thread = threading.Thread(target=lidar_data_reader, daemon=True)
        lidar_thread.start()
        return jsonify({"status": "LIDAR started", "device_response": resp})
    else:
        return jsonify({"status": "LIDAR already running"})

@app.route('/lidar/stop', methods=['POST'])
def lidar_stop():
    global lidar_streaming
    if lidar_streaming:
        resp = send_serial_command("LIDAR_STOP", expect_response=True)
        lidar_streaming = False
        return jsonify({"status": "LIDAR stopped", "device_response": resp})
    else:
        return jsonify({"status": "LIDAR not running"})

@app.route('/lidar/stream', methods=['GET'])
def lidar_stream():
    def generate():
        last_index = 0
        while lidar_streaming:
            if last_index < len(lidar_data_buffer):
                data = lidar_data_buffer[last_index]
                last_index += 1
                yield f"data: {data}\n\n"
            else:
                time.sleep(0.1)
    if not lidar_streaming:
        return jsonify({"error": "LIDAR is not running"}), 400
    return Response(generate(), content_type='text/event-stream')

# MAP GENERATION & SAVING

mapping_active = False
generated_map = None  # Simulation: {'pgm': bytes, 'yaml': str}

@app.route('/map/launch', methods=['POST'])
def map_launch():
    global mapping_active, generated_map
    if not mapping_active:
        mapping_active = True
        # Simulate map generation (replace with real device code)
        # Here, send mapping start command
        resp = send_serial_command("MAP_LAUNCH", expect_response=True)
        # Simulate map files
        generated_map = {
            'pgm': b'P5\n# Simulated map\n10 10\n255\n' + bytes([255]*100),
            'yaml': yaml.dump({'image': 'map.pgm', 'resolution': 0.05, 'origin': [0,0,0], 'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196})
        }
        return jsonify({"status": "Mapping started", "device_response": resp})
    else:
        return jsonify({"status": "Mapping already active"})

@app.route('/map/save', methods=['POST'])
def map_save():
    global mapping_active, generated_map
    if generated_map is None:
        return jsonify({"error": "No generated map to save"}), 404
    timestamp = int(time.time())
    pgm_path = os.path.join(MAP_DIR, f"map_{timestamp}.pgm")
    yaml_path = os.path.join(MAP_DIR, f"map_{timestamp}.yaml")
    with open(pgm_path, 'wb') as f:
        f.write(generated_map['pgm'])
    with open(yaml_path, 'w') as f:
        f.write(generated_map['yaml'])
    mapping_active = False
    return jsonify({"status": "Map saved", "pgm_file": pgm_path, "yaml_file": yaml_path})

@app.route('/map/files', methods=['GET'])
def list_map_files():
    files = [f for f in os.listdir(MAP_DIR) if f.endswith('.pgm') or f.endswith('.yaml')]
    return jsonify({"map_files": files})

@app.route('/map/files/<filename>', methods=['GET'])
def get_map_file(filename):
    filepath = os.path.join(MAP_DIR, filename)
    if os.path.isfile(filepath):
        return send_file(filepath)
    else:
        return jsonify({"error": "File not found"}), 404

# NAVIGATION

@app.route('/nav/launch', methods=['POST'])
def nav_launch():
    resp = send_serial_command("NAV_LAUNCH", expect_response=True)
    return jsonify({"status": "Navigation launched", "device_response": resp})

# LOCALIZATION

@app.route('/loc/launch', methods=['POST'])
def loc_launch():
    resp = send_serial_command("LOC_LAUNCH", expect_response=True)
    return jsonify({"status": "Localization launched", "device_response": resp})

# TELEOPERATION

@app.route('/teleop', methods=['POST'])
def teleop():
    # Expecting JSON: { "linear": x, "angular": y }
    cmd = request.json
    if not cmd or 'linear' not in cmd or 'angular' not in cmd:
        return jsonify({"error": "Invalid teleop command"}), 400
    command_str = f"TELEOP {cmd['linear']} {cmd['angular']}"
    resp = send_serial_command(command_str, expect_response=True)
    return jsonify({"status": "Teleop command sent", "device_response": resp})

# FIRMWARE UPLOAD

@app.route('/firmware', methods=['POST'])
def firmware_upload():
    if 'file' not in request.files:
        return jsonify({"error": "No file uploaded"}), 400
    file = request.files['file']
    filename = file.filename
    if not filename:
        return jsonify({"error": "No filename specified"}), 400
    save_path = os.path.join(FIRMWARE_UPLOAD_DIR, filename)
    file.save(save_path)
    # Simulate firmware flash
    resp = send_serial_command(f"FIRMWARE_UPLOAD {save_path}", expect_response=True)
    return jsonify({"status": "Firmware uploaded", "filename": filename, "device_response": resp})

# ROOT
@app.route("/", methods=["GET"])
def root():
    return jsonify({
        "device": "Huawei Ascend Atlas 200I DK A2 Developer Kit",
        "api": [
            {"method": "POST", "path": "/map/launch", "description": "Launch mapping process"},
            {"method": "POST", "path": "/lidar/start", "description": "Start LIDAR sensor"},
            {"method": "POST", "path": "/lidar/stop", "description": "Stop LIDAR sensor"},
            {"method": "GET",  "path": "/lidar/stream", "description": "HTTP event stream of LIDAR data"},
            {"method": "POST", "path": "/teleop", "description": "Manual teleop control"},
            {"method": "POST", "path": "/nav/launch", "description": "Launch navigation nodes"},
            {"method": "POST", "path": "/map/save", "description": "Save map to file"},
            {"method": "GET",  "path": "/map/files", "description": "List generated map files"},
            {"method": "GET",  "path": "/map/files/<filename>", "description": "Get a map file"},
            {"method": "POST", "path": "/loc/launch", "description": "Start localization"},
            {"method": "POST", "path": "/firmware", "description": "Upload firmware to device"}
        ]
    })

if __name__ == "__main__":
    app.run(host=HTTP_HOST, port=HTTP_PORT, threaded=True)