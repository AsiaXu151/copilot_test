import os
import io
import threading
import time
from flask import Flask, request, jsonify, Response, send_file
import serial
import yaml
import tempfile

# ========== Configuration from environment variables ==========
SERVER_HOST = os.environ.get("DRIVER_HTTP_HOST", "0.0.0.0")
SERVER_PORT = int(os.environ.get("DRIVER_HTTP_PORT", "8080"))

# Serial/UART configuration for LIDAR (example values, override with env vars if needed)
LIDAR_SERIAL_PORT = os.environ.get("LIDAR_SERIAL_PORT", "/dev/ttyUSB0")
LIDAR_BAUDRATE = int(os.environ.get("LIDAR_BAUDRATE", "115200"))

# Path for temporary map files
MAP_SAVE_DIR = os.environ.get("MAP_SAVE_DIR", "/tmp")

# ========== Global State ==========
lidar_thread = None
lidar_running = False
lidar_data_buffer = []
lidar_lock = threading.Lock()
map_files = {}
teleop_last_cmd = None

# ========== LIDAR Serial Communication ==========
def read_lidar_serial():
    global lidar_running, lidar_data_buffer
    try:
        with serial.Serial(LIDAR_SERIAL_PORT, LIDAR_BAUDRATE, timeout=1) as ser:
            while lidar_running:
                line = ser.readline()
                if line:
                    with lidar_lock:
                        lidar_data_buffer.append(line)
                        # Keep buffer size reasonable
                        if len(lidar_data_buffer) > 1000:
                            lidar_data_buffer = lidar_data_buffer[-1000:]
    except Exception as e:
        with lidar_lock:
            lidar_data_buffer.append(b"ERROR: " + str(e).encode())

def start_lidar():
    global lidar_running, lidar_thread, lidar_data_buffer
    if lidar_running:
        return
    lidar_data_buffer = []
    lidar_running = True
    lidar_thread = threading.Thread(target=read_lidar_serial, daemon=True)
    lidar_thread.start()

def stop_lidar():
    global lidar_running, lidar_thread
    lidar_running = False
    if lidar_thread:
        lidar_thread.join(timeout=2)
        lidar_thread = None

# ========== Flask HTTP API ==========
app = Flask(__name__)

@app.route("/map/launch", methods=["POST"])
def map_launch():
    # Simulate map building from lidar data
    if not lidar_running:
        return jsonify({"error": "LIDAR is not running"}), 400
    # For demo: Save fake map files
    pgm_path = os.path.join(MAP_SAVE_DIR, "map.pgm")
    yaml_path = os.path.join(MAP_SAVE_DIR, "map.yaml")
    with open(pgm_path, "wb") as f:
        f.write(b"P5\n# Demo map\n10 10\n255\n" + b"\x80" * 100)
    with open(yaml_path, "w") as f:
        yaml.dump({"image": "map.pgm", "resolution": 0.05, "origin": [0,0,0]}, f)
    map_files['pgm'] = pgm_path
    map_files['yaml'] = yaml_path
    return jsonify({"status": "mapping launched", "map_files": ["map.pgm", "map.yaml"]})

@app.route("/lidar/start", methods=["POST"])
def lidar_start():
    start_lidar()
    return jsonify({"status": "LIDAR started"})

@app.route("/lidar/stop", methods=["POST"])
def lidar_stop():
    stop_lidar()
    return jsonify({"status": "LIDAR stopped"})

@app.route("/lidar/stream", methods=["GET"])
def lidar_stream():
    def stream():
        last_idx = 0
        while lidar_running:
            time.sleep(0.1)
            with lidar_lock:
                if last_idx < len(lidar_data_buffer):
                    data = lidar_data_buffer[last_idx:]
                    last_idx = len(lidar_data_buffer)
                    for line in data:
                        yield b"data: " + line.strip() + b"\n\n"
    if not lidar_running:
        return jsonify({"error": "LIDAR is not running"}), 400
    return Response(stream(), mimetype="text/event-stream")

@app.route("/teleop", methods=["POST"])
def teleop():
    global teleop_last_cmd
    cmd = request.get_json(force=True)
    teleop_last_cmd = cmd
    # Here you would send the command to the Arduino over serial (not implemented)
    return jsonify({"status": "teleop command sent", "command": cmd})

@app.route("/nav/launch", methods=["POST"])
def nav_launch():
    # Simulate navigation node activation
    return jsonify({"status": "navigation launched"})

@app.route("/map/save", methods=["POST"])
def map_save():
    if 'pgm' not in map_files or 'yaml' not in map_files:
        return jsonify({"error": "No map to save"}), 400
    # For demo, just confirm the files exist
    return jsonify({"status": "map saved", "files": [map_files['pgm'], map_files['yaml']]})

@app.route("/loc/launch", methods=["POST"])
def loc_launch():
    # Simulate localization process
    return jsonify({"status": "localization launched"})

@app.route("/firmware", methods=["POST"])
def firmware_upload():
    if 'file' not in request.files:
        return jsonify({"error": "No firmware file uploaded"}), 400
    fw = request.files['file']
    fw_path = os.path.join(MAP_SAVE_DIR, fw.filename)
    fw.save(fw_path)
    # Actual firmware flashing not implemented for safety
    return jsonify({"status": "firmware uploaded", "file": fw.filename})

@app.route("/map/pgm", methods=["GET"])
def get_map_pgm():
    if 'pgm' not in map_files:
        return jsonify({"error": "No map available"}), 404
    return send_file(map_files['pgm'], mimetype="image/x-portable-graymap")

@app.route("/map/yaml", methods=["GET"])
def get_map_yaml():
    if 'yaml' not in map_files:
        return jsonify({"error": "No map available"}), 404
    return send_file(map_files['yaml'], mimetype="text/yaml")

@app.route("/lidar/raw", methods=["GET"])
def lidar_raw():
    # Returns last 100 lines of raw LIDAR data in plain text
    with lidar_lock:
        lines = lidar_data_buffer[-100:]
    return Response(b"".join(lines), mimetype="text/plain")

@app.route("/")
def index():
    return """
    <h1>Huawei Atlas 200I DK A2 LIDAR/Robot HTTP Driver</h1>
    <ul>
        <li>POST /lidar/start</li>
        <li>POST /lidar/stop</li>
        <li>GET  /lidar/stream (SSE, browser/CLI)</li>
        <li>GET  /lidar/raw (raw text)</li>
        <li>POST /map/launch</li>
        <li>GET  /map/pgm</li>
        <li>GET  /map/yaml</li>
        <li>POST /map/save</li>
        <li>POST /nav/launch</li>
        <li>POST /loc/launch</li>
        <li>POST /teleop (JSON body)</li>
        <li>POST /firmware (multipart form-data)</li>
    </ul>
    """

if __name__ == "__main__":
    app.run(host=SERVER_HOST, port=SERVER_PORT)