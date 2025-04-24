import os
import threading
import queue
import time
import json
from flask import Flask, request, Response, jsonify
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from jackal_msgs.msg import Drive, Feedback, Status
from std_msgs.msg import Header

# --- Environment Variable Config ---
ROS_MASTER_URI = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
ROS_HOSTNAME = os.environ.get('ROS_HOSTNAME', 'localhost')
ROS_IP = os.environ.get('ROS_IP', '127.0.0.1')
SERVER_HOST = os.environ.get('SERVER_HOST', '0.0.0.0')
SERVER_PORT = int(os.environ.get('SERVER_PORT', '8080'))

# --- ROS Node Initialization (in a background thread to avoid blocking Flask) ---
os.environ['ROS_MASTER_URI'] = ROS_MASTER_URI
os.environ['ROS_HOSTNAME'] = ROS_HOSTNAME
os.environ['ROS_IP'] = ROS_IP

app = Flask(__name__)
ros_ready = threading.Event()

# --- Queues for Latest Topic Data ---
data_queues = {
    'odometry': queue.Queue(maxsize=1),
    'imu': queue.Queue(maxsize=1),
    'navsat_fix': queue.Queue(maxsize=1),
    'navsat_vel': queue.Queue(maxsize=1),
    'feedback': queue.Queue(maxsize=1),
    'status': queue.Queue(maxsize=1),
}

def put_latest(q, data):
    try:
        if q.full():
            q.get_nowait()
    except queue.Empty:
        pass
    q.put(data)

# --- ROS Subscriber Callbacks ---
def odometry_cb(msg):
    put_latest(data_queues['odometry'], msg)

def imu_cb(msg):
    put_latest(data_queues['imu'], msg)

def navsat_fix_cb(msg):
    put_latest(data_queues['navsat_fix'], msg)

def navsat_vel_cb(msg):
    put_latest(data_queues['navsat_vel'], msg)

def feedback_cb(msg):
    put_latest(data_queues['feedback'], msg)

def status_cb(msg):
    put_latest(data_queues['status'], msg)

def ros_thread():
    rospy.init_node('jackal_ugv_http_driver', anonymous=True, disable_signals=True)
    rospy.Subscriber('/odometry/filtered', Odometry, odometry_cb)
    rospy.Subscriber('/imu/data', Imu, imu_cb)
    rospy.Subscriber('/navsat/fix', NavSatFix, navsat_fix_cb)
    rospy.Subscriber('/navsat/vel', Twist, navsat_vel_cb)
    rospy.Subscriber('/feed_back', Feedback, feedback_cb)
    rospy.Subscriber('/status', Status, status_cb)
    global cmd_vel_pub, cmd_driver_pub
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    cmd_driver_pub = rospy.Publisher('/cmd_driver', Drive, queue_size=1)
    ros_ready.set()
    rospy.spin()

ros_bg_thread = threading.Thread(target=ros_thread, daemon=True)
ros_bg_thread.start()
ros_ready.wait(timeout=15)

def ros_to_dict(msg):
    # Recursively convert ROS message to dict
    if hasattr(msg, '__slots__'):
        result = {}
        for slot in msg.__slots__:
            val = getattr(msg, slot)
            result[slot] = ros_to_dict(val)
        return result
    elif isinstance(msg, (list, tuple)):
        return [ros_to_dict(x) for x in msg]
    elif isinstance(msg, (float, int, str, bool)):
        return msg
    else:
        return str(msg)

def get_latest(q):
    try:
        return q.queue[-1]
    except (IndexError, AttributeError):
        return None

# --- API Endpoints ---

@app.route('/move', methods=['POST'])
def move():
    ros_ready.wait(timeout=5)
    data = request.get_json(force=True)
    try:
        linear = data.get('linear', {})
        angular = data.get('angular', {})
        twist = Twist()
        twist.linear.x = float(linear.get('x', 0))
        twist.linear.y = float(linear.get('y', 0))
        twist.linear.z = float(linear.get('z', 0))
        twist.angular.x = float(angular.get('x', 0))
        twist.angular.y = float(angular.get('y', 0))
        twist.angular.z = float(angular.get('z', 0))
        cmd_vel_pub.publish(twist)
        return jsonify({"result": "success"}), 200
    except Exception as e:
        return jsonify({"result": "error", "reason": str(e)}), 400

@app.route('/drive', methods=['POST'])
def drive():
    ros_ready.wait(timeout=5)
    data = request.get_json(force=True)
    try:
        drive_msg = Drive()
        # Allow generic mapping: all Drive fields can be set by JSON keys
        for field in drive_msg.__slots__:
            if field in data:
                setattr(drive_msg, field, data[field])
        cmd_driver_pub.publish(drive_msg)
        return jsonify({"result": "success"}), 200
    except Exception as e:
        return jsonify({"result": "error", "reason": str(e)}), 400

@app.route('/telemetry', methods=['GET'])
def telemetry():
    ros_ready.wait(timeout=5)
    telemetry_data = {}
    odom = get_latest(data_queues['odometry'])
    imu = get_latest(data_queues['imu'])
    navsat_fix = get_latest(data_queues['navsat_fix'])
    navsat_vel = get_latest(data_queues['navsat_vel'])
    feedback = get_latest(data_queues['feedback'])
    status = get_latest(data_queues['status'])
    telemetry_data['odometry'] = ros_to_dict(odom) if odom else None
    telemetry_data['imu'] = ros_to_dict(imu) if imu else None
    telemetry_data['navsat_fix'] = ros_to_dict(navsat_fix) if navsat_fix else None
    telemetry_data['navsat_vel'] = ros_to_dict(navsat_vel) if navsat_vel else None
    telemetry_data['feedback'] = ros_to_dict(feedback) if feedback else None
    telemetry_data['status'] = ros_to_dict(status) if status else None
    return jsonify(telemetry_data), 200

if __name__ == '__main__':
    app.run(host=SERVER_HOST, port=SERVER_PORT, threaded=True)