import os
import threading
import time
import queue
from flask import Flask, request, jsonify, Response
import json

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from jackal_msgs.msg import Drive, Feedback, Status

# ==== Configuration from environment ====
ROS_MASTER_URI = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
ROS_IP = os.environ.get("ROS_IP", "127.0.0.1")

HTTP_SERVER_HOST = os.environ.get("HTTP_SERVER_HOST", "0.0.0.0")
HTTP_SERVER_PORT = int(os.environ.get("HTTP_SERVER_PORT", "8080"))

# Topics
ODOM_TOPIC = os.environ.get("ODOM_TOPIC", "/odometry/filtered")
IMU_TOPIC = os.environ.get("IMU_TOPIC", "/imu/data")
NAVSAT_FIX_TOPIC = os.environ.get("NAVSAT_FIX_TOPIC", "/navsat/fix")
NAVSAT_VEL_TOPIC = os.environ.get("NAVSAT_VEL_TOPIC", "/navsat/vel")
FEEDBACK_TOPIC = os.environ.get("FEEDBACK_TOPIC", "/feed_back")
STATUS_TOPIC = os.environ.get("STATUS_TOPIC", "/status")
CMD_VEL_TOPIC = os.environ.get("CMD_VEL_TOPIC", "/cmd_vel")
CMD_DRIVER_TOPIC = os.environ.get("CMD_DRIVER_TOPIC", "/cmd_driver")

# ==== ROS Node Setup ====
os.environ["ROS_MASTER_URI"] = ROS_MASTER_URI
os.environ["ROS_IP"] = ROS_IP

app = Flask(__name__)

# Shared state for telemetry
telemetry_state = {
    "odometry": None,
    "imu": None,
    "navsat_fix": None,
    "navsat_vel": None,
    "feedback": None,
    "status": None
}
telemetry_lock = threading.Lock()

# --- ROS Callbacks ---
def odom_callback(msg):
    with telemetry_lock:
        telemetry_state["odometry"] = rosmsg_to_dict(msg)

def imu_callback(msg):
    with telemetry_lock:
        telemetry_state["imu"] = rosmsg_to_dict(msg)

def navsat_fix_callback(msg):
    with telemetry_lock:
        telemetry_state["navsat_fix"] = rosmsg_to_dict(msg)

def navsat_vel_callback(msg):
    with telemetry_lock:
        telemetry_state["navsat_vel"] = rosmsg_to_dict(msg)

def feedback_callback(msg):
    with telemetry_lock:
        telemetry_state["feedback"] = rosmsg_to_dict(msg)

def status_callback(msg):
    with telemetry_lock:
        telemetry_state["status"] = rosmsg_to_dict(msg)

def rosmsg_to_dict(msg):
    # Recursively convert ROS message to Python dictionary
    if hasattr(msg, '__slots__'):
        result = {}
        for slot in msg.__slots__:
            attr = getattr(msg, slot)
            if isinstance(attr, (list, tuple)):
                result[slot] = [rosmsg_to_dict(a) if hasattr(a, '__slots__') else a for a in attr]
            elif hasattr(attr, '__slots__'):
                result[slot] = rosmsg_to_dict(attr)
            else:
                result[slot] = attr
        return result
    else:
        return msg

def ros_thread():
    rospy.init_node('jackal_http_driver', anonymous=True, disable_signals=True)
    rospy.Subscriber(ODOM_TOPIC, Odometry, odom_callback)
    rospy.Subscriber(IMU_TOPIC, Imu, imu_callback)
    rospy.Subscriber(NAVSAT_FIX_TOPIC, NavSatFix, navsat_fix_callback)
    rospy.Subscriber(NAVSAT_VEL_TOPIC, Twist, navsat_vel_callback)
    rospy.Subscriber(FEEDBACK_TOPIC, Feedback, feedback_callback)
    rospy.Subscriber(STATUS_TOPIC, Status, status_callback)
    rospy.spin()

# Publisher queues
publisher_queues = {
    "cmd_vel": queue.Queue(),
    "cmd_driver": queue.Queue()
}

def ros_publishers_thread():
    pub_vel = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=5)
    pub_driver = rospy.Publisher(CMD_DRIVER_TOPIC, Drive, queue_size=5)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        try:
            while not publisher_queues["cmd_vel"].empty():
                twist_msg = publisher_queues["cmd_vel"].get_nowait()
                pub_vel.publish(twist_msg)
            while not publisher_queues["cmd_driver"].empty():
                drive_msg = publisher_queues["cmd_driver"].get_nowait()
                pub_driver.publish(drive_msg)
        except Exception:
            pass
        rate.sleep()

# ==== HTTP API ====

@app.route("/move", methods=["POST"])
def move():
    """
    Send velocity commands to the UGV (maps to /cmd_vel).
    Payload: {"linear": {"x": float, "y": float, "z": float}, "angular": {"x": float, "y": float, "z": float}}
    """
    data = request.get_json(force=True)
    twist = Twist()
    lin = data.get("linear", {})
    ang = data.get("angular", {})
    twist.linear.x = float(lin.get("x", 0))
    twist.linear.y = float(lin.get("y", 0))
    twist.linear.z = float(lin.get("z", 0))
    twist.angular.x = float(ang.get("x", 0))
    twist.angular.y = float(ang.get("y", 0))
    twist.angular.z = float(ang.get("z", 0))
    publisher_queues["cmd_vel"].put(twist)
    return jsonify({"status": "OK", "sent": rosmsg_to_dict(twist)})

@app.route("/drive", methods=["POST"])
def drive():
    """
    Issue low-level driver commands (maps to /cmd_driver).
    Payload: see jackal_msgs/Drive fields
    """
    data = request.get_json(force=True)
    drive_msg = Drive()
    for slot in drive_msg.__slots__:
        if slot in data:
            setattr(drive_msg, slot, data[slot])
    publisher_queues["cmd_driver"].put(drive_msg)
    return jsonify({"status": "OK", "sent": rosmsg_to_dict(drive_msg)})

@app.route("/telemetry", methods=["GET"])
def telemetry():
    """
    Retrieve aggregated telemetry from all main topics.
    """
    with telemetry_lock:
        snapshot = dict(telemetry_state)
    return Response(json.dumps(snapshot, default=str), mimetype="application/json")

def main():
    # Start ROS thread
    rt = threading.Thread(target=ros_thread, daemon=True)
    rt.start()
    pt = threading.Thread(target=ros_publishers_thread, daemon=True)
    pt.start()
    # Wait for ROS node to be ready
    while not rospy.core.is_initialized():
        time.sleep(0.2)
    # Start HTTP server
    app.run(host=HTTP_SERVER_HOST, port=HTTP_SERVER_PORT, threaded=True)

if __name__ == "__main__":
    main()