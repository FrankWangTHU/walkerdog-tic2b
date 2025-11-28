#!/usr/bin/env python
"""Flask web server that exposes simple controls and interacts with ROS.
This runs inside the ROS environment (source /catkin_ws/devel/setup.bash) and
publishes modes to `/dog_mode` topic. It also exposes a small API for non-ROS clients.
"""
from threading import Thread
from flask import Flask, request, jsonify
import rospy
from std_msgs.msg import String

app = Flask(__name__)

# We will initialize ROS in a separate thread once the server starts
ros_pub = None

@app.route('/api/mode', methods=['POST'])
def set_mode():
    data = request.get_json() or {}
    mode = data.get('mode', 'idle')
    if ros_pub:
        ros_pub.publish(String(data=mode))
    return jsonify({'status': 'ok', 'mode': mode})

@app.route('/api/status', methods=['GET'])
def status():
    # For demo purposes we only return a static message; a real implementation
    # would subscribe to /dog_state or query ROS master.
    return jsonify({'status': 'running'})

def ros_thread():
    global ros_pub
    rospy.init_node('dog_web_bridge', anonymous=True)
    ros_pub = rospy.Publisher('/dog_mode', String, queue_size=10)
    # Keep node alive to allow publishing when Flask handlers call ros_pub
    rospy.spin()

if __name__ == '__main__':
    # Start ROS in background thread
    t = Thread(target=ros_thread)
    t.daemon = True
    t.start()
    # Run Flask (for production use a WSGI server)
    app.run(host='0.0.0.0', port=5000)
