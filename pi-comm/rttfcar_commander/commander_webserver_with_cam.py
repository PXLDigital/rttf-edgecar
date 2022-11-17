#!/usr/bin/env python3
'''
    This Flask module can be run on a remote host device (e.g. Raspberry Pi) and listens for commands to execute on the host.
    An app or website can then be created to contact the Flask module and send commands to it from a smartphone.

    Requirements:
    - Flask

    Created for the Race to the Future event, usable in all kinds of setups.
'''

from flask import Flask, render_template, Response, request
from flask_cors import CORS
import roslibpy
import threading
import subprocess

NODE_NAME = "image_listener"
START_DRIVING_TOPIC = "/master/ai/start_driving"


app = Flask(__name__, template_folder='templates')
app.config['EXPLAIN_TEMPLATE_LOADING'] = True
CORS(app)

#threading.Thread(target=lambda: rospy.init_node('image_listener', disable_signals=True)).start()
client = roslibpy.Ros(host='localhost', port=9090)
client.run()

pub = roslibpy.Topic(client, START_DRIVING_TOPIC, 'std_msgs/Bool', None)


# FLASK ROUTES
@app.route('/')
def index():
    return render_template('with_cam.html')

@app.route('/execute', methods = ['POST'])
def run_command():
    global pub
    command = request.form

    print("Executing command named '" + command['title'] + "'")

    try:
        if command['title'] == 'Start AI':
            pub.publish(roslibpy.Message({'data': True}))
            print(f"Published True to {START_DRIVING_TOPIC}")
            return f"Published True to {START_DRIVING_TOPIC}"

        if command['title'] == 'Stop AI':
            pub.publish(roslibpy.Message({'data': False}))
            print(f"Published False to {START_DRIVING_TOPIC}")
            return f"Published False to {START_DRIVING_TOPIC}"

    except Exception as ex:
        print(ex)
        return ex

    #p = subprocess.Popen(command['command'], shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    #msg = p.communicate()[0]
    #msg = msg.decode()

def shutdown_server():
    func = request.environ.get('werkzeug.server.shutdown')
    if func is None:
        raise RuntimeError('Not running with the Werkzeug Server')
    func()
    
@app.get('/shutdown')
def shutdown():
    shutdown_server()
    return 'Server shutting down...'

# For the RPI
app.run(host='0.0.0.0', port=5000, debug=True)
