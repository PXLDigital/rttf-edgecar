#!/usr/bin/env python3
'''
    This Flask module can be run on a remote host device (e.g. Raspberry Pi) and listens for commands to execute on the host.
    An app or website can then be created to contact the Flask module and send commands to it from a smartphone.

    Requirements:
    - Flask

    Created for the Race to the Future event, usable in all kinds of setups.
'''

from flask import Flask
from flask import request
from flask_cors import CORS 

import subprocess

app = Flask(__name__)
CORS(app)

@app.route('/execute', methods = ['POST'])
def run_command():
   command = request.form

   print("Executing '" + command['command'] + "'")

   p = subprocess.Popen(command['command'], shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
   msg = p.communicate()[0]
   msg = msg.decode()

   return msg


# For the RPI
app.run(host='0.0.0.0', port=5000, debug=True)
 
# For testing on a normal Pc
app.run(debug=True)
