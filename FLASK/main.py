#Modified by smartbuilds.io
#Date: 27.09.20
#Desc: This web application serves a motion JPEG stream
# main.py
# import the necessary packages
from pickle import TRUE
from flask import Flask, render_template, Response, request
from flask_socketio import SocketIO, emit
#from flask_cors import CORS
from engineio.payload import Payload
from camera import VideoCamera
from vehicle import vehicleClass
import time
import threading
import os

Payload.max_decode_packets = 500

isDev = TRUE
pi_camera = VideoCamera(flip=False) # flip pi camera if upside down.

tank = vehicleClass()

# App Globals (do not edit)
app = Flask(__name__)
#CORS(app)
socketio = SocketIO(app)

@app.route('/')
def index():
    return render_template('index.html') #you can customze index.html here

def gen(camera):
    #get camera frame
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(gen(pi_camera),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@socketio.on('control', namespace='/control')
def control(message):
    data = message["data"]
    if "steeringData" in data.keys():
        print("received a steering mesage")
        tank.x_axis = data["steeringData"]["X"]
        tank.y_axis = data["steeringData"]["Y"]
        if isDev: print(tank)
        #linear.q.put(("left",x,y))
    elif "On" in data.keys():
        print("received a on message")
        tank.modeAutonomous = True
    elif "Off" in data.keys():
        print("received a off message")
        tank.modeAutonomous = False
    elif "Vert" in data.keys():
        print("received a Vert message")
        tank.servo1_pos = data["Vert"]
    elif "Horr" in data.keys():
        print("received a left message")
        tank.servo2_pos = data["Horr"]




@socketio.on_error_default
def default_error_handler(e):
    print("======================= ERROR =======================")
    print(request.event["message"])
    print(request.event["args"])
    print(e)

if __name__ == '__main__':

    socketio.run(app, host="0.0.0.0", debug=True, use_reloader=False)
    


