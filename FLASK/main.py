
from ast import arg
from pickle import TRUE
from flask import Flask, render_template, Response, request
from flask_socketio import SocketIO, emit
#from flask_cors import CORS
from engineio.payload import Payload
from camera import VideoCamera
from vehicle import vehicleClass
import time
import threading
from threading import Thread
import os
import RPi.GPIO as GPIO
import eventlet

GPIO.setmode(GPIO.BOARD)
GPIO.setup(40,GPIO.OUT)
GPIO.output(40,GPIO.LOW)

Payload.max_decode_packets = 500

isDev = TRUE
pi_camera = VideoCamera(flip=False) # flip pi camera if upside down.

vehicle = vehicleClass()

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
        eventlet.sleep(0.01)
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
        vehicle.x_axis = data["steeringData"]["X"]
        vehicle.y_axis = data["steeringData"]["Y"]
        vehicle.sendToSlave()
        if isDev: print(vehicle)
        #linear.q.put(("left",x,y))
    elif "On" in data.keys():
        print("received a on message")
        vehicle.modeAutonomous = True
        vehicle.sendToSlave()
        GPIO.output(40,GPIO.HIGH)
    elif "Off" in data.keys():
        print("received a off message")
        vehicle.modeAutonomous = False
        vehicle.sendToSlave()
        GPIO.output(40,GPIO.LOW)
    elif "Vert" in data.keys():
        print("received a Vert message")
        vehicle.servo1_pos = data["Vert"]
        vehicle.sendToSlave()
    elif "Horr" in data.keys():
        print("received a Horr message")
        vehicle.servo2_pos = data["Horr"]
        vehicle.sendToSlave()
    elif "info" in data.keys():
        print(data["info"])




@socketio.on_error_default
def default_error_handler(e):
    print("======================= ERROR =======================")
    print(request.event["message"])
    print(request.event["args"])
    print(e)

if __name__ == '__main__':

    socketio.run(app, host="0.0.0.0", debug=True, use_reloader=False)
    


