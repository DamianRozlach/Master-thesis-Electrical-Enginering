#Modified by smartbuilds.io
#Date: 27.09.20
#Desc: This web application serves a motion JPEG stream
# main.py
# import the necessary packages
from pickle import TRUE
from flask import Flask, render_template, Response, request
from flask_socketio import SocketIO, emit
from flask_cors import CORS
from engineio.payload import Payload
from camera import VideoCamera
from vehicle import vehicleClass
import time
import threading
import os

Payload.max_decode_packets = 50

isDev = TRUE
pi_camera = VideoCamera(flip=False) # flip pi camera if upside down.

tank = vehicleClass()

# App Globals (do not edit)
app = Flask(__name__)
CORS(app)
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
    print("received a control message")
    data = message["data"]
    if "steeringData" in data.keys():
        print("received a steering mesage")
        tank.x_axis = data[0]
        tank.y_axis = data[1]
        if isDev: print(tank)
        #linear.q.put(("left",x,y))
    elif "On" in data.keys():
        print("received a on message")
        x = data["right"][0]
        y = data["right"][1]
        if TRUE: print("[Server] Right: ",x,",",y)
        #servo.q.put(("right",x,y))
        #servo2.q.put(("right",y,x))
    elif "A" in data.keys():
        if TRUE: print("[Server] A")
        #binary.q.put(("A",1,0))
    elif "B" in data.keys():
        if True: print("[Server] B")
        #binary2.q.put(("B",1,0))

@socketio.on('message')
def handle_message(data):
    print('received message: ' + data)

@socketio.on_error_default
def default_error_handler(e):
    print("======================= ERROR")
    print(request.event["message"])
    print(request.event["args"])

if __name__ == '__main__':

    socketio.run(app, host="0.0.0.0", debug=True, use_reloader=False)
    


