import RPi.GPIO as GPIO
from time import *
GPIO.setmode(GPIO.BOARD)
pinList = [35,36,37,38]
counter = 0
pinState = [False,False,False,False]

for x in pinList:
    GPIO.setup(x,GPIO.OUT)
    GPIO.output(x,GPIO.LOW)
#end for

while True:
    for z in range(len(pinList)):
        pinState[z]= bool(counter & (1<<z))
        if(pinState[z]):
            GPIO.output(pinList[z],GPIO.HIGH)
        else:
            GPIO.output(pinList[z],GPIO.LOW)
        #endif
    #endfor
    print("Stan pinow",pinState)
    counter += 1
    if(counter >15):
        counter=0
    #endif
    sleep(0.8)

