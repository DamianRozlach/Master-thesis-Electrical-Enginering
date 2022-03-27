import smbus
import time
from random import randint
bus = smbus.SMBus(1)
adress=0x08
data =[0 for i in range(8)]
switch = 1
while True:
    if(switch == 0):
        for x in range(8):
            data[x] = randint(0,255)
        bus.write_block_data(adress,0,data)
    else:
        testStr = bus.read_i2c_block_data(adress,0,6)
        for x in range(len(testStr)):
            testStr[x] = chr(testStr[x])
        print(testStr)
    time.sleep(2)