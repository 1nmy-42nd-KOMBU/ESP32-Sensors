#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.iodevices import I2CDevice
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch

# Initialize the EV3
ev3 = EV3Brick()

timer = StopWatch()

# Initialize I2C Sensor
C1 = I2CDevice(Port.S1, 0x04)

wait(100)

start_time = timer.time()
for i in range(100):
    tuplea = C1.read(reg=0x04, length=2)
    print((tuplea[0] << 8) + tuplea[1])
    wait(100)

print(str((timer.time()-start_time)/100-100))
