#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.hubs import EV3Brick


# Initialize the sensor.

sensor = ColorSensor(Port.S4)
x = 200
y = 400
ev3 = EV3Brick()
ev3.speaker.beep(523.25, y)
wait(x)
ev3.speaker.beep(523.25, y)
wait(x)
ev3.speaker.beep(523.25,y)
wait(x)
ev3.speaker.beep(659.25, y)
wait(x)
ev3.speaker.beep(587.33, y)
wait(x)
ev3.speaker.beep(587.33, y)
wait(x)
ev3.speaker.beep(587.33, y)
wait(x)
ev3.speaker.beep(698.46, y)
wait(x)
ev3.speaker.beep(659.25, y)
wait(x)
ev3.speaker.beep(659.25, y)
wait(x)
ev3.speaker.beep(587.33, y)
wait(x)
ev3.speaker.beep(587.33, y)
wait(x)
ev3.speaker.beep(523.25, 4*y)