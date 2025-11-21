#!/usr/bin/env pybricks-micropython
# Underholdning nr 1: Spiller av jubellyd

from pybricks.hubs import EV3Brick
from pybricks.media.ev3dev import SoundFile
from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase

left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

robot = DriveBase(left_motor, right_motor, wheel_diameter=5, axle_track=100)
ev3 = EV3Brick()
Kjor = 0
while Kjor < 10000:
    left_motor.run(10000)
    right_motor.run(-10000)
    Kjor += 1


