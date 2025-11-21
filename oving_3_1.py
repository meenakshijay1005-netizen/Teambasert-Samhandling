#!/usr/bin/env pybricks-micropython



import time
from pybricks.ev3devices import Motor, ColorSensor, TouchSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from pybricks.hubs import EV3Brick
from pybricks.media.ev3dev import SoundFile
from pybricks.media.ev3dev import Image, ImageFile
from pybricks.tools import wait


# Initialize the motors.
left_motor = Motor(Port.C)
right_motor = Motor(Port.B)
touch = TouchSensor(Port.S3)
ev3 = EV3Brick()


# Initialize the color sensors.
line_sensor = ColorSensor(Port.S4)
line_sensor_h = ColorSensor(Port.S2)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

threshold = 0
Run = False
deviation = 0
deviation_2 = 0
count = 1
DRIVE_SPEED = 400

verdi = 50


def bjørnensover():
    robot.drive(0,0)
    x = 200
    y = 400
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
    ev3.speaker.beep(523.25, 2*y)

def show(count):
    if count == 1:
        bjørnensover()
    elif count == 2:
        robot.stop()
        for x in range(1,8):
            ev3.speaker.play_file(SoundFile.BRAVO)
    elif count == 3:
        robot.stop()
        img = Image(ImageFile.BOTTOM_LEFT) 
        ev3.screen.draw_image(0, 0, img)
        ev3.speaker.play_file(SoundFile.HELLO)
        wait(5000)
    elif count == 4:
        robot.stop()
        for i in range(20):
            robot.straight(10)
            robot.straight(-10)
        wait(2000)
            




# Start following the line endlessly.
while True:
    if touch.pressed():
        Run = True
        threshold = line_sensor.reflection()
        threshold2 = line_sensor_h.reflection()
        start_time = time.time()
        while Run:
            # Calculate the deviation from the threshold.
            deviation = line_sensor.reflection() - threshold
            deviation_2 = line_sensor_h.reflection() - threshold2
            turn_rate = 0
            # Calculate the turn rate.
            if time.time() - start_time >= 10:
                show(count)
                count += 1
                start_time = time.time()
            else:
                if deviation > verdi or deviation < (-1*verdi):
                    if deviation_2 < verdi and deviation_2 > (-1*verdi):
                        turn_rate = deviation*1.5
                elif deviation_2 > verdi or deviation_2 < (-1*verdi):
                    if deviation < verdi and deviation > (-1*verdi):
                        turn_rate = deviation_2*-1.5
                # Set the drive base speed and turn rate.
                robot.drive(DRIVE_SPEED, turn_rate)