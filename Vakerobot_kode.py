#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
stopH = TouchSensor(Port.S2) #høyre
stopV = TouchSensor(Port.S3) #venstre
stopM = TouchSensor(Port.S4)
obstacle_sensor = UltrasonicSensor(Port.S1)
ev3 = EV3Brick() #Initialiser EV3 Brick
obstacle_sensor = UltrasonicSensor(Port.S1) #Initialiser obstacle_sensor

# Initialiser to mototrer med default settinger i Port B og Port C
# Dette vil være venstre og høyre motorer for drivbasen
left_motor = Motor(Port.D)
right_motor= Motor(Port.C)
turn0_motor = Motor(Port.B)

# Drivbasen er komponert av to motorer, med et hjul på hver side
# Wheel_diameter og axle_track verdiene er brukt for å lage motorene
# bevege seg i riktig fart n år dui gir en motoren en kommando
# axle_track er avstanden mellom punktene hvor hjulene treffer bakken
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Spiller en beep for å fortelle oss når roboten er klar for å bevege
ev3.speaker.beep()

#konstant
speed1 = 300 # farten roboten skal kjøre framover (målt i mm)
limit_distance = 150 # grenseverdi i mm for å oppfatte hindring
TURN_DEG = 0
TURN_ANGLE_SENSOR = 80
SENSOR_SPEED = 200
TURN_ANGLE_OBSTACLE = 100

running = False


running = True
while True:
    # andre løkke styrer bevegelsen som kjøring og unngå hindringer
    if running:
        speed = speed1
        dist = obstacle_sensor.distance() # Leser avstanden fra ultralydsensoren
        if dist is not None and dist < limit_distance:
            speed = min(max(dist/speed * 100, 50), speed)
            ev3.screen.print(speed)
        if stopM.pressed() or dist <= 50:
            robot.stop()
            wait(1000)
            ev3.speaker.beep()
            robot.straight(100)
            turn0_motor.run_angle(SENSOR_SPEED,-TURN_ANGLE_SENSOR)
            avstand1 = obstacle_sensor.distance()
            wait(500)
            turn0_motor.run_angle(SENSOR_SPEED,2*TURN_ANGLE_SENSOR)
            avstand2 = obstacle_sensor.distance()
            turn0_motor.run_angle(SENSOR_SPEED,-TURN_ANGLE_SENSOR)
            wait(500)
            if avstand1 < limit_distance and avstand2 > limit_distance:
                robot.turn(-TURN_ANGLE_OBSTACLE)
            elif avstand1 > limit_distance and avstand2 < limit_distance:
                robot.turn(TURN_ANGLE_OBSTACLE)
            elif avstand1 > limit_distance and avstand2 > limit_distance:
                robot.turn(360)
            else:
                robot.turn(360) 
            if avstand1 < limit_distance and avstand2 < limit_distance:
                robot.turn(360)
        if stopH.pressed():
            ev3.speaker.beep()
            robot.stop()
            wait(1000)
            turn0_motor.run_angle(SENSOR_SPEED,-TURN_ANGLE_SENSOR)
            avstand1 = obstacle_sensor.distance()
            wait(500)
            robot.turn(-TURN_ANGLE_OBSTACLE/2)
            while avstand1 < limit_distance:
                avstand1 = obstacle_sensor.distance()
                robot.turn(-TURN_ANGLE_OBSTACLE/4)
            turn0_motor.run_angle(SENSOR_SPEED,TURN_ANGLE_SENSOR)
        if stopV.pressed():
            ev3.speaker.beep()
            robot.stop()
            wait(1000)
            turn0_motor.run_angle(SENSOR_SPEED,TURN_ANGLE_SENSOR)
            avstand1 = obstacle_sensor.distance()
            wait(500)
            robot.turn(TURN_ANGLE_OBSTACLE/2)
            while avstand1 < limit_distance:
                avstand1 = obstacle_sensor.distance()
                robot.turn(TURN_ANGLE_OBSTACLE/4)
            turn0_motor.run_angle(SENSOR_SPEED,-TURN_ANGLE_SENSOR)
        robot.drive(-speed,TURN_DEG) # Kjør rett frem med fart 500 mm/s og rotasjonshastighet 0