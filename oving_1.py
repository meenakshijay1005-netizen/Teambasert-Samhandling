from pybricks.hubs import EV3Brick 
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Initialiser EV3 Brick
ev3 = EV3Brick() 

# oppretter motor-objekter som er koblet til 2 porter vi selv velger
left_motor= Motor(Port.B)
right_motor= Motor(Port.C)


# spiller et pip med frekvesn 500 Hz og varighet i 100 milisekunder for å fortelle oss når roboten er klar for å bevege
ev3.speaker.beep(500,100)

# oppretter en DriveBase (roboten, som kan kjøre og svinge)
# Drivbasen er komponert av to motorer, med et hjul på hver side
# Wheel_diameter og axle_track verdiene er brukt for å lage motorene 
# bevege seg i riktig fart når du gir en motoren en kommando
# axle_track er avstanden mellom punktene hvor hjulene treffer bakken
drive_base = DriveBase(left_motor, right_motor, wheel_diameter = 50, axle_track= 130)

# Skriver "Hello World!" på EV3-klossens skjerm.
# Venter i 5000 ms (5 sekunder) slik at meldingen kan leses.
ev3.screen.print("Hello World!")
wait(5000)

# Kjør en firkant: i hver runde kjører roboten 200 mm rett frem, deretter svinger den 90 grader.
for i in range(4):               
    drive_base.straight(200)
    drive_base.turn(90)

# EV3 sier "Have a nice day" gjennom høyttaleren
ev3.speaker.say("Have a nice day")

# et lite pip før avslag 
# Slår av lyset på EV3-klossen
ev3.speaker.beep(500,100) 
ev3.light.off()
drive_base.stop()
