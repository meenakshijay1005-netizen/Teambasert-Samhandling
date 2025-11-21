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
power_button = TouchSensor(Port.S1) #TouchSensor er programmert til å være av og på knappen
ev3 = EV3Brick() #Initialiser EV3 Brick
obstacle_sensor = UltrasonicSensor(Port.S4) #Initialiser obstacle_sensor

# Initialiser to mototrer med default settinger i Port B og Port C
# Dette vil være venstre og høyre motorer for drivbasen
left_motor = Motor(Port.B)
right_motor= Motor(Port.C)

# Drivbasen er komponert av to motorer, med et hjul på hver side
# Wheel_diameter og axle_track verdiene er brukt for å lage motorene 
# bevege seg i riktig fart n år dui gir en motoren en kommando
# axle_track er avstanden mellom punktene hvor hjulene treffer bakken
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Spiller en beep for å fortelle oss når roboten er klar for å bevege
ev3.speaker.beep()

#konstant
speed = 200 # farten roboten skal kjøre framover (målt i mm)
limit_distance = 300 # grenseverdi i mm for å oppfatte hidnring
TURN_DEG = 90 # grader roboten skal svinge når den ,øter hindring

running = False # roboten staretr som av (False)

def wait_while_pressed(): # definerer en funksjon medn navnet wait_while_pressed
    while power_button.pressed(): # så lenge trykksensoren er trykket
        wait(10) # vent i 10 milisekunder

# Loopen for roboten til å bevege seg framover helt til den detekter en hindring
# Når den detekter en hindring vil den gå bakover og snu seg. 
# første løkken håndterer hendelsen når "knappen blir trykket"
while True: # koden kjører en evig løkkem til programmet blir avbrutt
    if power_button.pressed(): # sjekkeer om trykksensoren blir trykket inn
        wait_while_pressed() # venter til knappen slippes opp igjen (får ikke flere signaler i samme trykk)
        running = not running # setter runnign til det motsatte av hva det er på starten --> vi starter med at running er False og not False --> true --> da kjører den
        if running: # Hvis roboten skrus på 
            ev3.speaker.say("Exercise 2")  # EV3 sier "Exercise 2" når roboten starter
        else:
            robot.stop()  
            ev3.speaker.say("Exercise done") # EV3 sier "Exercise done"
            break # programmet avsluttes
    
# andre løkke styrer bevegelsen som kjøring og unngå hindringer 
    if running: # kjører koden bare hvis roboten skrus på
        dist = obstacle_sensor.distance() # Leser avstanden fra ultralydsensoren
        if dist is not None and dist < limit_distance:  
            robot.stop() # Stopp roboten umiddelbart
            robot.straight(-300) # Rygg ca. 300 mm bakover
            robot.turn(TURN_DEG) # Snu roboten et antall grader
        else:
                robot.drive(500,0) # Kjør rett frem med fart 500 mm/s og rotasjonshastighet 0
    else: #sørger for at roboten står stille når running = False
        robot.stop()   

    wait(10) 
            
            
            
            
            
            
            

  





