#!/usr/bin/env pybricks-micropython
import sys
import __init__
import math
import time

from pybricks import robotics
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor, TouchSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import ImageFile

# Robot def
ev3 = EV3Brick()

# Motor def
left_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE, gears = [12, 20])
right_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE, gears = [12, 20])
lift_motor = Motor(Port.A, Direction.CLOCKWISE, gears = [12, 36])

robot = DriveBase(left_motor, right_motor, wheel_diameter = 47, axle_track = 128)

# Sensor def
color_sensor = ColorSensor(Port.S3)
ultra_sensor = UltrasonicSensor(Port.S4)
touch_sensor = TouchSensor(Port.S1)

# Constant Values
DRIVE_SPEED = 60
TURN_RADIUS = 10
#robot.settings(DRIVE_SPEED, 100, 50)
PROPORTIONAL_GAIN = 1.2

robotLen = 150 # MÄT!

# Color Values
WHITE = 89
BLACK = 7
RED = 88
#BLUE = 
YELLOW = 11
#BROWN = 
#PINK = 
#GREEN = 

find_color_list = [BLACK, WHITE]

def path_find(find_color_list): #Follow a predetermened path
    use_color = colorMix(color_sensor.rgb())
    threshold = (use_color + WHITE) / 2
    while len(find_color_list) > 1:
        if color_sensor.rgb() == find_color_list[0]:
            find_color_list.pop(0)
            path_find(find_color_list)
        # Calculate the deviation from the threshold.
        deviation = colorMix(color_sensor.rgb()) - threshold

        # Calculate the turn rate.
        turn_rate = PROPORTIONAL_GAIN * deviation

        # Set the drive base speed and turn rate.
        robot.drive(DRIVE_SPEED, turn_rate)

def colorMix(rgbColor):
    color = 0
    for i in rgbColor:
        color += rgbColor[i]
        #color = rgbColor[0] + rgbColor[1] + rgbColor[2]
    return color

"""
def find_item():
    closestObj = 100000
    degTurn = 360
    for _ in range(72):
        robot.turn(5)
        distance = ultra_sensor.distance(silent=False)
        if closestObj > distance:
            closestObj = distance

    minDistance = closestObj - 10
    maxDistance = closestObj + 10

    done = False
    while done is False:
        robot.turn(5)
        degTurn -= 5
        if minDistance < ultra_sensor.dictance(silent=False) < maxDistance:
            hypotenuse = ultra_sensor.dictance(silent=False)
            robot.turn(degTurn)
            done = True

    if degTurn != 180:
        radTurn = math.radians(degTurn)
        closeCathetus = abs(hypotenuse * math.cos(radTurn))
        farCathetus = abs(hypotenuse * math.sin(radTurn))

        robot.straight(closeCathetus)
        if degTurn < 180:
            robot.turn(-90)
        elif degTurn > 180:
            robot.turn(90)
        robot.straight(farCathetus - robotLen)
    else:
        robot.drive(hypotenuse - robotLen)

    pick_up_item()
"""

def find_item_red_or_blue():
    use_color = YELLOW
    threshold = (use_color + BLACK) / 2
    while touch_sensor.pressed() == False:
        # Calculate the deviation from the threshold.
        deviation = colorMix(color_sensor.rgb()) - threshold

        # Calculate the turn rate.
        turn_rate = PROPORTIONAL_GAIN * deviation

        # Set the drive base speed and turn rate.
        robot.drive(DRIVE_SPEED, turn_rate)

def check_if_elevated_heavy():
    lift_motor.run_time(5, 5, Stop.HOLD, True)
    if lift_motor.angle() < 10:
        pick_up_elevated()

def check_if_elevated_sonic():
    distance = ultra_sensor.distance(silent=False)
    if distance >= 200:
        robot.straight(-50)
        pick_up_elevated()
    else:
        pick_up_item()

def check_pick_up():
    if touch_sensor.pressed() == False:
        ev3.screen.load_image('knocked_out.png')
        time.sleep(1)

def pick_up_item(): #Picks up an item
    #lift_motor.run_time(5, 5, Stop.HOLD, True)
    lift_motor.run_target(5, 15, Stop.HOLD, True)

    check_pick_up()

def pick_up_elevated():
    # Lift lift til pallet height
    lift_motor.run_target(5, 20, Stop.HOLD, True)
    
    robot.straight(50) #only if using with ultra sonic

    while touch_sensor.pressed() == False:
        robot.straight(5)
    
    lift_motor.run_target(5, 15, Stop.HOLD, True)

    check_pick_up()

def checkLight():
    lightLVL = color_sensor.rgb()
    print(lightLVL)
    time.sleep(1)

def main(): # Main method
    return 0

if __name__ == '__main__':
    sys.exit(main())
