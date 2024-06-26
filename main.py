#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (
    Motor,
    TouchSensor,
    ColorSensor,
    InfraredSensor,
    UltrasonicSensor,
    GyroSensor,
)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks import nxtdevices
import math
import statistics

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Tagging guide:
# !!/!: category/subcategory
# @: for calibrations
# $: for optimisations

# !! OBJECTS !! #

# brain
ev3 = EV3Brick()


# ! MOTORS ! #

# large driving motors (tank tracks)
left = Motor(Port.C, positive_direction=Direction.COUNTERCLOCKWISE)
right = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)

# medium motors
claw = Motor(Port.A)
sorter = Motor(Port.D, positive_direction=Direction.CLOCKWISE, gears=[8, 24])

# defining robot wheel motors
robot = DriveBase(left, right, 33, 187)
# @ variables: (left motor, right motor, diameter of the wheel (that is used to turn the tracks), distance between the wheels (from middle to middle of track)

# adjust the straight/turning speed/acceleration (for functions like robot.straight() and robot.turn())
#robot.settings(350, 150, 300, 200)
# @ variables: (straight_speed, straight_acceleration, turn_rate, turn_acceleration)


# ! SENSORS ! #

# colour sensors facing balls/deposit zones/obstacles
front_light_sensor = ColorSensor(Port.S1)

# colour sensors facing floor
left_light_sensor = ColorSensor(Port.S4)
right_light_sensor = ColorSensor(Port.S3)

#ultrasonic sensor
ultrasonic_sensor = UltrasonicSensor(Port.S2)


# !! VARIABLES !! #

# ! CALIBRATIONS ! #

left_min_red = 5
left_max_red = 55

left_min_green = 7
left_max_green = 46

left_min_blue = 11
left_max_blue = 100

right_min_red = 3
right_max_red = 50

right_min_green = 7
right_max_green = 46

right_min_blue = 3
right_max_blue = 90

left_red_range = left_max_red - left_min_red
left_green_range = left_max_green - left_min_green
left_blue_range = left_max_blue - left_min_blue

right_red_range = right_max_red - right_min_red
right_green_range = right_max_green - right_min_green
right_blue_range = right_max_blue - right_min_blue

# @ how to calibrate
# 1. Run the calibration function [calibration("rgb")]
# 2. Place the robot on white surface and record the values as maximum.
# 3. Place the robot on black surface and record the values as minimum.
# 4. Remember to remove the calibration function after calibration.


# ! OTHERS ! #

#so it can be referenced in the functions
left_val = None
right_val = None

# default
quit = False
exited = False
evac = False
last_black = 0
stored = [None, None, None]


# !! FUNCTIONS !! #

# covert raw values into calibrated values based off the min and max values
def calibrate_values():
    global left_val, right_val

    # max min to make sure values do not go below 0 and above 1
    left_val = left_light_sensor.rgb()

    left_red_cal = max(min((left_val[0] - left_min_red) / left_red_range, 1), 0)
    left_green_cal = max(min((left_val[1] - left_min_green) / left_green_range, 1), 0)
    left_blue_cal = max(min((left_val[2] - left_min_blue) / left_blue_range, 1), 0)

    right_val = right_light_sensor.rgb()

    right_red_cal = max(min((right_val[0] - right_min_red) / right_red_range, 1), 0)
    right_green_cal = max(
        min((right_val[1] - right_min_green) / right_green_range, 1), 0
    )
    right_blue_cal = max(min((right_val[2] - right_min_blue) / right_blue_range, 1), 0)

    return (
        left_red_cal,
        left_green_cal,
        left_blue_cal,
        right_red_cal,
        right_green_cal,
        right_blue_cal,
    )

# pure line track without green square detection
def only_line_track(l_red, l_green, l_blue, r_red, r_green, r_blue):
        print("only line track")
        error = l_green - r_green
        speed = (1 - abs(error)) * 80  # @ adjust the speed
        rotation = error * 225 # @ adjust degree of rotation
        robot.drive(speed, rotation)

# link track with green square detection
def line_track(l_red, l_green, l_blue, r_red, r_green, r_blue):
    global last_black
    if (
        max(l_red, l_green, l_blue, r_red, r_green, r_blue) < 0.45
    ):  # the calibrated value to trigger the double black detection
        print("double black detected")

        robot.stop()
        robot.straight(-17)  # @ Move back to check for green squares
        wait(100)

        val = calibrate_values() # Get values of the space under the black line
        nl_green = val[1]
        nr_green = val[4]
        nl_red = val[0]
        nr_red = val[3]

        # made for 135degree cases

        saw = ""
        if nl_green <= 0.6: # @ left black trigger
            robot.turn(-18) # @ amt to move to off set
            saw = "left"
        if nr_green <= 0.6: # @ right black trigger
            robot.turn(18) # @ amt to move to off set
            saw = "right"

        # recheck values if the robot had moved
        val = calibrate_values()
        nl_green = val[1]
        nr_green = val[4]
        nl_red = val[0]
        nr_red = val[3]

        # checking green
        if nl_red <= 0.4:  # @ low red trigger for left sensor
            if nr_red <= 0.3:  # @ low red trigger for right sensor
                #green channel check for black
                if nl_green <= 0.3: # @ low green trigger for left sensor
                    if nr_green <= 0.3: # @ low green trigger for right sensor
                        print("BOTH black")
                    single_line_track(100, "left", "left", 20)
                else:
                    print("BOTH green")  # If both are green
                    robot.turn(180)  # u-turn
                    
            else:  # If only left is green -> turn left
                print("LEFT green")
                single_line_track(110, "left", "left", 20)
        else:
            if nr_red <= 0.6:  # @ low red for right sensor
                print("RIGHT green")  # If only right is green -> turn right
                single_line_track(110, "right", "right", 20)
            else:  # If none is green
                robot.reset()
                print("BOTH white")
                if saw == "left":
                    single_line_track(110, "left", "left", 20)
                elif saw == "right":
                    single_line_track(110, "right", "right", 20)
                else:
                    while robot.distance() < 90: #calc new speed rotation
                        # can replace w only line track?
                        newval = calibrate_values()
                        error = newval[1] - newval[4]
                        speed = (1 - abs(error)) * 75  # @ adjust speed
                        rotation = error * 100 # @ adjust angle
                        robot.drive(speed, rotation)
    else:
        error = l_green - r_green
        speed = (0.8 - abs(error)) * 120  # @ adjust speed
        rotation = error * 250 # @ adjust angle
        if min(l_red, l_green, l_blue, r_red, r_green, r_blue) < 0.7: # @ black trigger
            last_black = robot.distance()
        gap = robot.distance() - last_black
        if gap > 100: # @ gap trigger
            #ev3.speaker.beep()
            print("gap")
            robot.straight(-50)
            robot.stop()
            robot.reset()
            while True:
                values = calibrate_values()
                l_red = values[0]
                l_green = values[1]
                l_blue = values[2]
                r_red = values[3]
                r_green = values[4]
                r_blue = values[5]
                if min(l_red, l_green, l_blue, r_red, r_green, r_blue) > 0.5:
                    if robot.angle() > -45:
                        #ev3.speaker.beep()
                        robot.drive(5, -20)
                    else:
                        break
                else:
                    break
            while True:
                values = calibrate_values()
                l_red = values[0]
                l_green = values[1]
                l_blue = values[2]
                r_red = values[3]
                r_green = values[4]
                r_blue = values[5]
                if min(l_red, l_green, l_blue, r_red, r_green, r_blue) > 0.5:
                    if robot.angle() < 45:
                        #ev3.speaker.beep()
                        robot.drive(5, 20)
                    else:
                        break
                else:
                    break
            while True:
                if robot.angle() > 0:
                    robot.drive(5, -20)
                else:
                    break

        robot.drive(speed, rotation)

# line track using only
def single_line_track(distance, sensor, side, speed=100.0, gain=100.0):
    print("single_line_track")
    error = 0.0
    robot.stop()
    robot.reset()
    while robot.distance() < distance:
        values = calibrate_values()
        l_green = values[1]
        r_green = values[4]
        if sensor is "left":
            if side is "left":
                error = l_green - 0.5
            else:
                error = 0.5 - l_green
        else:
            if side is "right":
                error = 0.5 - r_green
            else:
                error = r_green - 0.5
        robot.drive((1 - abs(error)) * 25, error * gain)

def silver_detection():
    more = 3
    global left_val, right_val, evac
    if evac is False:
        if (
            left_val[0] >= (left_max_red + more)
            and right_val[0] >= (right_max_red + more)
            and left_val[1] >= (left_max_green + more)
            and right_val[1] >= (right_max_green + more)
            and left_val[2] >= (left_max_blue)
            and right_val[2]
            >= (right_max_blue)  # blue maxs are 100, so no need to add more
        ):
        #prevent seesaw detected
            robot.stop()
            wait(1000)
            left_val = left_light_sensor.rgb()
            right_val = right_light_sensor.rgb()

            if (
                left_val[0] >= (left_max_red + more)
                and right_val[0] >= (right_max_red + more)
                and left_val[1] >= (left_max_green + more)
                and right_val[1] >= (right_max_green + more)
                and left_val[2] >= (left_max_blue)
                and right_val[2]
                >= (right_max_blue)  # blue maxs are 100, so no need to add more
            ):
                print("Silver Detected, stopping here.")
                robot.stop()
                evac = True

#clawfuncs
def pickup(action):
    angle = 500
    if action == "open":
        claw.run_angle(500, -abs(angle + 200))
    else:
        claw.run_angle(500, angle)

def red_line_detection(l_red, l_green, l_blue, r_red, r_green, r_blue):
    if max(l_red, l_green, l_blue, r_red, r_green, r_blue) > 0:
        if ((l_red + r_red) / (l_red + l_green + l_blue + r_red + r_green + r_blue)) >= 0.5:
            print("Red Detected, stopping here.")
            robot.stop()

def object_detection():
    global left_val, right_val, evac, stored

    front_val = sum(front_light_sensor.rgb())
    # check if anything there
    if front_val > 0:

            if stored[2] == None:

                randist = robot.distance()
                robot.drive(100, 0)
                claw.run_angle(200, 300)

                while robot.distance() > randist:  
                    robot.drive(-150, 0)
                robot.stop()

                front_val = sum(front_light_sensor.rgb()) #recheck the ball

                if front_val < 10: #black 0
                    if stored[0] == None:
                        sorter.run_target(100, 0)
                        stored[0] = "black"
                    elif stored[1] == None:
                        sorter.run_target(100, 120)
                        stored[1] = "black"
                    elif stored[2] == None:
                        sorter.run_target(100, 240)
                        stored[2] = "black"
                    pickup("close")
                    pickup("open")
                else: #white 74
                    if stored[0] == None:
                        sorter.run_target(100, 0)
                        stored[0] = "white"
                    elif stored[1] == None:
                        sorter.run_target(100, 120)
                        stored[1] = "white"
                    elif stored[2] == None:
                        sorter.run_target(100, 240)
                        stored[2] = "white"
                    pickup("close")
                    pickup("open")
                print(stored)

def obstacle_detection():
    front_val = front_light_sensor.rgb()
    # check if anything there
    if front_val[0] != 0 and front_val[1] != 0 and front_val[2] != 0:
        print((front_val[2]) / (front_val[0] + front_val[1] + front_val[2]))
        if ((front_val[2]) / (front_val[0] + front_val[1] + front_val[2])) < 0.5:
            print("Obstacle Detected, going around it.")
            robot.straight(-75)
            robot.turn(90)
            robot.straight(150)
            robot.turn(-90)
            robot.straight(300)
            robot.turn(-90)
            robot.straight(150)
            robot.turn(90)

            robot.drive(100, -10)
            print("line found")
            robot.reset()
            return

def scan_dist():
    global stored, quit
    initial_dist = robot.distance()
    dist_list = []

    while robot.distance() - initial_dist < 300: #step 0
        robot.drive(150, 0)
        object_detection()
        us = ultrasonic_sensor.distance()
        if us > 1050:
            pass
        else:
            dist_list.append(us)

    robot.stop()
    dist_list.sort()
    print(dist_list)
    count = len(dist_list)
    if count % 2 == 0:
        median = (dist_list[count // 2] + dist_list[count // 2 - 1]) / 2
    else:
        median = dist_list[count // 2]
    robot.straight(-300)
    print(median)
    foralignment = ultrasonic_sensor.distance()
    if median < 150: #special case
        robot.turn(90)
        initial_dist = robot.distance()
        dist_list = []

        while robot.distance() - initial_dist < 300:
            robot.drive(150, 0)
            object_detection()
            us = ultrasonic_sensor.distance()
            if us > 1050:
                pass
            else:
                dist_list.append(us)

        robot.stop()
        #find median manually 
        dist_list.sort()
        count = len(dist_list)
        if count % 2 == 0:
            median = (dist_list[count // 2] + dist_list[count // 2 - 1]) / 2
        else:
            median = dist_list[count // 2]
        robot.straight(-300)
        print(median)
        ev3.screen.print(median)

        if foralignment < 175:
            robot.stop()
            right.dc(-100)
            left.dc(-100)
            wait(2000)
            robot.stop()

        robot.turn(-90)
    else:
        robot.turn(-90)
    
    print(median)

    wall_track(median - 380)
    if quit:
        return None

    #scan depo
    robot.turn(45)
    robot.straight(100)
    #one wheel turn
    robot.stop()
    left.hold()
    current = robot.angle()
    while robot.angle() - current > -90:
        right.run(500)
    # check if evac is there
    # use front colour sensor to check if the value is red (dead ball), green (alive ball) or black (nothing)
    robot.stop()
    robot.straight(30)
    value = front_light_sensor.rgb()
    #red is 5,0,0
    #green is 2, 6, 4
    #none is 0 all
    wait(100)
    print(front_light_sensor.rgb())
    if max(value) == 0: #none
        print("none")
        robot.straight(-30)
        robot.stop()
        #use for 2B when there is time
        #left.hold()
        #current = robot.angle()
        #while robot.angle() - current > 90:
        #    right.run(-500)
        #robot.straight(-100)
        #robot.turn(-45)

        robot.turn(135)

        tile1 = ultrasonic_sensor.distance()
        robot.straight(250)
        tile2 = ultrasonic_sensor.distance()

        # 2 cases: no wall and wall
        if tile2 < 450:
            robot.turn(90)
            robot.stop()
            right.dc(-100)
            left.dc(-100)
            wait(1500)
            robot.straight(35)
        elif tile2 > 450:
            robot.turn(90)
            error = 60 - tile1
            robot.straight(error)

    if value[0] > value[1] and value[0] > value[2]: #red
        print("red")
        robot.straight(-100)
        robot.turn(180)
        
        if stored[0] == "black":
            stored[0] = ""
            sorter.run_target(100, 180)
            robot.stop()
            right.dc(-100)
            left.dc(-100)
            wait(1000)
            robot.stop()
            robot.straight(75)
        if stored[1] == "black":
            stored[1] = ""
            sorter.run_target(100, 300)
            robot.stop()
            right.dc(-100)
            left.dc(-100)
            wait(1000)
            robot.stop()
            robot.straight(75)
        if stored[2] == "black":
            stored[2] = ""
            sorter.run_target(100, 420)
            robot.stop()
            right.dc(-100)
            left.dc(-100)
            wait(1000)
            robot.stop()
            robot.straight(75)
        
        #after all balls depoed
        
        robot.turn(-45)
    
        #tile1 reading unlikely to be able to get
        tile1 = ultrasonic_sensor.distance()
        robot.straight(200)
        tile2 = ultrasonic_sensor.distance()

                # 2 cases: no wall and wall
        if tile2 < 450:
            robot.turn(90)
            robot.stop()
            right.dc(-100)
            left.dc(-100)
            wait(1500)
            robot.straight(35)
        elif tile2 > 450:
            robot.turn(90)
            error = 75 - tile1
            robot.straight(error)


    if value[1] > value[0] and value[1] > value[2]: #green
        print("green")
        robot.straight(-100)
        robot.turn(180)
        #select slots to depo balls not written
        
        if stored[0] == "white":
            stored[0] = ""
            sorter.run_target(100, 180)
            robot.stop()
            right.dc(-100)
            left.dc(-100)
            wait(1000)
            robot.stop()
            robot.straight(75)
        if stored[1] == "white":
            stored[1] = ""
            sorter.run_target(100, 300)
            robot.stop()
            right.dc(-100)
            left.dc(-100)
            wait(1000)
            robot.stop()
            robot.straight(75)
        if stored[2] == "white":
            stored[2] = ""
            sorter.run_target(100, 420)
            robot.stop()
            right.dc(-100)
            left.dc(-100)
            wait(1000)
            robot.stop()
            robot.straight(75)

        #after all balls depoed
        robot.turn(-45)
    
        #tile1 reading unlikely to be able to get
        tile1 = ultrasonic_sensor.distance()
        robot.straight(200)
        tile2 = ultrasonic_sensor.distance()

            # 2 cases: no wall and wall
        if tile2 < 450:
            robot.turn(90)
            robot.stop()
            right.dc(-100)
            left.dc(-100)
            wait(1500)
            robot.straight(35)
        elif tile2 > 450:
            robot.turn(90)
            error = 75 - tile1
            robot.straight(error)

def exit_scan_dist():
    # to be done: if gaps are found in corners (use phone ntoes ref) exit. if wall track gap is found exit. if wall track immediately left is found exit
    global stored, exited
    initial_dist = robot.distance()
    dist_list = []

    while robot.distance() - initial_dist < 300: #step 0
        robot.drive(150, 0)
        object_detection()
        us = ultrasonic_sensor.distance()
        if us > 1050:
            pass
        else:
            dist_list.append(us)

    robot.stop()
    dist_list.sort()
    count = len(dist_list)
    if count % 2 == 0:
        median = (dist_list[count // 2] + dist_list[count // 2 - 1]) / 2
    else:
        median = dist_list[count // 2]
    robot.straight(-300)
    print(median)
    foralignment = ultrasonic_sensor.distance()
    if median < 150: #special case
        robot.turn(90)
        initial_dist = robot.distance()
        dist_list = []

        while robot.distance() - initial_dist < 300:
            robot.drive(150, 0)
            object_detection()
            us = ultrasonic_sensor.distance()
            if us > 1050:
                pass
            else:
                dist_list.append(us)

        robot.stop()
        #find median manually 
        dist_list.sort()
        count = len(dist_list)
        if count % 2 == 0:
            median = (dist_list[count // 2] + dist_list[count // 2 - 1]) / 2
        else:
            median = dist_list[count // 2]
        robot.straight(-300)
        print(median)
        ev3.screen.print(median)

        if foralignment < 175:
            robot.drive(-100, 0)
            wait(1000)
            robot.stop()

        robot.turn(-90)
    else:
        robot.turn(-90)
    
    print(median)
    exit_wall_track(median - 360)
    if quit:
        return None

    #scan depo
    robot.turn(45)
    robot.straight(100)
    #one wheel turn
    robot.stop()
    left.hold()
    current = robot.angle()
    while robot.angle() - current > -90:
        right.run(500)
    # check if evac is there
    # use front colour sensor to check if the value is red (dead ball), green (alive ball) or black (nothing)
    robot.stop()
    robot.straight(30)
    value = front_light_sensor.rgb()
    #red is 5,0,0
    #green is 2, 6, 4
    #none is 0 all
    wait(100)
    print(front_light_sensor.rgb())
    if max(value) == 0: #none
        print("none")
        robot.straight(-30)
        robot.stop()
        #use for 2B when there is time
        #left.hold()
        #current = robot.angle()
        #while robot.angle() - current > 90:
        #    right.run(-500)
        #robot.straight(-100)
        #robot.turn(-45)

        robot.straight(50)
        robot.turn(45)

        left_wall = ultrasonic_sensor.distance()
        print(left_wall)
        if left_wall > 150:
            #exit
            robot.turn(-90)
            #drive til green
            print("exiting")
            robot.straight(100)
            return None

        #wall align
        robot.turn(90)

        tile1 = ultrasonic_sensor.distance()
        if tile1 > 150:
            #exit
            robot.turn(-90)
            robot.straight(100)
            return None
        robot.straight(200)
        tile2 = ultrasonic_sensor.distance()
        if tile2 > 150:
            #exit
            robot.turn(-90)
            robot.straight(100)
            return None
        
        # 2 cases: no wall and wall
        if tile2 < 450:
            robot.turn(90)
            robot.stop()
            right.dc(-100)
            left.dc(-100)
            wait(1500)
            robot.straight(35)

    if value[0] > value[1] and value[0] > value[2]: #red
        print("red")
        robot.straight(-100)
        robot.turn(180)
        
        if stored[0] == "black":
            sorter.run_target(100, 180)
            robot.stop()
            right.dc(-100)
            left.dc(-100)
            wait(2000)
            robot.stop()
            robot.straight(75)
        if stored[1] == "black":
            sorter.run_target(100, 300)
            robot.stop()
            right.dc(-100)
            left.dc(-100)
            wait(2000)
            robot.stop()
            robot.straight(75)
        if stored[2] == "black":
            sorter.run_target(100, 420)
            robot.stop()
            right.dc(-100)
            left.dc(-100)
            wait(2000)
            robot.stop()
            robot.straight(75)
        
        #after all balls depoed
        
        robot.turn(-45)
    
        #tile1 reading unlikely to be able to get
        tile1 = ultrasonic_sensor.distance()
        robot.straight(200)
        tile2 = ultrasonic_sensor.distance()
        print(tile2)
                # 2 cases: no wall and wall
        if tile2 < 450:
            robot.turn(90)
            robot.stop()
            right.dc(-100)
            left.dc(-100)
            wait(1500)
            robot.straight(35)
        if tile2 > 450:
            #exit
            robot.turn(-90)
            robot.straight(100)
            return None


    if value[1] > value[0] and value[1] > value[2]: #green
        print("green")
        robot.straight(-100)
        robot.turn(180)
        #select slots to depo balls not written
        
        if stored[0] == "white":
            sorter.run_target(100, 180)
            robot.stop()
            right.dc(-100)
            left.dc(-100)
            wait(2000)
            robot.stop()
            robot.straight(75)
        if stored[1] == "white":
            sorter.run_target(100, 300)
            robot.stop()
            right.dc(-100)
            left.dc(-100)
            wait(2000)
            robot.stop()
            robot.straight(75)
        if stored[2] == "white":
            sorter.run_target(100, 420)
            robot.stop()
            right.dc(-100)
            left.dc(-100)
            wait(2000)
            robot.stop()
            robot.straight(75)

        #after all balls depoed
        robot.turn(-45)
    
        #tile1 reading unlikely to be able to get
        tile1 = ultrasonic_sensor.distance()
        robot.straight(200)
        tile2 = ultrasonic_sensor.distance()

                # 2 cases: no wall and wall
        if tile2 < 450:
            robot.turn(90)
            robot.stop()
            right.dc(-100)
            left.dc(-100)
            wait(1500)
            robot.straight(35)
        if tile2 > 450:
            #exit
            robot.turn(-90)
            robot.straight(100)
            return None


def wall_track(distance):
    global quit, evac
    robot.stop()
    robot.reset() #this assums the robot is straight when start, coz it uses this as the initial 0 angle
    while robot.distance() < distance:
        # check reading for dist to left wall
        left_dist = ultrasonic_sensor.distance() #you either \use a or b, not both
        corrected_left_dist = left_dist * abs(math.cos(robot.angle() * 3.14 / 180))
        
        if corrected_left_dist > 200:
            error = 0
        else:
            error = corrected_left_dist - 85 #actual value is 75 #error is positive means robot too far, means should face negative angle
            #if youre just checking logic, you can change the 75, 75 very close to wall, hard to see if working

        target_angle = error * -0.23 # this affects the max angle the robot can tilt

        angle_error = robot.angle() - target_angle

        #print("current ", left_dist, "target", corrected_left_dist, "error", error, target_angle, "running angle", -angle_error * 10.0)
        #negative angle error should rutn right, so 

        object_detection()
        robot.drive(125, -angle_error * 12.0) #this affects how fast it turns back

        if len(ev3.buttons.pressed()) > 0:
            quit = True
            evac = False
            while len(ev3.buttons.pressed()) > 0:
                robot.stop()
            break
        

    robot.stop()

def exit_wall_track(distance):
    robot.stop()
    robot.reset() #this assums the robot is straight when start, coz it uses this as the initial 0 angle

    while robot.distance() < distance:
        # check reading for dist to left wall
        left_dist = ultrasonic_sensor.distance() #you either \use a or b, not both
        corrected_left_dist = left_dist * abs(math.cos(robot.angle() * 3.14 / 180))

        if corrected_left_dist > 200:
            robot.straight(100)
            robot.stop()
            robot.turn(-90)
            distance = 0
            robot.straight(100)
            return None
            
        else:
            error = corrected_left_dist - 85 #actual value is 75 #error is positive means robot too far, means should face negative angle
            #if youre just checking logic, you can change the 75, 75 very close to wall, hard to see if working

        target_angle = error * -0.23 # this affects the max angle the robot can tilt

        angle_error = robot.angle() - target_angle

        #print("current ", left_dist, "target", corrected_left_dist, "error", error, target_angle, "running angle", -angle_error * 10.0)
        #negative angle error should rutn right, so 

        object_detection()
        robot.drive(125, -angle_error * 12.0) #this affects how fast it turns back

        if len(ev3.buttons.pressed()) > 0:
            quit = True
            while len(ev3.buttons.pressed()) > 0:
                robot.stop()
            break

    robot.stop()

def evac_zone():
    global evac, exited, quit
    robot.straight(220) #move 20cm into evac
    for i in range(4):
        scan_dist()
        if quit:
            return None
    
    for i in range(4):
        exit_scan_dist()
        if quit:
            return None
    
        if exited is True:
            break
            evac = False


def calibration(mode):
     if mode == "rgb":
         while True:
             print("Calibration values: ", left_light_sensor.rgb(), right_light_sensor.rgb())
     elif mode == "calculated":
         while True:
                 print("Calibration values: ", calibrated_values())
     elif mode == "objcali":
         while True:
             front_val = front_light_sensor.rgb()
             if (front_val[0] + front_val[1] + front_val[2]) == 0:
                 pass
             else:
                 print(sum(front_val))


# !! MAIN LOOP !! #

initial_dist = robot.distance()
dist_list = []

while robot.distance() - initial_dist < 300: #step 0
    robot.drive(150, 0)
    object_detection()
    us = ultrasonic_sensor.distance()
    if us > 1050:
        pass
    else:
        dist_list.append(us)

robot.stop()
dist_list.sort()
print(dist_list)
count = len(dist_list)
if count % 2 == 0:
    median = (dist_list[count // 2] + dist_list[count // 2 - 1]) / 2
else:
    median = dist_list[count // 2]
robot.straight(-300)
print(median)

# starting beep
ev3.speaker.beep()

evac = True

while True:
    ev3.screen.print("press to start")
    robot.stop()
    robot.reset()
    while len(ev3.buttons.pressed()) == 0:
        pass
    while len(ev3.buttons.pressed()) > 0:
        pass
    quit = False
    while True:
        if evac is False:  # if not in evacuation zone
            # line tracking
            values = calibrate_values()
            l_red = values[0]
            l_green = values[1]
            l_blue = values[2]
            r_red = values[3]
            r_green = values[4]
            r_blue = values[5]
            line_track(l_red, l_green, l_blue, r_red, r_green, r_blue)

            # responsible for detecting obstacle
            obstacle_detection()

            # detect silver
            silver_detection()

            # detect red
            red_line_detection(l_red, l_green, l_blue, r_red, r_green, r_blue)

            if len(ev3.buttons.pressed()) > 0:
                quit = True
                while len(ev3.buttons.pressed()) > 0:
                    robot.stop()
                
            if quit:
                break

        else:  # if in evacuation zone
            evac_zone()
            if quit:
                break
