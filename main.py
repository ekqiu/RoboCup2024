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

# Create your objects here.
ev3 = EV3Brick()

## MOTORS

#drive motors
left = Motor(Port.C, positive_direction=Direction.COUNTERCLOCKWISE)
right = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)

claw = Motor(Port.A)
sorter = Motor(Port.D)

##robot
robot = DriveBase(left, right, 33, 187)

# adjust straight/turning speed and acl
#robot.settings(350, 150, 300, 200)

# Colour Sensors facing bricks.
front_light_sensor = ColorSensor(Port.S1)

# Colour Sensors facing floor.
left_light_sensor = ColorSensor(Port.S4)
right_light_sensor = ColorSensor(Port.S3)

#ultrasonic
ultrasonic_sensor = UltrasonicSensor(Port.S2)

left_min_red = 5
left_max_red = 55

left_min_green = 7
left_max_green = 46 # g30, w64

left_min_blue = 11
left_max_blue = 100

right_min_red = 3
right_max_red = 50

right_min_green = 7
right_max_green = 46 #g30, w63

right_min_blue = 3
right_max_blue = 90

left_red_range = left_max_red - left_min_red
left_green_range = left_max_green - left_min_green
left_blue_range = left_max_blue - left_min_blue

right_red_range = right_max_red - right_min_red
right_green_range = right_max_green - right_min_green
right_blue_range = right_max_blue - right_min_blue

# variables
left_val = None
right_val = None
evac = False
last_black = 0
stored = []

# Starting beep.
ev3.speaker.beep()

# Functions.
def calibrate_values():
    # max min to make sure values do not go below 0 and above 1
    global left_val, right_val
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

def only_line_track(l_red, l_green, l_blue, r_red, r_green, r_blue):
        error = l_green - r_green
        speed = (1 - abs(error)) * 80  # adjust speed
        rotation = error * 225
        robot.drive(speed, rotation)

def line_track(l_red, l_green, l_blue, r_red, r_green, r_blue):
    global last_black
    #val = calibrate_values()
    if (
        max(l_red, l_green, l_blue, r_red, r_green, r_blue) < 0.4
    ):  # Double black case .5 .6 white, .1 .2 green
        print("Double black detected")
        robot.stop()
        

        robot.straight(-15)  # Move back to check for green squares
        wait(100)
        val = calibrate_values()
        nl_green = val[1]
        nr_green = val[4]
        nl_red = val[0]
        nr_red = val[3]
        saw = ""
        if nl_green <= 0.6: #left black
            robot.turn(-18)
            saw = "left"
            #BLACK left



        if nr_green <= 0.6: #right black
            robot.turn(18)
            saw = "right"
            #BLACK right
        
    

        val = calibrate_values()
        nl_green = val[1]
        nr_green = val[4]
        nl_red = val[0]
        nr_red = val[3]
        # Checking green
        if nl_red <= 0.4:  # low red for left sensor
            if nr_red <= 0.3:  # low red for right sensor
                if nl_green <= 0.3:
                    if nr_green <= 0.3:
                        print("BOTH black")
                    single_line_track(100, "left", "left", 20)
                else:
                    print("BOTH green")  # If both are green
                    robot.turn(180)  # u-turn
                    
            else:  # If only left is green -> turn left
                print("LEFT green")
                single_line_track(110, "left", "left", 20)
        else:
            if nr_red <= 0.6:  # low red for right sensor
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
                        newval = calibrate_values()
                        error = newval[1] - newval[4]
                        speed = (1 - abs(error)) * 75  # adjust speed
                        rotation = error * 100
                        robot.drive(speed, rotation)
    else:
        error = l_green - r_green
        speed = (1 - abs(error)) * 100  # adjust speed
        rotation = error * 120
        if min(l_red, l_green, l_blue, r_red, r_green, r_blue) < 0.7:
            last_black = robot.distance()
        gap = robot.distance() - last_black
        print(gap)
        if gap > 50:
            ev3.speaker.beep()
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
                        ev3.speaker.beep()
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
                        ev3.speaker.beep()
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
#both black, 

def single_line_track(distance, sensor, side, speed=100.0, gain=100.0):
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
    more = 1
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
            print("Silver Detected, stopping here.")
            robot.stop()
            evac_zone()
            evac = True

#clawfuncs
def pickup(action):
    angle = 780
    if action == "open":
        claw.run_angle(500, -abs(angle))
    else:
        claw.run_angle(500, angle)

def red_line_detection(l_red, l_green, l_blue, r_red, r_green, r_blue):
    if ((l_red + r_red) / (l_red + l_green + l_blue + r_red + r_green + r_blue)) >= 0.5:
        print("Red Detected, stopping here.")
        robot.stop()

def object_detection():
    global  left_val, right_val, evac
    if evac is True:
        front_val = front_light_sensor.rgb()
        # check if anything there
        if front_val[0] != 0 and front_val[1] != 0 and front_val[2] != 0:
            
            randist = robot.distance()
            robot.drive(20, 0)
            claw.run_angle(50, 100)

            robot.straight(-abs(robot.distance() - randist))

            front_val = front_light_sensor.reflection() #recheck the ball

            if front_val < 20: #black
                stored.append("black")
                pickup("close")
                pickup("open")
                sorter.run_angle(100, 120)
            else: #white
                stored.append("white")
                pickup("close")
                pickup("open")
                sorter.run_angle(100, 120)
    else:
        front_val = front_light_sensor.rgb()
        # check if anything there
        if front_val[0] != 0 and front_val[1] != 0 and front_val[2] != 0:
            print((front_val[2]) / (front_val[0] + front_val[1] + front_val[2]))
            if ((front_val[2]) / (front_val[0] + front_val[1] + front_val[2])) < 0.7:
                print("Obstacle Detected, going around it.")
                robot.straight(-50)
                robot.turn(90)
                robot.straight(100)
                robot.turn(-90)
                while left_val[2] >= (left_min_green + 10) and right_val[2] <= (
                    right_min_green + 10
                ):
                    robot.drive(80, 10)
                robot.turn(90)

                robot.drive(100, -10)
                return
        else:
            front_val = front_light_sensor.rgb()
            if ((front_val[2]) / (front_val[0] + front_val[1] + front_val[2])) < 0.7:
                    print("Obstacle Detected, going around it.")
                    robot.straight(-50)
                    robot.turn(90)
                    robot.straight(100)
                    robot.turn(-90)
                    #USE ULTRASONIC SENSOR TO TRACK AROUND
                    robot.drive(80, 10)
                    robot.turn(90)

                    robot.drive(100, -10)
                    return

def scan_dist():
    initial_dist = robot.distance()
    dist_list = []

    while robot.distance() - initial_dist < 300: #step 0
        robot.drive(80, 0)
        #object_detection()
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
            robot.drive(80, 0)
            #object_detection()
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
    
    print(median)
    wall_track(median - 300)
    scan_for_depo()

def scan_for_depo():
    robot.turn(45)
    robot.straight(110)
    #one wheel turn
    robot.stop()
    current = robot.angle()
    while robot.angle() - current > -90:
        right.run(500)
        #object_detection()
    robot.straight(50)
    # check if evac is there
    # use front colour sensor to check if the value is red (dead ball), green (alive ball) or black (nothing)
    value = front_light_sensor.rgb()

def evac_zone():
    robot.straight(200) #move 20cm into evac

def wall_track(distance):
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

        print("current ", left_dist, "target", corrected_left_dist, "error", error, target_angle, "running angle", -angle_error * 10.0)
        #negative angle error should rutn right, so 

        robot.drive(50, -angle_error * 12.0) #this affects how fast it turns back

    robot.stop()
        #have to decide on sped first, then tune the multipliers
        #make it drive
    #nvm you print the values and test first
#error is how far off it is from 7.5cm
#so e.g. if the error is 0, then the robot should be heading traight forward
# so we need to calculate what angle it is supposed to be facing
# if its too close then we should be facing away from wall, too far face towards wall questions?    
#so next step is to compare our current angle and the angle we should be facing
# so if you just do error* 10, lets say the robot is currently against the wall, means error is -75
#but ofc if we make the robot turn to 75 degrees its too much
# so we add a multiplier
# so we can do smth like target angle is error * 0.1 so itll tilt by 7.5 deg

#this just calculates the target angle. then we need to figure the difference between current angle and the r
#so another error to calc
#then that error feeds to the drive
    
    #if the robot is tilted the measured dist from us sensor is longer than the actual dist the robot is to the wall

    #if you know the angle of the robot, its just using cos
    #its just cosine a is actual reading
    #the angle is the robot angle
    # but the angle need to convert to radians



#i got no voice

# this function you test on its own first #make new one just wall track youll use at many places in evac

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
                print((front_val[2]) / (front_val[0] + front_val[1] + front_val[2]))

#RUN
#Reset claw and depo position

claw.run_target(100, 10)
sorter.run_target(100, 0)

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

        # responsible for detecting objects
        object_detection()

        # detect silver
        silver_detection()

        # detect red
        red_line_detection(l_red, l_green, l_blue, r_red, r_green, r_blue)
    else:  # if in evacuation zone
        evac_zone()
