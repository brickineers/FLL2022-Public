# LEGO type:advanced slot:0
from time import sleep
from spike import PrimeHub, ColorSensor, Motor, MotorPair
from spike.control import wait_for_seconds
from hub import button
from util import time
from runtime.virtualmachine import VirtualMachine
import sys
import system
import hub
from math import *

phub = PrimeHub()
# Ports
wheels = MotorPair('E', 'F')
colorA = ColorSensor('A') #left
colorB = ColorSensor('B') #right
motorC = Motor('C')
motorD = Motor('D')
# Variables
target_light = 58
KP = 0.25
adaptive = 1
LEFT = 1
RIGHT = 2
CENTER = 3
FORWARD = 1
BACKWARD = 2
DEFAULT_SPEED = 40
DEFAULT_TURN_SPEED = 30
CONFIG = 1
RUNNING = 0

# Functions

# Get correction value for line following based on color
def get_line_correction(reflected_light, sign):
    global integral, last_error
    error = reflected_light - target_light
    derivative = error - last_error
    integral = integral + error
    last_error = error
    if adaptive:
        correction = ((KP * error) + (derivative * 1) + (integral * 0.001)) * sign
    else:
        correction = KP * error * sign
    return correction

# Check if sensor detects black
def is_color_black(sensor):
    return sensor.get_reflected_light() < 25 and sensor.get_color() == 'black'

# Check if sensor detects white
def is_color_white(sensor):
    return sensor.get_reflected_light() > 95 and sensor.get_color() == 'white'

# Set sign, sensors for line following and color check
def get_line_follow_parameters(port, align, color='black'):
    if align == RIGHT:
        sign = -1
    else:
        sign = 1
    if port == 'A':
        line_sensor = colorA
        stop_sensor = colorB
    elif port == 'B':
        line_sensor = colorB
        stop_sensor = colorA
    if color == 'black':
        stop_func = is_color_black
    else:
        stop_func = is_color_white
    return sign, line_sensor, stop_sensor, stop_func

# Follow line using port, till other color sensor matches black or white
def line_follow_till_color(port, align, speed, color):
    global integral, last_error
    integral, last_error = 0, 0
    sign, line_sensor, stop_sensor, stop_func = get_line_follow_parameters(port, align, color)
    while True:
        if stop_func(stop_sensor):
            wheels.stop()
            break
        correction = get_line_correction(line_sensor.get_reflected_light(), sign)
        wheels.start_tank_at_power(speed + int(correction), speed - int(correction))

# Follow line using port, till other color sensor moves out of black or white
def line_follow_out_of_color(port, align, speed, color='black'):
    global integral, last_error
    integral, last_error = 0, 0
    sign, line_sensor, stop_sensor, stop_func = get_line_follow_parameters(port, align, color)
    while True:
        if not stop_func(stop_sensor):
            wheels.stop()
            break
        correction = get_line_correction(line_sensor.get_reflected_light(), sign)
        wheels.start_tank_at_power(speed + int(correction), speed - int(correction))

# Follow line using port, for duration (seconds)
def line_follow_timer(port, align, speed, duration):
    global integral, last_error
    integral, last_error = 0, 0
    sign, line_sensor, stop_sensor, stop_func = get_line_follow_parameters(port, align)
    msec = int(duration*1000)
    end_time = time.get_time() + msec
    while True:
        if time.get_time() >= end_time:
            wheels.stop()
            break
        correction = get_line_correction(line_sensor.get_reflected_light(), sign)
        wheels.start_tank_at_power(speed + int(correction), speed - int(correction))

# Drive distance in cm using move_tank
def drive_distance_cm(distance, direction, lspeed = DEFAULT_SPEED, rspeed = None):
    if rspeed == None:
        rspeed = lspeed
    if direction != FORWARD:
        lspeed *= -1
        rspeed *= -1
    wheels.move_tank(distance, 'cm', lspeed, rspeed)

# Drive forward till sensor detects black or white color
def drive_till_color(port, speed, color):
    if port == 'B':
        stop_sensor = colorB
    elif port == 'A':
        stop_sensor = colorA
    if color == 'black':
        stop_func = is_color_black
    else:
        stop_func = is_color_white
    wheels.start_tank(speed, speed)
    while not stop_func(stop_sensor):
        pass
    wheels.stop()

def get_motor_speeds_for_turn(pivot_point, side, speed):
    # Turn slower for CENTER pivot to reduce error
    if pivot_point == CENTER:
        speed = int(speed / 2)
    if pivot_point == LEFT:
        left_speed = 0
    else:
        left_speed = speed
    if pivot_point == RIGHT:
        right_speed = 0
    else:
        right_speed = -speed
    if side == LEFT:
        left_speed *= -1
        right_speed *= -1
    return left_speed, right_speed

# Turn using gyro to given angle
def gyro_turn(degrees, pivot_point, side, speed = DEFAULT_TURN_SPEED):
    left_speed, right_speed = get_motor_speeds_for_turn(pivot_point, side, speed)
    phub.motion_sensor.reset_yaw_angle()
    wheels.start_tank(left_speed, right_speed)
    while True:
        if abs(phub.motion_sensor.get_yaw_angle()) >= degrees:
            wheels.stop()
            break
    phub.motion_sensor.reset_yaw_angle()

# Turn until sensor detects black or white color
def gyro_turn_till_color(pivot_point, side, port, color, speed = DEFAULT_TURN_SPEED):
    left_speed, right_speed = get_motor_speeds_for_turn(pivot_point, side, speed)
    phub.motion_sensor.reset_yaw_angle()
    if color == 'black':
        stop_func = is_color_black
    else:
        stop_func = is_color_white
    if port == 'A':
        stop_sensor = colorA
    elif port == 'B':
        stop_sensor = colorB
    wheels.start_tank(left_speed, right_speed)
    while True:
        if stop_func(stop_sensor):
            wheels.stop()
            break

# Drives in a straight line, not allowing robot to turn
def gyro_move(duration, speed, direction):
    if direction == FORWARD:
        power = speed
        sign = 16
    elif direction == BACKWARD:
        power = -speed
        sign = -16
    phub.motion_sensor.reset_yaw_angle()
    msec = int(duration*1000)
    end_time = time.get_time() + msec
    while True:
        if time.get_time() >= end_time:
            wheels.stop()
            break
        correction = sign * (power * (phub.motion_sensor.get_yaw_angle() / 180))
        wheels.start_tank_at_power(power - int(correction), power + int(correction))
    phub.motion_sensor.reset_yaw_angle()

# Moves arm up on motor D
def arm_up(degrees=None,speed=DEFAULT_SPEED):
    if not degrees:
        degrees = motorD.get_degrees_counted()
    motorD.run_for_degrees(degrees, speed)

# Moves arm down on motor D
def arm_down(degrees=None,speed=DEFAULT_SPEED):
    if not degrees:
        degrees = motorD.get_degrees_counted()
    motorD.run_for_degrees(-degrees, speed)

# Moves forklift up on motor C
def forklift_up(distance, speed=100):
    # 90 degrees is 1cm of worm
    motorC.run_for_degrees(int(-distance*90), speed)

# Moves forklift up on motor C
def forklift_down(distance, speed=100):
    # 90 degrees is 1cm of worm
    motorC.run_for_degrees(int(distance*90), speed)

# Missions
def Tv_Mission():
    # Does the watch TV mussion
    wait_for_seconds(0.5)
    gyro_move(2.6,45,FORWARD)

def Windmill_Mission():
    # Does the windmill mission, pushes 4th time for reliability
    drive_distance_cm(20.5,FORWARD)
    wait_for_seconds(0.7)
    drive_distance_cm(7, BACKWARD)
    drive_distance_cm(9, FORWARD)
    wait_for_seconds(0.7)
    drive_distance_cm(7, BACKWARD)
    drive_distance_cm(9, FORWARD)
    wait_for_seconds(0.7)
    drive_distance_cm(7, BACKWARD)
    drive_distance_cm(9, FORWARD)
    wait_for_seconds(0.7)
    drive_distance_cm(15, BACKWARD)
    gyro_turn(100,CENTER,RIGHT)
    drive_distance_cm(55,FORWARD,85,80)

def Hydrogen_Cell():
    # Picks up one cell and nudges the other
    drive_distance_cm(3, BACKWARD, 70)
    gyro_turn(55, CENTER, LEFT)
    forklift_up(4,100)
    drive_distance_cm(33,FORWARD,50)
    forklift_down(4,100)
    gyro_turn(20, CENTER, RIGHT)
    drive_distance_cm(12, FORWARD)
    gyro_turn(77,CENTER,RIGHT)

def Dinosaur_Transport():
    # Drives the large dinosaur
    gyro_move(5, 85, FORWARD)
    arm_down(155)

def Innovation_Project():
    # Drops off the innovation project in the target area
    gyro_move(1.5, 100, FORWARD)
    

def high_five():
    # High five mission with other team
    wheels.move(25, speed=-40, steering=10)
    arm_up(60)
    gyro_turn(15, CENTER, LEFT)
    drive_distance_cm(4.5, FORWARD)
    drive_distance_cm(1, BACKWARD, DEFAULT_SPEED)
    forklift_up(3)
    drive_distance_cm(3, FORWARD, DEFAULT_SPEED)
    forklift_down(3)
    drive_distance_cm(4, BACKWARD, 20)
    forklift_up(4)
    drive_distance_cm(1, BACKWARD)

def hydroelectric_dam():
    # Does the hydroelectric dam by pushing it
    gyro_move(0.63, 80, FORWARD)
    gyro_turn(36, CENTER, RIGHT,50)
    gyro_turn(15, CENTER, LEFT,50)
    forklift_up(6)
    wheels.move(5, speed=60, steering=0)
    wheels.move(2.5, speed=-60, steering=0)
    forklift_down(6)
    wheels.move(40, speed=-60, steering=15)


def oil_rig():
    # Does the oil rig mission by lifting lever with ramo on back of robot
    forklift_up(7)
    gyro_move(0.7, 75, FORWARD)
    line_follow_till_color('A', LEFT, 40, 'white')
    gyro_turn(85, RIGHT, RIGHT, 30)
    drive_distance_cm(15, BACKWARD)
    drive_distance_cm(5, FORWARD)
    drive_distance_cm(5, BACKWARD)
    drive_distance_cm(5, FORWARD)
    drive_distance_cm(5, BACKWARD)
    drive_distance_cm(6, FORWARD)
    gyro_turn_till_color(LEFT, RIGHT, 'A', 'white')
    gyro_turn_till_color(LEFT, LEFT, 'A', 'black')

def solar_farm():
    # Picks up 2 cells and nudges 3rd
    gyro_turn(35, CENTER, LEFT, 20)
    drive_distance_cm(21, FORWARD, 24, 20)
    forklift_down(4)
    gyro_turn(29, CENTER, LEFT, 20)
    drive_distance_cm(14, FORWARD, 20, 24)
    gyro_turn(15, CENTER, LEFT)
    forklift_up(4)
    gyro_turn(20, CENTER, RIGHT)
    drive_distance_cm(10, FORWARD)
    forklift_down(4)
    gyro_turn(70, CENTER, LEFT)
    drive_distance_cm(90, FORWARD, 75)

def power_plant():
    # Knocks lever down to finish powerplant
    wait_for_seconds(0.5)
    gyro_move(2.2, 75, FORWARD)
    drive_till_color('B', 35, 'black')
    drive_distance_cm(15, FORWARD, 30)
    gyro_turn_till_color(RIGHT, LEFT, 'A', 'black')
    drive_distance_cm(13, BACKWARD, 30)
    arm_down(120)
    line_follow_timer('A', LEFT, DEFAULT_SPEED, 1.7)
    drive_distance_cm(4, BACKWARD)
    arm_up(90, 100)
    drive_distance_cm(4, BACKWARD)
    gyro_turn(14, LEFT, LEFT)
    arm_down(120, DEFAULT_SPEED)
    gyro_turn(85, LEFT, RIGHT)
    drive_distance_cm(80, FORWARD, 90, 90)

def hybrid_car():
    # Completes hybrid car mission by knocking lever
    line_follow_timer('A', LEFT, 40, 0.2)
    line_follow_timer('A', LEFT, 60, 1.5)
    line_follow_till_color('A', LEFT, 40, 'black')
    line_follow_out_of_color('A', LEFT, 50, 'black')
    forklift_down(7)
    wheels.move(22, speed=40, steering=-35)
    drive_distance_cm(2, FORWARD)
    arm_up(160, 20)


def battery_crate():
    # Drops energy cells in battery crate
    wait_for_seconds(0.5)
    forklift_up(3, 100)
    gyro_move(1.5, 40, FORWARD)
    line_follow_timer('A', LEFT, 35, 1)
    line_follow_till_color('A', LEFT, 40, 'black')
    drive_distance_cm(2, FORWARD, 30)
    forklift_down(3, 100)
    drive_distance_cm(120, BACKWARD, 87, 98 )

def toy_factory():
    # Releases the dinosaur
    forklift_up(6, 100)
    gyro_move(1, 70, FORWARD)
    gyro_move(1, 30, FORWARD)
    wait_for_seconds(1)
    drive_distance_cm(45, BACKWARD, 70, 60)
    forklift_down(6, 100)

def run_1(vm, stack):
    Tv_Mission()
    Hydrogen_Cell()
    Windmill_Mission()

def run_2(vm, stack):
    toy_factory()

def run_3(vm, stack):
    power_plant()

def run_4(vm, stack):
    oil_rig()
    hybrid_car()
    high_five()
    solar_farm()

def run_5(vm, stack):
    hydroelectric_dam()

def run_6(vm, stack):
    battery_crate()

def run_7(vm, stack):
    Innovation_Project()

def show_battery_level():
    if hub.battery.capacity_left() < 81:
        hub.led(8)
    else:
        hub.led(6)


run_funcs = [None, run_1, run_2, run_3, run_4, run_5, run_6, run_7]
run_names = [None, "1", "2", "3", "4", "5", "6", "7"]

def update_config():    
    global CONFIG
    CONFIG += 1
    if CONFIG == len(run_funcs):
        CONFIG = 1
    phub.light_matrix.write(run_names[CONFIG])

# Called on RIGHT button pressed
# Moves to the next run
async def on_right_button(vm, stack):
    update_config()

# Called on LEFT button pressed
# Runs the displayed run and after, changes to the next run to streamline launching
async def on_left_button(vm, stack):
    global RUNNING
    if not RUNNING:
        RUNNING = 1
        before = time.get_time()
        run_funcs[CONFIG](vm, stack)
        after = time.get_time()
        print("Run {0} Time start={1}, end={2}, diff={3}".format(CONFIG, before, after, after - before))
        RUNNING = 0
        update_config()
    show_battery_level()

# Main Setup

async def on_start(vm, stack):
    phub.light_matrix.write(str(CONFIG))
    show_battery_level()


def setup(rpc, system, stop):
    vm = VirtualMachine(rpc, system, stop, "FLL2022")
    vm.register_on_button("on_left_button", on_left_button, "left", "pressed")
    vm.register_on_button("on_right_button", on_right_button, "right", "pressed")
    motorC.set_default_speed(DEFAULT_SPEED)
    motorD.set_default_speed(DEFAULT_SPEED)
    motorC.set_stop_action('brake')
    motorD.set_stop_action('brake')
    motorC.stop()
    motorD.stop()
    wheels.set_stop_action('brake')
    motorC.set_degrees_counted(0)
    motorD.set_degrees_counted(0)
    vm.register_on_start("another_unique_string", on_start)
    return vm

setup(None, system.system, sys.exit).start()
