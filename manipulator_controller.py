#!/usr/bin/python3.5

import sys
import signal
import argparse
import RPi.GPIO as GPIO
from time import sleep, perf_counter, time

# Import the PCA9685 module.
import Adafruit_PCA9685

from threading import Thread
from itertools import zip_longest

import paho.mqtt.client as mqtt
import json

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(50)

# Configure min and max servo pulse lengths
servo_min = 120  # Min pulse length out of 4096
servo_max = 540  # Max pulse length out of 4096

current_angles = [20, 83, 155, 50, 35, 165]
start_angles = (20, 83, 155, 50, 35, 165)

movement_end = [0, 0, 0, 0, 0, 0]


last_telemetry = {
    "horizontal_angle": current_angles[0],
    "grab_angle": current_angles[5],
    "vertical_position": 1}


client = mqtt.Client()
connected = False


help_info = {
    'turn':'''
turn - set turning angle of manipulator.

    Usage: turn <angle>
    
    Parameters:
        <angle>    Angle of manipulator turn. Between 0 and 180.
'''.strip(),
    'down': '''
down - set claw down.
'''.strip(),
    'up': '''
up - set claw up.
'''.strip(),
    'grab': '''
grab - set claw open angle.

    Usage: grab <angle>
    
    Parameters:
        <angle>    Claw opening angle. Between 120 and 180.
'''.strip(),
    'start': '''
start - set manipulator in start state.
'''.strip(),
	'help': '''
help - display help messages.

    Usage: help [<command name>]

    Parameters:
        <command name>    Show help for specifyed command.
'''.strip(),
    'exit': '''
exit - exit the program.
'''.strip()
}


def keyboard_interrupt_handler(signal, frame):
	print('To exit the program use exit command!')


def angle_to_pulse(angle):
    if angle < 0 or angle > 180:
        raise ValueError('Invalid angle: {}'.format(angle))

    return round((angle - 0) / (180 - 0) * (servo_max - servo_min) + servo_min)


def use_claw(angle):
    global current_angles
    if angle < 120 or angle > 180:
        raise ValueError('grab', angle)
    for i in range(len(current_angles)):
        if i == 5:
            thread = Thread(target=set_servo_angle, args=(i, angle))
        else:
            thread = Thread(target=hold_pulse, args=(i, angle_to_pulse(abs(current_angles[5] - angle))))
        thread.start()
    last_telemetry['grab_angle'] = angle


def down():
    global current_angles, vertical_position
    for i in range(len(current_angles)):
        if i == 2:
            thread = Thread(target=set_servo_angle, args=(i, 162))
        elif i == 3:
            thread = Thread(target=set_servo_angle, args=(i, 60))
        elif i == 5:
            thread = Thread(target=hold_pulse, args=(i, angle_to_pulse(63 - start_angles[3])))
        else:
            thread = Thread(target=hold_pulse, args=(i, angle_to_pulse(63 - start_angles[3])))
        thread.start()
    last_telemetry['vertical_position'] = 0
#    vertical_position = 0


def up():
    global current_angles, vertical_position
    for i in range(len(current_angles)):
        if i == 2:
            thread = Thread(target=set_servo_angle, args=(i, start_angles[i]))
        elif i == 3:
            thread = Thread(target=set_servo_angle, args=(i, start_angles[i]))
        elif i == 5:
            thread = Thread(target=hold_pulse, args=(i, angle_to_pulse(63 - start_angles[3])))
        else:
            thread = Thread(target=hold_pulse, args=(i, angle_to_pulse(63 - start_angles[3])))
        thread.start()
    last_telemetry['vertical_position'] = 1
#    vertical_position = 1


def turn(angle):
    if angle > 180 or angle < 0:
        raise ValueError('turn', angle)
    for i in range(len(current_angles)):
        if i == 0:
            thread = Thread(target=set_servo_angle, args=(i, angle))
        else:
            thread = Thread(target=hold_pulse, args=(i, angle_to_pulse(abs(current_angles[0] - angle))))
        thread.start()
    last_telemetry['horizontal_angle'] = angle


def set_all_servo_start():
    global start_angles

    for servo_num in range(len(current_angles)):
        thread = Thread(target=set_servo_start, args=(servo_num,))

        thread.start()

		
def set_servo_start(servo_num):
    global start_angles, current_angles, movement_end
    
    pulse = angle_to_pulse(start_angles[servo_num])
    current_pulse = angle_to_pulse(current_angles[servo_num])
    if pulse > current_pulse:
        for pulse_len in range(current_pulse, pulse):
            pwm.set_pwm(servo_num, 0, pulse_len)
    elif pulse < current_pulse:
        for pulse_len in range(current_pulse, pulse, -1):
            pwm.set_pwm(servo_num, 0, pulse_len)
    else:
        hold_pulse(servo_num, angle_to_pulse(60) - angle_to_pulse(start_angles[3]))

    current_angles[servo_num] = start_angles[servo_num]
    movement_end[servo_num] = 1


def set_servo_angle(servo_num, angle):
    global current_angles, movement_end
    pulse = angle_to_pulse(angle)
    current_pulse = angle_to_pulse(current_angles[servo_num])
    if pulse >= current_pulse:
        for pulse_len in range(current_pulse, pulse):
            pwm.set_pwm(servo_num, 0, pulse_len)
    else:
        for pulse_len in range(current_pulse, pulse, -1):
            pwm.set_pwm(servo_num, 0, pulse_len)
    current_angles[servo_num] = angle
    movement_end[servo_num] = 1
    

def hold_pulse(servo_num, pulse):
    global current_angles
    for i in range(pulse):
        pwm.set_pwm(servo_num, 0, angle_to_pulse(current_angles[servo_num]))
    movement_end[servo_num] = 1


def grouper(iterable, n, fillvalue=None):
    args = [iter(iterable)] * n
    return zip_longest(*args, fillvalue=fillvalue)


def send_telemetry():
    global client, last_telemetry

    client.publish('v1/devices/me/telemetry', json.dumps(last_telemetry))


def use_help_command(command_args):
    if len(command_args) == 0:
        print(*help_info.values(), sep='\n')
    elif len(command_args) == 1:
        if command_args[0] in help_info:
            print(help_info[command_args[0]])
        else:
            print('Error: Anspecified parametr of help: {}.'.format(command_args[0]))
    else:
        print('Error: To many parametrs for help command.')


def check_distance():
    GPIO.output(PIN_TRIGGER, GPIO.HIGH)

    sleep(0.00001)

    GPIO.output(PIN_TRIGGER, GPIO.LOW)

    while GPIO.input(PIN_ECHO) == 0:
        pulse_start_time = time()

    while GPIO.input(PIN_ECHO) == 1:
        pulse_end_time = time()

    pulse_duration = pulse_end_time - pulse_start_time
    distance = round(pulse_duration * 17150, 2)
    print(distance)

    if distance < 8.5:
        vertical_position = 0
    else:
        vertical_position = 1


def on_connect(client, userdata, flags, rc):
    global connected
    print('Connected with result code {}'.format(rc))

    connected = True


def on_disconnect(client, userdata, rc):
    global connected
    print('Disconnect with code {}'.format(rc))

    connected = False
    client.reconnect()


def main():
    global num_servos, end_send, vertical_position, movement_end

    set_all_servo_start()
    check_distance()
    send_telemetry()
    
    while True:		
        try:
            command_string = input("Command: ")
        except EOFError:
            print()
            command_string = ''
        
        if command_string == '':
            continue
        
        command_string = command_string.split()
        command_args = command_string[1:]
        command = command_string[0]

        try:
            for i in range(len(movement_end)):
               movement_end[i] = 0

            if command == 'turn':
                if len(command_args) > 1:
                    raise KeyError(command)
                turn(int(command_args[0]))
            elif command == 'down':
                if len(command_args) > 0:
                    raise KeyError(command)
                down()
            elif command == 'up':
                if len(command_args) > 0:
                    raise KeyError(command)
                up()
            elif command == 'grab':
                if len(command_args) > 1:
                    raise KeyError(command)
                use_claw(int(command_args[0]))
            elif command == 'start':
                if len(command_args) > 0:
                    raise KeyError(command)
                set_all_servo_start()
            elif command == 'help':
                use_help_command(command_args)
            elif command == 'exit':
                if len(command_args) > 0:
                    raise KeyError(command)
                end_send = True
                break
            else:
                print('''
                      Error: command '{}' isn\'t specifyed.\nUse command \'help\' to see list of avaliable commands
                      '''.strip().format(command))
                continue

            while not all(movement_end):
                pass

            check_distance()
            send_telemetry()
        except KeyError as e:
            print('Error: To many parameters for {} command'.format(e.args[0]))
        except ValueError as e:
            if len(e.args) == 1:
                print('Error: Incorrect parameter for {}: {}'.format(command, e.args[0].split(': ')[-1]))
            else:
                print('Error: Incorrect angle for {}: {}'.format(e.args[0], e.args[1]))


if __name__ == '__main__':
    signal.signal(signal.SIGINT, keyboard_interrupt_handler)

    client.username_pw_set('WydkkYetCeLl77wk0tOk')
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect

    client.connect('demo.thingsboard.io', 1883, 60)

    client.loop_start()
    
    while not connected:
        pass

    GPIO.setmode(GPIO.BCM)

    PIN_TRIGGER = 4
    PIN_ECHO = 17

    GPIO.setup(PIN_TRIGGER, GPIO.OUT)
    GPIO.setup(PIN_ECHO, GPIO.IN)

    GPIO.output(PIN_TRIGGER, GPIO.LOW)

    print('Waiting for sensor to settle')

    sleep(2)    

    main()
    client.loop_stop()
    client.disconnect()
    GPIO.cleanup()
    sys.exit()
