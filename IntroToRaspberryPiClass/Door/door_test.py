#Copyright by Slamivait LLC
#gpio pin library for sensors (door open/close & LED)
import RPi.GPIO as GPIO
import time

# setup LED
red_pin = 27
green_pin = 17
blue_pin = 22

door_sensor_pin = 16

def clear_led():
    GPIO.output(blue_pin, GPIO.LOW)
    GPIO.output(green_pin, GPIO.LOW)
    GPIO.output(red_pin, GPIO.LOW)

def control_led(on, color_pin):
    clear_led()
    if on:
        GPIO.output(color_pin, GPIO.HIGH)
    else:
        GPIO.output(color_pin, GPIO.LOW)

def clear():
    clear_led()
    time.sleep(0.5)

def door_is_open():
    door_state = GPIO.input(door_sensor_pin)
    if door_state == GPIO.HIGH:
        print("Door is open!")
        return True
    print("Door is closed")
    return False

if __name__ == "__main__":
    # setup
    # set pin read mode to GPIO/BCM
    GPIO.setmode(GPIO.BCM)

    # all are always outputting data
    GPIO.setup(blue_pin, GPIO.OUT)
    GPIO.setup(red_pin, GPIO.OUT)
    GPIO.setup(green_pin, GPIO.OUT) 

    # door sensor setup
    GPIO.setup(door_sensor_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP) 

    # start sequence
    while True:
        if door_is_open():
            control_led(True, red_pin)
        else:
            control_led(True, green_pin)
        time.sleep(0.5)
        clear()


