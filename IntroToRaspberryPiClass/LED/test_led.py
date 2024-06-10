#Copyright by Slamivait LLC
#gpio pin library for sensors (door open/close & LED)
import RPi.GPIO as GPIO
import time

# setup LED
red_pin = 27
green_pin = 17
blue_pin = 22

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
    time.sleep(0.5)
    clear_led()
    time.sleep(0.5)

if __name__ == "__main__":
    # setup
    # set pin read mode to GPIO/BCM
    GPIO.setmode(GPIO.BCM)

    # all are always outputting data
    GPIO.setup(blue_pin, GPIO.OUT)
    GPIO.setup(red_pin, GPIO.OUT)
    GPIO.setup(green_pin, GPIO.OUT)  

    # start sequence
    while True:
        # Red
        control_led(True, red_pin)
        clear()

        # Blue
        for i in range(2):
            control_led(True, blue_pin)
            clear()

        # Green
        for i in range(3):
            control_led(True, green_pin)
            clear()



