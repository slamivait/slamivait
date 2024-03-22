#AGPL-3.0 license
import time
import itertools
import io

#array/matrix based library
import numpy as np

#camera libraries
from picamera2 import Picamera2, Preview, MappedArray
from libcamera import *

#image processing library opencv
import cv2

#PIL image processing libraries
from PIL import Image

#pytorch libraries
from torchvision.io import read_image, ImageReadMode
from torchvision import transforms
from torchvision import models
import torch

#YOLO
from ultralytics import YOLO
#YOLOv8 logger
import comet_ml

#gpio pin library for sensors (door open/close & LED)
import RPi.GPIO as GPIO

class SensorManager:
    def __init__(self):
        # set pin read mode to GPIO/BCM
        GPIO.setmode(GPIO.BCM)

        # setup LED
        self.red_pin = 27
        self.green_pin = 17
        self.blue_pin = 22

        #Set the GPIO mode to BCM
        self.DOOR_SENSOR_PIN=16
        GPIO.setup(self.DOOR_SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # all are always outputting data
        GPIO.setup(self.blue_pin, GPIO.OUT)
        GPIO.setup(self.red_pin, GPIO.OUT)
        GPIO.setup(self.green_pin, GPIO.OUT)        

        # create Picamera object
        self.picam2 = Picamera2()    
        print("Created picam2")
        #self.picam2.start_preview(Preview.QTGL, x=100, y=200, width=800, height=600,transform=Transform(vflip=1))
        print("Started preview")
        self.faces = []
        self.picam2.start()
        print("Started")

        self.faces = [] 

        #setup pytorch to run optimally
        torch.backends.quantized.engine='qnnpack'
        torch.set_default_device('cpu')

        self.model = YOLO("yolov8n.pt")
        #self.model.set_classes(["person"])
        comet_ml.init()        

        #variables for tracking performance on neural network
        self.frame_count = 0  
        self.last_logged = 0

    def control_led(self, on, color_pin):
        self.clear_led()
        if on:
            GPIO.output(color_pin, GPIO.HIGH)
        else:
            GPIO.output(color_pin, GPIO.LOW)

    def clear_led(self):
        GPIO.output(self.blue_pin, GPIO.LOW)
        GPIO.output(self.green_pin, GPIO.LOW)
        GPIO.output(self.red_pin, GPIO.LOW)

    def door_is_open(self):
        door_state = GPIO.input(self.DOOR_SENSOR_PIN)
        if door_state == GPIO.HIGH:
            print("Door is open!")
            return True
        print("Door is closed")
        return False

    def start_monitoring(self):
        print("Starting video")
        try:
            while True:
                # process stream
                triggered = False
                image = self.picam2.capture_array("main")

                face_in_stream = self.detect_faces(image)
                if face_in_stream:
                    self.control_led(True, self.blue_pin)
                    triggered = True
                if self.door_is_open():
                    self.control_led(True, self.red_pin)
                    triggered = True
                if not triggered:
                    self.control_led(True, self.green_pin)
                #time.sleep(0.5)

                if cv2.waitKey(10) == 27:
                    self.picam2.close()
                    exit()
        except KeyboardInterrupt:
            self.picam2.close()
            exit()          

    # callback for image processing
    def detect_faces(self, image):
        #eliminate the 4th channel to ensure smooth programming
        image = image[:,:,0:3]

        #detect faces
        results = self.model(image)

        # determine whether any faces were detected, and return the results
        faces_detected = False
        for result in results:
            if len(result.boxes) > 0:
                faces_detected = True
        #print("Faces detected: ", faces_detected)

        # print and draw results
        #print("Results: ", results)
        self.draw_faces(results, image)
        return faces_detected

    def draw_faces(self, results, image):
        if image is None:
            print("Empty image!")
            return       
        for result in results:
            print("Detection count: ", result.boxes.shape[0])

            image = result.plot(conf=True, line_width=2, font_size=14, font='Arial.tff', pil=False, img = np.array(image), im_gpu=None, kpt_radius=5, kpt_line=True, labels=True, boxes=True, masks=True, probs=True, show=False)
        cv2.imshow('face_detect', np.array(image))
