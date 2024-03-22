This repo contains three programs which allows you to incrementally test your sensors. 

First, you'll want to program/use the LED program to ensure you've wired your LED correctly. 

Second, you'll want to program/use the Door program to ensure your door sensor is reactive and triggering the LED visual change. 

Third, you'll want to program/use the SecurityCamera program, which incorporates all three sensors (LED, door open/close sensor, and NoIR camera) so the LED is triggered if you open a door, or if a person is detected in your Yolov8 implementation of object detection. Note, you'll need to download the yolov8 pt file which contains the pretrained weights:

yolov8n.pt
https://github.com/ultralytics/ultralytics

some dependencies include:
numpy
picamera2
libcamera
opencv
PIL
torchvision
pytorch
ultralytics
RPi
