import socket
import picamera2
import time
import numpy as np
import cv2 as cv
from picamera2 import Picamera2

HOST = '192.168.1.3'	# Static IP of the central computing device
PORT=5050 		# Change as per use
s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST,PORT))
print("connected to server")
Picam2 = Picamera2()
preview_config = Picam2.create_preview_configuration({"size": (640,480), "format": "RGB888"},raw=Picam2.sensor_modes[5])
Picam2.configure(preview_config)
Picam2.start()

while True:
	image=Picam2.capture_array("main")
	#cv.imshow("frame", image)
	#cv.waitKey(1)
	
	# Image processing code to be put in here, extract useful info and compress to a string for socket
	
	str1="Your data in string"
	s.send(str1.encode())
s.close()
