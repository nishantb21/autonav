import cv2
import numpy as np
import sys
import time

st = time.time()

# Load the image file
img = cv2.imread(sys.argv[1])

# Convert the image to HSV color space
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# Define the lower and upper bounds of the red color in HSV
lower_red = np.array([0, 70, 50])
upper_red = np.array([10, 255, 255])

# Create a mask for the red color in the image
mask = cv2.inRange(hsv, lower_red, upper_red)

# Apply morphological operations to the mask to remove noise
kernel = np.ones((5, 5), np.uint8)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

# Detect contours in the mask
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Iterate over each contour and check if it is an octagon
for contour in contours[:5]:
    approx = cv2.approxPolyDP(contour, cv2.arcLength(contour, True) * 0.01, True)
    if len(approx) == 8:
        # If an octagon is found, return True
        et = time.time()
        print("True")
        elapsed_time = et - st
        print('Execution time:', elapsed_time*1000, 'ms')
        break
    else:
        # If no octagon is found, return False
        et = time.time()
        print("False")
        elapsed_time = et - st
        print('Execution time:', elapsed_time*1000, 'ms')
        break