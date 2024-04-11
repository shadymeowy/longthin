import timeit
import os
import cv2
from longthin import ParkDetector

# Create a ParkDetector object
detector = ParkDetector(1280, 720)

# Load images from a directory and process them
path = "images/"
files = os.listdir(path)
files.sort()

# Measure the time taken to process an image
file0 = files[0]
image = cv2.imread(path + file0)
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
time = timeit.timeit(lambda: detector.process_image(image), number=100)/100
print("Time taken to process an image:", time, "seconds")

# Process all images in the directory
for file in files:
    image = cv2.imread(path + file)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    detector.process_image(image, debug=True)
    cv2.waitKey(1)
