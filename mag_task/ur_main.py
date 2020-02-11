# Write your solution here

import time
import math
from ur_aux import VrepModel

# image processing
import numpy as np
import cv2 

#################### Functions ###########################

# Put value into range 0 - 255
def rng(x):
  return max(min(x,255),0)

# Find pegs in the image
# Return: [(x1,y1,rad1),(x2,y2,rad2)...]  
def findPegs(image, accumulator=10):
  _,gray = cv2.threshold(image[:,:,2], 180, 255, cv2.THRESH_TOZERO)  # pegs are red
  # find circles
  circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 50, param1=50, param2=accumulator, minRadius=10, maxRadius=40)
  if circles is None:
    return []
  else:
    circles = np.round(circles[0,:]).astype('int')    
    isRed = lambda p,q: image[q,p,0] < 100 and image[q,p,1] < 100 and image[q,p,2] > 180
    return [x for x in circles if isRed(rng(x[0]),rng(x[1]))]
   

# Show image and circles
# image - image array
# lst - list of found circles, could be empty
def visualize(image,lst=[]):
  img = image.copy()
  if lst:
    for (x,y,r) in lst:
      cv2.circle(img, (x,y), r, (0,255,0), 4)
      cv2.rectangle(img, (x-5,y-5), (x+5,y+5), (0,128,255), -1)      
  cv2.imshow('view', img)
  cv2.waitKey(0)

##################### Prepare ###############################

# port is in remoteApiConnections.txt
port = 19997
# simplify function call
rad = math.radians


##################### Start #############################

ur = VrepModel(port)   # create object
ur.startSimulation()   # start to work

#################### Solution ###########################

# Write your solution here
# You can use the following functions or write your own.

ur.ptp([0,rad(20),rad(-75),rad(-35),rad(90),0])     # go to some position in joint space

pos = ur.getToolPosition() # read current position

pos[1] -= 0.1
ur.lin(pos)            # go to new position in Cartesian space

ur.gripperOpen(True)   # use True/False to open/close the gripper

img = ur.getImage()    # get camera image
pegs = findPegs(img)   # try to find pegs

visualize(img,pegs)    # you can use it to visualize the picture, press any key to close the window


#################### Finish ###############################
ur.ptp([0,0,0,0,0,0])   # go to base 
ur.check()              # check the result
ur.stopSimulation()




