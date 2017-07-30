# File:          fuzzyController.py
# Date:          
# Description:   
# Author:        
# Modifications: 

import os, sys
lib_path = os.path.abspath('../../libraries')
sys.path.append(lib_path)

from controller import DifferentialWheels
import fuzzyMethods as fm


import numpy as np

import cv2


from math import atan2
import operator

  

def dump(obj):
      for attr in dir(obj):
	print "obj.%s = %s" % (attr, getattr(obj, attr))

class Enumerate(object):
  def __init__(self, names):
    for number, name in enumerate(names.split()):
      setattr(self, name, number)

# Here is the main class of your controller.
# This class defines how to initialize and how to run your controller.
# Note that this class derives Robot and so inherits all its functions
class fuzzyController (DifferentialWheels):
  
  Mode = Enumerate('STANDARD SEARCH STOP')
  timeStep = 64
  maxSpeed = 100.0
  limDist = 80.0
  
  from_search_to_standard = 0
  
  # User defined function for initializing and running
  # the fuzzyController class
  
  def init(self, reference_depth, reference_horizontal_distance, reference_speed):
    ### Set points
    self.reference_depth = reference_depth
    self.reference_horizontal_distance = reference_horizontal_distance
    self.reference_speed = reference_speed
    
    ### Speed estimation
    self.old_distance = 0
    
    ### Mode selection
    self.mode = self.Mode.STANDARD
    #self.mode = self.Mode.STOP
    
    ### Display initialization
    self.display_teste = self.getDisplay('display_teste')
    #self.display_teste = self.getDisplay('display_teste')
    
    ### Proximity sensors initialization
    self.ps = [self.getDistanceSensor(sensor_name) for
		  sensor_name in ['ps' + str(i) for i in range(8)]]
    for sensor in self.ps:
      sensor.enable(self.timeStep)
      
    ### Obstacle avoidance direction dictionary
    ### dir_id : (left_wheel_speed, right_wheel_speed)
    self.avoidance_directions = {
      0 : (1,0.9), 1 : (1,0.7),  2 : (1,0.4), 
      3 : (1,-0.1), 4 : (-0.1,1),  5 : (0.4,1), 
      6 : (0.7,1), 7 : (0.9,1)
    }           
    
    ### Camera initialization
    self.camera = self.getCamera('camera')
    self.camera.enable(self.timeStep)
    
    ### GPS initialization
    self.gps = self.getGPS('gps')
    self.gps.enable(self.timeStep)
    
    ### Inertial Unit initialization
    self.imu = self.getInertialUnit('inertial_unit')
    self.imu.enable(self.timeStep)  
    
    ###color id dictionary
    self.color_ids = {
    0 : 150, 1 : 50, 2 : 95
    }
    
    ### Assigning color id
    self.follow_color_id = sys.argv[1]
    #self.own_color_id = sys.argv[2]
    
  def getLocalLeaderAbsoluteSpeed(self, new_distance):
    new_distance = self.estimateDistance(new_distance)
    abs_speed = (new_distance - self.old_distance) + (self.getLeftSpeed() + self.getRightSpeed())/2
    #print "", (new_distance - self.old_distance)*self.timeStep
    #abs_speed = (new_distance - self.old_distance) + self.getLeftSpeed()
    self.old_distance = new_distance
    return abs_speed
  
  def estimateDistance(self, depth):
    estimated_distance = 100*0.2*(depth/50.0)
    return estimated_distance
  
  # Function used to control the robot movements and displays 
  def run(self):
    
    speeds = {
        "left_wheel" 		     : 0.0,
        "right_wheel" 	             : 0.0
    }
    
    last_pos = [0,0]
   
    # Main loop
    while True:

      ### IMAGE ACQUISITION
      dispArray = self.camera.getImageArray()
      cv_image = np.asarray(dispArray)
      cv_image = cv_image.astype('uint8')

      ### CONVERTING IMAGE COLOR SPACE
      cv_image = cv_image.swapaxes(0,1)	
      #cv_image = cv2.cvtColor(cv_image, cv2.cv.CV_BGR2RGB)
      	  
      ### IMAGE TREATEMENT
      ### OPENCV USES H VALUES FROM 0 TO 180 BECAUSE IT USES 8 BITS IMAGE
      hsv_image =  cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)
      lower_boundary = np.array([int(0.91*self.color_ids.get(int(self.follow_color_id))),50,50], np.uint8)
      upper_boundary = np.array([int(1.09*self.color_ids.get(int(self.follow_color_id))),255,255], np.uint8)

      mask = cv2.inRange(hsv_image, lower_boundary, upper_boundary )
	      
      contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
      
      ### DISTANCE ESTIMATION
      if len(contours) == 0:
	depth_real = -1
      else:
	depth_real = cv2.contourArea(contours[-1])
      
      cv_image_final = mask
      cv2.drawContours(cv_image_final, contours, -1, (255,150,150) , 0)
            
      ### MOMENTS CALCULATION
      M = cv2.moments(cv_image_final)
      if M['m00'] > 0:
	cy = int(M['m10']/M['m00'])
      else:
	cy = 0      
   
      ### DISPLAYING TREATED IMAGE 
      image_name = 'cv_temp' + str(self.follow_color_id) + '.jpg' 
      cv2.imwrite(image_name , cv_image_final)
      #cv2.imwrite('cv_temp.jpg', hsv_image)
      #cv2.imwrite('cv_temp.jpg', mask)
      imRef = self.display_teste.imageLoad(image_name)
      self.display_teste.imagePaste(imRef, 0, 0)
      self.display_teste.imageDelete(imRef)

      ### Error calculation
      depth_error = (depth_real - self.reference_depth)/self.reference_depth
      horizontal_distance_error = 2*float((cy - self.camera.getWidth()/2))/self.camera.getWidth()
      
      ### MODE CHOICE
      if self.mode != self.Mode.STOP:
	
	if depth_real > 0:
	  ### Initialization
	  if self.from_search_to_standard == 1:
	    speeds["left_wheel"] = 0
	    speeds["right_wheel"] = 0
	    self.from_search_to_standard = 0
	  
	  self.mode = self.Mode.STANDARD
	  
	elif depth_real < 0:
	  self.from_search_to_standard = 1
	  
	  self.mode = self.Mode.SEARCH
 
      ### CHOICE OF CONTROL STRATEGY 
      if self.mode == self.Mode.STANDARD:
	if depth_real > 0:
	  left_wheel_speed, right_wheel_speed = fm.calculateControl(depth_error, horizontal_distance_error)
	  speeds["left_wheel"] =  left_wheel_speed*self.maxSpeed*10
	  speeds["right_wheel"] = right_wheel_speed*self.maxSpeed*10
	  
      elif self.mode == self.Mode.SEARCH:
	speeds["left_wheel"] = 10*self.maxSpeed
	speeds["right_wheel"] = -10*self.maxSpeed
	
      elif self.mode == self.Mode.STOP:
	speeds["left_wheel"] = 0
	speeds["right_wheel"] = 0

      ### Obstacle identification
      if (self.mode != (self.Mode.STOP or self.Mode.Search)):
	if any(elem.getValue() > self.limDist for elem in self.ps):
	  i_max, max_sensor = max(enumerate(elem.getValue() for elem in self.ps), key=operator.itemgetter(1))
	  #i_min, min_sensor = min(enumerate(elem.getValue() for elem in self.ps), key=operator.itemgetter(1))
	  # HACK TENTANDO PEGAR A DIRECAO DIAMETRALMENTE OPOSTA
	  dir_id = (i_max + 3) % 7 #TODO VER SE EH MODELO 8 OU 7, E VER SE 3 EH VALIDO PRA TODOS OS CASOS, MEIO TRICKY
	  (left_speed, right_speed) = self.avoidance_directions.get(dir_id)
	  
	  #speeds["left_wheel"] = (1/6)*speeds["left_wheel"] + 4*self.maxSpeed*left_speed
	  #speeds["right_wheel"] = (1/6)*speeds["right_wheel"] + 4*self.maxSpeed*right_speed
	  speeds["left_wheel"] = (1/6)*speeds["left_wheel"] + 5*self.maxSpeed*left_speed
	  speeds["right_wheel"] = (1/6)*speeds["right_wheel"] + 5*self.maxSpeed*right_speed	
	
	
      ### SEND CALCULATED CONTROL TO THE WHEEL
      self.setSpeed(speeds["left_wheel"], speeds["right_wheel"])
      #abs_speed = self.getLocalLeaderAbsoluteSpeed(depth_real)
      #print 'MODO', self.mode
      #print speeds
      #print 'depth', depth_real
      
      
      ### Perform a simulation step of self.timeStep milliseconds
      ### and leave the loop when the simulation is over
      if self.step(self.timeStep) == -1: break
      

    
      ### Enter here exit cleanup code

      
# The main program starts from here

# This is the main program of your controller.
# It creates an instance of your Robot subclass, launches its
# function(s) and destroys it at the end of the execution.
# Note that only one instance of Robot should be created in
# a controller program.
controller = fuzzyController()
#controller = fuzzyController(40,0, 25)
controller.init(60,0, controller.maxSpeed/3) # 60 is the distance of one square in webots checkered floor
controller.run()
