# File:          leaderController.py
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
import numpy.linalg as alglin

import cv2


from math import atan2, sin, pi, ceil
from random import randint
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
class obstacleController (DifferentialWheels):
  
  Mode = Enumerate('STANDARD RANDOM STOP')
  timeStep = 64
  maxSpeed = 100.0  
  limDist = 80.0
  square_edge = 6e3
  dir_flag = 1
  counter = 0
  flag_counter = 0
  reference_dir = np.array(([1,0]))
  old_encoder_value = 0
  #TOL = 1e-3
  
  # User defined function for initializing and running
  # the fuzzyController class
  
  def init(self):
    ### Set points
    
    ### Assigning mode
    #self.mode = sys.argv[1]
    
    
    #self.mode = self.Mode.STANDARD
    #print self.mode
    #self.mode = self.Mode.STOP
    self.mode = self.Mode.RANDOM

    
  def run(self):
    speeds = {
      "left_wheel"  : 0.0,
      "right_wheel" : 0.0
    }
    time = 0.0
    k_right = 2
    k_left = 2   
    
    
    # Main loopF
    while True:
      

      if self.mode == self.Mode.STANDARD:
	speeds["left_wheel"] = 6*self.maxSpeed*round(sin(2*pi*time/(self.timeStep*10)))
	speeds["right_wheel"] = 6*self.maxSpeed*round(sin(2*pi*time/(self.timeStep*10)))
	  
      elif self.mode == self.Mode.RANDOM:
		    
	
	if time % 40 == 0:
	  move_pattern = randint(0,24) % 5
	  
	  if move_pattern == 0:
	    k_right = 6 
	    k_left = 4
	  elif move_pattern == 1:
	    k_right = 4
	    k_left = 6
	  elif move_pattern == 2:
	    k_right = 0
	    k_left = 2
	  elif move_pattern == 3:
	    k_right = 2
	    k_left = 0    
	  else:
	    k_right = 6
	    k_left = 6
	  
	speeds["left_wheel"] = k_left*self.maxSpeed
	speeds["right_wheel"] = k_right*self.maxSpeed
	
	
      elif self.mode == self.Mode.STOP:
	speeds["left_wheel"] = 0
	speeds["right_wheel"] = 0
	

	
      
      #### SEND CALCULATED CONTROL TO THE WHEEL
      self.setSpeed(speeds["left_wheel"], speeds["right_wheel"])
      
      
      ### Perform a simulation step of self.timeStep milliseconds
      ### and leave the loop when the simulation is over
      if self.step(self.timeStep) == -1: break
      time = time + 1.0
      

      


      
      ### Enter here exit cleanup code
      
# The main program starts from here

# This is the main program of your controller.
# It creates an instance of your Robot subclass, launches its
# function(s) and destroys it at the end of the execution.
# Note that only one instance of Robot should be created in
# a controller program.
controller = obstacleController()
controller.init() 
controller.run()
