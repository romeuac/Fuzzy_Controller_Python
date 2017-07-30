import os, sys
import fuzzy.storage.fcl.Reader as reader

dir = os.path.dirname(__file__)
filename = os.path.join(dir, '../rules/fuzzy_rules.fcl')
system = reader.Reader().load_from_file(filename)

# preallocate input and output values
controller_input = {
        "Depth_Error" 		     : 0.0,
        "Horizontal_Distance_Error" : 0.0
        }
controller_output = {
        "Left_Wheel_Speed"  : 0.0,
        "Right_Wheel_Speed" : 0.0
        }
 
def calculateControl(depth_error, horizontal_distance_error):
  # set input values
  controller_input["Depth_Error"] = depth_error
  controller_input["Horizontal_Distance_Error"] = horizontal_distance_error
  
  # calculate
  system.calculate(controller_input, controller_output)
  
  # now use outputs
  left_wheel_speed = controller_output["Left_Wheel_Speed"]
  right_wheel_speed = controller_output["Right_Wheel_Speed"]
  
  return left_wheel_speed, right_wheel_speed