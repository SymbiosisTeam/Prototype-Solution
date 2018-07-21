# TEAM SYMBIOSIS
# SIT302 - Project Delivery
# Trimester 2, 2018
# CrazyFlie 2.0 Drone - Flight-Control Solution Prototype Upgrade (v3)

# NOTE: This version takes command line inputs for X, Y, Z co-ordinate values
#       Designed to be used in conjunction with Unity / C#

"""
Bitcraze CrazyFlie 2.0 Drone
 - Automatic Flight-Control Prototype Solution

This program connects to the Crazyflie at the `URI` and runs a flight 
sequence to simulate how the drone will operate in the Deakin Motion Capture Lab.

This program does not utilise an external location system: it has been
tested with (and designed for) the Flowdeck in line with our Sprint 2 goals.

Written by:
Paul Hammond (for Symbiosis Team)
216171484
"""

import logging
import time
import math
import sys
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI = 'radio://0/80/250K'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

"""
ATTENTION: 	
USER TO ENTER VALUES FOR DESIRED WAYPOINT CO-ORDINATE (X, Y, Z)
NOTE:
X, Y, Z follows right-hand rule for 3-D positive axes
"""
# GET USER INPUT FOR (X, Y, Z) CO-ORDINATES VIA COMMAND LINE INPUTS
# eg:    python3 proto_v3.py 2 1 1
X = float(sys.argv[1])
Y = float(sys.argv[2])
Z = float(sys.argv[3])


'''
Methods
'''
# Method: Calculate Hypotenuse (using Pythagoras Theorem) and return value
def Hypotenuse(opposite, adjacent):

    return math.sqrt((opposite * opposite + adjacent * adjacent))


# Method: Fly straight line to designated point (x, y, z), returning final height value
def Traverse(vx, heightDrone, travelTime, destinationHeight):
    # Increase / Decrease height over distance
    climbRate = ((destinationHeight - heightDrone) / travelTime) / 10
    iterations = travelTime * 10

    # Traverse Flight Path
    print("\nTraversing to height of %f" %(destinationHeight))
    for i in range(iterations):
        heightDrone += climbRate
        cf.commander.send_hover_setpoint(vx, VELOCITY_SIDE, 0, heightDrone)
        time.sleep(0.1)

    return heightDrone


# Method: Hover Drone
def Hover(hoverHeight, hoverTime):
    # Convert time to iterations
    iterations = hoverTime * 10    

    # Hover Drone
    print("\nHovering at %f metres" %(hoverHeight))
    for i in range(iterations):
        cf.commander.send_hover_setpoint(0, 0, 0, hoverHeight)
        time.sleep(0.1)


# Method: Rotates the drone whilst in hover
def Rotate(rotationAngle, hoverHeight):
    YAWTIME = 4 	# seconds

    # Calculate yawRate and iterations
    # negative yaw is anti-clock-wise
    yawRate = -1 * (rotationAngle / YAWTIME)    # degrees / second           
    iterations = YAWTIME * 10

    # Rotate Drone
    print("\nRotating to %f degrees" %(rotationAngle))
    for i in range(iterations):
        cf.commander.send_hover_setpoint(0, 0, yawRate, hoverHeight)
        time.sleep(0.1)


'''	   
GENERAL ATTRIBUTES
'''
# Constants
HOVER_HEIGHT_STD = 0.5        # metres
HOVER_TIME_STD = 2            # seconds
DESTINATION_HOVER_TIME = 15   # seconds (to be set as desired)
INIT_VELOCITY_FWD = 0.5       # metres / sec
VELOCITY_SIDE = 0             # do not modify

# Variables
heightDrone = 0
yawRate = 0
climbRate = 0

# Equations
angle = math.degrees(math.atan(Y / X))                      # Orientation angle based on X, Y co-ords
distanceLevel = Hypotenuse(X, Y)                            # Ground level trajectory based on X, Y co-ords
distance = Hypotenuse(distanceLevel, Z - HOVER_HEIGHT_STD)  # Flight trajectory distance based on X, Y, Z
travelTime = (round)(distance / INIT_VELOCITY_FWD)          # Get time as an integer value
velocityForward = distance / travelTime                     # Recalculate drone velocity to accomodate for time as an integer



'''			
# Flight-Control Program
'''
if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)

        # TAKE-OFF
        heightDrone = HOVER_HEIGHT_STD
        Hover(heightDrone, HOVER_TIME_STD)
  
        # ROTATE
        # Rotate to correct orientation (if Y co-ord is entered)
        if (angle != 0):
            Rotate(angle, heightDrone)
            Hover(heightDrone, HOVER_TIME_STD)
        
        # TRAVERSE FLIGHT PATH TO DESTINATION WAYPOINT
        heightDrone = Traverse(velocityForward, heightDrone, travelTime, Z)

        # HOVER AT DESTINATION WAYPOINT
        # To demonstrate how drone will interact with actor during motion capture  
        Hover(heightDrone, DESTINATION_HOVER_TIME)

        # ROTATE
        Rotate(180, heightDrone)                # rotate 180 degrees
        Hover(heightDrone, HOVER_TIME_STD)      # stabilise hover
        
        # TRAVERSE FROM WAYPOINT TO ORIGIN
        heightDrone = Traverse(velocityForward, heightDrone, travelTime, HOVER_HEIGHT_STD)
        Hover(heightDrone, HOVER_TIME_STD)      # stabilise hover
        
        # ROTATE
        rotationAngle = 180 - angle             
        Rotate(rotationAngle, heightDrone)      # rotate back to original heading
        Hover(heightDrone, HOVER_TIME_STD)      # stabilise hover
                   
        # LAND and SHUTDOWN
        Hover(0.1, 1)                           # Land
        
        # Engine shut-off
        cf.commander.send_stop_setpoint()
