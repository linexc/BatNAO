#!/usr/bin/env python
import rospy
import time
import motion
import numpy as np
import almath
import sys
from naoqi import ALProxy
from move_direction.srv inport *
import tf

motionProxy = 0

def callback(req):

    # Send NAO to Pose Init
    postureProxy.goToPosture("StandInit", 0.5)

    # Example showing the moveTo command
    # The units for this command are meters and radians
    x  = req.x
    y  = req.y
    theta  = req.angle
    motionProxy.moveTo(x, y, theta)
    # Will block until move Task is finished

    # If moveTo() method does nothing on the robot,
    # read the section about walk protection in the
    # Locomotion control overview page.
def myServerFunction():
    rospy.init_node('move_foward_server')
    rospy.Service("move_forward_server",move_direction, callback)
    rospy.spin()

if __name__ == '__main__':
    
    #robotIP=str(sys.argv[1])
    robotIP="127.0.0.1"
    #PORT=int(sys.argv[2])
    PORT= 9559
    try:
        myServerFunction()
    except rospy.ROSInternalException:
        pass


