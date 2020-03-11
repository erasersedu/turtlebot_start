#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from dynamixel_msgs.msg import JointState as JointStateDynamixel
from sensor_msgs.msg import JointState
import tf

def printHelp():
    print "HEAD NODE. Options:"
    print "TODO: Print all argument options"

######################################
#### MOTOR POSITION  CALLBACK  #######
def callbackHeadPan(msg):
    global stateHeadPan
    stateHeadPan = msg.current_pos - msg.error

def callbackHeadTilt(msg):
    global stateHeadTilt
    stateHeadTilt = -1*(msg.current_pos - msg.error)

def main():
    print "INITIALIZING HEAD NODE..."
    ###Connection with ROS
    rospy.init_node("motors_states")
    jointStates = JointState()
    jointStates.name = ["head_pan_joint", "head_tilt_joint"]
    jointStates.position = [0 ,0]

    rospy.Subscriber('/head_pan_controller/state', JointStateDynamixel, callbackHeadPan)
    rospy.Subscriber('/head_tilt_controller/state', JointStateDynamixel, callbackHeadTilt)

    pubJointStates = rospy.Publisher("/joint_states", JointState, queue_size = 1)

    global stateHeadPan
    global stateHeadTilt
    stateHeadPan = 0
    stateHeadTilt = 0

    loop = rospy.Rate(30)
    while not rospy.is_shutdown():
        jointStates.header.stamp = rospy.Time.now()
        jointStates.position[0] = stateHeadPan
        jointStates.position[1] = stateHeadTilt
        pubJointStates.publish(jointStates)
        loop.sleep()

if __name__ == '__main__':
    try:
        if "--help" in sys.argv:
            printHelp()
        elif "-h" in sys.argv:
            printHelp()
        else:
            main()
    except rospy.ROSInterruptException:
        pass
