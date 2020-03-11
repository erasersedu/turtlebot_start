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

#ID 1
def callbackArmWaist(msg):
    global stateArmWaist
    stateArmWaist = msg.current_pos - msg.error

#ID 2
def callbackArmShoulder(msg):
    global stateArmShoulder
    stateArmShoulder = -1*(msg.current_pos - msg.error)

#ID 3
def callbackArmElbow(msg):
    global stateArmElbow
    stateArmElbow = msg.current_pos - msg.error

#ID 4
def callbackArmWrist(msg):
    global stateArmWrist
    stateArmWrist = msg.current_pos - msg.error

#ID 5
def callbackArmHand(msg):
    global stateArmHand
    stateArmHand = msg.current_pos - msg.error

#ID 6
def callbackHeadPan(msg):
    global stateHeadPan
    stateHeadPan = msg.current_pos - msg.error

#ID 7
def callbackHeadTilt(msg):
    global stateHeadTilt
    stateHeadTilt = -1*(msg.current_pos - msg.error)

def main():
    print "INITIALIZING MOTORS TF NODE..."

    global stateArmWaist #Motor ID 1
    global stateArmShoulder #Motor ID 2
    global stateArmElbow #Motor ID 3
    global stateArmWrist #Motor ID 4
    global stateArmHand #Motor ID 5
    global stateHeadPan #Motor ID 6
    global stateHeadTilt #Motor ID 7

    rospy.init_node("motors_states")
    jointStates = JointState()
    jointStates.name = ["arm_waist_joint", "arm_shoulder_joint", "head_pan_joint", "head_tilt_joint"]
    jointStates.position = [0 ,0, 0, 0]

    ###Connection with ROS
    rospy.Subscriber('/waist_controller/state', JointStateDynamixel, callbackArmWaist)
    rospy.Subscriber('/shoulder_controller/state', JointStateDynamixel, callbackArmShoulder)

    rospy.Subscriber('/head_pan_controller/state', JointStateDynamixel, callbackHeadPan)
    rospy.Subscriber('/head_tilt_controller/state', JointStateDynamixel, callbackHeadTilt)

    pubJointStates = rospy.Publisher("/joint_states", JointState, queue_size = 1)

    stateArmWaist = 0
    stateArmShoulder = 0
    stateArmElbow = 0
    stateArmWrist = 0
    stateArmHand = 0
    stateHeadPan = 0
    stateHeadTilt = 0

    loop = rospy.Rate(30)
    while not rospy.is_shutdown():
        jointStates.header.stamp = rospy.Time.now()

        jointStates.position[0] = stateArmWaist
        jointStates.position[1] = stateArmShoulder
        jointStates.position[2] = stateHeadPan
        jointStates.position[3] = stateHeadTilt

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
