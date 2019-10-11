#!/usr/bin/env python
import sys
import rospy
from sensor_msgs.msg import JointState

#print("Insert input according to this sequence:")
#print1("joint1 joint2 joint3 joint4 joint5 joint6")
#name = [ "joint1", "joint2", "joint3", "joint4", "joint5","joint6" ]
#print("[ joint1 joint2 joint3 joint4 joint5 joint6 ]")

#get input from user
#position_input = raw_input("Enter joint values in radian")
#position_msg = map( float, position_input.split() )
#velocity_input = raw_input( "Enter joint velocity radian/sec" )
#velocoty_msg = map( float, velocity_input.split() )
#torque_input = input( "Enter the torque in Nm" )


#I am deciding to use zero torque for now because my Jacobian is not ready
#torque_input = 6 * [0.0]

#to create a JointState message 
#msg = JointState()

# OOP state_publisher (incomplete)
#class state_publisher:
#    def __init__(self):
#        '''Initialize ros publisher'''
#        self.state_publisher = rospy.Publisher('joint_states', JointState, queue_size=1)

#simple state_publisher

def state_publisher():
    pub = rospy.Publisher('joint_states', JointState, queue_size=1)
    rospy.init_node('state_publisher')
    #msg = JointState()
#    rate = rospy.Rate(10) #10Hz
    while not rospy.is_shutdown():
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        
#        position_input = raw_input("Enter joint values in radian")
#       
#       Add a subroutine here to listen to a path planner
#       Add a subroutine here to solve the output of the path planner (Euclidean 3D) to 
#       configuration space
        msg.position =  [ 1.5, 0, 0, 0, 0, 0 ]
        
#        velocity_input = raw_input( "Enter joint velocity radian/sec" )
#        msg.velocity = map( float, velocity_input.split() )
        msg.velocity = [ 0, 0, 0, 0, 0, 0 ]       
        #torque_input = input( "Enter the torque in Nm" )
        msg.effort = 6 * [0.0]
        pub.publish(msg)

if __name__ == '__main__':
    try:
        state_publisher()
    except rospy.ROSInterruptException:
        pass
