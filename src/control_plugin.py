#!/usr/bin/env python3

import rospy
import math
import sys
# import tf2_ros
import time

from pdb import set_trace as bp
from statistics import mean
from std_msgs.msg import Header
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from control_msgs.msg import JointControllerState


#globals
TIRE_RADIUS = 0.055

# vehicle name
car_name = str(sys.argv[1])

# subscriber topics
gazebo_odom_topic = '/{}/ground_truth'.format(car_name)
LRW_state   = '/{}/left_rear_wheel_velocity_controller/state'.format(car_name)
RRW_state   = '/{}/right_rear_wheel_velocity_controller/state'.format(car_name)
LFW_state   = '/{}/left_front_wheel_velocity_controller/state'.format(car_name)
RFW_state   = '/{}/right_front_wheel_velocity_controller/state'.format(car_name)
command_topic     = '/{}/multiplexer/command'.format(car_name)

# state publisher topics
odom_pub_topic  = '/{}/base/odom'.format(car_name)
footprint_topic = '/{}/base/footprint'.format(car_name)

# control topics
LRW_topic   = '/{}/left_rear_wheel_velocity_controller/command'.format(car_name)
RRW_topic   = '/{}/right_rear_wheel_velocity_controller/command'.format(car_name)
LFW_topic   = '/{}/left_front_wheel_velocity_controller/command'.format(car_name)
RFW_topic   = '/{}/right_front_wheel_velocity_controller/command'.format(car_name)
LSH_topic   = '/{}/left_steering_hinge_position_controller/command'.format(car_name)
RSH_topic   = '/{}/right_steering_hinge_position_controller/command'.format(car_name)

# frame names
odom_frame = 'odom'
base_frame = '{}_base_link'.format(car_name)


# footprint parameters

global seq
global dt
global v
global omega_lfw, omega_lrw, omega_rfw, omega_rrw

omega_lrw = 0.0
omega_lfw = 0.0
omega_rrw = 0.0
omega_rfw = 0.0


v = Float64()

freq = 10.0

seq = 0
footprint = PolygonStamped()

side_A = Point32()
side_B = Point32()
side_C = Point32()
side_D = Point32()
side_E = Point32()

[side_A.x, side_A.y, side_A.z] = [-0.1, -0.2,  0.0]
[side_B.x, side_B.y, side_B.z] = [ 0.5, -0.2,  0.0]
[side_C.x, side_C.y, side_C.z] = [ 0.6,  0.0,  0.0]
[side_D.x, side_D.y, side_D.z] = [ 0.5,  0.2,  0.0]
[side_E.x, side_E.y, side_E.z] = [-0.1,  0.2,  0.0]

footprint.header.frame_id = base_frame
footprint.polygon.points  = [side_A, side_B, side_C, side_D, side_E]

# footprint visualizer

def footprint_visualizer():

    global seq

    footprint.header.seq = seq
    seq = seq + 1
    footprint.header.stamp = rospy.Time.now()
    footprint_pub.publish(footprint)

def LRW_callback(state):
    global omega_lrw
    omega_lrw = state.process_value

def LFW_callback(state):
    global omega_lfw
    omega_lfw = state.process_value

def RRW_callback(state):
    global omega_rrw
    omega_rrw = state.process_value

def RFW_callback(state):
    global omega_rfw
    omega_rfw = state.process_value

def publish_odom(event):
    global v
    v.data = mean([omega_lfw,omega_lrw,omega_rfw,omega_rrw])*TIRE_RADIUS
    odom_pub.publish(v)

def odom_callback(data):

    odom                      = Odometry()
    odom.header.frame_id      = odom_frame
    odom.child_frame_id       = base_frame
    odom.header.stamp         = rospy.Time.now()
    odom.pose                 = data.pose
    odom.twist = data.twist

    tf = TransformStamped(header         = Header(
                          frame_id       = odom.header.frame_id,
                          stamp          = odom.header.stamp),
                          child_frame_id = odom.child_frame_id,
                          transform      = Transform(
                          translation    = odom.pose.pose.position,
                          rotation       = odom.pose.pose.orientation))

    # visualize footprint everytime odom changes

    footprint_visualizer()

    odom_pub.publish(odom)
    # tf_pub.sendTransform(tf)

# command variables

global previous_speed

previous_speed   = 0.0
min_speed        = 0.0
max_speed        = 80.0  # 100.0
speed_delta      = 10.0  # 1.25
previous_speed   = 0.0

# command callback

def command_callback(data):

    global previous_speed

    steering_angle_msg = Float64()
    speed_msg          = Float64()

    steering_angle_msg.data = data.steering_angle
    speed_msg.data          = data.speed/TIRE_RADIUS     #Include logic to convert desired COM speed into angular velocity for the wheels

    pub_vel_LRW.publish(speed_msg)
    pub_vel_RRW.publish(speed_msg)
    pub_vel_LFW.publish(speed_msg)
    pub_vel_RFW.publish(speed_msg)

    pub_pos_LSH.publish(steering_angle_msg)
    pub_pos_RSH.publish(steering_angle_msg)

if __name__ == '__main__':

    try:

        rospy.init_node('control_plugin', anonymous = True)

        # rospy.Subscriber(gazebo_odom_topic, Odometry, odom_callback)
        rospy.Subscriber(LRW_state, JointControllerState, LRW_callback)
        rospy.Subscriber(RRW_state, JointControllerState, RRW_callback)
        rospy.Subscriber(RFW_state, JointControllerState, RFW_callback)
        rospy.Subscriber(LFW_state, JointControllerState, LFW_callback)
        rospy.Subscriber(command_topic, AckermannDrive, command_callback)

        # information publishers
        footprint_pub = rospy.Publisher(footprint_topic, PolygonStamped, queue_size = 1)
        # odom_pub      = rospy.Publisher(odom_pub_topic, Odometry, queue_size = 1)
        odom_pub      = rospy.Publisher(odom_pub_topic, Float64, queue_size = 1)
        # tf_pub        = tf2_ros.TransformBroadcaster()

        # control publishers
        pub_vel_LRW = rospy.Publisher(LRW_topic, Float64, queue_size = 1)
        pub_vel_RRW = rospy.Publisher(RRW_topic, Float64, queue_size = 1)
        pub_vel_LFW = rospy.Publisher(LFW_topic, Float64, queue_size = 1)
        pub_vel_RFW = rospy.Publisher(RFW_topic, Float64, queue_size = 1)
        pub_pos_LSH = rospy.Publisher(LSH_topic, Float64, queue_size = 1)
        pub_pos_RSH = rospy.Publisher(RSH_topic, Float64, queue_size = 1)

        # Initialize the ROS timer for the linear velocity publisher
        rospy.Timer(rospy.Duration(1.0/freq),publish_odom)

        rospy.spin()

    except rospy.ROSInterruptException:

        pass
