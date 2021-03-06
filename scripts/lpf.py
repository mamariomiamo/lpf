#!/usr/bin/env python
import rospy
from enum import Enum
from std_msgs.msg import Int64, Header, Byte
from std_srvs.srv import SetBool
import math
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, \
                            WaypointList, PositionTarget, AttitudeTarget, Thrust
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, WaypointClear, \
                            WaypointPush
from pymavlink import mavutil
from sensor_msgs.msg import NavSatFix, Imu
from six.moves import xrange
from threading import Thread
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np


class uavTaskType(Enum):
    Idle = 0
    TakeOff = 1
    Mission = 2
    Land = 3

class TaskManager:
    def __init__(self):
        self.altitude = Altitude()
        self.extened_state = ExtendedState()
        self.global_position = NavSatFix()
        self.imu_data = Imu()
        self.home_position = HomePosition()
        self.local_position = PoseStamped()
        self.attitude_sp = PoseStamped()
        self.pen_pose = PoseStamped()
        self.pen_pose_filtered = PoseStamped()
        self.quad_vel = TwistStamped()
        self.quad_vel_filtered = TwistStamped()
        self.pen_vel_filtered = TwistStamped()
        self.quad_pose = PoseStamped()
        self.relative_speed = TwistStamped()
        self.pen_vel = TwistStamped()
        self.state = State()
        self.local_velocity = TwistStamped()  # local_velocity initialize
        self.attitude_rate = AttitudeTarget()  # use for attitude setpoints pub
        self.thrust = Thrust()

        self.pos = PoseStamped()
        self.position = PositionTarget()  # thrust control commands

        self.task_state = uavTaskType.Idle
        self.euler = Vector3()  # Euler angles
        self.pos_sp = Vector3() #position setpoint

        # ROS publisher
        self.pos_control_pub = rospy.Publisher(
            'mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.position_pub = rospy.Publisher(
            'mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.attitude_sp_pub = rospy.Publisher(
            'mavros/setpoint_attitude/attitude', PoseStamped, queue_size=1)
        self.attitude_rate_sp_pub = rospy.Publisher(
            'mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
        self.attitude_thrust_pub = rospy.Publisher(
            'mavros/setpoint_attitude/thrust', Thrust, queue_size = 1)
        self.relative_speed_pub = rospy.Publisher('relative_speed', TwistStamped, queue_size = 1)
        self.quad_vel_filtered_pub = rospy.Publisher(
            'quadrotor_vel_filtered', TwistStamped, queue_size=1)
        self.pen_vel_filtered_pub = rospy.Publisher(
            'pendulum_vel_filtered', TwistStamped, queue_size=1)
        self.pen_pos_filtered_pub = rospy.Publisher(
            'pendulum_pos_filtered', PoseStamped, queue_size=1)
        # ROS subscribers
        self.quad_pos_sub = rospy.Subscriber('mavros/vision_pose/pose',PoseStamped, self.quad_pos_callback)
        self.vel_sub = rospy.Subscriber('mavros/local_position/velocity_local',
                                        TwistStamped, self.local_velocity_callback)  # local_velocity susbcriber
        self.quad_vel_sub = rospy.Subscriber('quadrotor_vel', TwistStamped, self.quad_vel_callback)
        self.pen_vel_sub = rospy.Subscriber('pendulum_vel',TwistStamped, self.pen_vel_callback)
        self.pen_pos_sub = rospy.Subscriber('vrpn_client_node/pen/pose', PoseStamped, self.pen_pose_callback)
        #self.vel_global_sub = rospy.Subscriber('mavros/local_position/velocity_local', TwistStamped, self.global_velocity_callback)
        # send setpoints in seperate thread to better prevent failsafe

    def local_velocity_callback(self, data):  # local_velocity callback
        self.local_velocity = data

    def quad_pos_callback(self, data):
        self.quad_pos = data

    def quad_vel_callback(self, data):
        self.quad_vel = data

    def pen_vel_callback(self, data):
        self.pen_vel = data

    def pen_pose_callback(self, data):
        self.pen_pose = data

if __name__ == '__main__':
    rospy.init_node('vicon_filter')
    print("hahaha")
    uavTask = TaskManager()

    uavTask.pos.pose.position.x = 0
    uavTask.pos.pose.position.y = 0
    uavTask.pos.pose.position.z = 0
    dX0 = 0
    dY0 = 0
    dx0 = 0
    dy0 = 0

    #uavTask.set_mode("OFFBOARD", 5)
    #uavTask.set_arm(True, 5)
    #xr_hat = np.array([[0,0]]).T
    #xs_hat = np.array([[0,0]]).T
    dt = 0.01

    #gamma = 0
    #beta = 0
    quad_vx_filter = 0
    quad_vy_filter = 0
    quad_vz_filter = 0
    pen_vx_filter = 0
    pen_vy_filter = 0
    pen_vz_filter = 0
    pen_px_filter = 0
    pen_py_filter = 0
    cutoff_freq = 10
    report_flag = 1

    while not rospy.is_shutdown():
        rate = rospy.Rate(30) #filter publishing frequency
        if 1:
            if report_flag:
               print("filtering")
               report_flag = 0
            #rospy.loginfo("Doing LQR takeoff")
            uavTask.pos_sp = [0, 0, 0.5]
            # Get position feedback from PX4
            x = uavTask.local_position.pose.position.x
            #x = uavTask.quad_pose.pose.position.x
            #print("current p_x is {0}".format(x))
            y = uavTask.local_position.pose.position.y
            #y = uavTask.quad_pose.pose.position.y
            #print("curent p_y is {0}".format(y))
            z = uavTask.local_position.pose.position.z  # ENU used in ROS
            #z = uavTask.quad_pose.pose.position.z
            quad_vx = uavTask.quad_vel.twist.linear.x
            quad_vy = uavTask.quad_vel.twist.linear.y
            quad_vz = uavTask.quad_vel.twist.linear.z

            pen_vx = uavTask.pen_vel.twist.linear.x
            pen_vy = uavTask.pen_vel.twist.linear.y
            pen_vz = uavTask.pen_vel.twist.linear.z

            pen_px = uavTask.pen_pose.pose.position.x
            pen_py = uavTask.pen_pose.pose.position.y

            filter_a = dt/(dt + 1/(2*3.14*cutoff_freq))

            quad_vx_filter = (1-filter_a)*quad_vx_filter + filter_a * quad_vx
            quad_vy_filter = (1-filter_a)*quad_vy_filter + filter_a * quad_vy
            quad_vz_filter = (1-filter_a)*quad_vz_filter + filter_a * quad_vz

            pen_vx_filter = (1-filter_a)*pen_vx_filter + filter_a * pen_vx
            pen_vy_filter = (1-filter_a)*pen_vy_filter + filter_a * pen_vy
            pen_vz_filter = (1-filter_a)*pen_vz_filter + filter_a * pen_vz

            pen_px_filter = (1-filter_a)*pen_px_filter + filter_a * pen_px
            pen_py_filter = (1-filter_a)*pen_py_filter + filter_a * pen_py

            uavTask.quad_vel_filtered.twist.linear.x = quad_vx_filter
            uavTask.quad_vel_filtered.twist.linear.y = quad_vy_filter
            uavTask.quad_vel_filtered.twist.linear.z = quad_vz_filter
            uavTask.quad_vel_filtered.header.stamp = rospy.Time.now()

            uavTask.pen_vel_filtered.twist.linear.x = pen_vx_filter
            uavTask.pen_vel_filtered.twist.linear.y = pen_vy_filter
            uavTask.pen_vel_filtered.twist.linear.z = pen_vz_filter
            uavTask.pen_vel_filtered.header.stamp = rospy.Time.now()

            uavTask.pen_pose_filtered.pose.position.x = pen_px_filter
            uavTask.pen_pose_filtered.pose.position.y = pen_py_filter
            uavTask.pen_pose_filtered.header.stamp = rospy.Time.now()

            uavTask.pen_vel_filtered_pub.publish(uavTask.pen_vel_filtered)
            uavTask.quad_vel_filtered_pub.publish(uavTask.quad_vel_filtered)
            uavTask.pen_pos_filtered_pub.publish(uavTask.pen_pose_filtered)

        rate.sleep()
    rospy.spin()
