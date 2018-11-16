#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SetModelState, GetModelState
from kobuki_msgs.msg import BumperEvent
from pyquaternion import Quaternion as Q
from ackermann_msgs.msg import AckermannDrive
import random
from math import sin, cos
from time import sleep
import tf
from graphs import SE2

class AckermannControl:
    def __init__(self):
        rospy.init_node("ackermann_vehicle_control_server")

        rospy.wait_for_service("/gazebo/get_model_state")
        self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        self.bumper_sub = rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.processBump)

        self.step_size = 10

    def processBump(self, data):
        rospy.signal_shutdown("Collision detected!")

    def set_state(self, position, vel):
        model_state_resp = self.get_model_state(model_name="ackermann_vehicle")
        model_state = SetModelState()
        model_state.model_name = "ackermann_vehicle"
        model_state.pose = model_state_resp.pose
        model_state.twist = Twist()
        model_state.reference_frame = "world"
        model_state.pose.position.x = position.X
        model_state.pose.position.y = position.Y
        quat = tf.transformations.quaternion_from_euler(0,0,position.theta)
        model_state.pose.orientation.x = quat[0]
        model_state.pose.orientation.y = quat[1]
        model_state.pose.orientation.z = quat[2]
        model_state.pose.orientation.w = quat[3]
        model_state.twist.linear.x = vel[0]
        model_state.twist.linear.y = vel[1]
        model_state.twist.angular.z = vel[2]
        self.set_model_state(model_state=model_state)

    def interpolate(self):
        model_state_resp = self.get_model_state(model_name="ackermann_vehicle")
        pos = model_state.pose.position
        quat = model_state.pose.orientation
        linAccMin = -10.0
        linAccMax = 10.0
        steerAccMin = -5.0
        steerAccMax = 5.0
        linAcc = random.uniform(linAccMin, linAccMax)
        steerAcc = random.uniform(steerAccMin, steerAccMax)
        time = random.random()
        inc = 0.05
        t0 = 0
        vx, vy, vs = vel[0], vel[1], vel[2]
        x, y, s  = pos.x, pos.y,
        while t0 < time:
            x += (vx*inc + linAcc*inc*inc) * cos(s) * cos(steerAcc)
            vx += linAcc * inc * cos(s) * cos(steerAcc)
            y += (vy*inc + linAcc*inc*inc) * sin(s) * cos(steerAcc)
            vy += linAcc * inc * sin(s) * cos(steerAcc)
            s += vs*inc + steerAcc*inc*inc * sin(steerAcc)
            vs += steerAcc * inc * sin(steerAcc)
            t0 += inc
            self.set_state(SE2(x,y,s), [vx,vy,vs])

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    mouseBot = AckermannControl()
    for i in range(1000):
        mouseBot.set_state(SE2(0,0,0), [0,0,0])
        position, vel = SE2.get_random_control(SE2(0,0,0),[0,0,0])
        print round(position.X, 3), round(position.Y, 3),round(position.theta, 3), round(vel[0], 3), round(vel[2], 3), round(vel[2], 3)
        sleep(1)
        #mouseBot.set_state(SE2(-1,1,0), [3,3,3])
        mouseBot.interpolate(SE2(0,0,0),[0,0,0])
        mouseBot.bumper_sub = rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, mouseBot.processBump)
        if mouseBot.bumper_sub:
            print "Test"
        sleep(1)
    #rocketPiano.run()
