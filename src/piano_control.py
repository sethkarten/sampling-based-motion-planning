#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SetModelState, GetModelState
from pyquaternion import Quaternion as Q
from random import uniform
from graphs import SE3
from math import sqrt, fabs
from time import sleep

class PianoControl:
    def __init__(self):
        rospy.init_node("piano_control_server")

        rospy.wait_for_service("/gazebo/get_model_state")
        self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        self.step_size = 10

    def set_position(self, position):
        model_state_resp = self.get_model_state(model_name="piano2")
        model_state = SetModelState()
        model_state.model_name = "piano2"
        model_state.pose = model_state_resp.pose
        model_state.twist = Twist()
        model_state.reference_frame = "world"
        model_state.pose.position.x = position.X
        model_state.pose.position.y = position.Y
        model_state.pose.position.z = position.Z
        self.set_model_state(model_state=model_state)

    def set_steering_angle(self, quat):
        model_state_resp = self.get_model_state(model_name="piano2")
        model_state = SetModelState()
        model_state.model_name = "piano2"
        model_state.pose = model_state_resp.pose
        model_state.twist = Twist()
        model_state.reference_frame = "world"
        model_state.pose.orientation.x = quat[0]
        model_state.pose.orientation.y = quat[1]
        model_state.pose.orientation.z = quat[2]
        model_state.pose.orientation.w = quat[3]
        self.set_model_state(model_state=model_state)

    def interpolate(self, b):
        current_model_state = self.get_model_state(model_name="piano2")
        quat = Q(current_model_state.pose.orientation.x, current_model_state.pose.orientation.y,\
         current_model_state.pose.orientation.z, current_model_state.pose.orientation.w)
        a = SE3(current_model_state.pose.position.x, current_model_state.pose.position.y,\
         current_model_state.pose.position.z, quat)
        inc = 0.1
        steering_inc = 0.03
        x_inc = b.X - a.X
        y_inc = b.Y - a.Y
        z_inc = b.Z - a.Z
        mag = sqrt(x_inc*x_inc + y_inc*y_inc + z_inc*z_inc)
        x_inc = x_inc / mag * inc
        y_inc = y_inc / mag * inc
        z_inc = z_inc / mag * inc
        x = a.X
        y = a.Y
        z = a.Z
        q = a.q
        cur = a
        while SE3.euclid_dist(cur, b) > inc\
        or fabs(Q.sym_distance(q, b.q)) > steering_inc:
            self.set_position(cur)
            self.set_steering_angle(cur.q)
            if SE3.euclid_dist(cur, b) > inc:
                x += x_inc
                y += y_inc
                z += z_inc
            if fabs(Q.sym_distance(q, b.q)) > steering_inc:
                q = Q.slerp(q, b.q, amount=steering_inc)
            cur = SE3(x, y, z, q)
            sleep(0.05)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    rocketPiano = PianoControl()
    for i in range(1000):
        sleep(1)
        position = SE3.get_random_state()
        rocketPiano.interpolate(position)
        #rocketPiano.set_position(position)
        #rocketPiano.set_steering_angle(pose)
    #rocketPiano.run()
