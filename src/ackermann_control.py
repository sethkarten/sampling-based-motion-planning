#!/usr/bin/env python
import rospy, tf
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SetModelState, GetModelState
from ackermann_msgs.msg import AckermannDrive
from time import sleep
from graphs import SE2
import sys
#sys.path.insert(0, '../ackermann/ackermann_vehicle_gazebo/nodes')

class AckermannControl:
    def __init__(self):

        self.ctrl = rospy.Publisher('ackermann_cmd', AckermannDrive, queue_size=10)

        rospy.init_node("ackermann_vehicle_control_server")

        rospy.wait_for_service("/gazebo/get_model_state")
        self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    def set_state(self, x, y):
        model_state_resp = self.get_model_state(model_name="ackermann_vehicle")
        model_state = SetModelState()
        model_state.model_name = "ackermann_vehicle"
        model_state.pose = model_state_resp.pose
        model_state.twist = Twist()
        model_state.reference_frame = "world"
        model_state.pose.position.x = x
        model_state.pose.position.y = y
        quat = tf.transformations.quaternion_from_euler(0,0,0)
        model_state.pose.orientation.x = quat[0]
        model_state.pose.orientation.y = quat[1]
        model_state.pose.orientation.z = quat[2]
        model_state.pose.orientation.w = quat[3]
        model_state.twist.linear.x = 0
        model_state.twist.linear.y = 0
        model_state.twist.angular.z = 0
        self.set_model_state(model_state=model_state)

    def control(self, lin_v, ang_v):
        msg = AckermannDrive()
        msg.speed = lin_v
        msg.steering_angle_velocity = ang_v
        self.ctrl.publish(msg)

if __name__ == "__main__":
    mouseBot = AckermannControl()
    mouseBot.control(50, 50)
    sleep(2)
    mouseBot.set_state(3, -5)
    for i in range(1000):
        vel = SE2.get_random_control()
        print vel[0], vel[1]
        mouseBot.control(vel[0], vel[1])
        sleep(2)
        print mouseBot.get_model_state(model_name="ackermann_vehicle")
        sleep(2)
