#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState

import math
import numpy as np
from numpy import cos, sin, pi

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

def rotz(ang):

    R = np.array([[cos(ang),-sin(ang),0],
                  [sin(ang),cos(ang),0],
                  [0,0,1]])

    return R

class Twist2OdomG99(Node):

    def __init__(self):
        super().__init__('twist_2_odom_g99')


        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_broadcaster2 = TransformBroadcaster(self)

        # callback function on each message
        self.subscription = self.create_subscription(Twist,'/cmd_vel',self.twist2odom_callback,10)

        self.subscription  # prevent unused variable warning

        # initial condition euler integration method
        self.dt = 0.1
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.theta = 0.0

        self.phi = 0.0

        # initialization of the transformation
        self.t = TransformStamped()
        self.t.header.frame_id = 'world'
        self.t.child_frame_id = 'base_link'

        self.get_logger().info("{0} started".format(self.nodeName))

    def twist2odom_callback(self, msg):

        # read the msg and extract the content
        x_dot = msg.linear.x
        y_dot = msg.linear.y 
        theta_dot = msg.angular.z  

        joint_state = JointState()

        # update joint_state
        now = self.get_clock().now()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = ['baselink_rwheellink']
        joint_state.position = [self.theta]

        # time stamp from ROS timer
        self.t.header.stamp = self.get_clock().now().to_msg()

        # Integrating angular velocity to angular position
        self.theta  = self.theta + theta_dot*self.dt

        # Transformation angular position Euler Angles to Quaternion
        q = quaternion_from_euler(0, 0, self.theta)
        self.t.transform.rotation.x = q[0]
        self.t.transform.rotation.y = q[1]
        self.t.transform.rotation.z = q[2]
        self.t.transform.rotation.w = q[3]

        # Integrating linear velocity to linear position
        self.x = self.x +  x_dot*self.dt      
        self.y = self.y +  y_dot*self.dt # this always is zero, but only for this example, z also is zero for this example.

        # vector form 
        pos = np.array([[self.x],[self.y],[self.z]])

        # rotating the position vector to correspond with the orientation
        new_pos = rotz(self.theta)@pos

        self.t.transform.translation.x = new_pos[0,0]
        self.t.transform.translation.y = new_pos[1,0]
        self.t.transform.translation.z = new_pos[2,0]

        # Send the transformation
        self.tf_broadcaster.sendTransform(self.t)

        self.joint_pub.publish(joint_state)


    
def main(args=None):
    rclpy.init(args=args)
    node = Twist2OdomG99()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()