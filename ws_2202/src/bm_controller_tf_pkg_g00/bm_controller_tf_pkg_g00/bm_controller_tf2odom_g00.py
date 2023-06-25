import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist

import math
import numpy as np
from numpy import cos, sin


# This function is a stripped down version of the code in
# https://github.com/matthew-brett/transforms3d/blob/f185e866ecccb66c545559bc9f2e19cb5025e0ab/transforms3d/euler.py
# Besides simplifying it, this version also inverts the order to return x,y,z,w, which is
# the way that ROS prefers it.
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


class FramePublisher(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_publisher')

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # callback function on each message
        self.subscription = self.create_subscription(Twist,'/cmd_vel',self.twist2odom_callback,10)

        self.subscription  # prevent unused variable warning

        self.dt = 0.1
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.theta = 0.0

        self.t = TransformStamped()
        self.t.header.frame_id = 'world'
        self.t.child_frame_id = 'odom'
  

    def twist2odom_callback(self, msg):

        x_dot = msg.linear.x
        y_dot = msg.linear.y 
        theta_dot = msg.angular.z  

        # Read message content and assign it to
        # corresponding tf variables
        self.t.header.stamp = self.get_clock().now().to_msg()
       

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        self.theta  = self.theta + theta_dot*self.dt

        q = quaternion_from_euler(0, 0, self.theta)
        self.t.transform.rotation.x = q[0]
        self.t.transform.rotation.y = q[1]
        self.t.transform.rotation.z = q[2]
        self.t.transform.rotation.w = q[3]


        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        self.x = self.x +  x_dot*self.dt      
        self.y = self.y +  y_dot*self.dt

        pos = np.array([[self.x],[self.y],[self.z]])

        new_pos = rotz(self.theta)@pos
        #print(new_pos)

        self.t.transform.translation.x = new_pos[0,0]
        self.t.transform.translation.y = new_pos[1,0]
        self.t.transform.translation.z = new_pos[2,0]

        # Send the transformation
        self.tf_broadcaster.sendTransform(self.t)

      

def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
