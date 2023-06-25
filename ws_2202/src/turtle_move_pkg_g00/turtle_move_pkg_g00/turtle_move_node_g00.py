#!/usr/bin/env python3

# libreria cliente de python
import rclpy
from rclpy.node import Node

#libreria de msj tipo twist
from geometry_msgs.msg import Twist

# libreria de msj turtlesim, mjs tipo pose
from turtlesim.msg import Pose

class TurtleMoveNodeG00(Node): # Redefine node class
    def __init__(self):
        super().__init__("turtle_move_node_g00") # Redefine node name

        # publisher obj (msg_type, topic_name, queue==buffer)
        self.cmd_pub = self.create_publisher(Twist,'/turtle1/cmd_vel',10)

        # create a topic subscriber
        # obj (msg_type,topic_name, callback_handler, buffer) 
        self.pose_subs = self.create_subscription(Pose,'/turtle1/pose',self.pose_callback,1)

        # create a timer function to send msg
        timer_period = 0.5 # in [s]
        self.timer = self.create_timer(timer_period,self.timer_callback)

    def timer_callback(self):
        # def twist variable
        cmd_vel = Twist()
        # msg
        cmd_vel.linear.x = 0.5  #float(sys.argv[1])#
        cmd_vel.angular.z = 1.0 #float(sys.argv[2])#

        # metodo
        self.cmd_pub.publish(cmd_vel)    
    
    def pose_callback(self,data):
        msg = 'X: {:.3f}, Y: {:.3f}, Theta: {:.3f}'.format(data.x,data.y,data.theta)
        self.get_logger().info(msg)



def main(args=None):
    #inicializador del nodo 
    rclpy.init(args=args)

    # objeto nodo 
    node = TurtleMoveNodeG00() # object definition (creation)

    rclpy.spin(node)
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()