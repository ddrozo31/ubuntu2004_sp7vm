#!/usr/bin/env python3

# libreria cliente de python
import rclpy
from rclpy.node import Node

class MyCustomNode(Node): # Redefine node class
    def __init__(self):
        super().__init__("node_name") # Redefine node name

def main(args=None):
    #inicializador del nodo 
    rclpy.init(args=args)

    # objeto nodo 
    node = MyCustomNode() # object definition (creation)

    rclpy.spin(node)
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()