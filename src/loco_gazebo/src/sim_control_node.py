#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from loco_pilot.msg import Command

class SimControlNode(Node):
    def __init__(self):
        super().__init__('sim_control_node')

        # --- CONFIGURATION ---
        self.max_force = 23.15 # Newtons per thruster
        
        # Internal State
        self.throttle_cmd = 0.0
        self.yaw_cmd = 0.0
        self.pitch_cmd = 0.0

        # --- PUBLISHERS ---
        # We publish to the remapped topics we set in the URDF
        self.pub_left = self.create_publisher(Wrench, "/left_thrust", 1)
        self.pub_right = self.create_publisher(Wrench, "/right_thrust", 1)
        self.pub_vert = self.create_publisher(Wrench, "/vertical_thrust", 1)
        
        # --- SUBSCRIBERS ---
        # Listen for keyboard commands
        self.create_subscription(Command, "/loco/command", self.command_callback, 10)

        # --- TIMER (The Heartbeat) ---
        # Run this loop at 20Hz (every 0.05 seconds)
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info("LoCO Physics Node (Timer Based) Started")

    def command_callback(self, msg):
        # Just store the latest command
        self.throttle_cmd = msg.throttle
        self.yaw_cmd = msg.yaw
        self.pitch_cmd = msg.pitch

    def control_loop(self):
        # 1. Calculate Forces based on stored commands
        
        # Vertical is simple direct mapping
        f_vert = self.max_force * self.pitch_cmd

        # Differential Thrust Logic (Tank Drive style mixing)
        # Left = Throttle + Yaw
        # Right = Throttle - Yaw
        f_left = self.max_force * (self.throttle_cmd - self.yaw_cmd)
        f_right = self.max_force * (self.throttle_cmd + self.yaw_cmd)

        # 2. Create Messages
        msg_left = Wrench()
        msg_left.force.x = float(f_left)

        msg_right = Wrench()
        msg_right.force.x = float(f_right)

        msg_vert = Wrench()
        msg_vert.force.z = float(f_vert) # Vertical pushes up/down (Z axis)

        # 3. Publish!
        self.pub_left.publish(msg_left)
        self.pub_right.publish(msg_right)
        self.pub_vert.publish(msg_vert)

        # Debug logging (Optional: Un-comment to see values in terminal)
        # self.get_logger().info(f"L: {f_left:.2f} | R: {f_right:.2f} | V: {f_vert:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = SimControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
