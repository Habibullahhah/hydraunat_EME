#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from loco_pilot.msg import Command

class SimControlNode(Node):
    def __init__(self):
        super().__init__('sim_control_node')

        self.max_force = 15.0 # Balanced power
        
        self.cmd_surge = 0.0
        self.cmd_yaw   = 0.0
        self.cmd_heave = 0.0

        # Topics match the URDF naming
        self.pub_fl = self.create_publisher(Wrench, "/loco/thruster_fl_cmd", 1)
        self.pub_fr = self.create_publisher(Wrench, "/loco/thruster_fr_cmd", 1)
        self.pub_bl = self.create_publisher(Wrench, "/loco/thruster_bl_cmd", 1)
        self.pub_br = self.create_publisher(Wrench, "/loco/thruster_br_cmd", 1)
        self.pub_cl = self.create_publisher(Wrench, "/loco/thruster_cl_cmd", 1)
        self.pub_cr = self.create_publisher(Wrench, "/loco/thruster_cr_cmd", 1)
        
        self.create_subscription(Command, "/loco/command", self.command_callback, 10)
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("LoCO Vectored Control Started")

    def command_callback(self, msg):
        self.cmd_surge = msg.throttle
        self.cmd_yaw   = msg.yaw
        self.cmd_heave = msg.pitch 

    def control_loop(self):
        # --- MIXING MATRIX (Diagonal Opposing Logic) ---
        
        # 1. Vertical
        f_cl = self.cmd_heave * self.max_force
        f_cr = self.cmd_heave * self.max_force

        # 2. Horizontal
        # To Turn (Yaw):
        # We need pairs (FR, BL) to fight (FL, BR)
        
        # Front Left:  Forward(+) + Yaw(-) -> Pushes back to turn left
        f_fl = (self.cmd_surge - self.cmd_yaw) * self.max_force
        
        # Front Right: Forward(+) + Yaw(+) -> Pushes forward to turn left
        f_fr = (self.cmd_surge + self.cmd_yaw) * self.max_force
        
        # Back Left:   Forward(+) + Yaw(+) -> Pushes forward to turn left
        f_bl = (self.cmd_surge + self.cmd_yaw) * self.max_force
        
        # Back Right:  Forward(+) + Yaw(-) -> Pushes back to turn left
        f_br = (self.cmd_surge - self.cmd_yaw) * self.max_force

        # Publish
        self.publish_force(self.pub_fl, f_fl)
        self.publish_force(self.pub_fr, f_fr)
        self.publish_force(self.pub_bl, f_bl)
        self.publish_force(self.pub_br, f_br)
        self.publish_force(self.pub_cl, f_cl, axis='z')
        self.publish_force(self.pub_cr, f_cr, axis='z')

    def publish_force(self, publisher, value, axis='x'):
        msg = Wrench()
        if axis == 'x':
            msg.force.x = float(value)
        elif axis == 'z':
            msg.force.z = float(value)
        publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
