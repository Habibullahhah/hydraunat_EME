#!/usr/bin/env python3

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from loco_pilot.msg import Command

msg = """
Reading from the keyboard
---------------------------
Planar Movement:
   q    w    e
   a         d
   z    x    c

i : up (+z)
k : down (-z)

Anything Else : stop

r/v : increase/decrease thruster power by 10% of full
---------------------------
CTRL-C to quit
"""

moveBindings = {
    'q': (0, -(2**0.5)/2, (2**0.5)/2, 0),
    'w': (0, 0, 1, 0),
    'e': (0, (2**0.5)/2, (2**0.5)/2, 0),
    'a': (0, -1, 0, 0),
    'd': (0, 1, 0, 0),
    'z': (0, -(2**0.5)/2, -(2**0.5)/2, 0),
    'x': (0, 0, -1, 0),
    'c': (0, (2**0.5)/2, -(2**0.5)/2, 0),
    'i': (1, 0, 0, 0),
    'k': (-1, 0, 0, 0),
}

speedBindings = {
    'r': (0.1, 0),
    'v': (-0.1, 0),
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('teleop_keyboard')
    pub = node.create_publisher(Command, '/loco/command', 1)

    node.declare_parameter('speed', 0.5)
    speed = node.get_parameter('speed').get_parameter_value().double_value

    pitch = 0.0
    yaw = 0.0
    throttle = 0.0
    status = 0

    settings = termios.tcgetattr(sys.stdin)

    try:
        print(msg)
        print(f"Thruster input power is currently at: {speed*100} %")

        while True:
            key = getKey(settings)

            if key in moveBindings.keys():
                pitch = moveBindings[key][0]
                yaw = moveBindings[key][1]
                throttle = moveBindings[key][2]
            elif key in speedBindings.keys():
                status = status + 1
                if (status == 14):
                    print(msg)
                    status = 0

                if speed + speedBindings[key][0] > 1:
                    print("Thruster power at maximum.")
                elif speed + speedBindings[key][0] < 0:
                    print("Thruster power at minimum.")
                else:
                    if speed + speedBindings[key][0] < 0.1:
                        speed = 0.0
                    else:
                        speed = speed + speedBindings[key][0]
                    print(f"Thruster input power is currently at: {speed*100} %")
            else:
                pitch = 0.0
                yaw = 0.0
                throttle = 0.0
                if (key == '\x03'):
                    break

            command = Command()
            command.pitch = float(pitch * speed)
            command.yaw = float(yaw * speed)
            command.throttle = float(throttle * speed)
            pub.publish(command)

    except Exception as e:
        print(e)

    finally:
        command = Command()
        command.pitch = 0.0
        command.yaw = 0.0
        command.throttle = 0.0
        pub.publish(command)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
