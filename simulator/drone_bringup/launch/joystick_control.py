#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

def scale_linear(value, in_min, in_max, out_min, out_max):
    # Scale 'value' from the joystick input range [in_min, in_max]
    # to the desired linear velocity range [out_min, out_max]
    if value < in_min:
        value = in_min
    elif value > in_max:
        value = in_max
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def joystick_callback(msg):
    # Map joystick axes to linear x and y velocities using scale_linear()
    linear_x = scale_linear(msg.axes[0], -1.0, 1.0, -1.0, 1.0)
    linear_y = scale_linear(msg.axes[1], -1.0, 1.0, -1.0, 1.0)

    twist = Twist()
    twist.linear.x = linear_x
    twist.linear.y = linear_y

    publisher.publish(twist)

def main():
    rclpy.init()
    node = rclpy.create_node('joystick_controller')

    # Subscribe to joystick inputs
    joystick_subscriber = node.create_subscription(Joy, 'joy', joystick_callback, 10)

    # Create a publisher to send Twist messages for drone control
    global publisher
    publisher = node.create_publisher(Twist, 'drone/cmd_vel', 10)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
