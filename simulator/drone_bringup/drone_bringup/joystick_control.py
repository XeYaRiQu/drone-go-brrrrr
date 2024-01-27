#!/usr/bin/env python3

import rclpy #ros client library for python
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist #twist is velocity in free space broken into its linear and angular parts

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
    throttle = scale_linear(msg.axes[2], -1.0, 1.0, -0.5, 0.5) #0.5 for easier control
    roll = scale_linear(msg.axes[0], -1.0, 1.0, -0.5, 0.5)
    pitch = scale_linear(msg.axes[1], -1.0, 1.0, -0.5, 0.5)
    yaw = scale_linear(msg.axes[3], -1.0, 1.0, -0.5, 0.5)

    twist = Twist()
    twist.linear.x = roll  
    twist.linear.y = pitch
    twist.linear.z = throttle  # Assuming upward motion for throttle
    twist.angular.z = yaw  # Assuming yaw controls angular rotation
    publisher.publish(twist)

def main():
    rclpy.init()
    node = rclpy.create_node('joystick_controller')

    # Subscribe to joystick inputs
    joystick_subscriber = node.create_subscription(Joy, 'joy', joystick_callback, 10) #subscribing to joy node, listen to messages on the joy topic, which are published by the joy node, and process them using the joystick_callback function

    # Create a publisher to send Twist messages for drone control
    global publisher
    publisher = node.create_publisher(Twist, 'drone/cmd_vel', 10) #publishing to gazebo?

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
