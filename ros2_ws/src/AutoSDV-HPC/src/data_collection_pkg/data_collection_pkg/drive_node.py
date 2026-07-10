import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
import marshal
import types
import os
import sys
import tty
import termios
import threading
import cv2
import numpy as np
import time
from datetime import datetime
from interfaces_pkg.msg import MotionCommand
 
def getch():
    """Gets a single character from stdin"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch
 
class DataCollector:
    def __init__(self, cam_num=0, max_steering=7, speed_increment=5):
        self.max_steering = max_steering
        self.speed_increment = speed_increment
        self.reset_values()
        
 
    def reset_values(self):
        self.steering = 0
        self.left_speed = 0
        self.right_speed = 0
        self.exit_flag = False
 
    def process_key(self, key):
        if key == 'q':
            self.exit_flag = True
            print("\nExiting...\n")
        elif key == 'r':  # Reset all values
            self.reset_values()
            print(f"\nSteering={self.steering}")
            print(f"\nReset all values to zero")
        elif key == 'a':  # Left
            self.steering = max(-self.max_steering, self.steering - 1)
            print(f"\nSteering={self.steering}")
        elif key == 'd':  # Right
            self.steering = min(self.max_steering, self.steering + 1)
            print(f"\nSteering={self.steering}")
        elif key == 'w':  # Speed up
            self.left_speed = min(255, self.left_speed + self.speed_increment)
            self.right_speed = min(255, self.right_speed + self.speed_increment)
        elif key == 's':  # Speed down
            self.left_speed = max(-255, self.left_speed - self.speed_increment)
            self.right_speed = max(-255, self.right_speed - self.speed_increment)
        elif key == ' ':  # Space for emergency stop
            self.steering = 0
            self.left_speed = 0
            self.right_speed = 0
            print("\nEmergency Stop!")
 
    def get_control_values(self):
        return {
            'steering': self.steering,
            'left_speed': self.left_speed,
            'right_speed': self.right_speed
        }
 
class DataCollectionNode(Node):
    def __init__(self):
        super().__init__('data_collection_node')
        
        # Parameters
        self.MAX_STEERING = 7
        self.SPEED_INCREMENT = 5
        
        # QoS Profile
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        
        # Publisher
        self.publisher = self.create_publisher(
            MotionCommand,
            'topic_control_signal',
            self.qos_profile
        )
        
        # Initialize data collector
        self.data_collector = DataCollector(
            max_steering=self.MAX_STEERING,
            speed_increment=self.SPEED_INCREMENT
        )
        
        # Create timer for regular publishing
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        
        # Start input thread
        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()
 
        # Print instructions
        self.print_instructions()
 
    def print_instructions(self):
        print("\n" + "="*60)
        print("ROBOT MOTION CONTROL")
        print("="*60)
        print("Controls:")
        print("  W/S  : Increase/Decrease speed (including reverse)")
        print("  A/D  : Turn left/right")
        print("  SPACE: Emergency stop")
        print("  R    : Reset all values to zero")
        print("  Q    : Quit")
        print("="*60)
        print("Initial Status:")
        print()
        
    def input_loop(self):
        while rclpy.ok():
            try:
                key = getch()
                self.data_collector.process_key(key)
                if self.data_collector.exit_flag:
                    break
            except KeyboardInterrupt:
                break
        
    def timer_callback(self):
        if self.data_collector.exit_flag:
            self.publish_stop_command()
            rclpy.shutdown()
            return
            
        # Get current control values
        control_values = self.data_collector.get_control_values()
        
        # Create and publish ROS2 message
        msg = MotionCommand()
        msg.steering = control_values['steering']
        msg.left_speed = control_values['left_speed']
        msg.right_speed = control_values['right_speed']
        self.publisher.publish(msg)
    
    def publish_stop_command(self):
        msg = MotionCommand()
        msg.steering = 0
        msg.left_speed = 0
        msg.right_speed = 0
        self.publisher.publish(msg)
        self.get_logger().info("Published stop command")
    
    def cleanup(self):
        self.data_collector.exit_flag = True
        if self.input_thread.is_alive():
            self.input_thread.join(timeout=1.0)
        self.publish_stop_command()
 
def main(args=None):
    rclpy.init(args=args)
    node = DataCollectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()
 
if __name__ == "__main__":
    main()