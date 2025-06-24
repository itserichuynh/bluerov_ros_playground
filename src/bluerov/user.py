#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
import time

# try:
#     import pubs
#     import subs
#     import video
# except:
#     import bluerov.pubs as pubs
#     import bluerov.subs as subs
#     import bluerov.video as video

from geometry_msgs.msg import TwistStamped
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool, String

from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import OverrideRCIn, RCIn, RCOut


class Code(Node):

    """Class to provide user access

    Attributes:
        cam (Video): Video object, get video stream
        pub (Pub): Pub object, do topics publication
        sub (Sub): Sub object, subscribe in topics
    """

    def __init__(self):
        super().__init__('user_node')

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        # Wait for the service to be available
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/mavros/cmd/arming service not available, waiting...')

        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Wait for the service to be available
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/mavros/set_mode service not available, waiting...')

        # self.sub = subs.Subs(self)
        # self.pub = pubs.Pubs(self)

        # self.arm()

        # self.pub.publish_topic(topic='/mavros/rc/override', msg_type=OverrideRCIn
        # self.pub.publish_topic(topic='/mavros/setpoint_velocity/cmd_vel', msg_type=TwistStamped)
        # self.pub.publish_topic('/BlueRov2/body_command', JointState)

        # self.sub.subscribe_topic(topic='/joy', msg_type=Joy)
        # self.sub.subscribe_topic(topic='/mavros/battery', msg_type=BatteryState)
        # self.sub.subscribe_topic(topic='/mavros/rc/in', msg_type=RCIn)
        # self.sub.subscribe_topic(topic='/mavros/rc/out', msg_type=RCOut)

        # self.joy_subscriber_ = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        # self.vel_publisher_ = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

        self.arm_subscriber_ = self.create_subscription(Bool, '/arm',self.arm, 10)
        self.disarm_subscriber_ = self.create_subscription(Bool, '/disarm',self.disarm, 10)
        self.setmode_subscriber_ = self.create_subscription(String, '/setmode',self.setmode, 10)

        # timer_period = 0.01  # seconds
        # self.timer = self.create_timer(timer_period, self.run)

        
    
    def arm(self, msg):
        if msg.data == True:
            self.get_logger().info("Sending arm command...")
            req = CommandBool.Request()
            req.value = True

            future = self.arming_client.call_async(req)
            future.add_done_callback(self.arm_callback)
        
    def disarm(self, msg):
        if msg.data == True:
            self.get_logger().info("Sending disarm command...")
            req = CommandBool.Request()
            req.value = False
            future = self.arming_client.call_async(req)
            future.add_done_callback(self.disarm_callback)
    
    def setmode(self, msg):
        self.get_logger().info("Sending mode command...")
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = msg.data
        future = self.mode_client.call_async(req)
        future.add_done_callback(self.setmode_callback)
    
    def setmode_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info("Set mode successfully.")
            else:
                self.get_logger().warn("Failed to set mode.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
    
    def arm_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Vehicle armed successfully.")
            else:
                self.get_logger().warn("Failed to arm vehicle.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def disarm_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Vehicle disarmed successfully.")
            else:
                self.get_logger().warn("Failed to disarm vehicle.")
        except Exception as e:
            self.get_logger().error(f"Disarm service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = Code()
    rclpy.spin(node)
    # try:
    #     rclpy.spin(node)
    # except KeyboardInterrupt:
    #     node.get_logger().info('Interrupted by user (Ctrl+C)')
    #     node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
