#!/usr/bin/env python3
#
#   letter_detector.py
#
#   Detect the tennis balls with OpenCV.
#
#   Node:           /letter_detector
#   Subscribers:    /usb_cam/image_raw          Source image
#   Publishers:     /letter_detector/image_raw     Debug image
#                   /letter_detector/letter_pos [ center_x, center_y, top_x, top_y, letter(0~49) ]
#
import cv2
import numpy as np

# ROS Imports
import rclpy
import cv_bridge

from rclpy.node         import Node
from sensor_msgs.msg    import Image
import detectors.util as util
from std_msgs.msg import Float32MultiArray, Int8, String

#
#  Detector Node Class
#
class CtrlNode(Node):
    # Pick some colors, assuming RGB8 encoding.
    red    = (255,   0,   0)
    green  = (  0, 255,   0)
    blue   = (  0,   0, 255)
    yellow = (255, 255,   0)
    white  = (255, 255, 255)

    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        self.word = 'robot'
        self.index = -1

        self.timer = self.create_timer(0.01, self.process)
        self.t  = 0.0
        self.dt = self.timer.timer_period_ns * 1e-9
        self.waiting = False

        # create_publisher
        
        # To robot:  decide action: flip/pile/pickup
        # [action:0~2, index, position]
        # pickup: [0, index, pos_x, pos_y, pos_top_x, pos_top_y]
        # flip:   [1,    -1, pos_x, pos_y, pos_top_x, pos_top_y]
        # pile:   [2,    -1, pos_x, pos_y]
        self.pub = self.create_publisher(Float32MultiArray, name+'/do_action', 10)
        # To camera: decide next target
        # 0~25 for letter, 26 for blank
        self.pub2 = self.create_publisher(Int8, name+'/settarget', 10)

        # create_subscription
        # From user:   decide next letter to pick up
        # From robot:  get robot done one action
        # From camera: get target(letter) position
        self.sub = self.create_subscription(
            String, '/set_letter', self.set_letter, 1)
        self.sub2 = self.create_subscription(
            Int8, '/cur_phase', self.handle_phase, 10
        )
        self.sub3 = self.create_subscription(
            Float32MultiArray, '/lettertarget', self.handle_letter, 10
        )

        # Report.
        self.get_logger().info("Brain running...")


    # Shutdown
    def shutdown(self):
        # No particular cleanup, just shut down the node.
        self.destroy_node()

    def set_letter(self, msg):
        self.word = msg.data
        self.index = -1
    
    def handle_phase(self, msg=None):
        self.get_logger().info('phase 0 get')
        phase = msg.data if msg is not None else 0
        self.index += 1
        # word done
        if self.index>=len(self.word): return
        # next phase
        if phase == 0:
            self.t = 0 # start waiting
            self.waiting = True
            
            letters = 'abcdefghijklmnopqrstuvwxyz'
            output = Int8()
            output.data = letters.find(self.word[self.index])
            self.pub2.publish(output)
            
    def handle_letter(self, msg):
        data = msg.data
        self.waiting = False
        [cx, cy, tx, ty, target] = data
        target = int(target)
        self.get_logger().info(str(target))
        letters = 'abcdefghijklmnopqrstuvwxyz'
        mymsg = Float32MultiArray()
        if 0<=target<26 and letters[target]==self.word[self.index]:
            mymsg.data = [0.0, float(self.index), cx, cy, tx, ty]
        elif target == 26:
            mymsg.data = [1.0, -1.0, cx, cy, tx, ty]
        elif target == -1:
            # pile
            mymsg.data = [2.0, -1.0, cx, cy]
        self.pub.publish(mymsg)

        


    # Process
    def process(self):
        if self.waiting:
            self.t += self.dt
            if self.t>10:
                self.waiting = False
                self.t = 0
                self.get_logger().info('try hit pile')
                output = Int8()
                output.data = -1
                self.pub2.publish(output)
                self.index -= 1
        else:
            pass
                # use flip or pile thing



#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the detector node.
    node = CtrlNode('brain')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
