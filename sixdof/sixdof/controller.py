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
import numpy as np

# ROS Imports
import rclpy
import cv_bridge

from rclpy.node         import Node
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

        self.word = 'cider'
        self.index = -1

        self.timer = self.create_timer(0.01, self.process)
        self.t  = 0.0
        self.dt = self.timer.timer_period_ns * 1e-9
        # -1: not looking
        #  0: looking for letter
        #  1: looking for tile
        #  2: looking for a pile
        self.waiting = -1
        self.failure_count = 0

        self.pile = None
        self.flip = None



        # create_publisher
        
        # To robot:  decide action: flip/pile/pickup
        # [action:0~2, index, position]
        # pickup: [0, index, pos_x, pos_y, pos_top_x, pos_top_y]
        # flip:   [1,    -1, pos_x, pos_y, pos_top_x, pos_top_y]
        # pile:   [2,    -1, pos_x, pos_y]
        self.action_pub = self.create_publisher(Float32MultiArray, name+'/do_action', 10)
        # To camera: decide next target
        # 0~25 for letter, 26 for blank
        self.camera_pub = self.create_publisher(Int8, name+'/settarget', 10)

        # create_subscription
        # From user:   decide next letter to pick up
        # From robot:  get robot done one action
        # From camera: get target(letter) position
        self.sub = self.create_subscription(
            String, '/set_letter', self.set_letter, 1)
        self.sub2 = self.create_subscription(
            Int8, '/cur_phase', self.handle_phase, 10
        )
        # Gets all the tiles and piles from camera
        self.sub3 = self.create_subscription(
            Float32MultiArray, '/lettertarget', self.select_phase, 10
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
   
    # listens to robot - if msg=1, then it failed to pick up tile
    def handle_phase(self, msg=None):
        self.get_logger().info('phase 0 get')
        phase = msg.data if msg is not None else 0
        if phase == 0: 
            self.failure_count = 0
            self.index += 1
        else:
            self.failure_count += 1
            # fail three times in a row
            if self.failure_count >=2:
                # look for pile
                # self.waiting = 2
                self.t = 0
                self.get_logger().info('try hit pile')
                output = Int8()
                output.data = -1
                self.camera_pub.publish(output)
                self.index -= 1
                return


        # word done
        if self.index>=len(self.word): return
        # next phase
        # if phase == 0:
        self.t = 0 # start waiting
        self.waiting = 0
        
        letters = 'abcdefghijklmnopqrstuvwxyz'
        output = Int8()
        self.get_logger().info(str((letters.find(self.word[self.index]), self.word[self.index], self.index, self.word)))
        output.data = letters.find(self.word[self.index])
        self.camera_pub.publish(output)


    # def handle_letter(self, msg):
    #     data = msg.data
    #     data = np.array(data).reshape(-1, 5)
    #     self.get_logger().info("data: " + str(data))
        # [cx, cy, tx, ty, target] = data
        # target = int(target)
        # self.get_logger().info(str((target, self.waiting)))
        # letters = 'abcdefghijklmnopqrstuvwxyz'
        # mymsg = Float32MultiArray()
        # if self.waiting != -1:
        #     if 0<=target<26 and letters[target]==self.word[self.index] and self.waiting==0:
        #         mymsg.data = [0.0, float(self.index), cx, cy, tx, ty]
        #         self.waiting = -1
        #     elif target == 26:
        #         mymsg.data = [1.0, -1.0, cx, cy, tx, ty]
        #     elif target == -1 and self.waiting==2: # pile
        #         self.waiting = -1
        #         mymsg.data = [2.0, -1.0, cx, cy]
        #     self.action_pub.publish(mymsg)
    
    # phase = waiting
    # -1: not looking
    #  0: looking for letter
    #  1: looking for tile
    #  2: looking for a pile

    # To robot:  decide action: flip/pile/pickup
    # [action:0~2, index, position]
    # pickup: [0, index, pos_x, pos_y, pos_top_x, pos_top_y]
    # flip:   [1,    -1, pos_x, pos_y, pos_top_x, pos_top_y]
    # pile:   [2,    -1, pos_x, pos_y]
    def select_phase(self, msg):
        self.get_logger().info("in select phase")
        # self.get_logger().info("waiting: " +  str(self.waiting))
        data = msg.data
        data = np.array(data).reshape(-1, 5)
        # token is tile/pile
        tokens = [token[0] for token in data]
        mymsg = Float32MultiArray()
        # if target is in data
        target = ord(self.word[self.index].lower()) - 97
        self.get_logger().info("target: " + str(target))
        self.get_logger().info("tokens: " + str(tokens))
        self.get_logger().info("-1?: " + str(-1 in tokens))
        if self.waiting == 0:
            if target in tokens:
                # look for letter
                self.get_logger().info("letter is found!")
                idx = tokens.index(target)
                # self.waiting = 0
                [letter, cx, cy, tx, ty] = data[idx] 
                targ_msg = [0.0, float(self.index), \
                            float(cx), float(cy), float(tx), float(ty)]
                mymsg.data = targ_msg
                self.action_pub.publish(mymsg)
                self.waiting = -1
            elif -1 in tokens:
                idx = tokens.index(-1)
                [letter, cx, cy, tx, ty] = data[idx] 
                targ_msg = [2.0, -1.0, \
                            float(cx), float(cy)]
                self.waiting = 0
                # mymsg.data = targ_msg
                # self.action_pub.publish(mymsg)
                # self.waiting = -1 
                self.pile = targ_msg
            elif 26 in tokens:
                idx = tokens.index(26)
                [letter, cx, cy, tx, ty] = data[idx] 
                targ_msg = [1.0, -1.0, \
                            float(cx), float(cy), float(tx), float(ty)]
                self.waiting = 0
                # mymsg.data = targ_msg
                # self.action_pub.publish(mymsg)
                # self.waiting = -1 
                self.flip = targ_msg
            # if pile is in data, but target tile is not found
            # self.get_logger().info("tokens: " + str(tokens))
            # elif -1 in tokens:
            #     self.get_logger().info("looking for a pile")
            #     idx = tokens.index(-1)
            #     [letter, cx, cy, tx, ty] = data[idx] 
            #     targ_msg = [1.0, -1.0, \
            #                 float(cx), float(cy), float(tx), float(ty)]
            #     mymsg.data = targ_msg
            #     self.action_pub.publish(mymsg)
            #     self.waiting = -1 
                
    
    # Process
    def process(self):
        # we have camera data
        if self.waiting == 0:
            self.t += self.dt
            if self.t%2<0.011: self.get_logger().info("self.waiting: " + str(self.waiting))
            # we have not found a tile
            if self.t>5:
                if self.pile is not None:
                    self.get_logger().info("trying to hit a pile")
                    mymsg = Float32MultiArray()
                    mymsg.data = self.pile
                    self.action_pub.publish(mymsg)
                    self.t = 0
                    self.waiting = -1
                    self.pile = None
                    self.flip = None
                elif self.flip is not None:
                    self.get_logger().info("trying to flip a tile")
                    mymsg = Float32MultiArray()
                    mymsg.data = self.flip
                    self.action_pub.publish(mymsg)
                    self.t = 0
                    self.waiting = -1
                    self.pile = None
                    self.flip = None
            #     # self.get_logger().info("WE CHANGED SELF.WAITING TO 2")
            #     self.waiting = -1
            #     self.t = 0
            #     self.get_logger().info('try hit pile')
            #     # output = Int8()
            #     # output.data = -1
            #     # self.camera_pub.publish(output)
            #     # self.index -= 1


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
