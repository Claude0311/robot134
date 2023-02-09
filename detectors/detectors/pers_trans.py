#!/usr/bin/env python3
#
#   pers_trans.py
#
#   Implement perspective transform
#
#   Node:           
#   Subscribers:    
#   Publishers:     
#
import cv2
import numpy as np

# ROS Imports
import rclpy

from rclpy.node         import Node
from std_msgs.msg import Float32MultiArray

#
#  Transformation Node Class
#
class TransNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # initialize tags
        self.tags = {}
        self.M = None

        # Create a publisher for the processed (debugging) image.
        # Store up to three images, just in case.
        self.pub = self.create_publisher(Float32MultiArray, '/target', 3)

        # Finally, subscribe to the incoming image topic.  Using a
        # queue size of one means only the most recent message is
        # stored for the next subscriber callback.
        self.sub = self.create_subscription(
            Float32MultiArray, '/box_list', self.process, 10)

        # Report.
        self.get_logger().info("perspective transformation running...")

    # Shutdown
    def shutdown(self):
        # No particular cleanup, just shut down the node.
        self.destroy_node()


    # Process the image (detect the aruco).
    def process(self, msg):
        data = msg.data
        if len(data)==0: return
        self.tags = {}
        box_num = len(data)//11
        for i in range(box_num):
            id = int(data[i*11])
            points = {
                'center': data[i*11+1 : i*11+3],
                'corners': data[i*11+3 : i*11+11],
            }
            if id not in self.tags:
                self.tags[id] = [points]
            else:
                self.tags[id].append(points)
        
        # self.get_logger().info(str(self.tags))

        target = 47
        if all(x in self.tags.keys() for x in [0,4,25,49, target]):
            pt_from = np.float32([self.tags[0][0]['center'], self.tags[4][0]['center'], self.tags[25][0]['center'], self.tags[49][0]['center']])
            x0 = -60.0 * 0.01
            y0 = 0.0 * 0.01
            dx = +13.5 * 0.01
            dy = -19.0 * 0.01
            pt_to = np.float32([[x0, y0], [x0+dx, y0], [x0, y0+dy], [x0+dx, y0+dy]])
            pt_target = np.float32(self.tags[target][0]['center'])
            self.M = cv2.getPerspectiveTransform(pt_from,pt_to)
            dst = self.warp_point(pt_target[0], pt_target[1])
            self.get_logger().info(str(dst))

            my_msg = Float32MultiArray()
            my_msg.data = list(dst)
            self.pub.publish(my_msg)

    def warp_point(self, x: int, y: int) -> tuple[int, int]:
        M = self.M
        d = M[2, 0] * x + M[2, 1] * y + M[2, 2]

        return (
            round((M[0, 0] * x + M[0, 1] * y + M[0, 2]) / d , 2), # x
            round((M[1, 0] * x + M[1, 1] * y + M[1, 2]) / d , 2), # y
        )


#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the detector node.
    node = TransNode('perspec_trans')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
