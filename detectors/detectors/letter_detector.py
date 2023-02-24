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
from std_msgs.msg import Float32MultiArray, Int8

#
#  Detector Node Class
#
class DetectorNode(Node):
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

        # Create a publisher for the processed (debugging) image.
        # Store up to three images, just in case.
        self.pub = self.create_publisher(Image, name+'/image_raw', 3)
        self.pub2 = self.create_publisher(Float32MultiArray, name+'/lettertarget', 3)

        # Set up the OpenCV bridge.
        self.bridge = cv_bridge.CvBridge()

        # setup log prob
        self.logprob = np.zeros((12,12), dtype='float64')
        self.accurate_pos = np.zeros((12,12,2), dtype='float64')

        self.target = 1

        # Finally, subscribe to the incoming image topic.  Using a
        # queue size of one means only the most recent message is
        # stored for the next subscriber callback.
        self.sub = self.create_subscription(
            Image, '/image_raw', self.process, 1)
        self.sub2 = self.create_subscription(
            Int8, '/settarget', self.settarget, 1)

        # Report.
        self.get_logger().info("Ball detector running...")

    # Shutdown
    def shutdown(self):
        # No particular cleanup, just shut down the node.
        self.destroy_node()

    def settarget(self, msg):
        self.get_logger().info('target is ######################################################')
        self.get_logger().info(str(msg.data))
        self.target = int(msg.data)
        if self.target >= 0:
            self.logprob = np.zeros((12,12), dtype='float64')
            self.accurate_pos = np.zeros((12,12,2), dtype='float64')

    # Process the image (detect the ball).
    def process(self, msg):
        # Confirm the encoding and report.
        assert(msg.encoding == "rgb8")
        # self.get_logger().info(
        #     "Image %dx%d, bytes/pixel %d, encoding %s" %
        #     (msg.width, msg.height, msg.step/msg.width, msg.encoding))

        # Convert into OpenCV image, using RGB 8-bit (pass-through).
        frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        drawframe = frame.copy()

        msg_data = util.aruco(frame, drawframe)

        M = util.perspective_transform(msg_data, ref=[46,47,48,49])

        # Convert the frame back into a ROS image and republish.
        

        if M is None or (self.target not in msg_data[:,0].tolist() and self.target!=-1): 
            self.pub.publish(self.bridge.cv2_to_imgmsg(drawframe, "rgb8"))
            return
        
        if self.target >= 0:
            self.pub.publish(self.bridge.cv2_to_imgmsg(drawframe, "rgb8"))
            # send target position
            target_ind = np.argwhere( msg_data[:,0]==self.target )[0,0]
            target = msg_data[target_ind]
            
            output = []
            # center position
            center_pos = util.warp_point(M, target[1], target[2])
            output.extend(center_pos.tolist())
            top_left_pos = util.warp_point(M, target[3], target[4])
            top_right_pos = util.warp_point(M, target[5], target[6])
            output.extend( ((top_left_pos+top_right_pos)/2).tolist() )
            output.append(float(self.target))

            my_msg = Float32MultiArray()
            my_msg.data = output
            self.pub2.publish(my_msg)
        else:
            
            util.piledetect(frame, drawframe, M, self.get_logger().info, self.logprob, self.accurate_pos)
            self.pub.publish(self.bridge.cv2_to_imgmsg(drawframe, "rgb8"))

            # send pile position
            x_grid, y_grid = np.unravel_index(np.argmin(self.logprob, axis=None), self.logprob.shape)
            my_msg = Float32MultiArray()
            if self.logprob[x_grid, y_grid]<-0.5:
                # x, y = util.grid2map(x_grid, y_grid)
                # my_msg.data = [x/100, y/100]
                # self.get_logger().info(str( self.accurate_pos[x_grid, y_grid]))
                my_msg.data = self.accurate_pos[x_grid, y_grid].tolist()
                my_msg.data.extend([-1.0, -1.0, -1.0])
            else:
                return
            
            self.pub2.publish(my_msg)


#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the detector node.
    node = DetectorNode('letter')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
