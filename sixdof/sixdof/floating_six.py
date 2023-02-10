#!/usr/bin/env python3
#
#   demo134.py
#
#   Demonstration node to interact with the HEBIs.
#
import numpy as np
import rclpy
import math

from rclpy.node         import Node
from sensor_msgs.msg    import JointState
from std_msgs.msg import Float32 as Float
from math import sin, cos, pi

#
#   Definitions
#
RATE = 100.0            # Hertz


#
#   DEMO Node Class
#
class DemoNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Create a temporary subscriber to grab the initial position.
        self.position0 = self.grabfbk()
        self.starttime = self.get_clock().now()
        self.position1 = -np.pi/6 * np.sign(self.position0)
        # self.position1 = -np.pi/2*np.sign(self.position0)
        self.amp = (self.position0-self.position1)/2
        self.offset = (self.position0+self.position1)/2
        self.get_logger().info("Initial positions: %r" % self.position0)

        self.gravity_scale = 1.0

        # Create a message and publisher to send the joint commands.
        self.cmdmsg = JointState()
        self.cmdpub = self.create_publisher(JointState, '/joint_commands', 10)

        # Create a subscriber to continually receive joint state messages.
        self.fbksub = self.create_subscription(
            JointState, '/joint_states', self.recvfbk, 10)

        # Create a timer to keep calculating/sending commands.
        rate       = RATE
        self.timer = self.create_timer(1/rate, self.sendcmd)
        self.get_logger().info("Sending commands with dt of %f seconds (%fHz)" %
                               (self.timer.timer_period_ns * 1e-9, rate))
        
        self.actpos = None
        self.statessub = self.create_subscription(
                JointState, '/joint_states', self.cb_states, 1)
        while self.actpos is None:
            rclpy.spin_once(self)
        self.get_logger().info("Initial positions: %r" % self.actpos)

        self.numbersub = self.create_subscription(Float, '/number', self.cb_number, 1)
        


    # Shutdown
    def shutdown(self):
        # No particular cleanup, just shut down the node.
        self.destroy_node()

    def cb_states(self, msg):
        # Save the actual position.
        self.actpos = msg.position
    
    def cb_number(self, msg):
        self.gravity_scale = msg.data
        self.get_logger().info("Received: %r" % msg.data)

    # Grab a single feedback - do not call this repeatedly.
    def grabfbk(self):
        # Create a temporary handler to grab the position.
        def cb(fbkmsg):
            self.grabpos   = list(fbkmsg.position)
            self.grabready = True

        # Temporarily subscribe to get just one message.
        sub = self.create_subscription(JointState, '/joint_states', cb, 1)
        self.grabready = False
        while not self.grabready:
            rclpy.spin_once(self)
        self.destroy_subscription(sub)

        # Return the values.
        return self.grabpos

    def gravity(self, pos):
        if pos is None: return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        scale = self.gravity_scale
        (A, B, C, D) = (0.0*scale, -0.1*scale, -0.5*scale, -2.75*scale)#(0.01*scale, 0.1*scale, -0.01*scale, -1.0*scale)
        (_,  t1, _, t2, t3, _) = list(pos)
        t1 = -t1
        t2 = -t2
        tau3 = 0.5 * cos(t1+t2+t3)  + 0.01 * sin(t1+t2+t3)
        tau2 = -tau3 - 5.0 * cos(t1+t2)
        tau1 = tau2 - 6.0 * cos(t1) * scale
        return (0.0, tau1, 0.0, tau2, tau3, 0.0)


    # Receive feedback - called repeatedly by incoming messages.
    def recvfbk(self, fbkmsg):
        # Just print the position (for now).
        # print(list(fbkmsg.position))
        pass

    # Send a command - called repeatedly by the timer.
    def sendcmd(self):
        # Build up the message and publish.
        self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
        t = (self.get_clock().now()-self.starttime).nanoseconds * 1e-9

        
        if t<5:
            theta = -1 + (-0.4)*t/5
            self.get_logger().info(str(t))
            self.get_logger().info(str(theta))
        else: theta = -1-0.4


        nan = float('nan')
        self.cmdmsg.name         = ['theta1', 'theta2', 'grip', 'theta3', 'theta4', 'theta5']
        self.cmdmsg.position     = [0.0, -np.pi/4, 0.9, 0.0, -np.pi/2, 0.0]
        self.cmdmsg.velocity     = (nan, nan, nan, nan, nan, nan)
        self.cmdmsg.effort       = self.gravity(self.actpos)
        self.cmdpub.publish(self.cmdmsg)


#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the DEMO node.
    node = DemoNode('demo')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
