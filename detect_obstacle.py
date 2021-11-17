#!/usr/bin/env python

import cv2
import numpy
import cv_bridge
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


class Detect_blockbar:
    def __init__(self):

        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        self.bar_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.bar_pub = rospy.Publisher('camera/rgb/image_raw/p2_bar', Image, queue_size=1)
        self.twist = Twist()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red = numpy.array([0, 30, 30])
        upper_red = numpy.array([15, 255, 110])
        img_mask = cv2.inRange(hsv, lower_red, upper_red)
        h, w = img_mask.shape
        block_mask = img_mask

   #     search_top = 1
    #   search_bot = 3 * h / 4
        block_mask[0:1, 0:w] = 0
        block_mask[3* h/4 :h, 0:w] = 0
        block_mask[0:h, 0:250] = 0

        M = cv2.moments(img_mask)
        self.twist.linear.x = 0.8
#        self.twist.angular.z = 1.0
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(image, (cx, cy), 10, (255, 0, 0), -1)
            self.twist.linear.x = 0.0
 #           rospy.loginfo('wait!!!')
        else:
            self.twist.linear.x = 0.8

        self.cmd_vel_pub.publish(self.twist)
        bar_image_msg = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        self.bar_pub.publish(bar_image_msg)
        cv2.imshow("window", img_mask)
        cv2.waitKey(3)


def main():
    rospy.init_node('Drive_bot')
    driving_bot = Detect_blockbar()
    rospy.spin()


if __name__ == "__main__":
    main()
