#!/usr/bin/env python
import cv2
import numpy
import cv_bridge
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class StopLine:  #Detect Stopline

    def __init__(self):  # creator Detector
        self.bridge = cv_bridge.CvBridge()
        # cv2.namedWindow("stopline_window", 1)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.stopline_image_pub = rospy.Publisher('camera/rgb/image_raw/stopline', Image, queue_size=1)  # detect stopline
        self.twist = Twist()
        self.stopline_count = 0
        self.next_dostop = True

    def image_callback(self, msg):
        stopline_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(stopline_image, cv2.COLOR_BGR2HSV)
        lower_white = numpy.array([0, 0, 225])
        upper_white = numpy.array([10, 10, 235])
        mask_stopline = cv2.inRange(hsv, lower_white, upper_white)

        h, w, d = stopline_image.shape # h, w, d = 480, 640, 3
        # search_top = 3 * h / 4 # = 360
        search_top = h - 110  # = 390
        search_bot = search_top + 20 # = 410

        mask_stopline[0:search_top, 0:w] = 0
        mask_stopline[search_bot:h, 0:w] = 0
        mask_stopline[0:h, 0:w / 2] = 0
        mask_stopline[0:h, (w / 2 + 20):w] = 0
        M = cv2.moments(mask_stopline)  # stopline mask

        if M['m00'] > 0:  # fisrt detect
            print('find stopline!')
            if self.next_dostop: # = True
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(stopline_image, (cx, cy), 20, (0, 0, 255), -1)

                self.twist.linear.x = 0.0
                rospy.sleep(3)
                self.next_dostop = False
                self.stopline_count += 1

            else: # self.dostop = False
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(stopline_image, (cx, cy), 20, (0, 0, 255), -1)
                self.twist.linear.x = 0.8
        else:
            print('no detect!')
            self.next_dostop = True
            self.twist.linear.x = 0.8

        self.cmd_vel_pub.publish(self.twist)
        stopline_image_msg = self.bridge.cv2_to_imgmsg(stopline_image, 'bgr8')
        self.stopline_image_pub.publish(stopline_image_msg)  # publish
        print('nextdostop = %r' % self.next_dostop)
        print('stoplinecount = %d' % self.stoplinecount)
        # cv2.imshow("stopline_window", stopline_image)
        # cv2.waitKey(3)

class Detect_blockbar:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        # cv2.namedWindow("blocking_bar_window", 1)
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

        block_mask[0:1, 0:w] = 0
        block_mask[3 * h/4 :h, 0:w] = 0
        block_mask[0:h, 0:250] = 0

        M = cv2.moments(img_mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(image, (cx, cy), 10, (255, 0, 0), -1)
            self.twist.linear.x = 0.0
        else:
            self.twist.linear.x = 0.8
        self.cmd_vel_pub.publish(self.twist)
        bar_image_msg = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        self.bar_pub.publish(bar_image_msg)
        # cv2.imshow("blocking_bar_window", img_mask)
        # cv2.waitKey(3)


def main():
    rospy.init_node('driving_bot')
    driving_bot = StopLine()
    driving_bot = Detect_blockbar()
    rospy.spin()


if __name__ == "__main__":
    main()