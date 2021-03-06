#!/usr/bin/env python
import cv2
import numpy
import cv_bridge
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class StopLine:  # Detect Stopline

    def __init__(self):  # creator Detector
        self.bridge = cv_bridge.CvBridge()
        # cv2.namedWindow("stopline_window", 1)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.stopline_image_pub = rospy.Publisher('camera/rgb/image_raw/stopline', Image,
                                                  queue_size=1)  # detect stopline
        self.twist = Twist()
        self.stopline_count = 0
        self.next_dostop = True

    def image_callback(self, msg):
        stopline_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(stopline_image, cv2.COLOR_BGR2HSV)
        lower_white = numpy.array([0, 0, 225])
        upper_white = numpy.array([10, 10, 235])
        mask_stopline = cv2.inRange(hsv, lower_white, upper_white)

        h, w, d = stopline_image.shape  # h, w, d = 480, 640, 3
        # search_top = 3 * h / 4 # = 360
        search_top = h - 110  # = 390
        search_bot = search_top + 20  # = 410

        mask_stopline[0:search_top, 0:w] = 0
        mask_stopline[search_bot:h, 0:w] = 0
        mask_stopline[0:h, 0:w / 2] = 0
        mask_stopline[0:h, (w / 2 + 20):w] = 0
        M = cv2.moments(mask_stopline)  # stopline mask

        if M['m00'] > 0:
            # print('find stopline!')
            if self.next_dostop:  # = True
                if self.stopline_count == 3:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    cv2.circle(stopline_image, (cx, cy), 20, (0, 0, 255), -1)
                    self.twist.linear.x = 0.8
                    self.next_dostop = False
                    self.stopline_count += 1
                else:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    cv2.circle(stopline_image, (cx, cy), 20, (0, 0, 255), -1)
                    self.twist.linear.x = 0.0
                    rospy.sleep(3)
                    self.next_dostop = False
                    self.stopline_count += 1
                    print('stop!')

            else:  # self.next_dostop = False
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(stopline_image, (cx, cy), 20, (0, 0, 255), -1)
                self.twist.linear.x = 0.8
        else:
            self.next_dostop = True
            self.twist.linear.x = 0.8
            # print('no detect stop line!')

        self.cmd_vel_pub.publish(self.twist)
        stopline_image_msg = self.bridge.cv2_to_imgmsg(stopline_image, 'bgr8')
        self.stopline_image_pub.publish(stopline_image_msg)  # publish
        # print('next_dostop = %r' % self.next_dostop)
        print('stopline_count = %d' % self.stopline_count)
        # cv2.imshow("stopline_window", stopline_image)
        # cv2.waitKey(1)


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
        block_mask[3 * h / 4:h, 0:w] = 0
        block_mask[0:h, 0:250] = 0

        M = cv2.moments(img_mask)
        if M['m00'] > 0:
            print('blocking bar @@@@')
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(image, (cx, cy), 10, (255, 0, 0), -1)
            self.twist.linear.x = 0.0
        else:
            self.twist.linear.x = 0.8
            print('GO')

        self.cmd_vel_pub.publish(self.twist)
        bar_image_msg = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        self.bar_pub.publish(bar_image_msg)
        # cv2.imshow("blocking_bar_window", img_mask)
        # cv2.waitKey(1)


class Right_YellowLine:
    def __init__(self):  # creator Detector
        self.bridge = cv_bridge.CvBridge()
        # cv2.namedWindow("stopline_window", 1)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.twist = Twist()

    def image_callback(self, msg):
        yellowline_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(yellowline_image, cv2.COLOR_BGR2HSV)

        lower_yellow = numpy.array([10, 10, 10])
        upper_yellow = numpy.array([255, 255, 250])
        mask_yellowline = cv2.inRange(hsv, lower_yellow, upper_yellow)

        lower_white = numpy.array([0, 0, 225])
        upper_white = numpy.array([10, 10, 235])
        mask_whiteline = cv2.inRange(hsv, lower_white, upper_white)

        h, w, d = yellowline_image.shape  # h, w, d = 480, 640, 3
        search_top = 5 * h / 6 + 10  # = 410
        search_bot = 5 * h / 6 + 30  # = 430

        mask_yellowline[0:w / 2 + 30, 0:w] = 0
        mask_yellowline[0:h, 0:search_bot + 10] = 0
        mask_yellowline[h - 15:h, 0:w] = 0
        mask_yellowline[0:h, w - 80:w] = 0
        M_yellow = cv2.moments(mask_yellowline)  # right yellow line mask

        mask_whiteline[0:h - 40, 0:w] = 0
        mask_whiteline[h - 20:h, 0:w] = 0
        mask_whiteline[h - 40:h - 20, 20:w] = 0
        M_white = cv2.moments(mask_whiteline)  # left white line mask

        if M_yellow['m00'] > 0:
            print('find Right Yellowline!!!!')
            cx_yellow = int(M_yellow['m10'] / M_yellow['m00'])
            cy_yellow = int(M_yellow['m01'] / M_yellow['m00'])
            # print('cx_yellow : ', cx_yellow, 'cy_yellow : ', cy_yellow)

            cx = (cx_yellow + cx_yellow - 350) // 2
            # print('cx : ', cx)

            cv2.circle(yellowline_image, (cx_yellow, cy_yellow), 10, (255, 0, 0), -1)

            if M_white['m00'] > 0:
                cx_white = int(M_white['m10'] / M_white['m00'])
                cy_white = int(M_white['m01'] / M_white['m00'])
                # print('cx_white : ', cx_white, 'cy_white : ', cy_white)

                cv2.circle(yellowline_image, (cx_white, cy_white), 10, (0, 255, 0), -1)

                self.twist.linear.x = 0.8
                print('find Left Whiteline @@@')
            else:
                err = cx - w / 2  # cx - 320
                self.twist.linear.x = 0.8
                self.twist.angular.z = -float(err) / 40
                print(-float(err) / 40)
                # print('no detect Left Whiteline $$$')
        # else:
            # print('no detect Right Yellowline!')

        self.cmd_vel_pub.publish(self.twist)
        # cv2.imshow("stopline_window", yellowline_image)
        # cv2.waitKey(1)


def main():
    rospy.init_node('driving_bot')
    driving_bot = StopLine()
    driving_bot = Detect_blockbar()
    driving_bot = Right_YellowLine()
    rospy.spin()


if __name__ == "__main__":
    main()