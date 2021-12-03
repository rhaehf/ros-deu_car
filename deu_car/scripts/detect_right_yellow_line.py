#!/usr/bin/env python
import cv2
import numpy
import cv_bridge
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Right_YellowLine:
    def __init__(self):  # creator Detector
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
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
                print('no detect Left Whiteline $$$')
        # else:
        #     print('no detect Right Yellowline!')

        self.cmd_vel_pub.publish(self.twist)
        cv2.imshow("window", yellowline_image)
        cv2.waitKey(1)


def main():
    rospy.init_node('driving_bot')
    driving_bot = Right_YellowLine()
    rospy.spin()


if __name__ == "__main__":
    main()