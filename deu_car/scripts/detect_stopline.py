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
        cv2.namedWindow("window", 1)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.stopline_image_pub = rospy.Publisher('camera/rgb/image_raw/stopline', Image, queue_size=1)  # detect stopline
        self.twist = Twist()
        self.stoplinecount = 0
        self.nextdostop = True

    def image_callback(self, msg):
        stoplineimage = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(stoplineimage, cv2.COLOR_BGR2HSV)
        lower_white = numpy.array([0, 0, 225])
        upper_white = numpy.array([10, 10, 235])
        stoplinemask = cv2.inRange(hsv, lower_white, upper_white)

        h, w, d = stoplineimage.shape # h, w, d = 480, 640, 3
        search_top = 3 * h / 4 # = 360
        search_bot = search_top + 20 # = 380

        stoplinemask[0:search_top, 0:w] = 0
        stoplinemask[search_bot:h, 0:w] = 0
        stoplinemask[0:h, 0:w / 2] = 0
        stoplinemask[0:h, (w / 2 + 20):w] = 0
        M = cv2.moments(stoplinemask)  # stopline mask

        if M['m00'] > 0:
            # print('find stopline!')
            if self.nextdostop: # = True
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(stoplineimage, (cx, cy), 20, (0, 0, 255), -1)

                self.twist.linear.x = 0.0
                rospy.sleep(3)
                self.nextdostop = False
                self.stoplinecount += 1
                print('stop!')
            else: # self.dostop = False
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(stoplineimage, (cx, cy), 20, (0, 0, 255), -1)
                self.twist.linear.x = 0.8
        else:
            self.nextdostop = True
            self.twist.linear.x = 0.8
            print('no detect stop line!')

        self.cmd_vel_pub.publish(self.twist)
        stoplineimage_msg = self.bridge.cv2_to_imgmsg(stoplineimage, 'bgr8')
        self.stopline_image_pub.publish(stoplineimage_msg)  # publish
        # print('nextdostop = %r' % self.nextdostop)
        print('stoplinecount = %d' % self.stoplinecount)
        cv2.imshow("window", stoplineimage)
        cv2.waitKey(3)

def main():
    rospy.init_node('driving_bot')
    driving_bot = StopLine()
    rospy.spin()


if __name__ == "__main__":
    main()