#!/usr/bin/env python

import rospy
import sys
import cv2
import cv2.cv as cv
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class SecCam(object):
    def __init__(self):
        self.threshold = rospy.get_param('~thresh', 0.1)

        self.img_buffer = []
        self.buflen = 30

        self.image_pub = None
        self.image_sub = None

        self.bridge = CvBridge()

        self.motion_detection_time = rospy.Time()

        self.image_sub = rospy.Subscriber("image_in/compressed", CompressedImage, self.image_callback)
        self.image_pub = rospy.Publisher("image_out/compressed", CompressedImage)



    def image_callback(self, ros_image):
        if (rospy.Time.now()-self.motion_detection_time).to_sec() < 5*60.0:
            self.image_pub.publish(ros_image)
            return
        else:
            # Use cv_bridge() to convert the ROS image to OpenCV format

            if 1:
                np_arr = np.fromstring(ros_image.data, np.uint8)
                rgb_frame  = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

            else:
                try:
                    rgb = self.bridge.imgmsg_to_cv(ros_image, "bgr8")
                except CvBridgeError, e:
                    print e

                # Convert the image to a Numpy array since most cv2 functions
                # require Numpy arrays.
                rgb_frame = np.array(rgb, dtype=np.uint8)

            new_img = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2GRAY)

            self.img_buffer.append(new_img)

            if len(self.img_buffer) < self.buflen:
                last_img = new_img
                return
            elif len(self.img_buffer) > self.buflen:
                self.img_buffer.pop(0)

            for i in range(len(self.img_buffer)):
                diff = cv2.absdiff(self.img_buffer[0], self.img_buffer[i])

                #cv2.imshow("Image window", diff)

                _,threshed = cv2.threshold(diff, 10, 255, cv2.THRESH_BINARY)

                #cv2.imshow("Image window", threshed)
                #cv2.waitKey(3)

                nz = cv2.countNonZero(threshed)
                nnz = float(nz)/float(threshed.size)
                #print nnz

                if nnz > self.threshold:
                    self.motion_detection_time = rospy.Time.now()
                    rospy.loginfo( 'detected motion @ '+str(self.motion_detection_time))
                    return
        #print 'no motion'

def main():

    rospy.init_node('seccam')

    sc = SecCam()

    rospy.spin()

if __name__ == '__main__':
    main()
