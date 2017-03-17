#! /usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import roslib
import rospy
from sensor_msgs.msg import CompressedImage

class ImageProcessing:
    
    def __init__(self):
        self.margin = 50
        #### end the initialization with the ros stuff ####
        self.sub    = rospy.Subscriber("/image_in/compressed",  CompressedImage, self.on_image,  queue_size = 1)
        self.pub    = rospy.Publisher ("/image_out/compressed", CompressedImage,                 queue_size = 1)
        
    def on_image(self, ros_data):

        #### From ros message to cv image ####
        compressed_in = np.fromstring(ros_data.data, np.uint8)
        # image_in      = cv2.imdecode(compressed_in, cv2.CV_LOAD_IMAGE_COLOR) --> obsolete
        image_in      = cv2.imdecode(compressed_in, cv2.IMREAD_COLOR)
        width         = image_in.shape[1]
        height        = image_in.shape[0]
        
        #### Processing ####
        
        subframe      = image_in[self.margin:(height-self.margin), 
                                 self.margin:(width-self.margin),  
                                 :]                      
        subframe[...] = 255 - subframe
        image_out     = image_in

        #### Publishing the result ####
        msg              = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format       = "jpeg"
        msg.data         = np.array(cv2.imencode('.jpg', image_out)[1]).tostring()
        self.pub.publish(msg)

    
if __name__ == '__main__':
    rospy.init_node('demo', anonymous=True)
    try:
        image_processing = ImageProcessing()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down demo_cv_py/demo.py"
