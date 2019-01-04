#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from cmvision.msg import *

def blobs_callback(data):
    if (data.blob_count > 0): #if you detect anything, then proceed
        for i in data.blobs: #go though the stuff detected
            if i.name == 'Teal': #if one of them is what you want
                twist_msg = Twist()
                horizontal_pos = i.x #get its x position 
                print (horizontal_pos)
                if horizontal_pos > 250:
                    twist_msg.angular.z = -0.1 #rotate ccw
                elif horizontal_pos < 150:
                    twist_msg.angular.z = 0.1 #totate cw
                else: #in the middle
                    twist_msg.angular.z = 0 #stop!
                twist_publisher.publish(twist_msg)#publish the twist message




if __name__ == "__main__":
    rospy.init_node("color_tracking_node")
    rospy.Subscriber("/blobs", Blobs, blobs_callback) # no need for assignemnt
    twist_publisher = rospy.Publisher("/cmd_vel", Twist , queue_size=1) #createing a publisher
    rospy.spin()
    
