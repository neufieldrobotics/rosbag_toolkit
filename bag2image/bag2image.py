import rosbag
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


'''
@author : Pushyami Kaveti
Simple script to take a rosbag and write images topics as images
'''

data_path = "/media/auv/Untitled/2017-11-14-13-51-31_9.bag"
topic_names = ['/camera_array/cam0/image_raw']#, '/camera_array/cam2/image_raw','/camera_array/cam3/#image_raw', '/camera_array/cam4/image_raw', '/camera_array/cam5/image_raw' ]
num_topics = len(topic_names)
bag = rosbag.Bag(data_path)
i=0
j=0
bridge = CvBridge()
i=range(0, 1000, 6)
for topic, msg, t in bag.read_messages(topics=topic_names):
    try:
	for i_i in i:
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")    
            for k in range(0, num_topics) :
                if topic == topic_names[k] :
                    print(msg.header.stamp)
                    cv2.imwrite(str(msg.header.stamp)+'.bmp', cv_image)
    except CvBridgeError as e:
        print(e)
bag.close()
