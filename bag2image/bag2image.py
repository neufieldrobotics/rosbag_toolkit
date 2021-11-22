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
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import numpy as np
import rosbag
import os
import argparse
import yaml

'''
@author: Pushyami Kaveti 
This is a tool to select only required images from a rosbag. 
The script goes through each image in the bag one by one and 
shows in a window, it has key bindings to  select the image 
which will be written to another bag file. It uses a config 
file to specify various options for selection like topics, 
time range, frequency etc.
'''

class Bag2Image:
    def __init__(self,read_bag_path, write_folder_path, write_bag_path, config):
        self.viz=False
        self.calib=False
        self.write_bag = rosbag.Bag(os.path.join(write_folder_path,write_bag_path), 'w')
        self.write_folder_path = write_folder_path
        self.bridge = CvBridge()
        self.topics =  self.image_pub_topics  = config['topics']

        self.num_topics = len(self.topics)
        for i in range(self.num_topics):
            pathe = os.path.join(self.write_folder_path,"cam"+str(i))
            if not os.path.exists(pathe):
                os.makedirs(pathe)
        rospy.init_node('Bag2Image', anonymous = True)
        self.read_bag = rosbag.Bag(read_bag_path)
        self.start = rospy.Time(config['start_time'] + self.read_bag.get_start_time())
        self.end = rospy.Time(config['end_time'] + self.read_bag.get_start_time())
        self.frequency = config['frequency']
        self.convertBag()


    def convertBag(self):
        oldt = 0;
        topiclist=[]
        msg_list=[]
        count = 0
        print(type(self.read_bag.get_start_time()))
        for topic, msg, t in self.read_bag.read_messages(topics=self.topics , start_time= self.start , end_time= self.end):
            if oldt == msg.header.stamp:
                topiclist.append(topic)
                msg_list.append(msg)
                #print(msg.header.frame_id)
            else:
                oldt =  msg.header.stamp
                if len(topiclist) == self.num_topics and topiclist==self.topics :
                    if count % self.frequency == 0:
                        print("Writing to bag:")
                        print('count:' + str(count))
                        print(t)
                        cv_img_list=[]
                        for im_msg in msg_list:
                            cv_image = self.bridge.imgmsg_to_cv2(im_msg, "mono8")
                            cv_img_list.append(cv_image)

                        for i in range(self.num_topics):
                            print(os.path.join(self.write_folder_path,"cam"+str(i),str(oldt.to_nsec())+".jpg"))
                            cv2.imwrite(os.path.join(self.write_folder_path,"cam"+str(i),str(oldt.to_nsec())+".jpg"), cv_img_list[i])
                    count = count + 1
                print("starting list\n")

                topiclist=[]
                msg_list=[]
                topiclist.append(topic)
                msg_list.append(msg)
        self.write_bag.close()


if __name__=="__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter,
                                     description='Selects specific messages from a ROSBAG and saves them to a bag file.\
                                      In case of images we can save image sto a fodler as well\n\n')
    parser.add_argument('-i','--input_bagfile', help='Input rosbag file to input', required=True)
    parser.add_argument('-o','--output_dir', help='Output dir for output bag file', default='.')
    parser.add_argument('-of','--output_filename', help='Output file name for output bag file' ,  default = 'output.bag')
    parser.add_argument('-c','--config_file', help='Yaml file which specifies the topic names, frequency of selection and time range',
                    default = 'config/config.yaml')

    args = parser.parse_args()

    with open(args.config_file, 'r') as f:
        config = yaml.safe_load(f)
    print(config)
    bs = Bag2Image(args.input_bagfile , args.output_dir , args.output_filename, config)
    #bs = BagSelector("2019_Aug6_indoor_rig_calib_selected.bag" , "/home/auv/testing_bagselector", "")
    #/media/auv/Untitled/2019-08-13-00-09-48.bag
