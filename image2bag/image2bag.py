import rosbag
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import std_msgs
import os
import glob
import numpy as np
import argparse
import yaml


'''
@author: Pushyami Kaveti 
This is a tool to convert images into a bag files.Each directory representss a topic.
It uses a config file to specify various options for selection like topics, 
time range, frequency etc.
'''

class Image2Bag:
    def __init__(self,topic_n, info_topics, freq, op_file, data_p="/home/auv/calib" ,img_t="bmp"):
        rospy.init_node('image_converter', anonymous=True)
        self.data_path=data_p
        self.topic_names = topic_n
        self.info_topic_names = info_topics
        self.num_topics = len(topic_n)
        self.img_type = img_t
        self.pubs = []
        self.cam_info_pubs = []
        self.im_list = []
        self.bridge = CvBridge()
        self.init_publishers()
        self.get_imgs()
        self.to_bag= True
        if self.to_bag :
            self.write_bag = rosbag.Bag(op_file, 'w')
        self.frequency = freq

    def init_publishers(self):
        for top_ind in range(self.num_topics):
            image_pub = rospy.Publisher(self.topic_names[top_ind], Image, queue_size=1)
            cam_info_pub = rospy.Publisher(self.info_topic_names[top_ind], CameraInfo, queue_size=1)
            self.pubs.append(image_pub)
            self.cam_info_pubs.append(cam_info_pub)

    def get_imgs(self):
        li_single = glob.glob(os.path.join(self.data_path ,"cam0/*."+self.img_type))
        #print(os.path.join(self.data_path ,"cam0/*.", self.img_type))
        self.num_imgs = len(li_single)
        # check all cam directories has the same number f images
        # check the number of diretories is equal to number of topics publishing

        dirs = [ name for name in os.listdir(self.data_path) if os.path.isdir(os.path.join(self.data_path, name)) ]
        #print(dirs)
        dirs.sort()
        if(len(dirs) != self.num_topics):
            print("ERROR: number of image directories is not equal to number of topics")
            exit()

        self.im_list = glob.glob(os.path.join(self.data_path ,"cam*/*."+self.img_type))
        self.im_list.sort()

    def run(self):
        r = rospy.Rate(self.frequency) # 10hz
        i=0
        print(self.num_imgs)
        while not rospy.is_shutdown():
            if ( i < self.num_imgs):
                h = std_msgs.msg.Header()
                h.stamp = rospy.Time.now()
                h.seq = i
                print("--------------------------------")
                imgs = []
                info_msgs = []
                for j in range(self.num_topics):
                    img = cv2.imread(self.im_list[i + j*self.num_imgs], cv2.IMREAD_GRAYSCALE)
                    print(self.im_list[i + j*self.num_imgs])
                    ros_img = self.bridge.cv2_to_imgmsg(img, "mono8")
                    ros_img.header =  h
                    imgs.append(ros_img)
                    cam_info_msg = CameraInfo()
                    cam_info_msg.header = h
                    info_msgs.append(cam_info_msg)
                for k in range(self.num_topics):
                    if self.to_bag:
                        print("writing"+str(i))
                        self.write_bag.write(self.topic_names[k], imgs[k], imgs[k].header.stamp)
                        self.write_bag.write(self.info_topic_names[k], info_msgs[k], info_msgs[k].header.stamp)
                    else:
                        self.pubs[k].publish(imgs[k])
                        self.cam_info_pubs[k].publish(info_msgs[k])
                i = i+1
                r.sleep()
            else:
                print(i)
                break
        r.sleep()
        if self.to_bag:
            self.write_bag.close()

if __name__=="__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter,
                                     description='Reads the images from directories in the path and \
                                                  saves them to individual topics in a bag file.\n\n')
    parser.add_argument('-i','--input_dir', help='Input directory path containing images', required=True)
    parser.add_argument('-o','--output_dir', help='Output dir for output bag file', default='.')
    parser.add_argument('-of','--output_filename', help='Output file name for output bag file' ,  default = 'output.bag')
    parser.add_argument('-c','--config_file', help='Yaml file which specifies the topic names, frequency of selection and time range',
                    default = 'config/config.yaml')
   
    args = parser.parse_args()

    with open(args.config_file, 'r') as f:
        config = yaml.safe_load(f)
    op_file = os.path.join(args.output_dir, args.output_filename)
    bs = Image2Bag(config['topics'], config['info_topics'], config['frequency'], op_file, args.input_dir , config['img_type'])
    bs.run()
