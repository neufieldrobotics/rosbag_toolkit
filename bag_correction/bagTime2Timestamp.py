#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Jan  8 13:37:41 2020

@author: Jagatpreet Singh
This is a tool to write the actual time stamp in bag time
"""

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
import numpy as np
import rosbag
import os
import argparse
import yaml
import time

parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter,
                                 description='Selects specific messages from a ROSBAG and saves them to a bag file.\
                                  In case of images we can save image sto a fodler as well\n\n')
parser.add_argument('-i','--input_bagfile', help='Input rosbag file to input', required=True)
parser.add_argument('-o','--output_dir', help='Output dir for output bag file', default='.')
parser.add_argument('-c','--config_file', help='Yaml file which specifies the topic names, frequency of selection and time range',
                default = 'config.yaml')
   
args = parser.parse_args()

with open(args.config_file, 'r') as f:
    config = yaml.safe_load(f)
print(config)
topics = config['topics']
read_bag_path = args.input_bagfile
output_filename = config['output_bag_name'][0]
write_folder_path = args.output_dir
read_bag = rosbag.Bag(read_bag_path)
start = rospy.Time(read_bag.get_start_time())
end =  rospy.Time(read_bag.get_end_time())

# Create output rosbag
filepath = os.path.join(write_folder_path,output_filename) 
output_bag = rosbag.Bag(filepath,'w')
start_time = time.time()
for topic, msg, t in read_bag.read_messages(topics=topics ,start_time= start , end_time= end):
    output_bag.write(topic,msg,t=msg.header.stamp)
elapsed_time = time.time() - start_time
print("elapsed time : %s" %(elapsed_time))    
read_bag.close()
output_bag.close()