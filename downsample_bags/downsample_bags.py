import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
import message_filters
import numpy as np
import rosbag
import os
import argparse
import yaml
import time

'''
@author: Jagatpreet Singh
This is a tool to downsample bags
'''

class BagSelector:
    def __init__(self,read_bag_path, config):
        rospy.init_node('BagSelector', anonymous = True)
        self.topics = config['topics']
        self.read_bag = rosbag.Bag(read_bag_path)
        self.base_imu_freq = config['base_imu_frequency'][0]
        self.base_camera_freq = config['base_camera_frequency'][0]
        #self.trim_time = config['trim_messages'][0]/self.base_imu_freq;
        self.start = rospy.Time(self.read_bag.get_start_time())
        self.end =  rospy.Time(self.read_bag.get_end_time())
              
            
    def downsample_to(self,file_path,imu_rate,cam_rate):
        imu_msg_gap = self.base_imu_freq/imu_rate
        cam_msg_gap = self.base_camera_freq/cam_rate
        bag = rosbag.Bag(file_path,'w')
        
        print("Imu rate: %s" %(imu_rate))
        print("Imu message gap: %s" %(imu_msg_gap))
        
        print("Cam rate: %s" %(cam_rate))
        print("cam message gap: %s" %(cam_msg_gap))
        print('start time : %s' %(self.read_bag.get_start_time()))
        print('end time : %s' %(self.read_bag.get_end_time()))
        count = 0;
        curr_imu_num = 0;
        prev_imu_num = 0;
        curr_image_num = 0;
        prev_image_num = 0;
        start_time = time.time()
        #raw_input('Press enter')
        for topic, msg, t in self.read_bag.read_messages(topics=self.topics ,start_time= self.start , end_time= self.end):
            print(curr_imu_num)
            print(curr_image_num)
            if topic== '/imu/imu':
                if curr_imu_num == 0:
                    bag.write(topic,msg,t=msg.header.stamp)
                if  curr_imu_num - prev_imu_num == imu_msg_gap:
                    bag.write(topic,msg,t=msg.header.stamp)
                    prev_imu_num = curr_imu_num
                curr_imu_num += 1
            if topic =='/cam0/image_raw_bin1' or topic == '/cam0/image_raw_bin2' or topic=='/cam0/image_raw_bin4':
                if curr_image_num == 0:
                    bag.write(topic,msg,t=msg.header.stamp)
                if curr_image_num - prev_image_num == cam_msg_gap:
                    bag.write(topic,msg,t=msg.header.stamp)
                    prev_image_num = curr_image_num
                curr_image_num += 1
               # print ('time : %s and end time: %s'%(t.to_sec(),self.start.to_sec()))
            count += 1
        bag.close()
        elapsed_time = time.time() - start_time
        print("elapsed time : %s" %(elapsed_time))
        print("Total messsages: %s" %(count))
        #raw_input('Press enter')


if __name__=="__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter,
                                     description='Selects specific messages from a ROSBAG and saves them to a bag file.\
                                      In case of images we can save image sto a fodler as well\n\n')
    parser.add_argument('-i','--input_bagfile', help='Input rosbag file to input', required=True)
    parser.add_argument('-o','--output_dir', help='Output dir for output bag file', default='.')
    parser.add_argument('-c','--config_file', help='Yaml file which specifies the topic names, frequency of selection and time range',
                    default = 'config/config.yaml')
   
    args = parser.parse_args()

    with open(args.config_file, 'r') as f:
        config = yaml.safe_load(f)
    print(config)
    bs = BagSelector(args.input_bagfile , config)
    write_folder_path = args.output_dir
    ds_cam_frequency = config['downsampled_camera_frequency']
    ds_imu_frequency = config['downsampled_imu_frequency']
    file_name = config['file_name']
    for cam_freq in ds_cam_frequency:
        filename = '%s_imu_%s_cam_%s.bag'%(file_name[0],ds_imu_frequency[0],cam_freq)
        full_bag_pathname = os.path.join(write_folder_path,filename)
        bs.downsample_to(full_bag_pathname,ds_imu_frequency[0],cam_freq)
    bs.read_bag.close()
