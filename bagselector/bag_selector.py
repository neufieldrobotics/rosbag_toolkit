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

class BagSelector:
    def __init__(self,read_bag_path, write_folder_path, write_bag_path, save_imgs, config):
        self.viz=False
        self.calib=False
        self.write_bag = rosbag.Bag(os.path.join(write_folder_path,write_bag_path), 'w')
        self.write_folder_path = write_folder_path

        self.bridge = CvBridge()
        self.topics =  self.image_pub_topics  = config['topics']

        self.num_topics = len(self.topics)
        rospy.init_node('BagSelector', anonymous = True)
        self.image_pubs = []
        for i in range(self.num_topics):
            pub = rospy.Publisher(self.image_pub_topics[i], Image, queue_size=1)
            self.image_pubs.append(pub)
        self.read_bag = rosbag.Bag(read_bag_path)
        self.start = rospy.Time(config['start_time'] + self.read_bag.get_start_time())
        self.end = rospy.Time(config['end_time'] + self.read_bag.get_start_time())
        self.frequency = config['frequency']
        self.bag_to_bag()


    def bag_to_bag(self):
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
                            if self.calib:
                                # Find the chess board corners
                                ret, corners = cv2.findChessboardCorners(cv_image, (9,16),None)
                                if ret == True:
                                    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                                    corners2 = cv2.cornerSubPix(cv_image,corners,(11,11),(-1,-1),criteria)
                                    # Draw and display the corners
                                    cv_image = cv2.drawChessboardCorners(cv_image, (9,16), corners2,ret)
                            cv_img_list.append(cv_image)

                        vis = np.concatenate(cv_img_list, axis=1)
                        height, width = vis.shape
                        target_size = (int( width/2), int(height/2))
                        vis = cv2.resize(vis, target_size, interpolation =cv2.INTER_NEAREST)
                        cv2.imshow("image", vis)
                        key = cv2.waitKey(0) & 0xFF
                        if key == ord('a') :
                            for i in range(self.num_topics):
                                self.write_bag.write(self.image_pub_topics[i], msg_list[i], msg_list[i].header.stamp)
                        elif key == ord('i') :
                            for i in range(self.num_topics):
                                print(os.path.join(self.write_folder_path,"cam"+str(i),str(oldt.secs)+".jpg"))
                                cv2.imwrite(os.path.join(self.write_folder_path,"cam"+str(i),str(oldt.secs)+".jpg"), cv_img_list[i])
                        elif key == 27:
                            self.write_bag.close()
                            return
                    count = count + 1
                print("starting list\n")

                topiclist=[]
                msg_list=[]
                topiclist.append(topic)
                msg_list.append(msg)


if __name__=="__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter,
                                     description='Selects specific messages from a ROSBAG and saves them to a bag file.\
                                      In case of images we can save image sto a fodler as well\n\n')
    parser.add_argument('-i','--input_bagfile', help='Input rosbag file to input', required=True)
    parser.add_argument('-o','--output_dir', help='Output dir for output bag file', default='.')
    parser.add_argument('-of','--output_filename', help='Output file name for output bag file' ,  default = 'output.bag')
    parser.add_argument('-c','--config_file', help='Yaml file which specifies the topic names, frequency of selection and time range',
                    default = 'config/config.yaml')
    parser.add_argument('-s','--saveimages', help='bool which specifies if images should be saved or rosbag',
                        default = False)

    args = parser.parse_args()

    with open(args.config_file, 'r') as f:
        config = yaml.safe_load(f)
    print(config)
    bs = BagSelector(args.input_bagfile , args.output_dir , args.output_filename, args.saveimages, config)
    #bs = BagSelector("2019_Aug6_indoor_rig_calib_selected.bag" , "/home/auv/testing_bagselector", "")
    #/media/auv/Untitled/2019-08-13-00-09-48.bag