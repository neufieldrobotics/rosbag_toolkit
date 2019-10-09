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

'''
@author: Pushyami Kaveti
Script to take images in a folder and convert them into bag file with synchronized camera_info messages
'''
class Image2Bag:
    def __init__(self,topic_n, info_topics, data_p="/home/auv/calib" , img_t="bmp"):
        rospy.init_node('image_converter', anonymous=True)
        self.data_path=data_p
        self.topic_names = topic_n #['/camera_array/cam0/image_raw', '/camera_array/cam1/image_raw','/camera_array/cam2/image_raw','/camera_array/cam3/image_raw', '/camera_array/cam4/image_raw']
        self.info_topic_names = info_topics
        self.num_topics = len(topic_n)
        self.img_type = img_t
        self.pubs = []
        self.cam_info_pubs = []
        self.im_list = []
        self.bridge = CvBridge()
        self.init_publishers()
        self.get_imgs()

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
        r = rospy.Rate(10) # 10hz
        i=0
        print(self.num_imgs)
        while not rospy.is_shutdown():
            if ( i < self.num_imgs):
                h = std_msgs.msg.Header()
                h.stamp = rospy.Time.now()
                print("--------------------------------")
                imgs = []
                info_msgs = []
                for j in range(self.num_topics):
                    img = cv2.imread(self.im_list[i + j*self.num_imgs], cv2.IMREAD_UNCHANGED)
                    print(self.im_list[i + j*self.num_imgs])
                    ros_img = self.bridge.cv2_to_imgmsg(img, "mono8")
                    ros_img.header =  h
                    imgs.append(ros_img)
                    cam_info_msg = CameraInfo()
                    cam_info_msg.header = h
                    info_msgs.append(cam_info_msg)
                for k in range(self.num_topics):
                    self.pubs[k].publish(imgs[k])
                    self.cam_info_pubs[k].publish(info_msgs[k])
                i = i+1
            else:
                break
            r.sleep()

if __name__=="__main__":
    top_list = ['/camera_array/cam0/image_raw', '/camera_array/cam1/image_raw','/camera_array/cam2/image_raw',\
                '/camera_array/cam3/image_raw', '/camera_array/cam4/image_raw']
    caminfo_top_list = ['/camera_array/cam0/camera_info', '/camera_array/cam1/camera_info',\
                        '/camera_array/cam2/camera_info','/camera_array/cam3/camera_info', '/camera_array/cam4/camera_info']

    #top_list = ['/camera_array/cam0/image_raw', '/camera_array/cam1/image_raw']
    bs = Image2Bag(top_list,caminfo_top_list, "/media/auv/Untitled1/drive-download-20190921T203356Z-001/isec_data_209_21_2019" , "bmp")
    bs.run()
