#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Aug 12 11:01:22 2020

@author: jagat
"""

import cv2 as cv
import os 

sourcepath = '/home/jagat/datasets/vio_rosbags/vio_rig/vio_motion_data_cam17197547_ap_f3_exp_2000us_right_shelf_data_3D_1/cam0'

outputpath = '/home/jagat/datasets/vio_rosbags/vio_rig/vio_motion_data_cam17197547_ap_f3_exp_2000us_right_shelf_data_3D_1/cam0_640x512'

onlyfiles = [f for f in os.listdir(sourcepath) if os.path.isfile(os.path.join(sourcepath, f))]

for filename in onlyfiles:
    filepath = os.path.join(sourcepath,filename)
    inputimg = cv.imread(filepath,0);
    outputimg = cv.resize(inputimg,(640,512))
    cv.imwrite(os.path.join(outputpath,filename),outputimg)
    
