#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@author: Vikrant Shah
A simple script that can converts rosmessages to pandas dataframe pkl files
Define the translation of each message type in a dictionary file and use a 
config file to specify all the topics to be converted.
"""
from __future__ import print_function

#import roslib
import rospy
#import rostopic

import argparse
#import importlib
import sys
import rospkg
import rosbag
import numpy as np
#import time
import sys
import os
import yaml
#import logging
import pandas as pd
import pickle
from tqdm import tqdm
import time

def save_topic2df(topic_name, msg_dict, bagfile):
    #bglog = logging.getLogger(__name__)
    expression = ",".join(msg_dict.values())
    field_names = msg_dict.keys()
            
    data_array = []
    if  bag.get_message_count(topic_name) > 0:
        for topic, msg, ts in tqdm(bagfile.read_messages(topics=[topic_name]), 
                                   total = bag.get_message_count(topic_name),
                                   leave = False, miniters = 2500, unit = 'msgs',
                                   desc = topic_name, position = 0):
            res = eval("{}".format(expression),{'np':np}, {'m': msg, 't':ts})
            data_array.append(res)
        
        if data_array:        
            data_np = np.array(data_array)
            df = pd.DataFrame(data = data_np, columns = field_names) 
            
            df['msg_time'] = pd.to_datetime(df['msg_time'], unit='s')
            
            if 'header_timestamp' in df.columns:
                df['header_timestamp'] = pd.to_datetime(df['header_timestamp'], unit='s')
                df.set_index('header_timestamp',inplace='True')
                
            else:
                df.set_index('msg_time',inplace='True')
                                    
            return df

    else:
        time.sleep(0.1)        
        tqdm.write("Notfound! Topic: "+topic_name )
        return None


parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter,
                                 description='Convert ROSBAG to pandas dataframe pkl file.\n\n'
                                 'A simple script that can converts rosmessages in a rossbag to pandas dataframes in a .pkl file. \
                                 \nDefine the translation of each message type in a dictionary file and use a config file to specify all the topics to be converted..\n\n'
                                 )
parser.add_argument('-i','--input_bagfile', help='Input rosbag file to input', required=True)
parser.add_argument('-o','--output_dir', help='Output dir for pkl file')
parser.add_argument('-c','--config_file', help='Yaml file which specifies topic names to convert',
                    default = 'config/bag2pddf_config.yaml')
parser.add_argument('-d','--dictionary', help='Dictionary file which specifies how to read the topic',
                    default = 'config/bag2pddf_dict.yaml')

args = parser.parse_args()

with open(args.dictionary, 'r') as f:
    exp_dic = yaml.safe_load(f)

with open(args.config_file, 'r') as f:
    config = yaml.safe_load(f)

if 'namespace' in config:
    ns_prefixes = config['namespace'] 
else:
    ns_prefixes = ['']

sys.stdout.write("Reading bagfile: "+args.input_bagfile+" ..")
sys.stdout.flush()
bag = rosbag.Bag(args.input_bagfile)
print("... done")

output_pkl_name = (os.path.basename(args.input_bagfile)).replace('.bag','.pkl')

if args.output_dir is None: 
    output_pkl_path = os.path.dirname(os.path.realpath(args.input_bagfile))+'/'
else:
    output_pkl_path = os.path.realpath(args.output_dir) +'/'

output_pkl = output_pkl_path + output_pkl_name

if not os.path.exists(output_pkl_path):
    os.makedirs(output_pkl_path)
    tqdm.write("output_dir doesn't exists, creating required path "+output_pkl)


if os.path.isfile(output_pkl):
    tqdm.write("File Already Exits, over-writing - "+output_pkl)
else:
    tqdm.write("Writing to file - "+output_pkl)

with open(output_pkl,'wb') as pickle_file:  
    
    start_time = bag.get_start_time()
    bag_topics = bag.get_type_and_topic_info().topics.keys()
    for ns_prefix in ns_prefixes:
        
        if any([topic.startswith('/'+ns_prefix) for topic in bag_topics]):
            final_data_dict = {}
            ns_dict = {}
            for topic in tqdm(config['topic_list'], unit = 'topic', desc = ns_prefix, position = 1):
                if ns_prefix:
                    topic_name = '/'+ns_prefix+topic[0]
                else:
                    topic_name = topic[0]
                msg_type = topic[1]
                
                variable_name = topic[2]
                msg_dict = exp_dic[msg_type]
                
                data_df = save_topic2df(topic_name, msg_dict , bag)
                
                if data_df is not None:
                    ns_dict[variable_name] = data_df
                                
            if ns_dict:
                if ns_prefix:
                    final_data_dict[ns_prefix] = ns_dict
                    tqdm.write("Wrote namespace: "+ns_prefix+"                           ")
                    
                else:
                    final_data_dict['robot'] = ns_dict
                    tqdm.write("Wrote namespace: robot                           ")
        else: 
            tqdm.write("Notfound! Namespace: "+ns_prefix+'\t\t\t')
        
    sys.stdout.write("Writing picklefile: "+output_pkl+" ..")
    sys.stdout.flush()    
    pickle.dump(final_data_dict, pickle_file)
    print("... done")
 
bag.close()
