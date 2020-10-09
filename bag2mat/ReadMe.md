# bag2mat

* Takes a rosbag file and converts it to a matlab mat file based on settings specified in a Dictionary and a yaml based config file.
  - Dictionary: The dictionary file specifies the list of variables(of interest) from each message type to be moved to matlab array. Definitions for many standard ros msgs are already included in in the config/msg_dict.yaml. Append your custom msgs definitions to the same file.
    eg `sensor_msgs/NavSatFix: t.to_sec(), m.latitude, m.longitude, m.altitude`
    
  - Configuration: This config file is used to specify topic names that should be transferred to a mat file and their associated variable name.
    Format`-[topic name, message type, name for variable in matfile]`
    See config/test_config.yaml for example.
* Dependencies:
  - rosbag
  - roslib
  - scipy
  - argparse
  - yaml
  - utm
  - logging
  
* Clone this repo in the src folder of your ros workspace and run 
  ```
  catkin_make
  source devel/setup.bash
  usage: bag2matpy [-h] -i INPUT_BAGFILE [-o OUTPUT_DIR] [-c CONFIG_FILE]
                 [-d DICTIONARY] [-s] [-iutm]
  ```
  ```
  Complete List of Arguments:
	
    -h, --help      show this help message and exit
    
    -i INPUT_BAGFILE, --input_bagfile INPUT_BAGFILE
                    Input a rosbag file
    -o OUTPUT_DIR, --output_dir OUTPUT_DIR
                    Output dir for mat file
    -c CONFIG_FILE, --config_file CONFIG_FILE
                    Yaml file specifying topic names to convert
    -d DICTIONARY, --dictionary DICTIONARY
                    Dictionary file which specifies how to read the topic
    -s, --subtract_start_time
                    Boolean flag to specify whether to include 		     
                    offset_time obtained by subtracting bag start_time from all timestamps
    -iutm, --ignore_ll2utm
                    Boolean flag to specify whether to ignore converting lat long msg to utm, defaults to False
  ```
```

```
