# bag2pandasdf
Convert rosbags to pandas dataframes for easy data manipulation and plotting

* Takes a rosbag file and converts it to a pndas dataframe pickle file based on settings specified in a Dictionary and a yaml config file.
  - Dictonary: The dicionary file specified which variables from each message type should be converted to a dataframe columns.
    eg `sensor_msgs/NavSatFix: t.to_sec(), m.latitude, m.longitude, m.altitude`
    See config/bag2mat_dic.yaml for full example.
  - Configuration: This file specified which topic names should be transferred to dataframes and under which variable name.
    A list of `-[topic name, message type, name for variable in matfile]`
    See config/bag2mat_config.yaml for full example.
* Currently needs the following packages:
  - rosbag
  - pandas
  - numpy
  - pyyaml
  - tqdm

* Usage
```
usage: bag2pandasdf.py [-h] -i INPUT_BAGFILE [-o OUTPUT_DIR] [-c CONFIG_FILE]
                       [-d DICTIONARY]

Convert ROSBAG to pandas dataframe pkl file.

A simple script that can converts rosmessages in a rossbag to pandas dataframes in a .pkl file.                                  
Define the translation of each message type in a dictionary file and use a config file to specify all the topics to be converted..

arguments:
  -i INPUT_BAGFILE, --input_bagfile INPUT_BAGFILE
                        Input rosbag file to input
                        
optional arguments:
  -h, --help            show this help message and exit
  -o OUTPUT_DIR, --output_dir OUTPUT_DIR
                        Output dir for pkl file
  -c CONFIG_FILE, --config_file CONFIG_FILE
                        Yaml file which specifies topic names to convert
  -d DICTIONARY, --dictionary DICTIONARY
                        Dictionary file which specifies how to read the topic
```
## Output pkl file data structure
The resulting output file while have a structure as shown below. This can be directly used with the [our_qtplot](https://github.com/neufieldrobotics/our_qt_plot) data plotting GUI.
![Alt text](https://g.gravizo.com/source/svg/input_ds?https%3A%2F%2Fraw.githubusercontent.com%2Fneufieldrobotics%2Fbag2pandasdf%2Fmaster%2FREADME.md)
<details> 
<summary></summary>
input_ds
digraph G {
"data_file.pkl" -> "full_data_dict";
"full_data_dict" -> "namespace1 dict" [color="orange"];
"full_data_dict" -> "namespace2 dict" [color="orange"];
n1t1 [label="topic1 pandas df"];
n1t2 [label="topic2 pandas df"];
n2t1 [label="topic1 pandas df"];
n2t2 [label="topic2 pandas df"];
"namespace1 dict" -> n1t1 [color="green"];
"namespace1 dict" -> n1t2 [color="green"];
"namespace2 dict" -> n2t1 [color="green"];
"namespace2 dict" -> n2t2 [color="green"];
n1t1t [label="Time\n(index)"];
n1t1f1 [label="Field1\ncolumn"];
n1t1f2 [label="Field2\ncolumn"];
n1t2t [label="Time\n(index)"];
n1t2f1 [label="Field1\ncolumn"];
n1t2f2 [label="Field2\ncolumn"];
n2t1t [label="Time\n(index)"];
n2t1f1 [label="Field1\ncolumn"];
n2t1f2 [label="Field2\ncolumn"];
n2t2t [label="Time\n(index)"];
n2t2f1 [label="Field1\ncolumn"];
n2t2f2 [label="Field2\ncolumn"];
n1t1 -> n1t1t [color="blue"];
n1t1 -> n1t1f1 [color="blue"];
n1t1 -> n1t1f2 [color="blue"];
n1t2 -> n1t2t [color="blue"];
n1t2 -> n1t2f1 [color="blue"];
n1t2 -> n1t2f2 [color="blue"];
n2t1 -> n2t1t [color="blue"];
n2t1 -> n2t1f1 [color="blue"];
n2t1 -> n2t1f2 [color="blue"];
n2t2 -> n2t2t [color="blue"];
n2t2 -> n2t2f1 [color="blue"];
n2t2 -> n2t2f2 [color="blue"];
}
input_ds
</details>
