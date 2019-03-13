# KittiTools
## **Description**
The tool could be used to convert Kitti velo data into rosbag format or publish a sequence of dataset in 10hz(default)

## Install
This tool requires clang++6.0 and cpp 17/14 at least. And ros tool is considered tobe well installed.
   
    source /opt/ros/<* ros distribution *>/setup.sh
    cd ~/catkin_ws/src 
    git clone * thisrepo *
    mv make_kittitools.sh ~/catkin_ws
    sh make_kittitools.sh
    
> ps::Tools cloud be found at ~/catkin_ws/devel/lib/KittiTools/ 

## Benchmark
Test with `google bench`, a sequence of data(about 4500 files) could be convert into bagfile(8GB) in 1min30s.

## Usage
In bin directory:

    // transfrom a sequence of data into bag file
    Kitti2Bag <bag path> <sequence_path> <times.txt path>    
    
    // publish a sequence of data   no need to store in local.
    kittiPublisher <sequence_path>  <times.txt path>

    // output a piece of data in a bin
    KittiParserTest
