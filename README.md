# ros_miku_viewing

## Overview  
See here  
[![](http://img.youtube.com/vi/lXIcyivqUMI/0.jpg)](https://www.youtube.com/watch?v=lXIcyivqUMI)

## How to use
### Prepare
1. Install ROS  
This package is confirmed to work in indigo (however it should works on other distributions)

1. Build the package  
    ```
    $ mkdir ~/catkin_ws
    $ cd ~/catkin_ws
    $ git clone https://github.com/mu-777/ros_miku_viewing.git src
    $ cd ~/catkin_ws
    $ catkin_make
    ```

1. Add MMD dae file  
    ```
    $ mkdir -p ~/catkin_ws/src/ros_miku_viewing/miku_rviz/robot/mmd_dae
    ```
Convert MMD or some models to .dae file and put it to mmd_dae folder    
(reference: http://nullorempry.jimdo.com/work-list/mmd/pmdeditor/)  
Change ``<property name="modelDataFileName" value="Tda-style_miku.dae"/>`` @ ``common/launch/parameter.launch`` to your .dae file name

1. Install Android apps  
Install the app using ``android_imu_publisher/imu_publisher/imu_publisher-release.apk``

### Run
1. Connect your phone and PC to the same network  
You can use your phone's tethering. In this case, you should disconnect mobile network(e.g. 4G, LTE or something)

1. Launch ROS node
    ```
    $ export ROS_IP=<your PC IP address>
    $ roscore
    ```
    @ another terminal
    ```
    $ export ROS_IP=<your PC IP address>
    $ cd ~/catkin_ws
    $ source devel/setup.bash
    $ roslaunch roslaunch common main_tda_miku.launch
    ```

1. Run Android app  
Launch your app and change ``http://localhost:11311`` to ``http://<your PC IP address>:11311`` at Master URI.  
Push 'connect' button.  
Run your camera app and see your model!







