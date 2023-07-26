# Dataset Tools (TfNSW dataset)

This packages contains tools required to work with the TfNSW dataset. This package can be built locally and run on your machine by following the steps in the [Local](#local) section below.

## Local

To build on a computer with ROS installed (tested on [Melodic](http://wiki.ros.org/melodic)), the following should work


1. Apply a bug fix known to effect ROS Melodic  
   ```bash
   sudo mv /usr/include/flann/ext/lz4.h /usr/include/flann/ext/lz4.h.bak
   sudo mv /usr/include/flann/ext/lz4hc.h /usr/include/flann/ext/lz4.h.bak
   sudo ln -s /usr/include/lz4.h /usr/include/flann/ext/lz4.h
   sudo ln -s /usr/include/lz4hc.h /usr/include/flann/ext/lz4hc.h
   ```  

2. Update package sources list and install packages that are not recognised in rosdep (to be fixed in a later version)  
   ```bash
   sudo apt update && apt install -y ros-melodic-tf-conversions ros-melodic-tf2-sensor-msgs ros-melodic-random-numbers
   ```  

3. Clone this repository and update submodules  

   **Note:** Replace ```~/catkin_ws/src``` with the absolute path to the source directory of your catkin workspace  

   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/chinitaberrio/dataset_tools.git
   cd dataset_tools
   ```  

4. Source your catkin workspace and install dependencies  

   **Note:** Replace ```~/catkin_ws``` with the absolute path of your catkin workspace  

   ```bash
   source ~/catkin_ws/devel/setup.bash
   rosdep install --from-paths ~/catkin_ws/src/dataset_tools --ignore-src --rosdistro=ROS_DISTRO
   ```  

5. Build the package  

   **Note:** Replace ```~/catkin_ws``` with the absolute path of your catkin workspace  
   
   ```bash
   cd ~/catkin_ws
   catkin build
   ```  

6. Launch the GUI  
   ```bash
   roslaunch dataset_playback run.launch
   ```  

   ![Dataset Tools GUI](https://gitlab.acfr.usyd.edu.au/its/dataset_metapackage/-/raw/master/screenshots/gui.png)  

7. In a new terminal, launch RViZ  
   ```bash
   rosrun rviz rviz
   ```  

8. Use the GUI to select a .bag file for playback  

   ![Select a file for playback](https://gitlab.acfr.usyd.edu.au/its/dataset_metapackage/-/raw/master/screenshots/dialog.png)  

   ![Select a file for playback](https://gitlab.acfr.usyd.edu.au/its/dataset_metapackage/-/raw/master/screenshots/file-selected.png)  

9. Add topics you want to visualise in RViZ, e.g. ```/sekonix_camera/port_a_cam_0/image_color```, ```/ouster/points``` (you may also want to change the Fixed Frame to ```base_link```)  

10. Use the GUI to play/pause/stop the .bag file playback or use the scrubber at the bottom of the GUI to move around the .bag file faster  
   
    ![Playback in RViZ](https://gitlab.acfr.usyd.edu.au/its/dataset_metapackage/-/raw/master/screenshots/rviz.png)  

An extract of playback from a dataset file (Week 1) is shown below  

![Playback in RViZ](https://gitlab.acfr.usyd.edu.au/its/dataset_metapackage/-/raw/master/screenshots/rviz-large.gif)  



