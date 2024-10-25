# iBTC
## iBTC: An Image-assisting Binary and Triangle Combined Descriptor for Place Recognition by Fusing LiDAR and Camera Measurements [RAL2024]

## 1. Introduction
**iBTC** is an improved version of BTC or STD. Rather than using only LiDAR measurements, it also integrates camera measurments for more robust loop detection performance against the scenes challenge for LiDAR measurements like long corridors.
<div align="center">
<img src="https://github.com/hku-mars/iBTC/blob/main/iBTC/description_images/ibtc_pipeline.jpg" width = 98% />
</div>

### 1.1 Paper
Our paper is available on https://ieeexplore.ieee.org/document/10720786 .

### 1.2 Demo Video
Our demo video is available on TODO. .

## 2. Tested Environment
### 2.0 **Ubuntu**
Our operation system is Ubuntu 18.

### 2.1 **ROS**
Following this [ROS Installation](http://wiki.ros.org/ROS/Installation) to install ROS and its additional pacakge. Our ROS version is Melodic.

### 2.2 **ceres**
Our ceres version is ceres-solver-1.14.0.

### 2.3 **Opencv**
Our Opencv version is Opencv-4.5.3 with Opencv-contrib-4.5.3.

### 2.4 **PCL**
Our pcl version is pcl-1.9.

### 2.5 **gcc/g++**
Our gcc/g++ version is gcc/g++ -9.

### 2.6 **Eigen3**

### 2.7 **TBB**
Our tbb: Threading Building Blocks 2019 Update 9

Download it from github and build it in a folder. Link it using hard directory in iBTC CMakelist.txt.

## 3. Build
Clone this repository and catkin_make:
```
cd ~/ws_iBTC/src/
git clone https://github.com/hku-mars/iBTC.git
cd ../
catkin_make
source ~/ws_iBTC/devel/setup.bash
```

## 4. Run our examples
Please download one set of the bag files, camera pose files, imu pose files to your computer. You can download from this link: https://connecthkuhk-my.sharepoint.com/:f:/g/personal/u3008067_connect_hku_hk/ElhEEvOhC0VHvF6xGBH41s8BTBs8WwhnhvHRGrEb5pSG8w?e=QbvxmR (password: iBTC). If the link is not available anymore, please draw an issue to tell us.

Please set the SaveDir so that the program can output evaluation result and debug file to your computer. Please set bag_file imu_pose_fname cam_pose_fname in the launch file so that program find the downloaded files. Otherwise, you may see that the program is not running.

Here is an example how to execute program:

```
cd ~/ws_iBTC/
source devel/setup.bash
roslaunch ibtc loop_test_rgb_avia_privatedata.launch
```
Press any key to start the program.

### 4.1 Run the online example for online LVIO SLAM system
In : a. registered and undistored LiDAR scan messages (/cloud_registered in PointXYZI) or undistored LiDAR scan messages (/cloud_undistort in PointXYZI) + scan pose (/aft_mapped_to_init in nav_msgs::Odometry)
b. undistored image (/camera/image_color_compressed in sensor_msgs::CompressedImage) + image pose (/camera/poses_inworld in nav_msgs::Odometry)
Out: loop closure transformation and pair frame ids (/loop_closure_tranformation in geometry_msgs::PoseWithCovariance)

Please make sure /cloud_undistort and /aft_mapped_to_init are using the same timestamp. Please make sure /camera/image_color_compressed and /camera/poses_inworld are using the same timestamp. Please make sure the output directory SaveDir in the launch file is set. Otherwise, you may see that the program is not running.

## 4.2 Do Percision-Recall evaluation or Recall_1 evaluation
Please check out iBTC/eval directory. Place your result file (like "lc_gt_doc.txt") to the correct place and run calc_pr_overlap_together.m (using Matlab).

## 5. Example results

TODO.

## 6. Report our problems and bugs
We know our packages might not completely stable at this stage, and we are working on improving the performance and reliability of our codes. So, if you have met any bug or problem, please feel free to draw an issue and I will respond ASAP. This code is using SSE2 acceleration and if you meet relevant problems please inform us.

## 7. Acknowledgments
In the development of iBTC, we stand on the shoulders of the following repositories:
STD, BTC.

## License
The source code is released under [GPLv2](http://www.gnu.org/licenses/) license.

If you use any code of this repo in your academic research, please cite **at least one** of our papers:
```
[1] Zou, Zuhao, et al. "iBTC: An Image-assisting Binary and Triangle Combined Descriptor for Place Recognition by Fusing LiDAR and Camera Measurements"
[2] Zou, Zuhao, et al. "LTA-OM: Long-Term Association LiDAR-Inertial
    Odometry and Mapping"
[3] Yuan, Chongjian, et al. "Std: Stable triangle descriptor for 3d place recognition"
[3] Yuan, Chongjian, et al. "Btc: A binary and triangle combined descriptor for 3-d place recognition"
```

For commercial use, please contact me < zuhaozouATyahoo.com > and Dr. Fu Zhang < fuzhangAThku.hk >.
