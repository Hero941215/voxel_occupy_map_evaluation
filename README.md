# voxel_occupy_map_evaluation

**Implementation of the voxel_occupy_map_evaluation is similar to the paper "2d slam quality evaluation methods", of Filatov et al. (FRUCT 2017).**

## 0. Features
We have developed a map quality evaluation algorithm suitable for 3D LiDAR. Construct a hash voxel map based on estimated trajectories and measurement point clouds. The number of voxels in the map can be used to analyze registration accuracy and map quality.

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
**Ubuntu >= 18.04**

ROS    >= Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2. **PCL && Eigen**
PCL    >= 1.8,   Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

Eigen  >= 3.3.3, Follow [Eigen Installation](http://eigen.tuxfamily.org/index.php?title=Main_Page).

## 2. Build

Clone the repository and catkin_make:

```
    cd ~/$A_ROS_DIR$/src
    git clone https://github.com/Hero941215/voxel_occupy_map_evaluation
    cd voxel_occupy_map_evaluation
    mkdir build
    cd build
    cmake ..
    make -j8
```

## 3. Run
### 3.1. **run demo**

    ./test_imls_sampling /your_pcd_path

## 4. Acknowledgments

Thanks for "2d slam quality evaluation methods"(Filatov et al.).

