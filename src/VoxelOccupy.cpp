#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <eigen3/Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include "HiltiPclPointType.h"

#include <unistd.h>
#include <thread>

class Vector3iHash {
public:
	size_t operator()(const Eigen::Vector3i& x) const {
		size_t seed = 0;
		boost::hash_combine(seed, x[0]);
		boost::hash_combine(seed, x[1]);
		boost::hash_combine(seed, x[2]);
		return seed;
	}
};

class PosewithTimestamp
{
public:
    PosewithTimestamp(double dTimestamp, Eigen::Isometry3d Tmi): 
    mdTimestamp(dTimestamp), mTmi(Tmi)
    {}

    PosewithTimestamp()
    {}

public:
    double mdTimestamp;
    Eigen::Isometry3d mTmi;
};

class MapVoxel
{
public:
    MapVoxel()
    {}

public:
    std::vector<Eigen::Vector3d> mvMappoints; // 存储所有在这个体素中的地图点

};

using VoxelMap = std::unordered_map<Eigen::Vector3i, std::shared_ptr<MapVoxel>, Vector3iHash, std::equal_to<Eigen::Vector3i>,
             Eigen::aligned_allocator<std::pair<const Eigen::Vector3i, std::shared_ptr<MapVoxel>>>>;

double dMaxMapVoxelSize = 0.1;
VoxelMap VM;

pcl::PointCloud<pcl::PointXYZI>::Ptr GetPC(std::string path)
{
    std::ifstream cur_file(path);
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    std::string line;
    // 读取    
    if (cur_file.is_open()) {
        std::getline(cur_file, line);  // 跳过第一行

        while (std::getline(cur_file, line)) {
            std::istringstream sline(line);
            std::string field;
            std::vector<std::string> fields;

            while (getline(sline, field, ',')) {
                fields.push_back(field);
            }

            // 处理fields中的数据...
            pcl::PointXYZI point;
            point.x = std::stod(fields[1]);
            point.y = std::stod(fields[2]);
            point.z = std::stod(fields[3]);
            point.intensity = 1.0;
            laser_cloud->push_back(point);
        }
        cur_file.close();
    }
    else
    {
        std::cout << "??" << std::endl;
    }

    return laser_cloud;
}

Eigen::Vector3i ComputeVoxelCoord(Eigen::Vector3d pw)
{
    double loc_xyz[3];
	for(int j=0; j<3; j++)
	{
		loc_xyz[j] = pw[j] / dMaxMapVoxelSize;
		if(loc_xyz[j] < 0)
		{
			loc_xyz[j] -= 1.0;
		}
	}

	Eigen::Vector3i VoxelCoord((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
	return VoxelCoord;
}

// 将点变换到地图系，计算体素坐标然后插入到地图
void UpdateVoxelMap(Eigen::Isometry3d gt_T, pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud)
{
    // 获取旋转和平移向量
    Eigen::Matrix3d R = gt_T.linear();
    Eigen::Vector3d t = gt_T.translation();

    std::cout << "t: " << t.transpose() << std::endl;

    pcl::VoxelGrid<pcl::PointXYZI> DownSizeFilter;
    DownSizeFilter.setLeafSize(0.1, 0.1, 0.1);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_ds_pc(new pcl::PointCloud<pcl::PointXYZI>);
    DownSizeFilter.setInputCloud(laser_cloud);
    DownSizeFilter.filter(*cur_ds_pc);  

    for(int i=0; i<cur_ds_pc->points.size(); i++)
    {
        pcl::PointXYZI point = cur_ds_pc->points[i];
        Eigen::Vector3d pl(point.x, point.y, point.z);

        Eigen::Vector3d pw = R*pl+t;
        Eigen::Vector3i vc = ComputeVoxelCoord(pw);

        auto found = VM.find(vc);
        if(found == VM.end()) // 创建一个新的地图体素
        {
            std::shared_ptr<MapVoxel> pMV(new MapVoxel());
            pMV->mvMappoints.push_back(pw);
            VM.emplace(vc, pMV);
        }
        else // 插入到已有体素中存储
        {
            found->second->mvMappoints.push_back(pw);
        }
    }
}

int main(int argc, char** argv)
{
    
    // 载入估计轨迹
    std::string root_path = ROOT_DIR;
    std::string param_path = root_path+"config/param_voxel_occupy.yaml";
    cv::FileStorage fSettings(param_path, cv::FileStorage::READ);

    std::string gt_pose_file_path = fSettings["lidar_est_path"];
    dMaxMapVoxelSize = fSettings["dMaxMapVoxelSize"];

    // 顺序读取，并存储到下面容器
    std::ifstream ground_truth_file(gt_pose_file_path, std::ifstream::in);
    std::vector<PosewithTimestamp> vGTPosewithTimestamp;

    std::string line;

    // 读取激光雷达真值轨迹
    int index = 0;
    while (std::getline(ground_truth_file, line))
    {
        std::stringstream pose_stream(line);
        std::string s;
        double ts;
        Eigen::Matrix<double, 7, 1> gt_pose;
        for(std::size_t j = 0; j < 8; j++)
        {
            std::getline(pose_stream, s, ' ');
            if(j==0)
                ts = stod(s);
            else
                gt_pose(j-1, 0) = stof(s);
        }
        
        Eigen::Vector3d gt_tmi(gt_pose.topLeftCorner<3, 1>());
        Eigen::Vector4d gt_rmi(gt_pose.bottomLeftCorner<4, 1>());
        std::cout << "gt_rmi: " << gt_rmi.transpose() << std::endl;
        Eigen::Quaterniond gt_qmi(gt_rmi);
        // std::cout << "gt_twl: " << gt_twl.transpose() << std::endl;
        // std::cout  << std::setprecision(12) << "ts: " << ts << std::endl;

        index++;

        Eigen::Isometry3d gt_T; gt_T.setIdentity();
        gt_T.rotate(gt_qmi);
        gt_T.pretranslate(gt_tmi);

        vGTPosewithTimestamp.push_back(PosewithTimestamp(ts, gt_T));
    }

    std::string pc_path = fSettings["dataset_folder"];

    int line_num = 0;
    // 插入第一帧
    Eigen::Isometry3d gt_T; gt_T.setIdentity();
    std::string first_pc_path = pc_path + "Hokuyo_" + std::to_string(line_num) + ".csv";

    // 创建一个pcl点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud = GetPC(first_pc_path);
    UpdateVoxelMap(gt_T, laser_cloud);
    
    line_num++;

    // 顺序载入测量点云，并将点云插入到VoxelMap
    for(int i=0; i<vGTPosewithTimestamp.size(); i++)
    {
        std::cout << "line_num: " << line_num << std::endl;
        std::string cur_pc_path = pc_path + "Hokuyo_" + std::to_string(line_num) + ".csv";
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr cur_laser_cloud = GetPC(cur_pc_path);

        std::cout << "i: " << i << " point num: " << cur_laser_cloud->points.size() << std::endl;

        PosewithTimestamp pTS = vGTPosewithTimestamp[i]; 
        std::cout << "pTS.mdTimestamp: " << pTS.mdTimestamp << std::endl;

        UpdateVoxelMap(pTS.mTmi, cur_laser_cloud);

        line_num++;
    }

    // 显示配准后的地图
    if(1)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr full_map(new pcl::PointCloud<pcl::PointXYZI>);
        for (auto it = VM.begin(); it != VM.end(); ++it) 
        {
            std::vector<Eigen::Vector3d> vMappoints = it->second->mvMappoints;
            for(int i=0; i<vMappoints.size(); i++)
            {
                pcl::PointXYZI p;
                p.x = vMappoints[i].x();
                p.y = vMappoints[i].y();
                p.z = vMappoints[i].z();
                full_map->push_back(p);
            }
        }

         // pcl 显示所有检测到的平面
        // pcl::visualization::PCLVisualizer::Ptr visualizer(
        //     new pcl::visualization::PCLVisualizer("PointCloud Visualizer"));
        // visualizer->setBackgroundColor(0, 0, 0);
        // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>
        //     rgb_color_handler(rgb_pointcloud);
        // visualizer->addPointCloud<pcl::PointXYZRGB>(rgb_pointcloud, rgb_color_handler,
        //                                             "RGB PointCloud");
        // visualizer->setPointCloudRenderingProperties(
        //     pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "RGB PointCloud");
        // visualizer->addCoordinateSystem(5.0);
        pcl::visualization::PCLVisualizer::Ptr visualizer(
            new pcl::visualization::PCLVisualizer("PointCloud Visualizer"));
        visualizer->setBackgroundColor(0, 0, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>
            single_color_handler(full_map, 0, 255, 0);
        visualizer->addPointCloud<pcl::PointXYZI>(full_map, single_color_handler,
                                                    "PointCloud");
        visualizer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "PointCloud");
        visualizer->addCoordinateSystem(5.0);                                                                     

        while (!visualizer->wasStopped()) {
          visualizer->spinOnce(100);
         std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

    }
    
    std::cout << "VoxelMap size: " << VM.size() << std::endl;

    return 0;
}
