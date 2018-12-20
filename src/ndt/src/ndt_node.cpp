#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/io/pcd_io.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ndt");
    ros::NodeHandle nh;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr laser1(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr laser2(new pcl::PointCloud<pcl::PointXYZ>());

    std::string fileName1, fileName2;
    nh.param(std::string("laser1"), fileName1, std::string("200.pcd"));
    nh.param(std::string("laser2"), fileName2, std::string("201.pcd"));

    pcl::io::loadPCDFile(fileName1, *laser1);
    pcl::io::loadPCDFile(fileName2, *laser2);

    //下采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_laser(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> filters;
    filters.setLeafSize(0.2, 0.2, 0.2);
    filters.setInputCloud(laser1);
    filters.filter(*filter_laser);

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setTransformationEpsilon(0.0001);
    ndt.setStepSize(0.1);
    ndt.setResolution(1.0);
    ndt.setMaximumIterations(1000);
    ndt.setInputSource(filter_laser);
    ndt.setInputTarget(laser2);
   
    // 先验知识(通过外部测量两个雷达的相对位置)
    Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
    guess(1, 3) = -2.6;
    
    ndt.align(*laser1, guess);
    std::cout << ndt.getFinalTransformation() << std::endl;
    return 0;
}
