#pragma once
#include<iostream>
#define BOOST_TYPEOF_EMULATION  //can slove problem of typeof_impl.hpp(125)
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/vfh.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <time.h>

struct MyPoint3d
{
	double x;
	double y;
	double z;
};

int getAnotherPointCloudFromOne(const char* ptr_pcl_file1, const char* ptr_pcl_file2, const Eigen::Matrix4d& rt);
int calculateError(const char* ptr_pcl_file1, const char* ptr_pcl_file2, double& error);
int calculateNumble(const char* ptr_pcl_file1, int& num);
void matrix2angle(Eigen::Matrix4f &result_trans, Eigen::Vector3f &result_angle);

 void removeNAN(pcl::PointCloud<pcl::PointXYZ>::Ptr &pc);
 pcl::PointCloud<pcl::PointXYZ>::Ptr downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr &pc, double gridSize);
 pcl::PointCloud<pcl::Normal>::Ptr pclNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr &pc, double radius);
 pcl::PointCloud<pcl::FPFHSignature33>::Ptr __stdcall fpfh(pcl::PointCloud<pcl::PointXYZ>::Ptr &pc, pcl::PointCloud<pcl::Normal>::Ptr &pc_normals, double radius); //FPFH
 void uaiRegisration(const char* file1, const char* file2, float T[4][3], const int ICP_ITERATIONS, const double SAMPLING_RADIUS);