#ifndef COOK_GEOMETRY_H_
#define COOK_GEOMETRY_H_
 
#include <vector>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <cmath>
#include <algorithm>
 
void FindXMinMax(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,float& x_min,float& x_max);
void FindXMinMax(const pcl::PointCloud<pcl::PointXYZ>& cloud,float& x_min,float& x_max);
void FindMinMax(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,std::vector<float>& min_max);
void FindMinMax(const pcl::PointCloud<pcl::PointXYZ>& cloud,std::vector<float>& min_max);
void FindZMinMax(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,float& z_min,float& z_max);
void FindZMinMax(const pcl::PointCloud<pcl::PointXYZ>& cloud,float& z_min,float& z_max);
void GetRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,float& radius);
void GetRadius(pcl::PointCloud<pcl::PointXYZ>& cloud,float& radius);
void PCACloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr& projection);
void CentroidOfCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,Eigen::Vector4f& centroid_of_model);
void CentroidOfCloud(pcl::PointCloud<pcl::PointXYZ>& cloud,Eigen::Vector4f& centroid_of_model);
float DistanceC2C(pcl::PointCloud<pcl::PointXYZ>::Ptr target,pcl::PointCloud<pcl::PointXYZ>::Ptr source);
float MeanZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
 
#endif // #ifndef COOK_GEOMETRY_H_
