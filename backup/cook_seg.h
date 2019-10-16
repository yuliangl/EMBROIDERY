//cook_seg.h -- calculate segment, surface. normal etc   
#ifndef COOK_SEG_H_
#define COOK_SEG_H_
 
#include <string>
#include <vector>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include "cook_basis.h"
 
void SegmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,pcl::PointCloud<pcl::Normal>::Ptr& normal_in,pcl::ModelCoefficients::Ptr& coefficients_plane,pcl::PointIndices::Ptr& inliers_plane,float threshold);
void SegmentCircle3D(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,pcl::PointCloud<pcl::Normal>::Ptr& normal_in,pcl::ModelCoefficients::Ptr& coefficients_circle,pcl::PointIndices::Ptr& inliers_circle,float r_min,float r_max,float threshold);
int ExtractClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,float tolerance,int max_size = 25000,int min_size = 100);
int ExtractClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters,float tolerance,int max_size,int min_size);
int ExtractClustersRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out,std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& clusters,float tolerance,int max_size,int min_size);
void RadiusGrouping(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,float x,float y,float z,float radius,bool state);
void SurfaceReconstr(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointNormal> & mls_points, float radius=0.03);
void SurfaceReconstrRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointNormal> & mls_points, float radius=0.03f);
void NormalEstimate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, float radius=0.005);
 
#endif //ifndef COOK_SEG_H_
