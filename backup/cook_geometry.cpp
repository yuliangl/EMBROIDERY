#include "cook_geometry.h"
 
/******************************************************************************
Function : CentroidOfCloud
Description : calculate the centroid of the cloud.
Input : target cloud.
Output : centroid.
******************************************************************************/
void CentroidOfCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,Eigen::Vector4f& centroid_of_model)
{
  CentroidOfCloud(*cloud,centroid_of_model);
}
 
void CentroidOfCloud(pcl::PointCloud<pcl::PointXYZ>& cloud,Eigen::Vector4f& centroid_of_model)
{
  pcl::compute3DCentroid<pcl::PointXYZ> (cloud,centroid_of_model);
  std::cout << "centroid of the model is: " << std::endl;
  std::cout << centroid_of_model << std::endl;
}
 
/******************************************************************************
Function : PCACloud
Description : move the cloud to the origin of the coordinate system with the method of Principle Component Analysis.
Input : target cloud.
Output : new cloud.
******************************************************************************/
void PCACloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr& projection)
{
  pcl::PCA<pcl::PointXYZ> pca (new pcl::PCA<pcl::PointXYZ>);
  pca.setInputCloud(cloud);
  pca.project(*cloud,*projection);
}
 
/******************************************************************************
Function : FindXMinMax
Description : find the min and max value on the x axis.
Input : target cloud.
Output : x_min, x_max.
******************************************************************************/
void FindXMinMax(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,float& x_min,float& x_max)
{
  FindXMinMax(*cloud,x_min,x_max);
}
 
void FindXMinMax(const pcl::PointCloud<pcl::PointXYZ>& cloud,float& x_min,float& x_max)
{
  x_min=FLT_MAX;
  x_max=-FLT_MAX;
  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      x_min = x_min < cloud.points[i].x ? x_min : cloud.points[i].x;
      x_max = x_max > cloud.points[i].x ? x_max : cloud.points[i].x;
    }
  }
  // NaN or Inf values could exist => check for them
  else
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud.points[i].x) || !pcl_isfinite (cloud.points[i].y) || !pcl_isfinite (cloud.points[i].z))
        continue;
      x_min = x_min < cloud.points[i].x ? x_min : cloud.points[i].x;
      x_max = x_max > cloud.points[i].x ? x_max : cloud.points[i].x;
    }
  }
}
 
 
/******************************************************************************
Function : FindMinMax
Description : calculate the min and max value on x,y and z axis.
Input : target cloud.
Output : min and max on each axis.
******************************************************************************/
void FindMinMax(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,std::vector<float>& min_max)
{
  FindMinMax(*cloud,min_max);
}
 
void FindMinMax(const pcl::PointCloud<pcl::PointXYZ>& cloud,std::vector<float>& min_max)
{
  min_max.clear();
  float x_min=FLT_MAX,y_min=FLT_MAX,z_min=FLT_MAX;
  float x_max=-FLT_MAX,y_max=-FLT_MAX,z_max=-FLT_MAX;
  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      x_min = x_min < cloud.points[i].x ? x_min : cloud.points[i].x;
      y_min = y_min < cloud.points[i].y ? y_min : cloud.points[i].y;
      z_min = z_min < cloud.points[i].z ? z_min : cloud.points[i].z;
 
      x_max = x_max > cloud.points[i].x ? x_max : cloud.points[i].x;
      y_max = y_max > cloud.points[i].y ? y_max : cloud.points[i].y;
      z_max = z_max > cloud.points[i].z ? z_max : cloud.points[i].z;
    }
  }
  // NaN or Inf values could exist => check for them
  else
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud.points[i].x) || !pcl_isfinite (cloud.points[i].y) || !pcl_isfinite (cloud.points[i].z))
        continue;
      x_min = x_min < cloud.points[i].x ? x_min : cloud.points[i].x;
      y_min = y_min < cloud.points[i].y ? y_min : cloud.points[i].y;
      z_min = z_min < cloud.points[i].z ? z_min : cloud.points[i].z;
 
      x_max = x_max > cloud.points[i].x ? x_max : cloud.points[i].x;
      y_max = y_max > cloud.points[i].y ? y_max : cloud.points[i].y;
      z_max = z_max > cloud.points[i].z ? z_max : cloud.points[i].z;
    }
  }
  min_max.push_back(x_min);
  min_max.push_back(x_max);
  min_max.push_back(y_min);
  min_max.push_back(y_max);
  min_max.push_back(z_min);
  min_max.push_back(z_max);
}
 
/******************************************************************************
Function : FindZMinMax
Description : calculate the min and max value on the z axis.
Input : target cloud.
Output : z_min, z_max.
******************************************************************************/
void FindZMinMax(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,float& z_min,float& z_max)
{
  FindZMinMax(*cloud,z_min,z_max);
}
 
void FindZMinMax(const pcl::PointCloud<pcl::PointXYZ>& cloud,float& z_min,float& z_max)
{
  z_min=FLT_MAX;
  z_max=-FLT_MAX;
  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      z_min = z_min < cloud.points[i].z ? z_min : cloud.points[i].z;
      z_max = z_max > cloud.points[i].z ? z_max : cloud.points[i].z;
    }
  }
  // NaN or Inf values could exist => check for them
  else
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud.points[i].x) || !pcl_isfinite (cloud.points[i].y) || !pcl_isfinite (cloud.points[i].z))
        continue;
      z_min = z_min < cloud.points[i].z ? z_min : cloud.points[i].z;
      z_max = z_max > cloud.points[i].z ? z_max : cloud.points[i].z;
    }
  }
}
 
/******************************************************************************
Function : GetRadius
Description : get the longest distance from the points to the origin of the coordinate system on the x and y axes and set it as the radius of the target cloud.
Input : target cloud.
Output : radius.
******************************************************************************/
void GetRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,float& radius)
{
  GetRadius(*cloud,radius);
}
 
void GetRadius(pcl::PointCloud<pcl::PointXYZ>& cloud,float& radius)
{
  radius = 0;
  std::vector<float> min_max;
  FindMinMax(cloud,min_max);
  std::vector<float> r;
  r.clear();
  float temp_r;
  for(int i=0;i<4;i++)
  {
    temp_r=fabs(min_max[i]);
    r.push_back(temp_r);
  }
  std::sort(r.begin(),r.end());
  radius = r[3];
  std::cout << "radius:  "  << radius << std::endl;
}
 
/******************************************************************************
Function : DistanceC2C
Description : calculate the distance between two point clouds.
Input : two point clouds.
Output : distance.
******************************************************************************/
float
DistanceC2C(pcl::PointCloud<pcl::PointXYZ>::Ptr target,pcl::PointCloud<pcl::PointXYZ>::Ptr source)
{
  std::vector<int> nn_index (1);
  std::vector<float> nn_distance (1);
 
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud (source);
 
  float error = 0;
 
  for (int i = 0; i < static_cast<int> (target->points.size ()); ++i)
  {
    tree.nearestKSearch (target->points[i], 1, nn_index, nn_distance);
    error += nn_distance[0];
  }
  error /= static_cast<int> (target->points.size ());  // calculate the average of the shortest distances.
  return (error);
}
 
/******************************************************************************
Function : MeanZ
Description : calculate the average value on z axis.
Input : target cloud
Output : MeanZ
******************************************************************************/
float
MeanZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  float sum=0;
  for(int i = 0; i < static_cast<int> (cloud->points.size()); i++)
  {
    sum += cloud->points[i].z;
  }
  return (sum / static_cast<int> (cloud->points.size()) );
}
