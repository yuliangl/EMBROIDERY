#include "cook_basis.h"
 
/******************************************************************************
Function : PrePassThrough
Description : use the passthrough filter to delete the points from the target cloud
Input : target cloud, min, max and axis for area determination,  state.
Output : new cloud
Others :
        state:
        if true:
            only the points outside of the area will be preserved.
        if false:
            only the points inside of the area will be preserved.
******************************************************************************/
void PrePassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,float min,float max,std::string axis,bool state)
{
  std::cout << " the size of the cloud for passthrough " << cloud_in->points.size() << std::endl;
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud_in);
  pass.setFilterFieldName (axis); //'x' 'y' 'z'
  pass.setFilterLimits (min,max);
  pass.setFilterLimitsNegative (state);
  pass.filter(*cloud_out);
  std::cerr << "PointCloud after filtering(passthrough) has: " << cloud_out->points.size() << " data points." << std::endl;
}


/******************************************************************************
Function : PassThroughRGB
Description : use the passthrough filter to delete the points from the target cloud(XYZRGB)
Input : target cloud(XYZRGB), min, max and axis for area determination,  state.
Output : new cloud(XYZRGB)
Others :
        state:
        if true:
            only the points outside of the area will be preserved.
        if false:
            only the points inside of the area will be preserved.
******************************************************************************/
void PassThroughRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out,float min,float max,std::string axis ,bool state)
{
  std::cout << " the size of the cloud for passthrough " << cloud_in->points.size() << std::endl;
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud_in);
  pass.setFilterFieldName (axis); //'x' 'y' 'z'
  pass.setFilterLimits (min,max);
  pass.setFilterLimitsNegative (state);
  pass.filter(*cloud_out);
  std::cerr << "PointCloud after filtering(passthrough) has: " << cloud_out->points.size() << " data points." << std::endl;
}
 
/******************************************************************************
Function : DownSampleCloud
Description : reduce the amount of points in the target cloud.
Input : target cloud, the size of the voxel for downsampling.
Output : new cloud.
Others :
        As for the function setLeafSize, the shape of the voxel can be set as cuboid, but for the convenience of parameters passing, here only cube has been used as the shape of the voxel.
******************************************************************************/
void DownSampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,float size)
{
  pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
  voxelGrid.setInputCloud(cloud_in);
  voxelGrid.setLeafSize(size,size,size); //0.003f
  voxelGrid.filter(*cloud_out);
  std::cerr << "PointCloud after downsampling has: " << cloud_out->points.size() << " data points." << std::endl;
}


/******************************************************************************
Function : DownSampleCloudRGB
Description : reduce the amount of points in the target cloud.
Input : target cloud(RGB), the size of the voxel for downsampling.
Output : new cloud(RGB).
Others :
        As for the function setLeafSize, the shape of the voxel can be set as cuboid, but for the convenience of parameters passing, here only cube has been used as the shape of the voxel.
******************************************************************************/
void DownSampleCloudRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out,float size)
{
  pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;
  voxelGrid.setInputCloud(cloud_in);
  voxelGrid.setLeafSize(size,size,size); //0.003f
  voxelGrid.filter(*cloud_out);
  std::cerr << "PointCloud after downsampling has: " << cloud_out->points.size() << " data points." << std::endl;
}
 
 
/******************************************************************************
Function : RemoveOutlier
Description : delete the points of noise out from the point cloud.
Input : target cloud, meanK for neighbour points searching and thresh for noise determination.
Output : new cloud.
Others :
        the average distance from the query point to its neighbours will be calculated and this distance will be used to evaluate if it is noise or not.
******************************************************************************/
void RemoveOutlier(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out, int meanK, float thresh)
{
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_in);
  sor.setMeanK(meanK);  //50
  sor.setStddevMulThresh(thresh); //0.1
  sor.filter(*cloud_out);
  std::cerr << "cloud after StatisticalOutlierRemoval has: " << cloud_out->points.size() << std::endl;
}
 
/******************************************************************************
Function : EstimateNormal
Description : estimate the nomals for each points in the target cloud.
Input : target cloud, rangeK for neighbour points searching.
Output : normals.
******************************************************************************/
void EstimateNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normal_out, int rangeK)
{
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setInputCloud (cloud_in);
  ne.setSearchMethod (tree);
  ne.setKSearch (rangeK);
  ne.compute (*normal_out);
}
 
/******************************************************************************
Function : TransformCloud
Description : transform the cloud to the required position.
Input : target cloud, x, y, z for translation, axis and angle for rotation.
Output : new cloud.
******************************************************************************/
void TransformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,float x,float y,float z,std::string axis,float angle)
{
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << x,y,z;
 
  if(axis == "x")
  {transform.rotate (Eigen::AngleAxisf (angle,Eigen::Vector3f::UnitX()));}
  else if(axis == "y")
  {transform.rotate (Eigen::AngleAxisf (angle,Eigen::Vector3f::UnitY()));}
  else if(axis == "z")
  {transform.rotate (Eigen::AngleAxisf (angle,Eigen::Vector3f::UnitZ()));}
  else {std::cout << "input an illegal parameter" << std::endl;}
 
  // Print the transformation
  printf ("\nusing an Affine3f\n");
  std::cout << transform.matrix() << std::endl;
 
  pcl::transformPointCloud (*cloud_in, *cloud_out, transform);
}
 
/******************************************************************************
Function : ExtractCloud
Description : extract the points out from the target cloud according to the given indices.
Input : target cloud, indices, state.
Output : new cloud.
Others :
        state:
        if true :
            outliers.
           false :
            inliers.
******************************************************************************/
void ExtractCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,pcl::PointIndices::Ptr inliers_in,pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,bool state)
{
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud_in);
  extract.setIndices (inliers_in);
  extract.setNegative (state);
  extract.filter (*cloud_out);
}
 
/******************************************************************************
Function : DeleteRedundency
Description : delete the overlap part between two point clouds with the method of radius searching.
Input : two point clouds.
Output : new cloud.
******************************************************************************/
void DeleteRedundency(pcl::PointCloud<pcl::PointXYZ>::Ptr& origin,pcl::PointCloud<pcl::PointXYZ>::Ptr& newCloud)
{
  float radius = 0.001;  // the radius for radius searching.  // 0.00425
  std::vector<int> indices;
  indices.clear();
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (origin);  // set up the kdtree of the origin point cloud.
  for(size_t i=0;i< newCloud->points.size();++i)   // for each points inside of the newCloud, find their neighbour points out from the origin.
  {
    std::vector<int> indices_part;
    std::vector<float> sqr_distances;
    indices_part.clear();
    kdtree.radiusSearch(newCloud->points[i], radius, indices_part, sqr_distances);
    indices.insert(indices.end(), indices_part.begin(), indices_part.end());
  }
 
  sort(indices.begin(),indices.end());
  std::vector<int>::iterator new_end=unique(indices.begin(),indices.end());  // delete the same indices from the vector.
  indices.erase(new_end,indices.end());
 
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
  inliers->indices=indices;
  ExtractCloud(origin,inliers,origin,true);  // delete the points of the overlap parts.
  *origin += *newCloud; // combine two clouds.
}
 
/******************************************************************************
Function : CalculateAngle
Description : calculate the angle of the point relative to the position of the base
Input: position of the target point and the base.
Output : angle
******************************************************************************/
float
CalculateAngle(float x, float y, float base_x, float base_y)
{
  float dx = x - base_x;
  float dy = y - base_y;
  float theta = atan(fabs(dy)/fabs(dx));
  if(dx > 0 && dy > 0)
  {
    ;
  }
  else if(dx < 0 && dy > 0)
  {
    theta = pi - theta;
  }
  else if(dx < 0 && dy < 0)
  {
    theta +=pi;
  }
  else
  {
    theta = 2*pi - theta;
  }
  return theta;
}
 
 
/******************************************************************************
Function : GetAngleRange
Description : calculate the angle_max and angle_min for each points from the clusters relative to a pre-defined point
Input: clusters for calculation, the position of the pre-defined point
Output : angles(max and min)
******************************************************************************/
int
GetAngleRange(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters, std::vector<angleRanges>& angles, float x, float y)
{
  if( clusters.size() == 0 ) // check the size of the clusters first in case of an empty one.
  {
    return 0;
  }
  angles.clear();
  angleRanges angleRange;
  float theta_min,theta_max,theta;
  theta_min = 2 * pi;
  theta_max = 0;
 
  std::vector<angleRanges> angles_temp;
  angles_temp.clear();
  //for each cluster, calculate the maximum and minimum of the angles
  for(unsigned int i = 0; i < clusters.size(); i++)
  {
    for(unsigned int j = 0; j < clusters[i]->points.size(); j++)
    {
      theta = CalculateAngle(clusters[i]->points[j].x, clusters[i]->points[j].y, x, y);
      theta_min = theta_min < theta ? theta_min : theta;
      theta_max = theta_max > theta ? theta_max : theta;
    }
 
    angleRange.theta_min = theta_min;
    angleRange.theta_max = theta_max;
    angles_temp.push_back(angleRange);
    theta_min = 2 * pi;
    theta_max = 0;
  }
 
  SortAngleRange(angles_temp);
  CombineOverlap(angles_temp, angles);
  return 1;
}
 
/******************************************************************************
Function : SortAngleRange
Description : sort the angleRange according to the value of theta_min
Input: the vector of the angleRanges
Output : the sorted vector
******************************************************************************/
void
SortAngleRange(std::vector<angleRanges>& angles_temp)
{
  // sort the ranges according to the value of theta_min
  for(unsigned int i = 0; i < angles_temp.size()-1; i++)
  {
    for(unsigned int j = i+1; j < angles_temp.size(); j++)
    {
      if(angles_temp[i].theta_min > angles_temp[j].theta_min)
      {
        Swap(angles_temp[i].theta_min,angles_temp[j].theta_min);
        Swap(angles_temp[i].theta_max,angles_temp[j].theta_max);
        Swap(angles_temp[i].theta_range,angles_temp[j].theta_range);
      }
    }
  }
}
 
/******************************************************************************
Function : CombineOverlap
Description : Combine two angleRanges if they have overlap parts.
Input: the vector of the angleRange
Output : the combined vector
Others:
        *Be careful that this function can only be used after the angleRange has been sorted from the smallest to the biggest!
******************************************************************************/
bool
CombineOverlap(std::vector<angleRanges>& angles_temp,std::vector<angleRanges>& angles)
{
  angleRanges angleRange;
  // check if there have any overlap parts between the angle ranges.
  for(unsigned int i = 0; i < angles_temp.size(); )
  {
    if(i==angles_temp.size()-1) // if reach the end of the vector, stop searching.
    {
      angleRange.theta_max = angles_temp[i].theta_max;
      angleRange.theta_min = angles_temp[i].theta_min;
      angles.push_back(angleRange);
      return 1;
    }
    int count = 0;
    unsigned int j = i+1;
    while(angles_temp[i].theta_max > angles_temp[j].theta_min) // from the first range, check if theta_max is bigger than theta_min of the latter ranges until it is not satisfied.
    {
        j++;
        count++;
        if(j > angles_temp.size()-1) // be careful about this part.
        {
          break;
        }
    }
    if(count == 0) // if no overlap part, output the information of this range and keep on searching.
    {
      angleRange.theta_max = angles_temp[i].theta_max;
      angleRange.theta_min = angles_temp[i].theta_min;
      angles.push_back(angleRange);
      i++;
    }
    else
    {
      float max_temp = 0;
      for(unsigned int m = i; m < j; m++)  // find the maximum of the angle from the ranges which have overlap parts with each other.
      {
        max_temp = max_temp > angles_temp[m].theta_max ? max_temp : angles_temp[m].theta_max;
      }
      angles_temp[j-1].theta_max = max_temp;    // use the maximum to keep on finding overlap parts.
      angles_temp[j-1].theta_min = angles_temp[i].theta_min;
      i = j-1;
    }
  }
}
 
 
/******************************************************************************
Function : Swap
Description : swap two variables.
Input: two variables
Output : two variables after swapping
******************************************************************************/
void Swap(float& x,float& y)
{
  float temp = x;
  x = y;
  y = temp;
}
