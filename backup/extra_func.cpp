//ifndef __EXTRA_FUNC_H_
//define __EXTRA_FUNC_H_
 
#include "cook_seg.h"
#include "cook_basis.h"
#include "cook_io.h"
 
 
struct PlateInformation
{
  float x;           //0
  float y;           //1
  float radius;      //2
  float top2desk;    //3
  float bottom2deskC;//4
  //float topself;     //4
  float bottom2desk; //5
  float bottomself;  //6
  float angle;       //7
  int target;        //8
  float bottomselfC; //9
 
  float food;
  int order;
  int extracted;
};
 
void DeleteCircle(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, float x, float y, float radius, bool state = true);
void ExtractCircle(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, float x, float y,float radius,bool state = true);
void ExtractRing(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, float x, float y, float radius_min, float radius_max);
bool GetObstacleAngles(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, float x, float y, std::vector<angleRanges>& angles, float tolerance);
bool GetDirection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, float x,float y,float z,float radius, float& angle,float tolerance);
float NormalizeAngle(float angle_in);
float HeightDetermination(float r);
void CheckOrderWithAngle(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_angle, std::vector<PlateInformation>& infor_pre, std::vector<PlateInformation>& infor_af);
void checkFood(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,PlateInformation& infor);
 
bool GetSizeOrder(std::vector<PlateInformation>& infor);
void GetSequence(std::vector<PlateInformation>& infor_af);
 

//include "extra_func.h"
 
/*
Aim: Extract the points of the plate neighbour out (range size: [radius_min,radius_max])
*/
 
 
 
/******************************************************************************
Function : DeleteCircle
Description : Delete the point cloud of a circle from the target.
Input : target cloud, center of the circle, radius  and  state.
Output : the new cloud
Others :
        for state:
            if true:
                delete the circle
            if false:
                keep the circle and delete the points outside of the circle
******************************************************************************/
void
DeleteCircle(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, float x, float y, float radius, bool state)
{
  float thresh = pow(radius,2.0);
  std::vector<int> indices;
  indices.clear();
  for(unsigned int i = 0; i < cloud_in->points.size(); i++)
  {
    if( fabs(cloud_in->points[i].x - x) > radius || fabs(cloud_in->points[i].y - y) > radius )
    {
      ;
    }
    else if ( pow(fabs(cloud_in->points[i].x - x), 2.0) + pow(fabs(cloud_in->points[i].y - y), 2.0) > thresh )
    {
      ;
    }
    else
    {
      indices.push_back(i);
    }
  }
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
  inliers->indices = indices;
  ExtractCloud(cloud_in,inliers,cloud_out,state);
}
 
/******************************************************************************
Function : ExtractCircle
Description : Extract the point cloud of a circle out from the target.
Input : target cloud, center of the circle, radius  and  state.
Output : the new cloud
Others :
        for state:
            if true:
                only the points inside of the circle will be left
            if false:
                the points outside of the circle will be left
******************************************************************************/
void
ExtractCircle(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, float x, float y, float radius, bool state)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::cout << " cloud size of the target for circle extracting " << cloud_in->points.size() << std::endl;
 
  if(state == true)     // if only want the points inside of the circle to be preserved, the points outside of the circumscribing square of the circle can be delete first.
  {
    PrePassThrough(cloud_in,cloud,x-radius,x+radius,"x",false);
    //ShowCloud(cloud);
    PrePassThrough(cloud,cloud,y-radius,y+radius,"y",false);
    //ShowCloud(cloud);
  }
  else                 // the points outside of the circumscribing square of the circle can not be deleted if the points outside of the circle should be preserved.
  {
    cloud = cloud_in;
  }
 
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
  std::vector<int> indices_out;
  indices_out.clear();
  float distance;
  float threshold = pow(radius,2.0);     // use the square of the radius as the threshold for sorting.
  for(unsigned int i = 0; i < cloud->points.size(); i++)
  {
    // calculate the distance from each points to the center of the circle and compare with the threshold to sort the point cloud.
    distance = pow(fabs(cloud->points[i].x - x), 2.0) + pow(fabs(cloud->points[i].y - y), 2.0);
    if(distance > threshold)
    {
      indices_out.push_back(i);   // here, only the index of the points outside of the circle will be stored in the vector.
    }
  }
  inliers->indices = indices_out;
  ExtractCloud(cloud,inliers,cloud_out,state);
}
 
 
/******************************************************************************
Function : ExtractRing
Description : Extract the points inside of the ring out from the target.
Input: target cloud, center of the ring, radius_max and radius_min
Output : point cloud of the ring
******************************************************************************/
void
ExtractRing(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, float x, float y, float radius_min, float radius_max)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //std::cout << " cloud size of the target " << cloud_in->points.size() << std::endl;
  //ShowCloud(cloud_in);
  ExtractCircle(cloud_in,cloud,x,y,radius_max,true);      // Extract the points which are inside of the bigger circle.
  //std::cout << "cloud of the outer circle" << std::endl;
  //ShowCloud(cloud);
  ExtractCircle(cloud,cloud_out,x,y,radius_min,false);    // Extract the points which are outside of the smaller circle.
  //std::cout << " cloud size of the ring " << cloud->points.size() << std::endl;
  //std::cout << "cloud of the ring" << std::endl;
  //ShowCloud(cloud_out);
  if(cloud_out->points.size() > 10)
  {
    //WriteCloud(cloud_out,"ring.pcd");
    ;
  }
}
 
 
 
/******************************************************************************
Function : GetObstacleAngles
Description : get the angleRange of the obstacle in the point cloud.
Input: target cloud, center of the ring, tolerance for segmentation
Output : angleRanges
******************************************************************************/
bool
GetObstacleAngles(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, float x, float y, std::vector<angleRanges>& angles, float tolerance)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_upper (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lower (new pcl::PointCloud<pcl::PointXYZ>);
 
  PrePassThrough(cloud_in,cloud_upper, y,y + 0.2, "y");
  if(cloud_upper->points.size() > 10)
  {
    //ShowCloud(cloud_upper);
    //WriteCloud(cloud_upper,"cloud_upper.pcd");
    std::cout << "10+ points inside of the upper part." << std::endl;
  }
  else
  {
    std::cout << "no points inside of the upper part." << std::endl;
  }
 
  PrePassThrough(cloud_in,cloud_lower,y - 0.2, y, "y");
  if(cloud_lower->points.size() > 10)
  {
    //ShowCloud(cloud_lower);
    //WriteCloud(cloud_lower,"cloud_lower.pcd");
    std::cout << "10+ points inside of the lower part." << std::endl;
  }
  else
  {
    std::cout << "no points inside of the lower part." << std::endl;
  }
 
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_upper;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_lower;
  // extract the clusters which can be considered as the obstacles during picking out.
  ExtractClusters(cloud_upper, clusters_upper, tolerance, 100000, 10);   // tolerance is the distance for segmentation
  ExtractClusters(cloud_lower, clusters_lower, tolerance, 100000, 10);
  if(clusters_lower.size() == 0 && clusters_upper.size() == 0)
  {
    // no obstacles
    std::cout << "no obstacles" << std::endl;
    return 0;
  }
  else
  {
    std::vector<angleRanges> angles_upper;
    std::vector<angleRanges> angles_lower;
 
    GetAngleRange(clusters_upper,angles_upper,x,y);
    GetAngleRange(clusters_lower,angles_lower,x,y);
    angles.insert(angles.end(),angles_upper.begin(),angles_upper.end());
    angles.insert(angles.end(),angles_lower.begin(),angles_lower.end());
    return 1; // with obstacles
  }
}
 
 
/******************************************************************************
Function : GetDirection
Description : Calculate the best angle for plates catching
Input: target cloud, location of the plate, tolerance for cluster extraction
Output : angle for catching
******************************************************************************/
bool
GetDirection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, float x,float y,float z,float radius, float& angle,float tolerance)
{
  float interval[9] = {};
  float h_tool = HeightDetermination(radius);
 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::PointCloud<pcl::PointXYZ> > clouds;
  clouds.clear();
 
  int interval_num;
  if(radius > 0.152) // 0.16
  {
    std::cout << "the plate is too big to be caught!" << std::endl;
    angle = pi;
    return 0;
  }
  else if(radius > 0.142)  // 0.15
  {
    interval[0] = radius+0.008;
    interval[1] = 0.16;
    interval_num = 1;
  }
  else if(radius > 0.132)  // 0.14
  {
    interval[0] = radius+0.008;
    interval[1] = 0.15;
    interval[2] = 0.16;
    interval_num = 2;
  }
  else if(radius > 0.122)   // 0.13
  {
    interval[0] = radius+0.008;
    interval[1] = 0.14;
    interval[2] = 0.15;
    interval[3] = 0.16;
    interval_num = 3;
  }
  else if(radius > 0.112)   //0.12
  {
    interval[0] = radius+0.008;
    interval[1] = 0.13;
    interval[2] = 0.14;
    interval[3] = 0.15;
    interval[4] = 0.16;
    interval_num = 4;
  }
  else if(radius > 0.102)  //0.11
  {
    interval[0] = radius + 0.008;
    interval[1] = 0.12;
    interval[2] = 0.13;
    interval[3] = 0.14;
    interval[4] = 0.15;
    interval[5] = 0.16;
    interval_num = 5;
  }
  else if(radius > 0.092)  //0.10
  {
    interval[0] = radius + 0.008;
    interval[1] = 0.11;
    interval[2] = 0.12;
    interval[3] = 0.13;
    interval[4] = 0.14;
    interval[5] = 0.15;
    interval[6] = 0.16;
    interval_num = 6;
  }
  else if(radius > 0.082)  //0.09
  {
    interval[0] = radius + 0.008;
    interval[1] = 0.10;
    interval[2] = 0.11;
    interval[3] = 0.12;
    interval[4] = 0.13;
    interval[5] = 0.14;
    interval[6] = 0.15;
    interval[7] = 0.16;
    interval_num = 7;
  }
  else                     // 0.08
  {
    interval[0] = radius + 0.008;
    interval[1] = 0.09;
    interval[2] = 0.10;
    interval[3] = 0.11;
    interval[4] = 0.12;
    interval[5] = 0.13;
    interval[6] = 0.14;
    interval[7] = 0.15;
    interval[8] = 0.16;
    interval_num = 8;
  }
 
  for(int i = 0; i < interval_num; i++)
  {
    if(i == 0 && interval[1]-interval[0] < 0.001)
    {
      continue;
    }
    std::cout << " ------------ " << i << " ring -------------" << std::endl;
    float delta = sqrt(0.18*0.18 - interval[i] * interval[i]);
    std::cout << " interval : " << interval[i] << " ; " << interval[i+1] << std::endl;
    std::cout << "delta : " << delta << std::endl;
    std::cout << "z : " << z << std::endl;
    std::cout << "h_tool : " << h_tool << std::endl;
    ExtractRing(cloud_in,cloud,x,y,interval[i],interval[i+1]);
    //ShowCloud(cloud);
    PrePassThrough(cloud,cloud,z + h_tool - delta - 0.02,z+h_tool+1,"z");            // h_tool is in need
    std::cout << "here shows the height for the filter : " << z+h_tool-delta-0.02 << std::endl;
    //std::cout << "here shows the " << i << " interval point cloud" << std::endl;
    //ShowCloud(cloud);
    if(cloud->points.size() > 10)
    {
      clouds.push_back(*cloud);
      //std::cout << "cloud point size (should be) : " << cloud->points.size() << std::endl;
      //std::cout << "clouds[0] point size : " << clouds[0].points.size() << std::endl;
      WriteCloud(cloud,"obstacle.pcd");
      std::cout << "clouds size : " << clouds.size() << std::endl;
    }
  }
 
  if(clouds.size() == 0)   // if amount of the points left is smaller than 10, it can be seen as no obstacle.
  {
    angle =  pi/2;   // this angle is set as 90 deg, because the initial angle of the tool is 90 deg.
    return 1;
  }
 
  std::vector<angleRanges> angles_all;
  angles_all.clear();
  for(unsigned int i = 0; i < clouds.size(); i++)
  {
    std::cout << "----- " << i << " -------------" << std::endl;
    std::cout << "cloud points amount : " << clouds[i].points.size() << std::endl;
    GetObstacleAngles(clouds[i].makeShared(),x,y,angles_all,tolerance);
  }
  if(angles_all.size() == 0)
  {
    angle = pi/2;
    return 1;
  }
 
  SortAngleRange(angles_all);
  std::vector<angleRanges> angles_unique;
  angles_unique.clear();
  CombineOverlap(angles_all,angles_unique);
 
  std::vector<angleRanges> angles_blank;
  angles_blank.clear();
  angleRanges angle_temp;
 
  // transfer the obstacle angle ranges into blank part angle ranges.
  if(angles_unique.size() != 0)
  {
    angle_temp.theta_min = angles_unique[angles_unique.size()-1].theta_max - 2 * pi;
    angle_temp.theta_max = angles_unique[0].theta_min;
    angles_blank.push_back(angle_temp);
    for(unsigned int i = 0; i < angles_unique.size()-1; i++)
    {
      angle_temp.theta_min = angles_unique[i].theta_max;
      angle_temp.theta_max = angles_unique[i+1].theta_min;
      angles_blank.push_back(angle_temp);
    }
  }
  else
  {
    angle = pi/2;
    return 1;
  }
 
  // calculate the width for each range.
  for(unsigned int i = 0; i < angles_blank.size(); i++)
  {
    angles_blank[i].theta_range = angles_blank[i].theta_max - angles_blank[i].theta_min;
  }
 
  float angle_inneed;
  angle_inneed = asin(0.045/radius) * 2;
  std::cout << "angle_inneed : " << angle_inneed << std::endl;
 
  // delete angles which are smaller than angel_inneed
  std::vector<angleRanges> angles_poss;
  angles_poss.clear();
  for(unsigned int i = 0; i < angles_blank.size(); i++)
  {
    if(angles_blank[i].theta_range > angle_inneed)
    {
      angles_poss.push_back(angles_blank[i]);
    }
  }
  std::cout <<"delete small intervals " << std::endl;
 
  if(angles_poss.size() == 0)
  {
    angle = pi;
    return 0;
  }
 
  // re-arrange the ranges according to the size of the range.
  for(unsigned int i = 0; i < angles_poss.size()-1; i++)
  {
    for(unsigned int j = i + 1; j < angles_poss.size(); j++)
    {
      if(angles_poss[i].theta_range < angles_poss[j].theta_range)
      {
        Swap(angles_poss[i].theta_max,angles_poss[j].theta_max);
        Swap(angles_poss[i].theta_min,angles_poss[j].theta_min);
        Swap(angles_poss[i].theta_range,angles_poss[j].theta_range);
      }
    }
  }
  std::cout << "rearrange intervals " << std::endl;
 
  float angle_result;
  // check the first interval
  std::cout << "check the first interval " << std::endl;
  if(angles_poss[0].theta_range > pi + angle_inneed)   // check if the biggest interval has already covered the position to catch.
  {
    angle_result = NormalizeAngle((angles_poss[0].theta_max + angles_poss[0].theta_min)/2 - pi/2);
    std::cout << "angle_result(first interval): " << angle_result << std::endl;
    if(angle_result > 1.5 * pi)
    {
      angle = angle_result - 2 * pi;
    }
    else if(angle_result > 0.5 * pi)
    {
      angle = angle_result - pi;
    }
    else
    {
      angle = angle_result;
    }
    std::cout << "angel_result(first interval)(final) : " << angle << std::endl;
    return 1;
  }
 
  //check opposite side
  std::cout << "check the opposite side" << std::endl;
  float oppo_max;
  float oppo_min;
  float final_max;
  float final_min;
  float final_range;
  for(unsigned int i = 0; i < angles_poss.size(); i++)
  {
    std::cout << "  " << i << "  interval----------" << std::endl;
    oppo_max = NormalizeAngle(pi + angles_poss[i].theta_max);
    oppo_min = NormalizeAngle(pi + angles_poss[i].theta_min);
    for(unsigned int j = 0; j < angles_poss.size(); j++)
    {
      if(i == j)
      {
        ;
      }
      else
      {
        std::cout << "  --- " << j << "interval ------------" <<std::endl;
 
        float diff;
        diff = NormalizeAngle(angles_poss[j].theta_max - oppo_min);
        if(diff > pi )
        {
          break;
        }
        else if (diff < angles_poss[i].theta_range)
        {
          final_max = angles_poss[j].theta_max;
        }
        else
        {
          final_max = oppo_max;
        }
 
        float diff1,diff2;
        diff1 = NormalizeAngle(final_max - oppo_min);
        diff2 = NormalizeAngle(final_max - angles_poss[j].theta_min);
        final_range = diff1 > diff2 ? diff2 : diff1;
        final_min = diff1 > diff2 ? angles_poss[j].theta_min : oppo_min;
        std::cout << "final_range : " << final_range << std::endl;
        if(final_range > angle_inneed)
        {
          angle_result = NormalizeAngle((final_max + final_min)/2);
          std::cout << "angle_result : " << angle_result << std::endl;
          if(angle_result > 1.5 * pi)
          {
            angle = angle_result - 2 * pi;
          }
          else if(angle_result > 0.5 * pi)
          {
            angle = angle_result - pi;
          }
          else
          {
            angle = angle_result;
          }
          return 1;
        }
        else
        {
          ;
        }
      }
    }
  }
  angle = pi;
  return  0;
}
 
float
HeightDetermination(float r)
{
  double width_p;
  double theta;
  width_p = sqrt(r * r - 0.044 * 0.044);
  theta = asin((width_p - 0.00325) / 0.174);
  double height_h;
  height_h = 0.174 * cos(theta) + 0.02 * sin(theta + 3/180 * pi);
  return height_h;
}
 
 
void
CheckOrderWithAngle(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_angle, std::vector<PlateInformation>& infor_pre, std::vector<PlateInformation>& infor_af)
{
  std::vector<PlateInformation> infor_temp;
  infor_temp.clear();
 
  for(unsigned int i = 0; i < infor_pre.size(); i++)
  {
    std::cout << " check the " << i << " th information element" << std::endl;
    std::cout << "here show the top and bottom to the desk : " << infor_pre[i].top2desk << " ; " << infor_pre[i].bottom2desk << std::endl;
    infor_pre[i].extracted = GetDirection(cloud_angle,infor_pre[i].x,infor_pre[i].y,infor_pre[i].bottom2desk,infor_pre[i].radius,infor_pre[i].angle,0.01);
    std::cout << " finish getting the angle" << std::endl;
    infor_temp.push_back(infor_pre[i]);
    if(infor_pre[i].extracted == 1)
    {
      std::cout << "succeed in deleting " << std::endl;
      infor_af.push_back(infor_pre[i]);
      DeleteCircle(cloud_angle,cloud_angle,infor_pre[i].x,infor_pre[i].y,infor_pre[i].radius + 0.006);         // delete the points of the plate from the angle_detecting Point Cloud.
      //ShowCloud(cloud_angle);
      WriteCloud(cloud_angle,"delete.pcd");
      int flag = 1;
      while(flag)
      {
        flag =0;
        for(int j = infor_temp.size() -1; j >= 0; j--)
        {
          if(infor_temp[j].extracted == 0)  // hasn't been extracted
          {
            infor_temp[j].extracted = GetDirection(cloud_angle,infor_temp[j].x,infor_temp[j].y,infor_temp[j].bottom2desk,infor_temp[j].radius,infor_temp[j].angle,0.01);
            if(infor_temp[j].extracted == 0)
            {
              ;  // still can't be caught
            }
            else
            {
              infor_af.push_back(infor_temp[j]);
              DeleteCircle(cloud_angle,cloud_angle,infor_temp[j].x,infor_temp[j].y,infor_temp[j].radius + 0.006);
              WriteCloud(cloud_angle,"delete.pcd");
              //ShowCloud(cloud_angle);
              flag = 1;                      // once their exist one plate that can newly be caught
            }
          }
          else                              // has already been extracted
          {
            ;
          }
        }
      }
    }
  }
}
 
 
bool
GetSizeOrder(std::vector<PlateInformation>& infor)
{
  if(infor.size() < 1)
  {
    return 0;
  }
  else if (infor.size() < 2)
  {
    infor[0].order = 0;
    return 1;
  }
 
  for(unsigned int i = 0; i < infor.size(); i++)
  {
    int order = 0;
    for(unsigned int j = 0; j < infor.size(); j++)
    {
      if(j == i)
      {
        continue;
      }
      if(infor[i].radius < infor[j].radius)
      {
        order++;
      }
    }
    infor[i].order = order;
  }
 
  int i = 1;
  while(i)
  {
    i=0;
    for(unsigned int j = 0; j < infor.size(); j ++)
    {
      for(unsigned int k = 0; k < infor.size(); k++)
      {
        if(j != k && infor[j].order == infor[k].order)
        {
          infor[j].order ++;
          i = 1;
        }
      }
    }
  }
 
  return 1;
}
 
 
 
 
void
GetSequence(std::vector<PlateInformation>& infor_af)
{
  for(unsigned int i = 0; i < infor_af.size(); i++ )
  {
    if(infor_af[i].order == 0)
    {
      infor_af[i].target = -1;
      continue;
    }
    for(unsigned int j = 0; j < infor_af.size(); j++)
    {
      if(infor_af[i].order == infor_af[j].order + 1)
      {
        infor_af[i].target = j;
        break;
      }
    }
  }
}
 
 
 
 
void
checkFood(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,PlateInformation& infor)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  ExtractCircle(cloud_in,cloud,infor.x,infor.y,infor.radius - 0.015,true);
  PrePassThrough(cloud,cloud,infor.top2desk + 0.006, infor.top2desk + 1,"z");  // modify : plus 0.005  2018/06/13
  if(cloud->points.size() > 100)
  {
    std::cout << "show the cloud of the food on the plate !" << std::endl;
    //ShowCloud(cloud);
    WriteCloud(cloud,"food.pcd");
  }
 
  int num;
  num = ExtractClusters(cloud,0.01,10000,100);
  if(num > 0)
  {
    infor.food = 1;
  }
  else
  {
    infor.food = 0;
  }
}
 
 
 
 
 
/******************************************************************************
Function : NormalizeAngle
Description : normalize the angle into the interval [0, 2 * pi]
Input: target angle
Output : new angle
******************************************************************************/
float NormalizeAngle(float angle_in)
{
  float angle_ = angle_in ;
  if(angle_in < 0)
  {
    int integer = (-angle_in) / (2 * pi)+1 ;
    angle_ = angle_in + integer * 2 * pi;
  }
  else if (angle_in > 2 * pi)
  {
    int integer = angle_in / (2 * pi);
    angle_ = angle_in - integer * 2 * pi;
  }
  return angle_;
}
