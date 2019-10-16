#include "cook_io.h"
 
/******************************************************************************
Function : ReadCloud
Description : read a cloud from a PCD file
Input : file name
Output : cloud
******************************************************************************/
void
ReadCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,std::string name)
{
  pcl::PCDReader reader;
  reader.read (name,*cloud_in);
  std::cerr << "PointCloud has: " << cloud_in->points.size () << " data points." << std::endl;
}
 
/******************************************************************************
Function : ReadCloud
Description : read a cloud from a PCD file
Input : file name and extension (numbers can be included)
Output : cloud
******************************************************************************/
void
ReadCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,std::string name,int num, std::string ext)
{
  std::ostringstream sstr;
  sstr << num;
  std::string str = sstr.str();
  std::string final_name = name + str + ext;
  pcl::PCDReader reader;
  reader.read (final_name,*cloud_in);
  std::cerr << "PointCloud has: " << cloud_in->points.size () << " data points." << std::endl;
}
 
/******************************************************************************
Function : WriteCloud
Description : write a cloud to a PCD file
Input : cloud and file name
Output :
******************************************************************************/
void
WriteCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,std::string name)
{
  pcl::PCDWriter writer;
  struct timeval tv;
  gettimeofday(&tv, NULL);
  std::string us = std::to_string(tv.tv_usec);
  std::string s = std::to_string(tv.tv_sec);
  s += (us + name);
  writer.write<pcl::PointXYZ> (s, *cloud_in);
}


/******************************************************************************
Function : WriteCloudRGB
Description : write a cloud to a PCD file
Input : cloud and file name
Output :
******************************************************************************/
void WriteCloudRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,std::string name)
{
  pcl::PCDWriter writer;
  struct timeval tv;
  gettimeofday(&tv, NULL);
  std::string us = std::to_string(tv.tv_usec);
  std::string s = std::to_string(tv.tv_sec);
  s += (us + name);
  writer.write<pcl::PointXYZRGB> (s, *cloud_in);
}


/******************************************************************************
Function : WriteCloud
Description : write a cloud with a name base on the local time
Input : cloud
Output : file
******************************************************************************/
void WriteCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{
  pcl::PCDWriter writer;
 
  struct timeval tv;
  gettimeofday(&tv, NULL);
  std::string us = std::to_string(tv.tv_usec);
  std::string s = std::to_string(tv.tv_sec);
  std::string ext = ".pcd";
  s += (us+ext);
  std::cout << s << std::endl;
  writer.write<pcl::PointXYZ> (s, *cloud_in);
}
 
 
/******************************************************************************
Function : ShowCloud
Description : show a cloud on the screen.
Input : cloud, coordinate size, and background color.
Output :
******************************************************************************/
void ShowCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,double coor_size,int R,int G,int B)
{
  if (cloud_in->points.empty())
    std::cerr << " No points exist." << std::endl;
  else
  {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Vierwer"));
    viewer->setBackgroundColor(R,G,B);
    viewer->addPointCloud<pcl::PointXYZ> (cloud_in,"result");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"result");
    viewer->addCoordinateSystem(coor_size);
    viewer->initCameraParameters();
 
    while(!viewer->wasStopped())
    {
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }
}
 
void normalsVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::PointNormal>::ConstPtr normals)
{

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 1);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZ> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addPointCloudNormals<pcl::PointXYZ, pcl::PointNormal> (cloud, normals, 10, 0.01f, "normals");
  viewer->addCoordinateSystem (0.5);
  viewer->initCameraParameters ();
  while(!viewer->wasStopped())
  {
    viewer->spinOnce(100);
    boost::this_thread::sleep (boost::posix_time::microseconds(100000));
  }
}

/******************************************************************************
Function : LoadCloud
Description : load cloud from a PCD file or a PLY file.
Input : argc and argv for file names
Output : cloud
******************************************************************************/
int
LoadCloud(int argc,char** argv,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if( argc < 2)
    {
        printf("No target Cloud files given!\n");
    }
 
    std::vector<int> filenames;
    bool file_is_pcd = false;
 
    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");
 
    if (filenames.size () != 1)  {
      filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
 
      if (filenames.size () != 1) {
        printf("input error!\n");
        return -1;
      } else {
        file_is_pcd = true;
      }
    }
 
    if (file_is_pcd) {
      if (pcl::io::loadPCDFile (argv[filenames[0]], *cloud) < 0)  {
        std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
        return -1;
      }
    } else {
      if (pcl::io::loadPLYFile (argv[filenames[0]], *cloud) < 0)  {
        std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
        return -1;
      }
    }
    return 0;
}
