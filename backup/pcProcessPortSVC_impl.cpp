// -*-C++-*-
/*!
 * @file  pcProcessPortSVC_impl.cpp
 * @brief Service implementation code of pcProcessPort.idl
 *
 */

#include "pcProcessPortSVC_impl.h"  
#include "pcProcessLib.h"

#include <pcl/point_cloud.h>      //pcl library
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcProcessLib.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/tracking/impl/hsv_color_coherence.hpp>

#include "cook_basis.h"           // my library
#include "cook_seg.h"
#include "cook_io.h"
#include "cook_geometry.h"
#include "Visualizer.h"
#include "Define_PointCloud_Algorithm.h"
#include "Functions.h"
#include <pcl/console/time.h>
#include <time.h>
#include <pcl/registration/icp.h>
#include <math.h>
#include <pcl/registration/ndt.h>

pcl::PointCloud<pcl::PointXYZ> ComPcProcessSVC_impl::cloud_merge;
pcl::PointCloud<pcl::PointXYZRGB> ComPcProcessSVC_impl::cloud_color;
pcl::PointCloud<pcl::PointXYZ>::Ptr ComPcProcessSVC_impl::cloud_init;     //realsense get pc
pcl::PointCloud<pcl::PointNormal>::Ptr ComPcProcessSVC_impl::cloud_normals_line; //normal of pointline
float ComPcProcessSVC_impl::nor_sort[300][3];
float ComPcProcessSVC_impl::band;
/*
 * Example implementational code for IDL interface ComPcProcess
 */
ComPcProcessSVC_impl::ComPcProcessSVC_impl() {
    // Please add extra constructor code here.
}
ComPcProcessSVC_impl::ComPcProcessSVC_impl(
        RTC::PointCloud *m_p,
        RTC::InPort<RTC::PointCloud> *m_p_i,
        RTC::PointCloud *m_p2,
        RTC::InPort<RTC::PointCloud> *m_p_i2,
        RTC::PointCloud *m_p3,
        RTC::InPort<RTC::PointCloud> *m_p_i3,
        RTC::PointCloud *m_p4,
        RTC::InPort<RTC::PointCloud> *m_p_i4) {

    m_pointCloud_in		= m_p;
    m_pointCloud_inIn	= m_p_i;
    m_pointCloud_in2	= m_p2;
    m_pointCloud_in2In	= m_p_i2;
    m_pointCloud_in3	= m_p3;
    m_pointCloud_in3In	= m_p_i3;
    m_pointCloud_in4	= m_p4;
    m_pointCloud_in4In	= m_p_i4;

    // Please add extra constructor code here.
    std::cout << "Constructor ComPcProcessSVC_impl!" << std::endl;
    ComPcProcessSVC_impl::cloud_init = PCXYZ_Ptr(new PCXYZ);     //realsense get pc
    ComPcProcessSVC_impl::cloud_normals_line = PCXYZN_Ptr(new PCXYZN);
    band = 0.05f;
}

ComPcProcessSVC_impl::~ComPcProcessSVC_impl() {
    // Please add extra destructor code here.
}


ComPcProcess::MultiPointCloud_slice* ComPcProcessSVC_impl::get_pointCloud(::CORBA::Short CameraBitMask, ::CORBA::Boolean& flag)
{
    ComPcProcess::MultiPointCloud_slice* pc = new RTC::PointCloud[4];

    std::cout << "exe get_pointCloud" << std::endl;

    // check new point cloud
    if (m_pointCloud_inIn->isNew()) {
        // read new point cloud
        m_pointCloud_inIn->read();

        std::cout << "discover new data" << std::endl;

        flag = true;
        pc[0] = *m_pointCloud_in;

        return pc;
    }

    std::cout << "not discover new data" << std::endl;

    flag = false;
    return pc;
}


static std::vector<std::string> StringSplit(const std::string &str, char sep)
{
	std::vector<std::string> v;
	std::stringstream ss(str);
	std::string buffer;
	while( std::getline(ss, buffer, sep) )
		v.push_back(buffer);
	return v;
}

/**
* @brief Combine the std::string that in std::vector, and seprated by [seprator]. \n
* @param[in] std::vector<std::string> src		Mask of which stream you want to setup and enable.
* @param[in] char seprator						Seprator. Default is none.
**/
static std::string StringCombine(std::vector<std::string> src, char seprator = '/')
{
	std::string res = "";
	for(std::string tmp : src)
		res += tmp + seprator;
	return res.erase(res.size()-1,1);
}

static std::string ProcessPathString(std::string src_path, std::string ChannelStr)
{
	char seprator = '/';

	auto path_list = StringSplit(src_path, seprator);
	std::string FileName = path_list[path_list.size() - 1];

	FileName.insert(0, ChannelStr);

	path_list[path_list.size() - 1] = FileName;
	return StringCombine(path_list, seprator);
}

::CORBA::Boolean ComPcProcessSVC_impl::save_pointCloud(::CORBA::Short CameraBitMask, const char* str)
{
    std::cout << "exe save_pointCloud" << std::endl;
    bool fSuccess = false;
    if (m_pointCloud_inIn->isNew()) {
        // read new point cloud
        m_pointCloud_inIn->read();
        std::string Path(str);

        std::cout << "Channel1 discover new data" << std::endl;

        // ========== point cloud convert RTC -> PCL ==============
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cvt_RTCpc_to_PCLpc(*m_pointCloud_in, cloud);

        // ========== cut near 0 (under 0.001) ====================
        pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
        cut_pointCloud_z(cloud, cloud_filtered, 0.001);

        // ========== save point cloud ==============
		pcl::io::savePCDFileBinaryCompressed(ProcessPathString(Path, "Channel1_"), cloud);
        std::cout << "save data over!" << std::endl;
        fSuccess = true;
    }
    if (m_pointCloud_in2In->isNew()) {
        // read new point cloud
        m_pointCloud_in2In->read();
        std::string Path(str);

        std::cout << "Channel2 discover new data" << std::endl;

        // ========== point cloud convert RTC -> PCL ==============
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cvt_RTCpc_to_PCLpc(*m_pointCloud_in2, cloud);

        // ========== cut near 0 (under 0.001) ====================
        pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
        cut_pointCloud_z(cloud, cloud_filtered, 0.001);

        // ========== save point cloud ==============
		pcl::io::savePCDFileBinaryCompressed(ProcessPathString(Path, "Channel2_"), cloud);
        std::cout << "save data over!" << std::endl;
        fSuccess = true;
    }
    if (m_pointCloud_in3In->isNew()) {
        // read new point cloud
        m_pointCloud_in3In->read();
        std::string Path(str);

        std::cout << "Channel3 discover new data" << std::endl;

        // ========== point cloud convert RTC -> PCL ==============
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cvt_RTCpc_to_PCLpc(*m_pointCloud_in3, cloud);

        // ========== cut near 0 (under 0.001) ====================
        pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
        cut_pointCloud_z(cloud, cloud_filtered, 0.001);

        // ========== save point cloud ==============
		pcl::io::savePCDFileBinaryCompressed(ProcessPathString(Path, "Channel3_"), cloud);
        std::cout << "save data over!" << std::endl;
        fSuccess = true;
    }
    if (m_pointCloud_in4In->isNew()) {
        // read new point cloud
        m_pointCloud_in4In->read();
        std::string Path(str);

        std::cout << "Channel4 discover new data" << std::endl;

        // ========== point cloud convert RTC -> PCL ==============
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cvt_RTCpc_to_PCLpc(*m_pointCloud_in4, cloud);

        // ========== cut near 0 (under 0.001) ====================
        pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
        cut_pointCloud_z(cloud, cloud_filtered, 0.001);

        // ========== save point cloud ==============
		pcl::io::savePCDFileBinaryCompressed(ProcessPathString(Path, "Channel4_"), cloud);
        std::cout << "save data over!" << std::endl;
        fSuccess = true;
    }
    if(fSuccess)
        return true;
    else
    {
        std::cout << "not discover new data" << std::endl;
        return false;
    }
}
/**********************************************************/


// End of example implementational code
void ComPcProcessSVC_impl::xyzabc_mean_pc(const ::ComPcProcess::array nothing, const char* axis, ::ComPcProcess::array info) {}

//static pcl::PointCloud<pcl::PointXYZ> cloud_merge;
//static pcl::PointCloud<pcl::PointXYZRGB> cloud_color;
//static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_init (new pcl::PointCloud<pcl::PointXYZ>);     //realsense get pc
//static pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals_line (new pcl::PointCloud<pcl::PointNormal>); //normal of pointline
//static float nor_sort[300][3];
//static float band = 0.05f;

bool comparePointDown(const pcl::PointNormal& p1, const pcl::PointNormal& p2)
{
        if (p1.z < p2.z)
            return true;
        else
            return false;
}
bool comparePointUp(const pcl::PointNormal& p1, const pcl::PointNormal& p2)
{
        if (p1.z > p2.z)
            return true;
        else
            return false;
}

void sort0fPoint(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals, pcl::PointCloud<pcl::PointNormal>::Ptr normals_sorted)
{
    float z_max = -FLT_MAX;
    uint32_t k = 0;
    for (uint32_t i = 0; i<cloud_normals->points.size(); i++)
    {
        if (z_max < cloud_normals->points[i].z)
        {
            z_max = cloud_normals->points[i].z;
            k = i;
        }
    }
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals_left (new pcl::PointCloud<pcl::PointNormal>);

    for (uint32_t i = 0; i<cloud_normals->points.size(); i++)
    {
        if (cloud_normals->points[i].y >= cloud_normals->points[k].y)
        {
            if (cloud_normals->points[i].z >= 0.09f+ComPcProcessSVC_impl::band) //0.09
                normals_sorted ->push_back(cloud_normals->points[i]);
        }
        else
        {
            if (cloud_normals->points[i].z >= 0.07f+ComPcProcessSVC_impl::band) //0.07
                cloud_normals_left->push_back(cloud_normals->points[i]);
        }
    }
    std::sort(cloud_normals_left->points.begin(), cloud_normals_left->points.end(), comparePointUp);
    std::sort(normals_sorted->points.begin(), normals_sorted->points.end(), comparePointDown);
    for (uint32_t i=0; i<cloud_normals_left->points.size(); i++)
        normals_sorted->push_back(cloud_normals_left->points[i]);
}

void extractPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals)
{
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    pcl::PointXYZ p;

    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud (cloud);
    feature_extractor.compute ();
    feature_extractor.getAABB (min_point_AABB, max_point_AABB);

    std::cout<<" minmum point: "<< min_point_AABB
             <<" maxmum point: "<< max_point_AABB << std::endl;

    for (unsigned int i = 0; i < cloud->points.size(); i++)
    {
        float a = (min_point_AABB.x + max_point_AABB.x)/2;
        if (cloud->points[i].x >= a-0.0003f && cloud->points[i].x <= a+0.0003f)//(cloud->points[i].z == min_point_AABB.z)
         {
            p.x = cloud->points[i].x;
            p.y = cloud->points[i].y;
            p.z = cloud->points[i].z;
            cloud_line->push_back(p);

            ComPcProcessSVC_impl::cloud_normals_line->push_back(cloud_normals->points[i]);

         }
    }
}

void nor_to_xyz(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointXYZ p;
    for (unsigned int i = 0; i < cloud_normals->points.size(); i++)
    {
        p.x = cloud_normals->points[i].x;
        p.y = cloud_normals->points[i].y;
        p.z = cloud_normals->points[i].z;
        cloud->push_back(p);
    }
}

void ComPcProcessSVC_impl::merge_pointCloud(const ::ComPcProcess::Matrix4X4F matrix_in) //get and merge point cloud
 {
    std::cout << "is here??????????????????????????????????" << std::endl;
        if (m_pointCloud_inIn->isNew())
    {
        // read new point cloud
        m_pointCloud_inIn->read();
        std::cout << "Channel1 discover new data" << std::endl;
        // ========== point cloud convert RTC -> PCL ==============
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cvt_RTCpc_to_PCLpc(*m_pointCloud_in, cloud);     // target_cloud

        Eigen::Matrix4f matrix_c2b;

        for(int i=0;i<4;i++)
        {
          for(int j=0;j<4;j++)
          {
            matrix_c2b(i,j)=matrix_in[i][j];
          }
        }
    pcl::PointCloud<pcl::PointXYZ> cloud_base;
    pcl::transformPointCloud (cloud, cloud_base, matrix_c2b);//pointcloud coordinate c2b

    ComPcProcessSVC_impl::cloud_merge += cloud_base;
    ComPcProcessSVC_impl::cloud_init = cloud_merge.makeShared();
    }
}

void ComPcProcessSVC_impl::color_pointCloud(const ::ComPcProcess::Matrix4X4F matrix_in)
{
    if (m_pointCloud_inIn->isNew())
    {
        // read new point cloud
        m_pointCloud_inIn->read();
        std::cout << "Channel1 discover new data" << std::endl;
        // ========== point cloud convert RTC -> PCL ==============
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cvt_RTCpc_to_PCLpc(*m_pointCloud_in, cloud);     // target_cloud

        Eigen::Matrix4f matrix_c2b;

        for(int i=0;i<4;i++)
        {
          for(int j=0;j<4;j++)
          {
            matrix_c2b(i,j)=matrix_in[i][j];
          }
        }
        pcl::PointCloud<pcl::PointXYZRGB> cloud_base;
        pcl::transformPointCloud (cloud, cloud_base, matrix_c2b);//pointcloud coordinate c2b
        ComPcProcessSVC_impl::cloud_color += cloud_base;
        std::cout<< "color_pointCloud done" <<std::endl;
    }
}

void normal_reserve(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals)
{
    for (size_t i=0; i<cloud_normals->points.size(); i++)
    {
        cloud_normals->points[i].normal_x *= -1;
        cloud_normals->points[i].normal_y *= -1;
        cloud_normals->points[i].normal_z *= -1;
    }

}

void ComPcProcessSVC_impl::toolCoordinateCalibration(const ::ComPcProcess::Matrix4X4F matrix_in, ::ComPcProcess::array0fThr caliPoint)
{
    ShowCloud(cloud_init,0.2);
    WriteCloud(cloud_init,"init_calibration.pcd");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough (new pcl::PointCloud<pcl::PointXYZ>);
    PrePassThrough(cloud_init,cloud_passthrough,0.03f,0.08f,"z");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough1 (new pcl::PointCloud<pcl::PointXYZ>);
    PrePassThrough(cloud_passthrough,cloud_passthrough1,-0.820f,-0.480f,"y");
    /**
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough2 (new pcl::PointCloud<pcl::PointXYZ>);
    PrePassThrough(cloud_init,cloud_passthrough2,0.550f,0.700f,"x");
    **/
    ShowCloud(cloud_passthrough1,0.2);
    WriteCloud(cloud_init,"passth_calibration.pcd");

    float y_max = -FLT_MAX;
    uint32_t k = 0;
    for (uint32_t i = 0; i<cloud_passthrough1->points.size(); i++)
    {
        if (y_max < cloud_passthrough1->points[i].y)
        {
            y_max = cloud_passthrough1->points[i].y;
            k = i;
        }
    }
    caliPoint[0] = cloud_passthrough1->points[k].x;
    caliPoint[1] = cloud_passthrough1->points[k].y;
    caliPoint[2] = cloud_passthrough1->points[k].z;
}



::CORBA::Long ComPcProcessSVC_impl::apple_PC(const ::ComPcProcess::Matrix4X4F matrix_in, ::ComPcProcess::point_line point_coor)
{
    //ReadCloud(cloud_init,"init_apple.pcd");
//    ShowCloud(cloud_init,0.2);
    WriteCloud(cloud_init,"init_apple.pcd");
	
		
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough1 (new pcl::PointCloud<pcl::PointXYZ>);
    PrePassThrough(cloud_init,cloud_passthrough1,0.04f+band,0.12f+band,"z");                               //extract 0 to 0.5 in z of axis
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough2 (new pcl::PointCloud<pcl::PointXYZ>);
    PrePassThrough(cloud_passthrough1,cloud_passthrough2,-0.70f,-0.54f,"y");  //-0.54                    //extract -0.22,-0.08 in y of axis
//    WriteCloud(cloud_passthrough2,"passtr_apple.pcd");
	
    const float voxel_grid_size = 0.0007f;                                                     //3D box
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel (new pcl::PointCloud<pcl::PointXYZ>);
    DownSampleCloud(cloud_passthrough2,cloud_voxel,voxel_grid_size);
//    WriteCloud(cloud_voxel,"voxel_apple.pcd");
	
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);	   //find the maximum cluster of point cloud
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
	int num_clusters = 0;
    num_clusters = ExtractClusters(cloud_voxel,cloud_cluster,clusters,0.007f,45000,100);
//	ShowCloud(cloud_cluster,0.02);
	WriteCloud(cloud_cluster,"cluster_apple.pcd");
	
	pcl::PointCloud<pcl::PointNormal> mls_points;                                             //reconstruct surface of point cloud
	SurfaceReconstr(cloud_cluster, mls_points);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
    cloud_normals = mls_points.makeShared();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outlier (new pcl::PointCloud<pcl::PointXYZ>);
    nor_to_xyz(cloud_normals,cloud_outlier);                                                  //reconstructed surface of pointsXYZ cloud
    WriteCloud(cloud_outlier,"apple_reconst.pcd");
//    ShowCloud(cloud_outlier,0.2);

    Eigen::Vector4f centroid;
    CentroidOfCloud(cloud_outlier,centroid);                                                // calculate centroid of cloud
    for (size_t i=0; i<cloud_normals->points.size(); i++)                                   // make normals orientation point to centroid
    {
        pcl::flipNormalTowardsViewpoint (cloud_normals->points[i], centroid(0), centroid(1),centroid(2), cloud_normals->points[i].normal_x,cloud_normals->points[i].normal_y,cloud_normals->points[i].normal_z);
    }
    normal_reserve(cloud_normals);                                                           // reserve normals orientation

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lineout(new pcl::PointCloud<pcl::PointXYZ>);
    extractPoint(cloud_outlier,cloud_lineout,cloud_normals);                                 //extract points and normal in line

    pcl::PointCloud<pcl::PointNormal>::Ptr normals_sorted (new pcl::PointCloud<pcl::PointNormal>); //sorted points of Pc trajectory(pnormal)
    sort0fPoint(cloud_normals_line,normals_sorted);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudLineSorted(new pcl::PointCloud<pcl::PointXYZ>);       //sorted points of Pc trajectory(PXYZ)
    nor_to_xyz(normals_sorted,cloudLineSorted);
//    ShowCloud(cloudLineSorted,0.2);
    WriteCloud(cloudLineSorted,"apple_line.pcd");
//    normalsVis(cloudLineSorted,normals_sorted);
    //point_coor[300][3] = { 0 };
    uint32_t count = normals_sorted->points.size();
    uint32_t endf = count < 300 ? count : 300;
    for(uint32_t i = 0; i < endf; i++)
	{
        point_coor[i][0] = normals_sorted->points[i].x;
        point_coor[i][1] = normals_sorted->points[i].y;
        point_coor[i][2] = normals_sorted->points[i].z;
        ComPcProcessSVC_impl::nor_sort[i][0] = normals_sorted->points[i].normal_x;
        ComPcProcessSVC_impl::nor_sort[i][1] = normals_sorted->points[i].normal_y;
        ComPcProcessSVC_impl::nor_sort[i][2] = normals_sorted->points[i].normal_z;
        std::cout<< normals_sorted->points[i].x << " "
                 << normals_sorted->points[i].y << " "
                 << normals_sorted->points[i].z << std::endl;

	}
    std::cout<<"count : "<< count << std::endl;
    return count;
}
::CORBA::Long ComPcProcessSVC_impl::transform_tc2b(const ::ComPcProcess::Matrix4X4F matrix_in, ::ComPcProcess::point_line point_coor)
{

    for(unsigned int i = 0; i < 300; i++)
    {
        point_coor[i][0] = ComPcProcessSVC_impl::nor_sort[i][0];
        point_coor[i][1] = ComPcProcessSVC_impl::nor_sort[i][1];
        point_coor[i][2] = ComPcProcessSVC_impl::nor_sort[i][2];

        std::cout<< "point_coor " << i << ":" << point_coor[i][0] << " "
                                              << point_coor[i][1] << " "
                                              << point_coor[i][2] << std::endl;
    }

    return 0;
}

void ComPcProcessSVC_impl::boundaryNormal(const ::ComPcProcess::Matrix4X4F matrix_in, ::ComPcProcess::point_line normal, ::CORBA::Long& boundarysizes)
{
    pcl::PCDWriter writer;
    pcl::PCDReader reader;
    /**
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<PointXYZRGB>);
    //reader.read<pcl::PointXYZRGB> ("init.pcd", *cloud);
    cloud = ComPcProcessSVC_impl::cloud_color.makeShared();
     ViewerPara paramater;
     {
         paramater.NeedBaseCoord = true;
         paramater.NeedRGBPC_SegmentMode = false;
     }
//    PCShow(cloud, "initial", 1, paramater);
    WriteCloudRGB(cloud, "initRGB.pcd");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_passthrough1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    PassThroughRGB(cloud,cloud_passthrough1,0.04f+band,0.12f+band,"z");                               //extract 0 to 0.5 in z of axis
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_passthrough2 (new pcl::PointCloud<pcl::PointXYZRGB>);
    PassThroughRGB(cloud_passthrough1,cloud_passthrough2,-0.70f,-0.54f,"y");
    //PCShow(cloud_passthrough2, "passthrough", 1, paramater);
    WriteCloudRGB(cloud_passthrough2, "passthroughRGB.pcd");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);	   //find the maximum cluster of point cloud
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
    int num_clusters = 0;
    num_clusters = ExtractClustersRGB(cloud_passthrough2,cloud_cluster,clusters,0.007f,45000,100);
    //PCShow(cloud_cluster, "cluster", 1, paramater);
    WriteCloudRGB(cloud_cluster, "clusterRGB.pcd");
    **/
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    reader.read<pcl::PointXYZRGB> ("clusterRGB.pcd", *cloud_cluster);
    pcl::PointCloud<pcl::PointNormal> mls_points;                                             //reconstruct surface of point cloud
    SurfaceReconstrRGB(cloud_cluster, mls_points);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
    cloud_normals = mls_points.makeShared();

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsvCloud (new pcl::PointCloud<pcl::PointXYZHSV>);
    pcl::PointXYZHSV hsv;
    for (uint32_t i=0; i<cloud_cluster->size(); i++){
        hsvCloud->push_back(hsv);
        hsvCloud->points[i].x = cloud_normals->points[i].x;
        hsvCloud->points[i].y = cloud_normals->points[i].y;
        hsvCloud->points[i].z = cloud_normals->points[i].z;
        int r = cloud_cluster->points[i].r;
        int g = cloud_cluster->points[i].g;
        int b = cloud_cluster->points[i].b;

        pcl::tracking::RGB2HSV(r,g,b,hsvCloud->points[i].h,hsvCloud->points[i].s,hsvCloud->points[i].v);
        }
    pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPulp (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ xyz_p;
    for (uint32_t i=0; i<hsvCloud->size(); i++){
        if (hsvCloud->points[i].h*256>25 && hsvCloud->points[i].h*256<230){
            xyz_p.x = hsvCloud->points[i].x;
            xyz_p.y = hsvCloud->points[i].y;
            xyz_p.z = hsvCloud->points[i].z;
            CloudPulp->push_back(xyz_p);
         }
    }
//    ShowCloud(CloudPulp,0.2);
    writer.write<pcl::PointXYZ> ("CloudPulp.pcd", *CloudPulp);


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_pulp (new pcl::PointCloud<pcl::PointXYZ>);	   //find the maximum cluster of point cloud
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters1;
    ExtractClusters(CloudPulp,cloud_cluster_pulp,clusters1,0.002f,45000,100);
//    ShowCloud(cloud_cluster_pulp,0.2);
    WriteCloud(cloud_cluster_pulp, "CloudPulp.pcd");

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_boundary (new pcl::PointCloud<pcl::PointNormal>);
    float x_max = -FLT_MAX;
    for (uint32_t i=0; i<cloud_cluster_pulp->size(); i++)
        x_max = x_max > cloud_cluster_pulp->points[i].x ? x_max : cloud_cluster_pulp->points[i].x;
    for (unsigned int i = 0; i < cloud_normals->points.size(); i++)
    {
        if (cloud_normals->points[i].x >= x_max-0.0005f && cloud_normals->points[i].x <= x_max+0.0005f)//(cloud->points[i].z == min_point_AABB.z)
         {
          cloud_boundary->push_back(cloud_normals->points[i]);
         }
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    nor_to_xyz(cloud_boundary,cloud_boundary_xyz);
    writer.write<pcl::PointXYZ> ("cloud_boundary_xyz.pcd", *cloud_boundary_xyz);

    boundarysizes = int(cloud_boundary->size());
    uint32_t endf = boundarysizes < 300 ? boundarysizes : 300;
    for (uint32_t i = 0; i < endf; i++){
        normal[i][0] = cloud_boundary->points[i].normal_x;
        normal[i][1] = cloud_boundary->points[i].normal_y;
        normal[i][2] = cloud_boundary->points[i].normal_z;
    }
    std::cout<< "boundarySizes: " << boundarysizes <<endl;
    ComPcProcessSVC_impl::cloud_merge.clear();
    ComPcProcessSVC_impl::cloud_color.clear();
    ComPcProcessSVC_impl::cloud_init->clear();
    ComPcProcessSVC_impl::cloud_normals_line->clear();
}

void ComPcProcessSVC_impl::evaluation(const ::ComPcProcess::Matrix4X4F matrix_in, ::CORBA::Float& pulp, ::CORBA::Float& apple)
{
    ViewerPara paramater;
    {
        paramater.NeedBaseCoord = true;
        paramater.NeedRGBPC_SegmentMode = false;
    }
//   PCShow(cloud, "initial", 1, paramater);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<PointXYZRGB>);
    cloud = ComPcProcessSVC_impl::cloud_color.makeShared();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_passthrough0 (new pcl::PointCloud<pcl::PointXYZRGB>);
    PassThroughRGB(cloud,cloud_passthrough0,0.45f,0.65f,"x");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_passthrough1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    PassThroughRGB(cloud_passthrough0,cloud_passthrough1,0.032f,0.10f,"z");                              //extract 0 to 0.5 in z of axis
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_passthrough2 (new pcl::PointCloud<pcl::PointXYZRGB>);
    PassThroughRGB(cloud_passthrough1,cloud_passthrough2,-0.85f,-0.70f,"y");
    PCShow(cloud_passthrough2, "passthrough", 1, paramater);
    WriteCloudRGB(cloud_passthrough2, "passthrough.pcd");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);	   //find the maximum cluster of point cloud
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
    int num_clusters = 0;
    num_clusters = ExtractClustersRGB(cloud_passthrough2,cloud_cluster,clusters,0.005f,100000,600);
    PCShow(cloud_cluster, "cluster", 1, paramater);
    WriteCloudRGB(cloud_cluster, "handclusterRGB.pcd");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_voxel (new pcl::PointCloud<pcl::PointXYZRGB>);  //3D box(0.001mm)
    DownSampleCloudRGB(cloud_cluster, cloud_voxel);
    PCShow(cloud_voxel, "voxel", 1, paramater);
    WriteCloudRGB(cloud_voxel, "handvoxelRGB.pcd");

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsvCloud (new pcl::PointCloud<pcl::PointXYZHSV>);
    pcl::PointXYZHSV hsv;
    for (uint32_t i=0; i<cloud_voxel->size(); i++){
        hsv.x = cloud_voxel->points[i].x;
        hsv.y = cloud_voxel->points[i].y;
        hsv.z = cloud_voxel->points[i].z;
        hsvCloud->push_back(hsv);
        int r = cloud_voxel->points[i].r;
        int g = cloud_voxel->points[i].g;
        int b = cloud_voxel->points[i].b;

        pcl::tracking::RGB2HSV(r,g,b,hsvCloud->points[i].h,hsvCloud->points[i].s,hsvCloud->points[i].v);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPulp (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ xyz_p;
    for (uint32_t i=0; i<hsvCloud->size(); i++){
        if (hsvCloud->points[i].h*256>25 && hsvCloud->points[i].h*256<230){
            xyz_p.x = hsvCloud->points[i].x;
            xyz_p.y = hsvCloud->points[i].y;
            xyz_p.z = hsvCloud->points[i].z;
            CloudPulp->push_back(xyz_p);
         }
    }
    pulp = CloudPulp->size();
    apple = cloud_voxel->size();
    ComPcProcessSVC_impl::cloud_color.clear();

}
