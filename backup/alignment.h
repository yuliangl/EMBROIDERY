#ifndef __ALIGNMENT_H_
#define __ALIGNMENT_H_
 
#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include "cook_basis.h"
#include "cmath"
#include "cook_io.h"
#include "cook_geometry.h"
#include "cook_seg.h"
#include <pcl/console/time.h>
#include <pcl/registration/icp.h>
#include <extra_func.h>
 
 
 
class FeatureCloud
{
  public:
    // A bit of shorthand
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
    typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;
 
    FeatureCloud () :
      search_method_xyz_ (new SearchMethod),
      normal_k_(30), //30
      feature_k_(35),  //35
      normal_radius_ (0.02f),
      feature_radius_ (0.018f)
    {}
 
    ~FeatureCloud () {}
 
    // Process the given cloud
    void
    setInputCloud (PointCloud::Ptr xyz)
    {
      xyz_ = xyz;
      processInput ();
    }
 
    // Load and process the cloud in the given PCD file
    void
    loadInputCloud (const std::string &pcd_file)
    {
      xyz_ = PointCloud::Ptr (new PointCloud);
      pcl::io::loadPCDFile (pcd_file, *xyz_);
      processInput ();
    }
 
    // Get a pointer to the cloud 3D points
    PointCloud::Ptr
    getPointCloud () const
    {
      return (xyz_);
    }
 
    // Get a pointer to the cloud of 3D surface normals
    SurfaceNormals::Ptr
    getSurfaceNormals () const
    {
      return (normals_);
    }
 
    // Get a pointer to the cloud of feature descriptors
    LocalFeatures::Ptr
    getLocalFeatures () const
    {
      return (features_);
    }
 
  protected:
    // Compute the surface normals and local features
    void
    processInput ()
    {
      computeSurfaceNormals ();
      computeLocalFeatures ();
    }
 
    // Compute the surface normals
    void
    computeSurfaceNormals ()
    {
      normals_ = SurfaceNormals::Ptr (new SurfaceNormals);
 
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
      norm_est.setInputCloud (xyz_);
      norm_est.setSearchMethod (search_method_xyz_);
      norm_est.setKSearch(normal_k_);
      //norm_est.setRadiusSearch (normal_radius_);
      norm_est.compute (*normals_);
    }
 
    // Compute the local feature descriptors
    void
    computeLocalFeatures ()
    {
      features_ = LocalFeatures::Ptr (new LocalFeatures);
 
      pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
      fpfh_est.setInputCloud (xyz_);
      fpfh_est.setInputNormals (normals_);
      fpfh_est.setSearchMethod (search_method_xyz_);
      fpfh_est.setKSearch(feature_k_);
      //fpfh_est.setRadiusSearch (feature_radius_);
      fpfh_est.compute (*features_);
    }
 
  private:
    // Point cloud data
    PointCloud::Ptr xyz_;
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_;
    SearchMethod::Ptr search_method_xyz_;
 
    // Parameters
    int normal_k_;
    int feature_k_;
    float normal_radius_;
    float feature_radius_;
};
 
 
 
void ICPCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr target_in,pcl::PointCloud<pcl::PointXYZ>::Ptr model_in,pcl::PointCloud<pcl::PointXYZ>::Ptr& target_out,pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>& icp);
 
class TemplateAlignment
{
  public:
    // A struct for storing alignment results
    struct Result
    {
      float fitness_score;
      float icp_score;
      Eigen::Matrix4f sac_ia_transformation;
      Eigen::Matrix4f icp_transformation;
      Eigen::Matrix4f final_transformation;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
 
    TemplateAlignment () :
      min_sample_distance_ (0.07f),//0.05
      max_correspondence_distance_ (0.01f*0.01f),
      //max_correspondence_distance_(0.001f),
      nr_iterations_ (250) //175
    {
      // Initialize the parameters in the Sample Consensus Initial Alignment (SAC-IA) algorithm
      sac_ia_.setMinSampleDistance (min_sample_distance_);
      sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
      sac_ia_.setMaximumIterations (nr_iterations_);
    }
 
    ~TemplateAlignment () {}
 
 
 
    // Set the given cloud as the target to which the templates will be aligned
    void
    setTargetCloud (FeatureCloud &target_cloud)
    {
      target_ = target_cloud;
      sac_ia_.setInputTarget (target_cloud.getPointCloud ());
      sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures ());
    }
 
    // Add the given cloud to the list of template clouds
    void
    addTemplateCloud (FeatureCloud &template_cloud)
    {
      templates_.push_back (template_cloud);
    }
 
    // Align the given template cloud to the target specified by setTargetCloud ()
    void
    align (FeatureCloud &template_cloud, TemplateAlignment::Result &result)
    {
      std::cout << " " << std::endl;
      std::cout << "------try one model from the templates to align with the target-----------" << std::endl;
      sac_ia_.setInputCloud (template_cloud.getPointCloud ());
      sac_ia_.setSourceFeatures (template_cloud.getLocalFeatures ());
 
      pcl::PointCloud<pcl::PointXYZ> registration_output;
      sac_ia_.align (registration_output);
 
      result.fitness_score = (float) sac_ia_.getFitnessScore (max_correspondence_distance_);
      result.sac_ia_transformation = sac_ia_.getFinalTransformation ();
 
      std::cout << "sac-ia score: " << result.fitness_score << std::endl;
 
      if(result.fitness_score < 0.000040)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
        pcl::transformPointCloud (*template_cloud.getPointCloud (), *transformed_cloud, result.sac_ia_transformation);
/*
        int index = 0;
        float temp = 0;
        for(unsigned int k = 0;k< transformed_cloud->points.size();k++)
        {
          if(temp < transformed_cloud->points[k].x)
          {
            temp = transformed_cloud->points[k].x;
            index = k;
          }
        }
        if(transformed_cloud->points[index].z < MeanZ(transformed_cloud))
        {
          TransformCloud(transformed_cloud,transformed_cloud, 0, 0, 0, "x", 3.1415926);
          std::cout << "transform (upside down)" << std::endl;
        }
*/
 
        pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
        ICPCloud(transformed_cloud,target_.getPointCloud(),icp_cloud,icp_);
        result.icp_score = icp_.getFitnessScore();
        result.icp_transformation = icp_.getFinalTransformation();
        result.final_transformation =  result.icp_transformation * result.sac_ia_transformation ;
      }
      else
      {
        result.icp_score = 1;
        result.final_transformation = result.sac_ia_transformation;
      }
      std::cout << "--------align one finished-------------" << std::endl;
    }
 
    // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
    void
    alignAll (std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
    {
      results.resize (templates_.size ());
      for (size_t i = 0; i < templates_.size (); ++i)
      {
        align (templates_[i], results[i]);
 
        if(results[i].icp_score < 0.000005)    //   change on 7/16
        {
          if(i < templates_.size()-1)
          {
            for(size_t j = i+1; j < templates_.size(); j++)
            {
              results[j].icp_score = 1;
            }
          }
          break;
        }
 
      }
    }
 
    // Align all of template clouds to the target cloud to find the one with best alignment score
    int
    findBestAlignment (TemplateAlignment::Result &result)
    {
      // Align all of the templates to the target cloud
      std::vector<Result, Eigen::aligned_allocator<Result> > results;
      alignAll (results);
 
      // Find the template with the best (lowest) fitness score
      float lowest_score = std::numeric_limits<float>::infinity ();
      int best_template = 0;
      for (size_t i = 0; i < results.size (); ++i)
      {
        const Result &r = results[i];
        if (r.icp_score < lowest_score)
        {
          lowest_score = r.icp_score;
          best_template = (int) i;
        }
      }
 
      // Output the best alignment
      result = results[best_template];
      return (best_template);
    }
 
  private:
    // A list of template clouds and the target to which they will be aligned
    std::vector<FeatureCloud> templates_;
    FeatureCloud target_;
 
    // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
    float min_sample_distance_;
    float max_correspondence_distance_;
    int nr_iterations_;
};
 
 
 
 
void InputStream(std::vector<FeatureCloud>& object_templates,char* fname);
void AlignCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& target_in,std::vector<FeatureCloud>& templates_in,int& best_index,TemplateAlignment::Result& best_alignment);
int ExtractAlignedModel(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr& model_in,TemplateAlignment::Result& best_alignment,float voxel_size_in);
int EvaluateTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr& model_in,TemplateAlignment::Result& best_alignment,float radius);
int AlignAllModels(pcl::PointCloud<pcl::PointXYZ>::Ptr& target_in,std::vector<FeatureCloud>& templates_in,float fit_threshold,std::vector<PlateInformation>& infor, std::vector<PlateInformation>& food);
 


