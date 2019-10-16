#include "alignment.h"
 
//input models
void InputStream(std::vector<FeatureCloud>& object_templates,char* fname)
{
  std::ifstream input_stream (fname);
  object_templates.resize (0);
  std::string pcd_filename;
  while (input_stream.good ())
  {
    std::getline (input_stream, pcd_filename);
    if (pcd_filename.empty () || pcd_filename.at (0) == '#') // Skip blank lines or comments
      continue;
 
    FeatureCloud template_cloud;
    template_cloud.loadInputCloud (pcd_filename);
    object_templates.push_back (template_cloud);
  }
  input_stream.close ();
}
 
//first step: align models with targets
void AlignCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& target_in,std::vector<FeatureCloud>& templates_in,int& best_index,TemplateAlignment::Result& best_alignment)
{
  pcl::console::TicToc ttt;
  std::cerr << "Aligning one...\n", ttt.tic ();
 
  FeatureCloud target_cloud;
  target_cloud.setInputCloud (target_in);
 
  // Set the TemplateAlignment inputs
  TemplateAlignment template_align;
  for (size_t i = 0; i < templates_in.size (); ++i)
  {
    template_align.addTemplateCloud (templates_in[i]);
  }
  template_align.setTargetCloud (target_cloud);
 
  // Find the best template alignment
  best_index = template_align.findBestAlignment (best_alignment);
 
  // Print the alignment fitness score (values less than 0.00002 are good)
  printf ("Best fitness score: %f\n", best_alignment.icp_score);
  std::cerr << ">> Done: " << ttt.toc () << " ms\n";
}
 
//second step: evaluate the alignments and extract the neighbours of the models if it is a good alignment.
int ExtractAlignedModel(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr& model_in,TemplateAlignment::Result& best_alignment,float voxel_size_in)
{
  pcl::console::TicToc tt;
  std::cerr << "Extracting...\n", tt.tic ();
 
  pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>);
  DownSampleCloud(model_in,model,voxel_size_in);
  float radius=sqrt(3)*voxel_size_in/2;
 
  std::vector<int> indices;
  indices.clear();
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (cloud);
  for(size_t i=0;i< model->points.size();++i)
  {
    std::vector<int> indices_part;
    std::vector<float> sqr_distances;
    indices_part.clear();
    kdtree.radiusSearch(model->points[i],radius,indices_part,sqr_distances);
    indices.insert(indices.end(),indices_part.begin(),indices_part.end());
  }
 
  sort(indices.begin(),indices.end());
  std::vector<int>::iterator new_end=unique(indices.begin(),indices.end());
  indices.erase(new_end,indices.end());
 
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
  inliers->indices=indices;
  ExtractCloud(cloud,inliers,cloud,true);
  std::cerr << ">> Done: " << tt.toc () << " ms\n";
  return 1;
}
 
//entire procedure:
//
int AlignAllModels(pcl::PointCloud<pcl::PointXYZ>::Ptr& target_in,std::vector<FeatureCloud>& templates_in,float fit_threshold,std::vector<PlateInformation>& infor, std::vector<PlateInformation>& food)
{
  pcl::console::TicToc tt;
  std::cerr << "Aligning ALL...\n", tt.tic ();
 
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_angle (new pcl::PointCloud<pcl::PointXYZ>);
  //*cloud_angle = *target_in;
  pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
 
  // ---------------------------------------------------------------------------------------------------------------------
  // try to delete all planes whose amount is larger than the threshold.
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane (new pcl::PointCloud<pcl::PointXYZ>);
  while(1)
  {
    EstimateNormal(target_in,normal,50);
    SegmentPlane(target_in,normal,coefficients_plane,inliers_plane,0.01);  // the thickness of the plane.
    ExtractCloud(target_in,inliers_plane,plane,false);
    if(plane->points.size() > 2000)                                    // threshold that can be changed to determinate if the plane should be deleted or not.
    {
      ExtractCloud(target_in,inliers_plane,target_in,true);
      //RemoveOutlier(target_in,target_in,50,2);
    }
    else
    {
      break;
    }
    //ShowCloud(target_in);
  }
  // ---------------------------------------------------------------------------------------------------------------------
  // two point cloud here : target_in    is the point cloud for plate recognization
  //                        cloud_angle  is the point cloud for angle determination
 
  int j = 0;
  int try_times = 0;
  float fit_score = 0.0f;
  int best_index=-1;
  TemplateAlignment::Result best_alignment;
  infor.clear();
  food.clear();
  pcl::PCDWriter writer;
  std::vector<PlateInformation> all_infor;
  all_infor.clear();
  while(fit_score <= fit_threshold || try_times < 1)
  {
    // Assign to the target FeatureCloud
    std::cout<<"--------------------------------aligncloud-----------------------------------"<<std::endl;
    AlignCloud(target_in,templates_in,best_index,best_alignment);
    fit_score = best_alignment.icp_score;
 
    const FeatureCloud &best_template = templates_in[best_index];
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*best_template.getPointCloud (), *transformed_cloud, best_alignment.final_transformation);
    ExtractAlignedModel(target_in,transformed_cloud,best_alignment,0.01f);         // delete the points of the plate from the target_in Point Cloud.
    //ShowCloud(target_in);
    WriteCloud(target_in,"extract.pcd");
    if(fit_score <= fit_threshold)
    {
      try_times = 0;
 
      PlateInformation oneInfor;
      Eigen::Vector3f T2C = best_alignment.final_transformation.block<3,1>(0, 3);
      oneInfor.x=T2C[0];
      oneInfor.y=T2C[1];
 
      oneInfor.food = 0;
      GetRadius(*best_template.getPointCloud (),oneInfor.radius);
      checkFood(target_in,oneInfor);
      float top2desk;
      float topself;
      float bottom2desk;
      float bottomself;
      FindZMinMax(*best_template.getPointCloud(),bottomself,topself);
      FindZMinMax(*transformed_cloud,bottom2desk,top2desk);
      oneInfor.top2desk = top2desk;
      //oneInfor.topself = topself;
      oneInfor.bottom2deskC = bottom2desk;
      oneInfor.bottom2desk = bottom2desk;
      oneInfor.bottomself = bottomself;
      oneInfor.bottomselfC = bottomself;
 
      if(oneInfor.food == 0)
      {
        infor.push_back(oneInfor);
      }
      else
      {
        food.push_back(oneInfor);
      }
 
      std::stringstream ss;
      ss << "model_aligned_" << j << ".pcd";
      writer.write<pcl::PointXYZ> (ss.str (), *transformed_cloud, false); //*
      printf ("t = < %0.3f, %0.3f, %0.3f >\n", T2C[0], T2C[1], T2C[2]);
      std::cout<<"find a plate "<<std::endl;
      j++;
      std::cout << "--------------------" << std::endl;
      std::cout << fit_score << std::endl;
    }
    else
    {
      try_times += 1;
      std::cout<<"find a invalid target"<<std::endl;
    }
  }
  std::cout<< j << " plates have been found"<<std::endl;
  std::cerr << ">> Done: " << tt.toc () << " ms\n";
  return j;
}
 
 
 
void
ICPCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr target_in,pcl::PointCloud<pcl::PointXYZ>::Ptr model_in,pcl::PointCloud<pcl::PointXYZ>::Ptr& target_out,pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>& icp)
{
  pcl::console::TicToc tt;
  std::cerr << "ICP...\n", tt.tic ();
  icp.setInputSource(target_in);
  icp.setInputTarget(model_in);
  icp.setMaximumIterations(50);
  icp.setRANSACIterations (50);
  //icp.setMaxCorrespondenceDistance(0.01f*0.1f);
  icp.align(*target_out);
  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  std::cerr << ">> Done: " << tt.toc () << " ms\n";
}
