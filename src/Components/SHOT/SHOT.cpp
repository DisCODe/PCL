/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "SHOT.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>


#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>



typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

using namespace std;
namespace Processors {
namespace SHOT {

SHOT::SHOT(const std::string & name) :
		Base::Component(name)  {

}

SHOT::~SHOT() {
}

void SHOT::prepareInterface() {
	// Register data streams, events and event handlers HERE!
registerStream("in_pcl", &in_pcl);
registerStream("out_keypoints", &out_keypoints);
	// Register handlers
	h_shot.setup(boost::bind(&SHOT::shot, this));
	registerHandler("shot", &h_shot);
	addDependency("shot", &in_pcl);

}

bool SHOT::onInit() {

	return true;
}

bool SHOT::onFinish() {
	return true;
}

bool SHOT::onStop() {
	return true;
}

bool SHOT::onStart() {
	return true;
}

void SHOT::shot() {
  pcl::PointCloud<PointType>::Ptr cloud = in_pcl.read();
  pcl::PointCloud<NormalType>::Ptr normals (new pcl::PointCloud<NormalType> ());
  pcl::PointCloud<PointType>::Ptr keypoints (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<DescriptorType>::Ptr descriptors (new pcl::PointCloud<DescriptorType> ());
  cout<<"dupa 1"<<endl;
  float model_ss_ = 0.01f;
  float descr_rad_ = 0.02f;
  cout<<"dupa 2"<<endl;
  //normals
  pcl::NormalEstimation<PointType, NormalType> norm_est;
  cout<<"dupa 2.1"<<endl;
  norm_est.setKSearch (0);
  cout<<"dupa 2.2"<<endl;
  norm_est.setRadiusSearch (0.1);
  cout<<"dupa 2.3"<<endl;
  norm_est.setInputCloud (cloud);
  cout<<"dupa 2.4"<<endl;
  norm_est.compute (*normals);
cout<<"dupa 3"<<endl;

  //indeces
  pcl::PointCloud<int> sampled_indices;
  pcl::UniformSampling<PointType> uniform_sampling;
  uniform_sampling.setInputCloud (cloud);
  uniform_sampling.setRadiusSearch (model_ss_);
  uniform_sampling.compute (sampled_indices);
  pcl::copyPointCloud (*cloud, sampled_indices.points, *keypoints);
  std::cout << "Model total points: " << cloud->size () << "; Selected Keypoints: " << keypoints->size () << std::endl;
cout<<"dupa 4"<<endl;

  //SHOT
  pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
  descr_est.setRadiusSearch (descr_rad_);
cout<<"dupa 5"<<endl;
  descr_est.setInputCloud (keypoints);
  descr_est.setInputNormals (normals);
  descr_est.setSearchSurface (cloud);
  descr_est.compute (*descriptors);
  
  //out_keypoints.write(keypoints);
cout<<"dupa 6"<<endl;

}



} //: namespace SHOT
} //: namespace Processors
