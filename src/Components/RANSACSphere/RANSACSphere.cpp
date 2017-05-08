/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "RANSACSphere.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

using namespace std;
namespace Processors {
namespace RANSACSphere {

RANSACSphere::RANSACSphere(const std::string & name) :
		Base::Component(name)  {

}

RANSACSphere::~RANSACSphere() {
}

void RANSACSphere::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_pcl", &in_pcl);
	registerStream("out_outliers", &out_outliers);
	registerStream("out_inliers", &out_inliers);
	// Register handlers
	h_ransac.setup(boost::bind(&RANSACSphere::ransac, this));
	registerHandler("ransac", &h_ransac);
	addDependency("ransac", &in_pcl);

}

bool RANSACSphere::onInit() {

	return true;
}

bool RANSACSphere::onFinish() {
	return true;
}

bool RANSACSphere::onStop() {
	return true;
}

bool RANSACSphere::onStart() {
	return true;
}

void RANSACSphere::ransac() {

	pcl::PointCloud<pcl::PointXYZ> cloud = in_pcl.read();
	
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_SPHERE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud.makeShared ());
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    //PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    cout<<"Could not estimate a planar model for the given dataset."<<endl;
  }
//info
  std::cout << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  std::cout << "Model inliers: " << inliers->indices.size () << std::endl;
  for (size_t i = 0; i < inliers->indices.size (); ++i)
    std::cout << inliers->indices[i] << "    " << cloud.points[inliers->indices[i]].x << " "
                                               << cloud.points[inliers->indices[i]].y << " "
                                               << cloud.points[inliers->indices[i]].z << std::endl;	
//////////////////////////

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliers (new pcl::PointCloud<pcl::PointXYZ> ());

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud.makeShared());
    extract.setIndices (inliers);
    extract.setNegative (false);

    extract.filter (*cloud_inliers);
    //std::cout << "PointCloud representing the planar component: " << cloud_inliers->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_outliers);
    //*cloud_filtered = *cloud_f;



out_outliers.write(cloud_outliers);
out_inliers.write(cloud_inliers);	
	
	
}



} //: namespace RANSACSphere
} //: namespace Processors
