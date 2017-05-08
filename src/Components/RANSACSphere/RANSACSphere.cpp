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
		Base::Component(name),
		radius("radius", 100),
		radius_dev("radius_dev", 10),
		distance("distance", 10)  {
	
	radius.addConstraint("1");
	radius.addConstraint("1000");
	registerProperty(radius);
	
	radius_dev.addConstraint("0");
	radius_dev.addConstraint("100");
	registerProperty(radius_dev);
	
	distance.addConstraint("1");
	distance.addConstraint("100");
	registerProperty(distance);
	
}

RANSACSphere::~RANSACSphere() {
}

void RANSACSphere::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_pcl", &in_pcl);
	registerStream("out_outliers", &out_outliers);
	registerStream("out_inliers", &out_inliers);
	registerStream("out_colored", &out_colored);
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
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_pcl.read();

	// RANSAC objects: model and algorithm. 
	pcl::SampleConsensusModelSphere<pcl::PointXYZRGB>::Ptr model(new pcl::SampleConsensusModelSphere<pcl::PointXYZRGB>(cloud)); 
	pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model); 
	model->setRadiusLimits(0.001*(radius-radius_dev), 0.001*(radius+radius_dev));  // The actual value is 7.25cm, but it even doesn't work with logically larger intervals as well. 
	ransac.setMaxIterations(100000); 
	ransac.setDistanceThreshold(0.001 * distance); 
	ransac.computeModel(); 

	std::vector<int> inliers; 
	ransac.getInliers(inliers); 
	if (inliers.size() == 0) 
	{ 
		CLOG(LERROR) << "Could not estimate a sphere model for the given dataset."; 
	} else {
		CLOG(LNOTICE) << inliers.size();
		for (int i = 0; i < inliers.size(); ++i) {
			(*cloud).at(inliers[i]).r = 255;
			(*cloud).at(inliers[i]).g = 0;
			(*cloud).at(inliers[i]).b = 0;
		}
	}
	
	out_colored.write(cloud);
}



} //: namespace RANSACSphere
} //: namespace Processors
