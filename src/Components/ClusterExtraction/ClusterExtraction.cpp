/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "ClusterExtraction.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>


using namespace std;
namespace Processors {
namespace ClusterExtraction {

ClusterExtraction::ClusterExtraction(const std::string & name) :
		Base::Component(name),
		clusterTolerance("clusterTolerance", 0.02),
		minClusterSize("minClusterSize", 100),
		maxClusterSize("maxClusterSize", 25000)  {
			registerProperty(clusterTolerance);
			registerProperty(minClusterSize);
			registerProperty(maxClusterSize);
			minClusterSize.addConstraint("0");
			minClusterSize.addConstraint("25000");
			maxClusterSize.addConstraint("100");
			maxClusterSize.addConstraint("100000");
}

ClusterExtraction::~ClusterExtraction() {
}

void ClusterExtraction::prepareInterface() {
	// Register data streams, events and event handlers HERE!
registerStream("in_pcl", &in_pcl);
registerStream("out_indices", &out_indices);
registerStream("out_clusters", &out_clusters);
	// Register handlers
	h_extract.setup(boost::bind(&ClusterExtraction::extract, this));
	registerHandler("extract", &h_extract);
	addDependency("extract", &in_pcl);

}

bool ClusterExtraction::onInit() {

	return true;
}

bool ClusterExtraction::onFinish() {
	return true;
}

bool ClusterExtraction::onStop() {
	return true;
}

bool ClusterExtraction::onStart() {
	return true;
}

void ClusterExtraction::extract() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_pcl.read();
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (clusterTolerance); // 2cm
  ec.setMinClusterSize (minClusterSize);
  ec.setMaxClusterSize (maxClusterSize);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
  
  //int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (cloud->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

	clusters.push_back(cloud_cluster);
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    //std::stringstream ss;
    //ss << "cloud_cluster_" << j << ".pcd";
    //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    //j++;
  }	
	
	//std::cout<<"j=="<<j<<endl;
	//std::cout<<clusters.size()<<endl;
	out_indices.write(cluster_indices);
	out_clusters.write(clusters);
	
}



} //: namespace ClusterExtraction
} //: namespace Processors
