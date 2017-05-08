/*!
 * \file
 * \brief 
 * \author Micha Laszkowski
 */

#ifndef CLUSTEREXTRACTION_HPP_
#define CLUSTEREXTRACTION_HPP_

#include <Base/Component_Aux.hpp>
#include <Base/Component.hpp>
#include <Base/DataStream.hpp>
#include <Base/Property.hpp>
#include <Base/EventHandler2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace std;
namespace Processors {
namespace ClusterExtraction {

/*!
 * \class ClusterExtraction
 * \brief ClusterExtraction processor class.
 *
 * ClusterExtraction processor.
 */
class ClusterExtraction: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	ClusterExtraction(const std::string & name = "ClusterExtraction");

	/*!
	 * Destructor
	 */
	virtual ~ClusterExtraction();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();

protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();


// Input data streams

		Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr> in_pcl;
		
		Base::DataStreamOut<std::vector<pcl::PointIndices> > out_indices;
		Base::DataStreamOut<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> > out_clusters;

// Output data streams

	// Handlers
	Base::EventHandler2 h_extract;
	
	// Handlers
	void extract();
	
	Base::Property<float> clusterTolerance;
	Base::Property<int> minClusterSize;
	Base::Property<int> maxClusterSize;

};

} //: namespace ClusterExtraction
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("ClusterExtraction", Processors::ClusterExtraction::ClusterExtraction)

#endif /* CLUSTEREXTRACTION_HPP_ */
