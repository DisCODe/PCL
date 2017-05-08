/*!
 * \file
 * \brief 
 * \author Micha Laszkowski
 */

#ifndef RANSACSPHERE_HPP_
#define RANSACSPHERE_HPP_

#include <Base/Component_Aux.hpp>
#include <Base/Component.hpp>
#include <Base/DataStream.hpp>
#include <Base/Property.hpp>
#include <Base/EventHandler2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>

using namespace std;
namespace Processors {
namespace RANSACSphere {

/*!
 * \class RANSACSphere
 * \brief RANSACSphere processor class.
 *
 * RANSACSphere processor.
 */
class RANSACSphere: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	RANSACSphere(const std::string & name = "RANSACSphere");

	/*!
	 * Destructor
	 */
	virtual ~RANSACSphere();

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

		Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ> > in_pcl;

// Output data streams

		Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr> out_outliers;
		Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr> out_inliers;
	// Handlers
	Base::EventHandler2 h_ransac;

	
	// Handlers
	void ransac();

};

} //: namespace RANSACSphere
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("RANSACSphere", Processors::RANSACSphere::RANSACSphere)

#endif /* RANSACSPHERE_HPP_ */
