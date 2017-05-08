/*!
 * \file
 * \brief 
 * \author Micha Laszkowski
 */

#ifndef SPHEREGENERATOR_HPP_
#define SPHEREGENERATOR_HPP_

#include <Base/Component_Aux.hpp>
#include <Base/Component.hpp>
#include <Base/DataStream.hpp>
#include <Base/Property.hpp>
#include <Base/EventHandler2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
namespace Processors {
namespace SphereGenerator {

/*!
 * \class SphereGenerator
 * \brief SphereGenerator processor class.
 *
 * SphereGenerator processor.
 */
class SphereGenerator: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	SphereGenerator(const std::string & name = "SphereGenerator");

	/*!
	 * Destructor
	 */
	virtual ~SphereGenerator();

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


// Output data streams

		Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr> out_pcl_ptr;
		Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ> > out_pcl;

	// Handlers
	Base::EventHandler2 h_Generate;

	
	// Handlers
	void Generate();



		pcl::PointCloud<pcl::PointXYZ> cloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr;
		
		
		Base::Property<float> r;
		Base::Property<float> x;
		Base::Property<float> y;
		Base::Property<float> z;
		Base::Property<float> mi;
		Base::Property<float> sigma;
		Base::Property<int> nr_of_points;
		Base::Property<int> nr_of_outliers;

};

} //: namespace SphereGenerator
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("SphereGenerator", Processors::SphereGenerator::SphereGenerator)

#endif /* SPHEREGENERATOR_HPP_ */
