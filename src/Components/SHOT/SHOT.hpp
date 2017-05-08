/*!
 * \file
 * \brief 
 * \author Micha Laszkowski
 */

#ifndef SHOT_HPP_
#define SHOT_HPP_

#include <Base/Component_Aux.hpp>
#include <Base/Component.hpp>
#include <Base/DataStream.hpp>
#include <Base/Property.hpp>
#include <Base/EventHandler2.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace std;
namespace Processors {
namespace SHOT {

/*!
 * \class SHOT
 * \brief SHOT processor class.
 *
 * SHOT processor.
 */
class SHOT: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	SHOT(const std::string & name = "SHOT");

	/*!
	 * Destructor
	 */
	virtual ~SHOT();

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

// Output data streams

		Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr> out_keypoints;
		

	// Handlers
	Base::EventHandler2 h_shot;

	
	// Handlers
	void shot();

};

} //: namespace SHOT
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("SHOT", Processors::SHOT::SHOT)

#endif /* SHOT_HPP_ */
