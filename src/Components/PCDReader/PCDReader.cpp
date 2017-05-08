/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "PCDReader.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>


using namespace std;
namespace Processors {
namespace PCDReader {

PCDReader::PCDReader(const std::string & name) :
	Base::Component(name),
	filename("filename", std::string("")),
	read_on_init("read_on_init", true),
	prop_return_xyz("cloud.xyz", false),
	prop_return_xyzrgb("cloud.xyzrgb", false),
	prop_return_xyzsift("cloud.xyzsift", false)

{
	// Register property.
	registerProperty(filename);
	registerProperty(prop_return_xyz);
	registerProperty(prop_return_xyzrgb);
	registerProperty(prop_return_xyzsift);
	registerProperty(read_on_init);

	CLOG(LTRACE) << "Hi PCDReader\n";

}

PCDReader::~PCDReader() {
	CLOG(LTRACE) << "Bye PCDReader\n";
}

void PCDReader::prepareInterface() {
	// Register data streams.
	registerStream("in_trigger", &in_trigger);
	registerStream("out_cloud_xyz", &out_cloud_xyz);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);

	// Register handlers
	registerHandler("Read", boost::bind(&PCDReader::Read, this));

	registerHandler("onTriggeredLoadNextCloud", boost::bind(&PCDReader::onTriggeredLoadNextCloud, this));
	addDependency("onTriggeredLoadNextCloud", &in_trigger);
}

bool PCDReader::onInit() {
	// If propery set - read point cloud at start.
	if (read_on_init)
		Read();
	return true;
}

bool PCDReader::onFinish() {
	return true;
}

bool PCDReader::onStop() {
	return true;
}

bool PCDReader::onStart() {
	return true;
}

void PCDReader::onTriggeredLoadNextCloud(){
    CLOG(LDEBUG) << "PCDReader::onTriggeredLoadNextCloud";
    in_trigger.read();
    Read();
}

void PCDReader::Read() {
	CLOG(LTRACE) << "PCDReader::Read";
	  
	if (prop_return_xyz){
		// Try to read the cloud of XYZ points.
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
		if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud_xyz) == -1){
			CLOG(LWARNING) <<"Cannot read PointXYZ cloud from "<<filename;
		}else{
			out_cloud_xyz.write(cloud_xyz);
			CLOG(LINFO) <<"PointXYZ cloud of size "<< cloud_xyz->size() << " loaded properly from "<<filename;
		}//: else
	}//: else

	if (prop_return_xyzrgb){
		// Try to read the cloud of XYZRGB points.
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename, *cloud_xyzrgb) == -1){
			CLOG(LWARNING) <<"Cannot read PointXYZRGB cloud from "<<filename;
		}else{
			out_cloud_xyzrgb.write(cloud_xyzrgb);
			CLOG(LINFO) <<"PointXYZRGB cloud of size "<< cloud_xyzrgb->size() << " loaded properly from "<<filename;
		}//: else
	}//: else

	if (prop_return_xyzsift){
		// Try to read the cloud of XYZSIFT points.
		pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift (new pcl::PointCloud<PointXYZSIFT>);
		if (pcl::io::loadPCDFile<PointXYZSIFT> (filename, *cloud_xyzsift) == -1){
			CLOG(LWARNING) <<"Cannot read PointXYZSIFT cloud from "<<filename;
		}else{
			out_cloud_xyzsift.write(cloud_xyzsift);
			CLOG(LINFO) <<"PointXYZSIFT cloud of size "<< cloud_xyzsift->size() << " loaded properly from "<<filename;
		}//: else
	}//: else

}



} //: namespace PCDReader
} //: namespace Processors
