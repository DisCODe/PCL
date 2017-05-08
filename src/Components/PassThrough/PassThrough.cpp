/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "PassThrough.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

using namespace std;
namespace Processors {
namespace PassThrough {

PassThrough::PassThrough(const std::string & name) :
		Base::Component(name) , 
        xa("x.min", 0),
        xb("x.max", 0),
        ya("y.min", 0),
        yb("y.max", 0),
        za("z.min", 0),
        zb("z.max", 0),
        negative_x("negative_x", false),
        negative_y("negative_y", false),
        negative_z("negative_z", false),
	pass_through("pass_through", false)
{
	registerProperty(xa);
	registerProperty(xb);
	registerProperty(ya);
	registerProperty(yb);
	registerProperty(za);
	registerProperty(zb);
	registerProperty(negative_x);
	registerProperty(negative_y);
	registerProperty(negative_z);
	registerProperty(pass_through);
}

PassThrough::~PassThrough() {
}

void PassThrough::prepareInterface() {
    // Register data streams, events and event handlers HERE!
    registerStream("in_cloud_xyz", &in_cloud_xyz);
    registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
    registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
    registerStream("out_cloud_xyz", &out_cloud_xyz);
    registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
    registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);
    // Register handlers
    registerHandler("filter_xyz", boost::bind(&PassThrough::filter_xyz, this));
    addDependency("filter_xyz", &in_cloud_xyz);
    registerHandler("filter_xyzrgb", boost::bind(&PassThrough::filter_xyzrgb, this));
    addDependency("filter_xyzrgb", &in_cloud_xyzrgb);
    registerHandler("filter_xyzsift", boost::bind(&PassThrough::filter_xyzsift, this));
    addDependency("filter_xyzsift", &in_cloud_xyzsift);
}

bool PassThrough::onInit() {

	return true;
}

bool PassThrough::onFinish() {
	return true;
}

bool PassThrough::onStop() {
	return true;
}

bool PassThrough::onStart() {
	return true;
}

void PassThrough::filter_xyz() {
	CLOG(LTRACE) <<"filter_xyz()";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();

	if (!pass_through) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud (cloud);
		pass.setFilterFieldName ("x");
		pass.setFilterLimits (xa, xb);
		pass.setFilterLimitsNegative (negative_x);
		pass.filter (*cloud_filtered);
		// Set resulting cloud as input.
		pass.setInputCloud (cloud_filtered);
		pass.setFilterFieldName ("y");
		pass.setFilterLimits (ya, yb);
		pass.setFilterLimitsNegative (negative_y);
		pass.filter (*cloud_filtered);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (za, zb);
		pass.setFilterLimitsNegative (negative_z);
		pass.filter (*cloud_filtered);
		out_cloud_xyz.write(cloud_filtered);
	} else 
		out_cloud_xyz.write(cloud);
}

void PassThrough::filter_xyzrgb() {
	LOG(LTRACE) <<"filter_xyzrgb()";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();

	if (!pass_through) {
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PassThrough<pcl::PointXYZRGB> pass;
		pass.setInputCloud (cloud);
		pass.setFilterFieldName ("x");
		pass.setFilterLimits (xa, xb);
		pass.setFilterLimitsNegative (negative_x);
		pass.filter (*cloud_filtered);
		// Set resulting cloud as input.
		pass.setInputCloud (cloud_filtered);
		pass.setFilterFieldName ("y");
		pass.setFilterLimits (ya, yb);
		pass.setFilterLimitsNegative (negative_y);
		pass.filter (*cloud_filtered);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (za, zb);
		pass.setFilterLimitsNegative (negative_z);
		pass.filter (*cloud_filtered);
		out_cloud_xyzrgb.write(cloud_filtered);
	} else 
		out_cloud_xyzrgb.write(cloud);

}

void PassThrough::filter_xyzsift() {
    LOG(LTRACE) <<"filter_xyzsift()";
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud = in_cloud_xyzsift.read();

	if (!pass_through) {
		pcl::PointCloud<PointXYZSIFT>::Ptr cloud_filtered (new pcl::PointCloud<PointXYZSIFT>);
		applyFilter(cloud, *cloud_filtered, "x", xa, xb, negative_x);
		applyFilter(cloud_filtered, *cloud_filtered, "y", ya, yb, negative_y);
		applyFilter(cloud_filtered, *cloud_filtered, "z", za, zb, negative_z);
		out_cloud_xyzsift.write(cloud_filtered);
	} else 
		out_cloud_xyzsift.write(cloud);

}

void PassThrough::applyFilter (pcl::PointCloud<PointXYZSIFT>::Ptr input, pcl::PointCloud<PointXYZSIFT> &output, std::string filter_field_name, float min, float max, bool negative)
{
    CLOG(LTRACE) << "applyFilter() " << filter_field_name;
    output.header = input->header;
    output.sensor_origin_ = input->sensor_origin_;
    output.sensor_orientation_ = input->sensor_orientation_;

    std::vector<int> indices;

    output.is_dense = true;
    applyFilterIndices (indices, input, filter_field_name, min, max, negative);
    CLOG(LTRACE)<< "Number of indices: "<< indices.size() <<endl;
    copyPointCloud (*input, indices, output);

}

void PassThrough::applyFilterIndices (std::vector<int> &indices, pcl::PointCloud<PointXYZSIFT>::Ptr input, std::string filter_field_name, float min, float max, bool negative)
{
    CLOG(LTRACE) << "applyFilterIndices() " << filter_field_name;
    pcl::IndicesPtr indices_(new vector<int>);
    for(int i = 0; i < input->size(); i++){
        indices_->push_back(i);
    }

    // The arrays to be used
    indices.resize (indices_->size ());
    int oii = 0; // oii = output indices iterator, rii = removed indices iterator
    // Has a field name been specified?

    if (filter_field_name.empty ())
    {
        // Only filter for non-finite entries then
        for (int iii = 0; iii < static_cast<int> (indices_->size ()); ++iii) // iii = input indices iterator
        {
            // Non-finite entries are always passed to removed indices
            if (!pcl_isfinite (input->points[(*indices_)[iii]].x) ||
                    !pcl_isfinite (input->points[(*indices_)[iii]].y) ||
                    !pcl_isfinite (input->points[(*indices_)[iii]].z))
            {
                continue;
            }
            indices[oii++] = (*indices_)[iii];
        }
    }
    else
    {
        // Attempt to get the field name's index
        std::vector<pcl::PCLPointField> fields;
        int distance_idx = pcl::getFieldIndex (*input, filter_field_name, fields);
        if (distance_idx == -1)
        {
            CLOG(LWARNING) << "applyFilterIndices Unable to find field name in point type.";
            indices.clear ();
            return;
        }
        // Filter for non-finite entries and the specified field limits
        for (int iii = 0; iii < static_cast<int> (indices_->size ()); ++iii) // iii = input indices iterator
        {
            // Non-finite entries are always passed to removed indices
            if (!pcl_isfinite (input->points[(*indices_)[iii]].x) ||
                    !pcl_isfinite (input->points[(*indices_)[iii]].y) ||
                    !pcl_isfinite (input->points[(*indices_)[iii]].z))
            {
                continue;
            }
            // Get the field's value
            const uint8_t* pt_data = reinterpret_cast<const uint8_t*> (&input->points[(*indices_)[iii]]);
            float field_value = 0;
            memcpy (&field_value, pt_data + fields[distance_idx].offset, sizeof (float));
            // Remove NAN/INF/-INF values. We expect passthrough to output clean valid data.
            if (!pcl_isfinite (field_value))
            {
                continue;
            }
            // Outside of the field limits are passed to removed indices
            if (!negative && (field_value < min || field_value > max))
            {
                continue;
            }
            // Inside of the field limits are passed to removed indices if negative was set
            if (negative && field_value >= min && field_value <= max)
            {
                continue;
            }
            // Otherwise it was a normal point for output (inlier)
            indices[oii++] = (*indices_)[iii];
        }
    }
    // Resize the output arrays
    indices.resize (oii);
}



} //: namespace PassThrough
} //: namespace Processors
