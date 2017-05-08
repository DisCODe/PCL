/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "PlaneGenerator.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <boost/random.hpp> 
#include <boost/random/normal_distribution.hpp> 

using namespace std;
namespace Processors {
namespace PlaneGenerator {

PlaneGenerator::PlaneGenerator(const std::string & name) :
		Base::Component(name),
		nr_of_points("nr_of_points", 150),
		nr_of_outliers("nr_of_outliers", 10),
		a("equation.a", 1.0),
		b("equation.b", 0.0),
		c("equation.c", 0.0),
		d("equation.d", 0.0),
		mi("noise.mi", 0.0),
		sigma("noise.sigma", 0.001)    {
			
			registerProperty(nr_of_points);
			registerProperty(nr_of_outliers);
			registerProperty(a);
			registerProperty(b);
			registerProperty(c);
			registerProperty(d);
			registerProperty(mi);
			registerProperty(sigma);
			nr_of_points.addConstraint("0");
			nr_of_points.addConstraint("10000");
			nr_of_outliers.addConstraint("0");
			nr_of_outliers.addConstraint("1000");
}

PlaneGenerator::~PlaneGenerator() {
}

void PlaneGenerator::prepareInterface() {
	// Register data streams, events and event handlers HERE!
registerStream("out_pcl", &out_pcl);
	// Register handlers
	h_Generate.setup(boost::bind(&PlaneGenerator::Generate, this));
	registerHandler("Generate", &h_Generate);

}

bool PlaneGenerator::onInit() {
	Generate();

	return true;
}

bool PlaneGenerator::onFinish() {
	return true;
}

bool PlaneGenerator::onStop() {
	return true;
}

bool PlaneGenerator::onStart() {
	return true;
}

void PlaneGenerator::Generate() {
	
if (nr_of_outliers > nr_of_points)
		nr_of_outliers = 0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  
  cloud->width  = nr_of_points;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

// Create a set of planar coefficients 
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = a;
  coefficients->values[1] = b;
  coefficients->values[2] = c;
  coefficients->values[3] = d;


  // Create the filtering object
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud);


 // Set a few outliers
   for (int i = 0; i < nr_of_outliers; ++i){
	   cloud->points[i].x += (rand () / (RAND_MAX + 1.0f) * 5) +1;
	   if (rand()%2)
			cloud->points[i].x=-cloud->points[i].x;
	   cloud->points[i].y += (rand () / (RAND_MAX + 1.0f) * 5) +1;
	   if (rand()%2)
			cloud->points[i].y=-cloud->points[i].y;
	   cloud->points[i].z += (rand () / (RAND_MAX + 1.0f) * 5) +1;
	   if (rand()%2)
			cloud->points[i].z=-cloud->points[i].z;
   }
	
//noise
        struct timeval start; 
        gettimeofday (&start, NULL); 
        boost::mt19937 rng; 
        rng.seed (start.tv_usec); 
        boost::normal_distribution<> nd (mi, sigma); 
        boost::variate_generator<boost::mt19937&, 
			boost::normal_distribution<> > var_nor (rng, nd); 
        // Noisify each point in the dataset 
        for (size_t i = nr_of_outliers; i < cloud->points.size (); ++i) 
        { 
          cloud->points[i].x += var_nor ();
          cloud->points[i].y += var_nor ();
          cloud->points[i].z += var_nor (); 
        } 



	out_pcl.write(cloud); 
	
}



} //: namespace PlaneGenerator
} //: namespace Processors
