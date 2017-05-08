/*!
 * \file
 * \brief
 * \author Micha Laszkowski
 */

#include <memory>
#include <string>

#include "SphereGenerator.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <boost/random.hpp> 
#include <boost/random/normal_distribution.hpp> 

using namespace std;
namespace Processors {
namespace SphereGenerator {

SphereGenerator::SphereGenerator(const std::string & name) :
		Base::Component(name),
		r("r", 1),
		x("center.x", 0),
		y("center.y", 0),
		z("center.z", 0),
		nr_of_points("nr_of_points", 150),
		nr_of_outliers("nr_of_outliers", 10),
		mi("noise.mi", 0),
		sigma("noise.sigma", 0.001)   {
			registerProperty(r);
			registerProperty(x);
			registerProperty(y);
			registerProperty(z);
			registerProperty(nr_of_points);
			registerProperty(nr_of_outliers);
			registerProperty(mi);
			registerProperty(sigma);
			nr_of_points.addConstraint("0");
			nr_of_points.addConstraint("10000");
			nr_of_outliers.addConstraint("0");
			nr_of_outliers.addConstraint("1000");
}

SphereGenerator::~SphereGenerator() {
}

void SphereGenerator::prepareInterface() {
	// Register data streams, events and event handlers HERE!
registerStream("out_pcl_ptr", &out_pcl_ptr);
registerStream("out_pcl", &out_pcl);
	// Register handlers
	h_Generate.setup(boost::bind(&SphereGenerator::Generate, this));
	registerHandler("Generate", &h_Generate);

}

bool SphereGenerator::onInit() {
	Generate();

for (size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud.points[i].x << " " 
                        << cloud.points[i].y << " " 
                        << cloud.points[i].z << std::endl;	

	return true;
}

bool SphereGenerator::onFinish() {
	return true;
}

bool SphereGenerator::onStop() {
	return true;
}

bool SphereGenerator::onStart() {
	return true;
}

void SphereGenerator::Generate() {
//generate 
	if (nr_of_outliers > nr_of_points)
		nr_of_outliers = 0;
 
  // Fill in the cloud data
  cloud.width  = nr_of_points;
  cloud.height = 1;
  cloud.points.resize (cloud.width * cloud.height);

  // Generate the data
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x =  rand () / (RAND_MAX + 1.0f) * r;
    if (rand()%2)
		cloud.points[i].x = - cloud.points[i].x;
    cloud.points[i].y =  rand () / (RAND_MAX + 1.0f) * sqrt(r*r- cloud.points[i].x*cloud.points[i].x);
    if (rand()%2)
		cloud.points[i].y = - cloud.points[i].y;
    cloud.points[i].z =  sqrt(r*r - cloud.points[i].x*cloud.points[i].x - cloud.points[i].y*cloud.points[i].y);
    if (rand()%2)
		cloud.points[i].z = - cloud.points[i].z;
		
		cloud.points[i].x+=x;
		cloud.points[i].y+=y;
		cloud.points[i].z+=z;
  }




  // Set a few outliers
   for (int i = 0; i < nr_of_outliers; ++i){
	   cloud.points[i].x += (rand () / (RAND_MAX + 1.0f) * 10) -5;
	   cloud.points[i].y += (rand () / (RAND_MAX + 1.0f) * 10) -5;
	   cloud.points[i].z += (rand () / (RAND_MAX + 1.0f) * 10) -5;
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
        for (size_t i = nr_of_outliers; i < cloud.points.size (); ++i) 
        { 
          cloud.points[i].x += var_nor ();
          cloud.points[i].y += var_nor ();
          cloud.points[i].z += var_nor (); 
        } 

  

	
	out_pcl.write(cloud);	
	cloudPtr = cloud.makeShared();
	out_pcl_ptr.write(cloudPtr);
	
}


} //: namespace SphereGenerator
} //: namespace Processors
