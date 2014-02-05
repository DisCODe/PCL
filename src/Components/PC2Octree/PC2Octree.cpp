/*!
 * \file
 * \brief
 * \author Tomek Kornuta,,,
 */

#include <memory>
#include <string>

#include "PC2Octree.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace PC2Octree {

PC2Octree::PC2Octree(const std::string & name) :
		Base::Component(name)  {

}

PC2Octree::~PC2Octree() {
}

void PC2Octree::prepareInterface() {
	// Register data streams.
	registerStream("in_cloud", &in_cloud_xyz);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	// Register handlers
	h_cloud_xyzrgb_to_octree.setup(boost::bind(&PC2Octree::cloud_xyzrgb_to_octree, this));
	registerHandler("cloud_xyzrgb_to_octree", &h_cloud_xyzrgb_to_octree);
	addDependency("cloud_xyzrgb_to_octree", &in_cloud_xyzrgb);


}

bool PC2Octree::onInit() {

	return true;
}

bool PC2Octree::onFinish() {
	return true;
}

bool PC2Octree::onStop() {
	return true;
}

bool PC2Octree::onStart() {
	return true;
}

void PC2Octree::cloud_xyzrgb_to_octree() {
	LOG(LTRACE) << "PC2Octree::cloud_xyzrgb_to_octree";
	// Read from dataport.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
    LOG(LINFO) << "features quantity: " << cloud -> size();

	// Set voxel resolution.
	float voxelSize = 0.01f;
	pcl::octree::OctreePointCloud<pcl::PointXYZRGB> octree (voxelSize);
	// Set input cloud.
	octree.setInputCloud(cloud);
	// Calculate bounding box of input cloud.
	octree.defineBoundingBox();

	// Add points from input cloud to octree.
	octree.addPointsFromInputCloud ();

	//...?


  // breadth-first iterator test

  unsigned int lastDepth = 0;
  unsigned int branchNodeCount = 0;
  unsigned int leafNodeCount = 0;

  bool leafNodeVisited = false;

  pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::BreadthFirstIterator bfIt;
  const pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::BreadthFirstIterator bfIt_end = octree.breadth_end();

  for (bfIt = octree.breadth_begin(); bfIt != bfIt_end; ++bfIt)
  {
    // tree depth of visited nodes must grow
    //ASSERT_EQ( bfIt.getCurrentOctreeDepth()>=lastDepth, true);
    lastDepth = bfIt.getCurrentOctreeDepth ();
    LOG(LWARNING) << "lastDepth" << lastDepth;
    
//    LOG(LWARNING) << "bfIt.getTreeDepth()" << bfIt.getTreeDepth();
//getLeafCount
//getBranchCount
    
//    bool 	branchHasChild
    if (bfIt.isBranchNode ())
    {
      branchNodeCount++;
      // leaf nodes are traversed in the end
      // ASSERT_EQ( leafNodeVisited, false);
      int childCount =0;
      for(int i=0; i<8; i++) {
//          LOG(LINFO) << "before dynamic cast " << bfIt.getCurrentOctreeNode();
          pcl::octree::OctreeBranchNode<pcl::PointXYZRGB>* branchNode;
          try {
          branchNode = static_cast< pcl::octree::OctreeBranchNode<pcl::PointXYZRGB>* >(bfIt.getCurrentOctreeNode());
          } catch(const std::bad_cast &ex) {
              LOG(LINFO) << "bad cast!";
              LOG(LINFO) << ex.what();
          }

//          LOG(LINFO) << "passed dynamic cast " << branchNode;
//          bfIt.getCurrentOctreeNode();
          if(branchNode->hasChild(i))
              childCount++;
      }
      LOG(LINFO) << "node child count: " << childCount;
    }

    if (bfIt.isLeafNode ())
    {
      leafNodeCount++;
      leafNodeVisited = true;
    }
  }

LOG(LWARNING) << "branchNodeCount: " << branchNodeCount;
LOG(LWARNING) << "leafNodeCount: " << leafNodeCount;

	// Delete octree data structure (pushes allocated nodes to memory pool!)
	// octree.deleteTree ();
}


} //: namespace PC2Octree
} //: namespace Processors
