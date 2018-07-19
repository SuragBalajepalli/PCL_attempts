//Trying to implement RANSAC using PCL
//Parameter tuning here
//Lets hope you work out because Hough Transforms are going over my head

#include <iostream>
#include <ros/ros.h>
//#include <pcl/control/parse.h> // nope
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h> // Loading clouds from .pcd?
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h> /// MVP
#include <pcl/sample_consensus/sac_model_plane.h> //Because I'm trying to fit a plane
#include <pcl/visualization/pcl_visualizer.h> // No need to save first and then load in rviz!
#include <boost/thread/thread.hpp> //Wat

float distance_threshold_ = 0.1;
int plane_ness = 50; //Bad job. But necessary for testing. Only for use while creating a random point cloud

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)   //Learn about this
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0,0,0);
	viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample_cloud");
	viewer->initCameraParameters ();
	return (viewer);
}

int main (int argc, char ** argv) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); // I want a pointer to a pointcloud of XYZ points. Call it cloud. Main cloud. Replace with model. 
	pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>); // After the above cloud is ransac'd, store it here

	//This section pertains to creating a random cloud. Must be removed when loading from pcd
	cloud->width = 50;
	cloud->height = 10;
	cloud->points.resize(cloud->width * cloud->height); //assigning a random width and height to the cloud and resizing to make it an ordered point cloud. 
	for( size_t i = 0; i < cloud->points.size(); ++i) {
		cloud->points[i].x = 1024 * rand()/(RAND_MAX + 1.0);
		cloud->points[i].y = 1024 * rand()/(RAND_MAX + 1.0);
		if (i > cloud->points.size() - plane_ness) {
			cloud->points[i].z = 1000;
		}
		else {
			cloud->points[i].z = 1024 * rand()/(RAND_MAX + 1.0);
		}

	}
	// Here ends population of random point cloud. Should have ideally made it a function but whatever

	std::vector<int> inliers;   // A vector to contain all the points that lie within our ransac limits

	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud)); //Object of SampleConsensusModelPlane 
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_plane); //Obj of RandomSampleConsensus with an object of SampleConsensusModelPlane as arg. Nice
	ransac.setDistanceThreshold(distance_threshold_); //Tunable parameter
	ransac.computeModel(); // Sets things in motion. MVP in action
	ransac.getInliers(inliers); // All the points that lie within ('inliers') the distance threshold from a computed plane
	// choses three random points, computes a plane between them. Finds all points that lie within a distance from the plane (distance threshold). 
	//Choses another three points randomly and computes the same stuff. If more points in new plane, it 'remembers' this one.
	//Returns largest plane (with most number of points) !!! Huge problem for finding spots for vacuum grasping. 
	
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final); //copies all points with indices as mentioned in 'inliers' from 'cloud' to 'final'. "This is a cat"
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer = simpleVis(final);
	while(!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}