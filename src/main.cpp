#include <stdio.h>
#include <stdlib.h>

#include "../include/HeightEstimator.h"
#include "../include/SplitPoints.h"

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <vector>


int main(int argc, char **argv)
{	
	ros::init(argc, argv, "Project1");
	
	//~ HeightEstimator H;
	MatrixAccumulator M;	

	ros::spin();
	return 0;
}
