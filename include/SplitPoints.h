/*
 * SplitPoints.h
 *
 *  Created on: 29 sept. 2015
 *      Author: yann
 */

#ifndef SPLITPOINTS_H_
#define SPLITPOINTS_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <vector>
#include <map>


#define minY -20
#define maxY 20
#define minX -20
#define maxX 20

#define TRAV_COLOR 64
#define NONTRAV_COLOR 255

#define OBSTACLE 3
#define TRAVERSABLE 1
#define NOT_TRAVERSABLE 2 
#define UNKNOWN 0

//Number of points necessary to perform an estimation


#define MAX_RANGE 5

class MatrixAccumulator{
    public:
        MatrixAccumulator();
        virtual ~MatrixAccumulator();	
	void splitPointsCallback(const sensor_msgs::PointCloud2ConstPtr);
	double MatrixAccumulator::normV(pcl::PointXYZ pt1, pcl::PointXYZ pt2 );
		
    protected:
	
    private:	

	std::string base_frame_;
	double max_range_;
	int nb_points_est_;
	
	double W_OBST_MAX2;
	double distance_secu;
	
	// Parameter of Hough
	int hgh_dp;
	int hgh_min_dist;
	int hgh_param1;
	int hgh_param2;
	int hgh_min_radius;
	int hgh_max_radius;

	double stepX, stepY;
	int nbX; 
	int nbY; 
	//int dims[2] = {nbX,nbY};
	
	ros::Subscriber sub;
	ros::Publisher marker_plane_pub_;
	ros::Publisher marker_cylinder_pub_;

	ros::Publisher arm_cmd_pub;
	ros::Publisher armPos_cmd_pub;
	ros::Publisher armTwist_cmd_pub;
	
	tf::TransformListener listener_;
	pcl::PointCloud<pcl::PointXYZ> lastpc_;
	pcl::PointCloud<pcl::PointXYZ> lastpcBBR_;
	
	//cv::Mat_<Accumulator> tab = cv::Mat_<Accumulator>(2,dims);
	std::map< std::pair<int, int>, std::vector<pcl::PointXYZ> > tab;
	std::map< std::pair<int, int>, uint > mapStat;
	std::map< std::pair<int, int>, double > mapBias;
	std::map< std::pair<int, int>, double[9]> mapPlane; // a,b,c,d,bias,nbPoint, B_tm1_T, B_tm1_NT, B_tm1_O
	std::map< std::pair<int, int>, double[7]> mapCircle; // x,y,r,z_min,z_max,bias,nbPoint
	
	
	visualization_msgs::MarkerArray mArray = visualization_msgs::MarkerArray();
	ros::NodeHandle n;
	
	cv::Mat imgResTrav;
	cv::Mat imgResNonTrav;
	sensor_msgs::ImagePtr imgResROS;
	sensor_msgs::ImagePtr imgResROS2;
	sensor_msgs::ImagePtr imgResROS3;
	sensor_msgs::ImagePtr imgResROS4;
	image_transport::Publisher imgPub4;
	image_transport::Publisher imgPub;
	image_transport::Publisher imgPub2;
	image_transport::Publisher imgPub3;
	
	
	// For the map of height	
	std::map< std::pair<int, int>, double[7] > mapHeight; // z_mean, z_biais, z_min, z_max, mu_tm1, sigma_tm1, errorZ
	std::map< std::pair<int, int>, double > mapConfidence;
	cv::Mat imgResHeight;
	cv::Mat imgResSigma;
	cv::Mat imgResError;
	
	
};

#endif /* SPLITPOINTS_H_ */

