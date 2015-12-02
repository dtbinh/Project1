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
#include "../include/planeRegression.h"
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
		
		double p_Zt_Xt(int Zt, int Xt);
		void setColorImgHeight( cv::Mat imgResHeight, int u, int v, double z );
		void setColorImgSigma( cv::Mat imgResSigma, int u, int v, double z );
		void setColorImgError( cv::Mat imgResError, int u, int v, double z );
		void splitPointsCallback(const sensor_msgs::PointCloud2ConstPtr);
		
    protected:
	
    private:	
	
	int countBeforeHough;
	int houghLimit;
	
	std::string base_frame_;
	double max_range_;
	int nb_points_est_;
	
	// Parameter of Hough
	int hgh_dp;
	int hgh_min_dist;
	int hgh_param1;
	int hgh_param2;
	int hgh_min_radius;
	int hgh_max_radius;

	// Parameters Estimation of the Statut
	double p_T_NT;
	double p_NT_NT;
	double p_O_NT;

	double p_T_T;
	double p_NT_T;
	double p_O_T;

	double p_T_O;
	double p_NT_O;
	double p_O_O;

	double p_T_U;
	double p_NT_U;
	double p_O_U;
	
	// Parameters Estimation Heightd
	double A_t, B_t, C_t; // State
	double R_t, Q_t; // Noise
	double sigma_t0; // Initial value of the gaussian variable of the height
	
	double stepX, stepY;
	int nbX; 
	int nbY; 
	//int dims[2] = {nbX,nbY};
	
	ros::Subscriber sub;
	ros::Publisher marker_plane_pub_;
	ros::Publisher marker_cylinder_pub_;
	
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

