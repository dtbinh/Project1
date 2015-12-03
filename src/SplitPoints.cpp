#include "../include/SplitPoints.h"

static float H_THRES = 0.01;

MatrixAccumulator::MatrixAccumulator(): n("~") {

	n.param("base_frame",base_frame_,std::string("/world"));
	n.param("max_range",max_range_,5.0);
	n.param("nb_points_est", nb_points_est_, 100);
	
	n.param("stepX", stepX, 0.1);
	n.param("stepY", stepY, 0.1);
	nbX = (int)floor((maxX - minX)/stepX);
	nbY = (int)floor((maxY - minY)/stepY);
	ROS_INFO("%d", nbY);
	
	// Make sure TF is ready
	ros::Duration(1).sleep();

	//~ sub = n.subscribe("/vrep/depthSensor", 1, &MatrixAccumulator::splitPointsCallback, this);
	sub = n.subscribe("/vrep/hokuyoSensor", 1, &MatrixAccumulator::splitPointsCallback, this);
	marker_plane_pub_ = n.advertise<visualization_msgs::MarkerArray>("floor_plane",1);
	marker_cylinder_pub_ = n.advertise<visualization_msgs::MarkerArray>("floor_cylinder",1);

	//OpenCV image initialization
 	cv::Mat imgR(nbX, nbY, CV_8UC3, cv::Scalar(0,0,0));
	cv::Mat imgR2(nbX, nbY, CV_8UC3, cv::Scalar(0,0,0));
	cv::Mat imgR3(nbX, nbY, CV_8UC3, cv::Scalar(0,0,0));
	cv::Mat imgR4(nbX, nbY, CV_8UC3, cv::Scalar(0,0,0));
	cv::Mat imgR5(nbX, nbY, CV_8UC3, cv::Scalar(0,0,0));
	imgResTrav = imgR;
	imgResNonTrav = imgR2;
	imgResHeight = imgR3;
	imgResSigma = imgR4;
	imgResError = imgR5;

	//Image publisher initialization
	ros::NodeHandle(nh);
	image_transport::ImageTransport it(nh);
	image_transport::ImageTransport it2(nh);
	image_transport::ImageTransport it3(nh);
	image_transport::ImageTransport it4(nh);
	imgPub4 = it4.advertise("imageErr", 1);
	imgPub3 = it3.advertise("imageSig", 1);
	imgPub2 = it2.advertise("imageHei", 1);
	imgPub = it.advertise("imageNav", 1);

	ROS_INFO("-------------- Ready ----------------");
}


MatrixAccumulator::~MatrixAccumulator(){}

void MatrixAccumulator::splitPointsCallback(const sensor_msgs::PointCloud2ConstPtr msg){
		
	pcl::PointCloud<pcl::PointXYZ> temp;
	pcl::fromROSMsg(*msg, temp);
	// Make sure the point cloud is in the base-frame
	listener_.waitForTransform(base_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
	pcl_ros::transformPointCloud(base_frame_,msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);
	pcl_ros::transformPointCloud("/Hokuyo", msg->header.stamp, temp, msg->header.frame_id, lastpcBBR_, listener_);

	mArray.markers.clear();
	unsigned int n = temp.size();
	std::vector<size_t> pidx;

	for (int i=n-1; i>=0; i--) {	
		if(lastpc_[i].z > H_THRES)
		{
			ROS_INFO("Begin of potential obstacle at %d", i);
			while (lastpc_[i].z > 0.00 && i > 0) i--;
			if (i == 0) 
			{
				ROS_INFO("%0.2f %0.2f %0.2f %d", lastpc_[i].x, lastpc_[i].y, lastpc_[i].z, i);
				ROS_INFO("%0.2f %0.2f %0.2f %d", lastpcBBR_[i].x, lastpcBBR_[i].y, lastpcBBR_[i].z, i);
				break;
			}
			else 
			{
				ROS_INFO("End of obstacle at %d", i);				
				continue;
			}
		}
		
	}
}


