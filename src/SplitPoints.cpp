/*
 * SplitPoints.cpp
 *
 *  Created on: 29 sept. 2015
 *      Author: yann
 */

#include "../include/SplitPoints.h"


MatrixAccumulator::MatrixAccumulator(): n("~") {
	ROS_INFO(" Hello World ! :) ");

	n.param("base_frame",base_frame_,std::string("/VSV/platform"));
	n.param("max_range",max_range_,5.0);
	n.param("nb_points_est", nb_points_est_, 100);
	
	n.param("stepX", stepX, 0.1);
	n.param("stepY", stepY, 0.1);
	nbX = (int)floor((maxX - minX)/stepX);
	nbY = (int)floor((maxY - minY)/stepY);
	ROS_INFO("%d", nbY);
	
	// --------------------------
	// Parameters for Hough transform
	n.param("hgh_dp",hgh_dp,1);
	n.param("hgh_min_dist", hgh_min_dist, nbY/2);
	n.param("hgh_param1",hgh_param1,1);
	n.param("hgh_param2",hgh_param2,10);
	n.param("hgh_min_radius",hgh_min_radius,0); // 0 disable
	n.param("hgh_max_radius",hgh_max_radius,0); // 0 disable

	n.param("HoughLimit", houghLimit, 200);
	
	// --------------------------
	// Parameters Estimation of the Statut
	n.param("p_T_NT",p_T_NT,0.);
	n.param("p_NT_NT", p_NT_NT, 1.);
	n.param("p_O_NT", p_O_NT, 0.);

	n.param("p_T_T",p_T_T,1.);
	n.param("p_NT_T", p_NT_T, 0.);
	n.param("p_O_T", p_O_T, 0.);

	n.param("p_T_O",p_T_T,0.);
	n.param("p_NT_O", p_NT_T, 0.);
	n.param("p_O_O", p_O_T, 1.);

	n.param("p_T_U",p_T_U,0.7);
	n.param("p_NT_U",p_NT_U,0.2);
	n.param("p_O_U",p_O_U,0.1);
	
	// --------------------------
	// Parameters Estimation of the Height	
	//n.param("z_MIN2",z_MIN2,-2.0);
	//n.param("z_MAX2",z_MAX2,2.0);
	n.param("A_t",A_t,0.1);
	n.param("B_t",B_t,0.0);
	n.param("C_t",C_t,0.1);
	n.param("R_t",R_t,0.1);
	n.param("Q_t",Q_t,0.1);
	n.param("sigma_t0",sigma_t0,0.1);
	
	// Make sure TF is ready
	ros::Duration(1).sleep();

	//~ sub = n.subscribe("/vrep/depthSensor", 1, &MatrixAccumulator::splitPointsCallback, this);
	sub = n.subscribe("/vrep/hokuyoSensor", 1, &MatrixAccumulator::splitPointsCallback, this);
	marker_plane_pub_ = n.advertise<visualization_msgs::MarkerArray>("floor_plane",1);
	marker_cylinder_pub_ = n.advertise<visualization_msgs::MarkerArray>("floor_cylinder",1);

	ROS_INFO(" Hello World ! :) 2 ");

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

	countBeforeHough = 0;
}


MatrixAccumulator::~MatrixAccumulator(){
}




double MatrixAccumulator::p_Zt_Xt(int Zt, int Xt){
	double res = 0;
	if(Zt == Xt){
		res = 0.85;
	}
	else{
		res = 0.15;
	}


	return res;
}
 

void MatrixAccumulator::setColorImgHeight( cv::Mat imgResHeight, int u, int v, double z ){
		
	double z_MIN2 = -0.05;
	double z_MAX2 = 0.7;
	double rangeZMid = (z_MAX2 - z_MIN2)/2.0;
	double stepColor = 400 / rangeZMid;
		
		
	if(z < z_MAX2 && z > z_MIN2){
		int redMinus = (z - z_MIN2)* stepColor;
		redMinus = redMinus > 200 ? 200 : redMinus;
		redMinus = redMinus < 0 ? 0 : redMinus;
		
		//int greenMinus = (z - z_MIN2 - rangeZMid) * stepColor;
		int greenMinus = (z) * stepColor;
		greenMinus = greenMinus < 0 ? 0 : greenMinus;	
		greenMinus = greenMinus > 200 ? 200 : greenMinus;	
		
		(imgResHeight.at<cv::Vec3b>(u, v))[0] = 51;					// B
		(imgResHeight.at<cv::Vec3b>(u, v))[1] = 253 - greenMinus;	// G
		(imgResHeight.at<cv::Vec3b>(u, v))[2] = 51  + redMinus;		// R
	}
	else{
		
		// If flat
		(imgResHeight.at<cv::Vec3b>(u, v))[0] = 204;		// B
		(imgResHeight.at<cv::Vec3b>(u, v))[1] = 0;			// G
		(imgResHeight.at<cv::Vec3b>(u, v))[2] = 204;		// R
	}
			
}

void MatrixAccumulator::setColorImgError( cv::Mat imgResError, int u, int v, double z ){
		
	double z_MIN2 = 0;
	double z_MAX2 = 0.0001;
	double rangeZMid = (z_MAX2 - z_MIN2)/2.0;
	double stepColor = 400 / rangeZMid;
		
		
	if(z <= z_MAX2 && z >= z_MIN2){
		int redMinus = (z - z_MIN2)* stepColor;
		redMinus = redMinus > 200 ? 200 : redMinus;
		redMinus = redMinus < 0 ? 0 : redMinus;
		
		//int greenMinus = (z - z_MIN2 - rangeZMid) * stepColor;
		int greenMinus = (z) * stepColor;
		greenMinus = greenMinus < 0 ? 0 : greenMinus;	
		greenMinus = greenMinus > 200 ? 200 : greenMinus;	
		
		(imgResError.at<cv::Vec3b>(u, v))[0] = 51;					// B
		(imgResError.at<cv::Vec3b>(u, v))[1] = 253 - greenMinus;	// G
		(imgResError.at<cv::Vec3b>(u, v))[2] = 51  + redMinus;		// R
	}
	else{
		
		// If flat
		(imgResError.at<cv::Vec3b>(u, v))[0] = 204;		// B
		(imgResError.at<cv::Vec3b>(u, v))[1] = 0;			// G
		(imgResError.at<cv::Vec3b>(u, v))[2] = 204;		// R
	}
			
}


void MatrixAccumulator::setColorImgSigma( cv::Mat imgResSigma, int u, int v, double z ){
	double z_MIN2 = 0;
	double z_MAX2 = 0.011;
	double rangeZMid = (z_MAX2 - z_MIN2)/2.0;
	double stepColor = 200 / rangeZMid;
		
		
	if(z < z_MAX2 && z > z_MIN2){
		int redMinus = (z - z_MIN2) * stepColor;
		redMinus = redMinus > 200 ? 200 : redMinus;
		
		int greenMinus = (z - z_MIN2 - rangeZMid) * stepColor;
		greenMinus = greenMinus < 0 ? 0 : greenMinus;	
		
		(imgResSigma.at<cv::Vec3b>(u, v))[0] = 51;					// B
		(imgResSigma.at<cv::Vec3b>(u, v))[1] = 253 - greenMinus;	// G
		(imgResSigma.at<cv::Vec3b>(u, v))[2] = 51  + redMinus;		// R
	}
	else{
		
		// If flat
		(imgResSigma.at<cv::Vec3b>(u, v))[0] = 204;		// B
		(imgResSigma.at<cv::Vec3b>(u, v))[1] = 0;		// G
		(imgResSigma.at<cv::Vec3b>(u, v))[2] = 204;		// R
	}
			
}


void MatrixAccumulator::splitPointsCallback(const sensor_msgs::PointCloud2ConstPtr msg){
		
	pcl::PointCloud<pcl::PointXYZ> temp;
	pcl::fromROSMsg(*msg, temp);
	// Make sure the point cloud is in the base-frame
	listener_.waitForTransform(base_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
	pcl_ros::transformPointCloud(base_frame_,msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);
	//~ pcl_ros::transformPointCloud("/kinect_visionSensor", msg->header.stamp, temp, msg->header.frame_id, lastpcBBR_, listener_);
	pcl_ros::transformPointCloud("/Hokuyo", msg->header.stamp, temp, msg->header.frame_id, lastpcBBR_, listener_);

	mArray.markers.clear();
	unsigned int n = temp.size();
	std::vector<size_t> pidx;
	
	double newPlane[9]; // a,b,c,d,bias,nbPoint, B_tm1_T, B_tm1_NT, B_tm1_O
	double newCellData[6]; // z_mean, z_biais, z_min, z_max, mu_tm1, sigma_tm1
	//double newCircle[7]; // x,y,r,z_min,z_max,bias,nbPoint
	// First count the useful points
	for (unsigned int i=0;i<n;i++) {	
	
		/*************************************/
		/**********  Check Outlier  **********/
		/*************************************/
		const pcl::PointXYZ & T = temp[i];
		double d = hypot(T.x,T.y);
		// In the sensor frame, this point would be inside the camera
		if (d < 1e-2) {
			// Bogus point, ignore
			continue;
		}
		if(lastpc_[i].x < minX || lastpc_[i].x > maxX) continue;
		if(lastpc_[i].y < minY || lastpc_[i].y > maxY) continue;
		d = hypot(lastpcBBR_[i].x, lastpcBBR_[i].y);
		if (d > max_range_) continue;

		//ROS_INFO("Point found x: %0.2f", lastpc_[i].x);
		//ROS_INFO("Point found y: %0.2f", lastpc_[i].y);
		
		// Compute index of the right accumulator		
		int u = (int) floor((lastpc_[i].x - minX)/stepX);
		int v = (int) floor((lastpc_[i].y - minY)/stepY);

		tab[{u,v}].push_back(lastpc_[i]);

		//ROS_INFO("---------i- %d ", tab[{u,v}].size());
		if(tab[{u,v}].size() < nb_points_est_){
			// too few points, ignore
			continue;
		}
	}
	
	//Display the image
	cv::imshow("Mapping result", imgResTrav);
	//cv::imshow("Mapping of obstacles", imgResNonTrav);
	cv::imshow("Mapping of the Height", imgResHeight);
	cv::imshow("Mapping of the Sigma", imgResSigma);

	//Waits for D ms
	cv::waitKey(1);

	//Build ROS message
	// imgResROS = cv_bridge::CvImage(std_msgs::Header(), "rgba8", imgResTrav).toImageMsg();
	imgResROS2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgResHeight).toImageMsg();
	imgResROS = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgResTrav).toImageMsg();
	imgResROS3 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgResSigma).toImageMsg();
	imgResROS4 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgResError).toImageMsg();
	imgPub4.publish(imgResROS4);
	imgPub.publish(imgResROS);
	imgPub2.publish(imgResROS2);
	imgPub3.publish(imgResROS3);
	countBeforeHough++;
}


