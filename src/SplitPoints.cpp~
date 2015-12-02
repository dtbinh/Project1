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

		
		/*****************************************/
		/**********  Compute the plane  **********/
		/*****************************************/
		floorRansac(tab[{u,v}], newPlane);		
		// Add circle and Plane if not have already been initialized yet
		if( mapPlane.count({u,v}) == 0){
			(mapPlane[{u,v}])[0] = newPlane[0];
			(mapPlane[{u,v}])[1] = newPlane[1];
			(mapPlane[{u,v}])[2] = newPlane[2];
			(mapPlane[{u,v}])[3] = newPlane[3];
			(mapPlane[{u,v}])[4] = newPlane[4];
			(mapPlane[{u,v}])[5] = 0;
			(mapPlane[{u,v}])[6] = p_T_U;
			(mapPlane[{u,v}])[7] = p_NT_U;
			(mapPlane[{u,v}])[8] = p_O_U;
		}
		
		
		//ROS_INFO("------plane  : - a= %0.2f,  b= %0.2f, c= %0.2f, d= %0.2f ", (mapPlane[{u,v}])[0], (mapPlane[{u,v}])[1], (mapPlane[{u,v}])[2], (mapPlane[{u,v}])[3]);
		//ROS_INFO("------circle : - a= %0.2f,  b= %0.2f, r= %0.2f, z_min= %0.2f, z_max= %0.2f", (mapCircle[{u,v}])[0], (mapCircle[{u,v}])[1], (mapCircle[{u,v}])[2], (mapCircle[{u,v}])[3], (mapCircle[{u,v}])[4]);


		/**************************************************************/
		/**********  Check the orientation of the new plane  **********/
		/**************************************************************/
		Eigen::Vector3f nNormal;
		nNormal << newPlane[0],newPlane[1],newPlane[2];
		 
		if(nNormal.norm() == 0){
			// not a plane, ignore
			continue;
		}
		nNormal /= nNormal.norm();
		
		Eigen::Vector3f zA;
		zA << 0.0, 0.0, 1.0;
		
		// If plane angle > Pi/2, Then Not Traversable, Else Traversable
		if( fabs(zA.dot(nNormal)) < 0.01  ){
			mapStat[{u,v}] = OBSTACLE;
			//imgResTrav.at<uchar>(u, v) = NONTRAV_COLOR;
			(imgResTrav.at<cv::Vec3b>(u, v))[0] = 153;
			(imgResTrav.at<cv::Vec3b>(u, v))[1] = 153;
			(imgResTrav.at<cv::Vec3b>(u, v))[2] = 253;
			imgResNonTrav.at<uchar>(u, v, 0) = NONTRAV_COLOR;
			//ROS_INFO("------------------  NOT  --------- Traversable");
		}
		else if (fabs(zA.dot(nNormal)) < 0.80  ){  
			mapStat[{u,v}] = NOT_TRAVERSABLE;
			//imgResTrav.at<uchar>(u, v) = 196;
			(imgResTrav.at<cv::Vec3b>(u, v))[0] = 253;
			(imgResTrav.at<cv::Vec3b>(u, v))[1] = 153;
			(imgResTrav.at<cv::Vec3b>(u, v))[2] = 253;
			//imgResNonTrav.at<uchar>(u, v, 0) = NONTRAV_COLOR;
			//ROS_INFO("------------------  NOT  --------- Traversable");
		}
		else{
			mapStat[{u,v}] = TRAVERSABLE;
			//imgResTrav.at<uchar>(u, v) = TRAV_COLOR;
			//(imgResTrav.at<cv::Vec3b>(u, v))[0] = 0;
			// Vert claire
			(imgResTrav.at<cv::Vec3b>(u, v))[0] = 153;
			(imgResTrav.at<cv::Vec3b>(u, v))[1] = 254;
			(imgResTrav.at<cv::Vec3b>(u, v))[2] = 153;
			//ROS_INFO("------------------Traversable ");
		}
		
		(mapPlane[{u,v}])[0] = newPlane[0];
		(mapPlane[{u,v}])[1] = newPlane[1];
		(mapPlane[{u,v}])[2] = newPlane[2];
		(mapPlane[{u,v}])[3] = newPlane[3];
		(mapPlane[{u,v}])[4] = newPlane[4];
		(mapPlane[{u,v}])[5] ++; // Count the number of time that the process happen on this cell
		


		/*****************************************************/
		/**********  Bayesian recursive estimation  **********/
		/*****************************************************/
		double B_tm1_T  = (mapPlane[{u,v}])[6];
		double B_tm1_NT = (mapPlane[{u,v}])[7];
		double B_tm1_O  = (mapPlane[{u,v}])[8];

		// Try each possibilities of statut
		/*double B_t_T  = p_Zt_Xt(mapStat[{u,v}], TRAVERSABLE)     * ( p_T_T *B_tm1_T + p_T_NT *B_tm1_NT  + p_T_O *B_tm1_O );
		double B_t_NT = p_Zt_Xt(mapStat[{u,v}], NOT_TRAVERSABLE) * ( p_NT_T*B_tm1_T + p_NT_NT*B_tm1_NT  + p_NT_O*B_tm1_O );
		double B_t_O  = p_Zt_Xt(mapStat[{u,v}], OBSTACLE)        * ( p_O_T *B_tm1_T + p_O_NT *B_tm1_NT  + p_O_O *B_tm1_O );*/
		double B_t_T  = p_Zt_Xt(mapStat[{u,v}], TRAVERSABLE)     * ( 0.7 *B_tm1_T + 0.1 *B_tm1_NT  + 0.0 *B_tm1_O );
		double B_t_NT = p_Zt_Xt(mapStat[{u,v}], NOT_TRAVERSABLE) * ( 0.3 *B_tm1_T + 0.9 *B_tm1_NT  + 0.0*B_tm1_O );
		double B_t_O  = p_Zt_Xt(mapStat[{u,v}], OBSTACLE)        * ( 0.0 *B_tm1_T + 0.0 *B_tm1_NT  + 1. *B_tm1_O );
		
		double normalize = 1;
		if(!(B_t_T == 0 && B_t_NT == 0 && B_t_O == 0)){
			normalize /= ( B_t_T + B_t_NT + B_t_O )	;		
		}
		B_t_T *= normalize;
		B_t_NT*= normalize;
		B_t_O *= normalize;

		// Find the best
		double maxB_t = max(B_t_T,max(B_t_NT,B_t_O));
	
		// Create the right color for the map
		int redu = 225000*(maxB_t);
		int reduP = 0;
		if(redu > 150){
			reduP = redu / 150;
			redu = 150;
		}
		if(reduP > 150){
			reduP = 150;
		}
		if(maxB_t == B_t_T){
			if(mapStat[{u,v}] != TRAVERSABLE){ // Check if the status change between 2 iterations
				//(mapPlane[{u,v}])[5] = 0;
				//(mapPlane[{u,v}])[5]--;
				//(mapPlane[{u,v}])[5]--;
				(imgResHeight.at<cv::Vec3b>(u, v))[0] = 0;
				(imgResHeight.at<cv::Vec3b>(u, v))[1] = 0;
				(imgResHeight.at<cv::Vec3b>(u, v))[2] = 0;
				(imgResSigma.at<cv::Vec3b>(u, v))[0] = 0;
			}
			mapStat[{u,v}] = TRAVERSABLE;
			//imgResTrav.at<uchar>(u, v, 0) = TRAV_COLOR;
			(imgResTrav.at<cv::Vec3b>(u, v))[0] = 153 - redu;
			(imgResTrav.at<cv::Vec3b>(u, v))[1] = 254 - reduP;
			(imgResTrav.at<cv::Vec3b>(u, v))[2] = 153 - redu;
			(imgResNonTrav.at<cv::Vec3b>(u, v))[0] = 0;
		}
		else if(maxB_t == B_t_NT){
			if(mapStat[{u,v}] != NOT_TRAVERSABLE){ // Check if the status change between 2 iterations
				//(mapPlane[{u,v}])[5] = 0;
				//(mapPlane[{u,v}])[5]--;
				//(mapPlane[{u,v}])[5]--;
				(imgResHeight.at<cv::Vec3b>(u, v))[0] = 0;
				(imgResHeight.at<cv::Vec3b>(u, v))[1] = 0;
				(imgResHeight.at<cv::Vec3b>(u, v))[2] = 0;
				(imgResSigma.at<cv::Vec3b>(u, v))[0] = 0;
			}
			mapStat[{u,v}] = NOT_TRAVERSABLE;
			//imgResTrav.at<uchar>(u, v, 0) = NONTRAV_COLOR;
			(imgResTrav.at<cv::Vec3b>(u, v))[0] = 253 - reduP;
			(imgResTrav.at<cv::Vec3b>(u, v))[1] = 153 - redu;
			(imgResTrav.at<cv::Vec3b>(u, v))[2] = 253 - reduP;
			(imgResNonTrav.at<cv::Vec3b>(u, v))[0] = NONTRAV_COLOR;
		}
		else{
			if(mapStat[{u,v}] != OBSTACLE){ // Check if the status change between 2 iterations
				//(mapPlane[{u,v}])[5] = 0;
				//(mapPlane[{u,v}])[5]--;
				//(mapPlane[{u,v}])[5]--;
				(imgResHeight.at<cv::Vec3b>(u, v))[0] = 0;
				(imgResHeight.at<cv::Vec3b>(u, v))[1] = 0;
				(imgResHeight.at<cv::Vec3b>(u, v))[2] = 0;
				(imgResSigma.at<cv::Vec3b>(u, v))[0] = 0;
			}
			mapStat[{u,v}] = OBSTACLE;
			//imgResTrav.at<uchar>(u, v, 0) = NONTRAV_COLOR;
			(imgResTrav.at<cv::Vec3b>(u, v))[0] = 153 - redu;
			(imgResTrav.at<cv::Vec3b>(u, v))[1] = 153 - redu;
			(imgResTrav.at<cv::Vec3b>(u, v))[2] = 253 - reduP;
			(imgResNonTrav.at<cv::Vec3b>(u, v))[0] = NONTRAV_COLOR;
		}
		(mapPlane[{u,v}])[6] = B_t_T;
		(mapPlane[{u,v}])[7] = B_t_NT;
		(mapPlane[{u,v}])[8] = B_t_O;
		//ROS_INFO("------Height : - B_t_T= %0.2f, B_t_NT= %0.2f,  B_t_O= %0.2f", (mapPlane[{u,v}])[6], (mapPlane[{u,v}])[7], (mapPlane[{u,v}])[8]);
		
		
		/****************************************************/
		/**********  Get proprieties of this cell  **********/
		/****************************************************/
		if(mapStat[{u,v}] != OBSTACLE){			
			floorHeight(tab[{u,v}], newCellData);
			// Init mapHeight if needed
			if( mapHeight.count({u,v}) == 0){
				(mapHeight[{u,v}])[0] = newCellData[0]; // z_mean
				(mapHeight[{u,v}])[1] = newCellData[1]; // z_biais
				(mapHeight[{u,v}])[2] = newCellData[2]; // z_min
				(mapHeight[{u,v}])[3] = newCellData[3]; // z_max
				(mapHeight[{u,v}])[4] = newCellData[0]; // mu_tm1 = z_mean at t0
				(mapHeight[{u,v}])[5] = newCellData[1]; //	// sigma_tm1
				//(mapHeight[{u,v}])[5] = 0.0; 	// errorZ
				setColorImgHeight( imgResHeight, u, v, newCellData[0]);
				setColorImgSigma( imgResSigma, u, v, sigma_t0);	
				setColorImgError( imgResError, u, v, 0.0);	
				//ROS_INFO("------Height : - z_min= %0.2f, z_max= %0.2f,  sigma= %0.2f", (mapHeight[{u,v}])[2], (mapHeight[{u,v}])[3], (mapHeight[{u,v}])[5]);
			}		
		 
		
			/*******************************************************/
			/**********  Update proprieties of this cell  **********/
			/*******************************************************/
			// --- Kalman Filter :
			double mu_b_t = A_t * (mapHeight[{u,v}])[4]; // A*mu_tm1 + B_t*u_t =  A*mu_tm1
			double sigma_b_t = A_t*A_t * (mapHeight[{u,v}])[5]  + R_t; // A*sigma_tm1*A + R_t
				
			double K_t = sigma_b_t * C_t / ( C_t*C_t*sigma_b_t + Q_t);
			double mu_t = mu_b_t + K_t*(newCellData[0] - C_t*mu_b_t); // mu_b_t + K_t(z_t - C*mu_b_t)
			double sigma_t = (1 - K_t*C_t)*sigma_b_t;

			double errorZ = (newCellData[0] - mu_t);
			errorZ *= errorZ;
			
			
			(mapHeight[{u,v}])[0] = newCellData[0];
			(mapHeight[{u,v}])[1] = newCellData[1];
			(mapHeight[{u,v}])[2] = newCellData[2];
			(mapHeight[{u,v}])[3] = newCellData[3];
			(mapHeight[{u,v}])[4] = mu_t; 			// mu_tm1
			(mapHeight[{u,v}])[5] = sigma_t; 		// sigma_tm1
			(mapHeight[{u,v}])[6] = errorZ; 		// errorZ
			setColorImgHeight( imgResHeight, u, v, mu_t);		
			setColorImgSigma( imgResSigma, u, v, sigma_t);	
			setColorImgError( imgResError, u, v, errorZ);	
			//ROS_INFO("------Height : - sigma_t= %0.4f, mu_t= %0.2f, errorZ= %0.8f", sigma_t, mu_t, errorZ);				
		}
		
		
		
		
		tab[{u,v}].clear();
		
		/****************************************/
		/**********  Create the Plane  **********/
		/****************************************/
		if((mapPlane[{u,v}])[2] == 0){
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


