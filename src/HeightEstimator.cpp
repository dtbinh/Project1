#include "../include/HeightEstimator.h"
#include "../include/planeRegression.h"

static int cpt = 0;
static double MARGIN = 0.1;
static double RANGE = 2;
static double RANGE_MIN = 2;
static int SQUARE_SIZE = 25;
static double RANGE_OBJ = 1.0;
static float H_THRES = 0.05;

static vector<pair<int, int>> contour;

void HeightEstimator::setColor(double z_max, double z_min, double z, int u, int v, cv::Mat img)
{
	double rangeZMid = (z_max - z_min) * 0.5;
	double stepColor = 400 / rangeZMid;

	(img.at<cv::Vec3b>(u, v))[1] = 253;
	/*
	if(z < z_max && z > z_min)
	{
		int redMinus = (z - z_min)* stepColor > 200 ? 200 : redMinus;
		redMinus = redMinus < 0 ? 0 : redMinus;

		int greenMinus = z * stepColor < 0 ? 0 : greenMinus;	
		greenMinus = greenMinus > 200 ? 200 : greenMinus;	
		
		(img.at<cv::Vec3b>(u, v))[0] = 51;			// B
		(img.at<cv::Vec3b>(u, v))[1] = 253 - greenMinus;	// G
		(img.at<cv::Vec3b>(u, v))[2] = 51  + redMinus;		// R
		
		
	}
	else
	{
		(img.at<cv::Vec3b>(u, v))[0] = 204;		// B
		(img.at<cv::Vec3b>(u, v))[1] = 0;		// G
		(img.at<cv::Vec3b>(u, v))[2] = 204;		// R
	}*/
}

HeightEstimator::HeightEstimator(): n("~") 
{
	n.param("base_frame",base_frame_,std::string("/world"));
	n.param("max_range",max_range_,50.0);
	n.param("nb_points_est", nb_points_est_, 100);
	
	n.param("stepX", stepX, 0.1);
	n.param("stepY", stepY, 0.1);
	nbX = (int)floor((maxX - minX)/stepX);
	nbY = (int)floor((maxY - minY)/stepY);
	
	// Parameters Estimation of the Height	
	n.param("A_t",A_t,0.1);
	n.param("B_t",B_t,0.0);
	n.param("C_t",C_t,0.1);
	n.param("R_t",R_t,0.1);
	n.param("Q_t",Q_t,0.1);
	n.param("sigma_t0",sigma_t0,0.1);
	
	// Make sure TF is ready
	ros::Duration(1).sleep();

	sub = n.subscribe("/vrep/depthSensor", 1, &HeightEstimator::splitPointsCallback, this);
	marker_target_pub_ = n.advertise<visualization_msgs::MarkerArray>("target",1);

	robotTwist_cmd_pub = n.advertise<geometry_msgs::Twist>("/vsv_driver/twistCommand", 1000);

	//OpenCV image initialization
	cv::Mat imgR3(nbX, nbY, CV_8UC3, cv::Scalar(0,0,0));
	imgResHeight = imgR3;

	//Image publisher initialization
	ros::NodeHandle(nh);
	image_transport::ImageTransport it2(nh);
	imgPub2 = it2.advertise("imageHei", 1);

	ROS_INFO("------Height Estimator Ready-------");
}

HeightEstimator::~HeightEstimator(){}

double HeightEstimator::p_Zt_Xt(int Zt, int Xt)
{
	double res = (Zt == Xt) ? 0.85 : 0.15;
	return res;
}
 
void HeightEstimator::setColorImgHeight( cv::Mat imgResHeight, int u, int v, double z )
{
	setColor(0.7, -0.01, z, u, v, imgResHeight);		
}

void HeightEstimator::splitPointsCallback(const sensor_msgs::PointCloud2ConstPtr msg){

	/*
	 * Set up target point in front of the robot
	 */
	pcl::PointCloud<pcl::PointXYZ> temp;
	pcl::fromROSMsg(*msg, temp);
	listener_.waitForTransform(base_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
	pcl_ros::transformPointCloud(base_frame_,msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);
	pcl_ros::transformPointCloud("/world", msg->header.stamp, temp, msg->header.frame_id, lastpcBBR_, listener_);

	geometry_msgs::PointStamped dummy, target;
	dummy.header.frame_id = "/VSV/platform";
	dummy.header.stamp = msg->header.stamp; //ros::Time::now();
	dummy.point.x = 0.5;
	dummy.point.y = 0;
	dummy.point.z = 0;
	listener_.transformPoint("/world", dummy, target);
	double min_dist = 10000000.0, min_x = 0, min_y = 0;
	

	unsigned int n = temp.size();
	std::vector<size_t> pidx;
	double newCellData[6]; // z_mean, z_biais, z_min, z_max, mu_tm1, sigma_tm1
	double mu_t=0;
	for (unsigned int i=0;i<n;i++) 
	{	
		/*************************************/
		/**********  Check Outlier  **********/
		/*************************************/
		const pcl::PointXYZ & T = temp[i];
		double d = hypot(T.x,T.y);
		// In the sensor frame, this point would be inside the camera
		if (d < 1e-2) continue;
		if(lastpc_[i].x < minX || lastpc_[i].x > maxX) continue;
		if(lastpc_[i].y < minY || lastpc_[i].y > maxY) continue;
		d = hypot(lastpcBBR_[i].x, lastpcBBR_[i].y);
		if (d > max_range_) continue;

		// Compute index of the right accumulator		
		int u = (int) floor((lastpc_[i].x - minX)/stepX);
		int v = (int) floor((lastpc_[i].y - minY)/stepY);

		tab[{u,v}].push_back(lastpc_[i]);

		if(tab[{u,v}].size() < (unsigned) nb_points_est_) continue;


		/****************************************************/
		/**********  Get properties of this cell  **********/
		/****************************************************/			
		floorHeight(tab[{u,v}], newCellData);
		if(mapHeight.count({u,v}) == 0)
		{
			(mapHeight[{u,v}])[0] = newCellData[0]; // z_mean
			(mapHeight[{u,v}])[1] = newCellData[1]; // z_biais
			(mapHeight[{u,v}])[2] = newCellData[2]; // z_min
			(mapHeight[{u,v}])[3] = newCellData[3]; // z_max
			(mapHeight[{u,v}])[4] = newCellData[0]; // mu_tm1 = z_mean at t0
			(mapHeight[{u,v}])[5] = newCellData[1]; // sigma_tm1
			setColorImgHeight(imgResHeight, u, v, newCellData[0]);	
		}		

		/****************************************************
	robotTwist_cmd_pub = n.advertise<geometry_msgs::Twist>("/vsv_driver***/
		/**********  Update properties of this cell  **********/
		/*******************************************************/
		double mu_b_t = A_t * (mapHeight[{u,v}])[4]; // A*mu_tm1 + B_t*u_t =  A*mu_tm1
		double sigma_b_t = A_t*A_t * (mapHeight[{u,v}])[5]  + R_t; // A*sigma_tm1*A + R_t
		
		double K_t = sigma_b_t * C_t / ( C_t*C_t*sigma_b_t + Q_t);
		mu_t = mu_b_t + K_t*(newCellData[0] - C_t*mu_b_t); // mu_b_t + K_t(z_t - C*mu_b_t)
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
	
		tab[{u,v}].clear();
		
		/*
		 * Get closest point to the objective
		 */
		 
		if((mapHeight[{u,v}])[0] > H_THRES)
		{
			double temp_dist = sqrt(pow((lastpc_[i].x - target.point.x),2) + pow((lastpc_[i].y - target.point.y),2));
			if (temp_dist < min_dist)
			{
				min_dist = temp_dist;
				min_x = u;
				min_y = v;
			}
		}
	}
	
	int u = min_x;
	int v = min_y;
	setColorImgHeight(imgResHeight, u, v, mu_t);		
	
	nbtarget++;
	visualization_msgs::Marker m;
	m.header.stamp = ros::Time::now();
	m.header.frame_id = "world";//base_frame_;
	m.ns = "target";
	m.id = nbtarget;
	m.type = visualization_msgs::Marker::CYLINDER;
	m.action = visualization_msgs::Marker::ADD;
	m.pose.position.x = u*stepX + minX;
	m.pose.position.y = v*stepY + minY;
	m.pose.position.z = 0;
	m.scale.x = 0.1;
	m.scale.y = 0.1;
	m.scale.z = 0.05;
	m.color.a = 0.5;
	m.color.r = 1.0;
	m.color.g = 1.0;
	m.color.b = 0.0;
	mArray.markers.push_back(m);
	marker_target_pub_.publish(mArray);
		
		
		
	vector<pcl::PointXY> Tang;
	double tangCoeff[2];
	for(int cptX = 0 ; cptX < SQUARE_SIZE ; cptX++)
	{
		pcl::PointXY pt_tmp;
		for(int cptY = 0 ; cptY < SQUARE_SIZE ; cptY++)
		{
			if(u + cptX >= nbX || u - cptX <= 0 || v + cptY >= nbY || v - cptY <= 0)
				continue;
			if((mapHeight[{u + cptX, v - cptY}])[0] > H_THRES[1])
			{				
				pt_tmp.x = u + cptX;
				pt_tmp.y = v - cptY;
				Tang.push_back(pt_tmp);
			}
			if((mapHeight[{u + cptX, v + cptY}])[0] > H_THRES[1])
			{				
				pt_tmp.x = u + cptX;
				pt_tmp.y = v + cptY;
				Tang.push_back(pt_tmp);
			}
			if((mapHeight[{u - cptX, v - cptY}])[0] > H_THRES[1])
			{				
				pt_tmp.x = u - cptX;
				pt_tmp.y = v + cptY;
				Tang.push_back(pt_tmp);
			}
			if((mapHeight[{u - cptX, v - cptY}])[0] > H_THRES[1])
			{				
				pt_tmp.x = u - cptX;
				pt_tmp.y = v - cptY;
				Tang.push_back(pt_tmp);
			}
		}
	}
	floorLinearRegression(Tang, tangCoeff);
	
	geometry_msgs::Vector3Stamped tangDir, tangDirWorld;
	tangDirWorld.header.frame_id = "/world";
	tangDirWorld.header.stamp = msg->header.stamp;//ros::Time::now();
	tangDirWorld.vector.x = tangCoeff[0];
	tangDirWorld.vector.y = tangCoeff[1];
	tangDirWorld.vector.z = 0;
	listener_.transformVector("/VSV/platform", tangDirWorld, tangDir);
	double alpha = atan2(tangDirWorld.vector.y,tangDirWorld.vector.x);
	
	
	double v_Robot = 0.5;
	double K_1 = 0.5;
	double K_2 = 0.5;
	double minW = -3;
	double maxW = 3;
	
	geometry_msgs::Twist cmd;
	cmd.linear.x = v_Robot;				
	cmd.angular.z = K_1*(min_dist-RANGE_MIN) + K_2*alpha;
	ROS_INFO("%lf, %lf", min_dist-RANGE_MIN, alpha);	
	if(cmd.angular.z < minW)
			cmd.angular.z = minW;
	if(cmd.angular.z > maxW)
			cmd.angular.z = maxW;
	robotTwist_cmd_pub.publish(cmd);


	//Display the image
	cv::imshow("Height mapping", imgResHeight);
	cv::waitKey(1);

	//Build ROS message
	imgResROS2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgResHeight).toImageMsg();
	imgPub2.publish(imgResROS2);
}


