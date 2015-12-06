#include "../include/HeightEstimator.h"
#include "../include/planeRegression.h"

static int cpt = 0;
static double MARGIN = 0.1;
static double RANGE = 2;
static int SQUARE_SIZE = 25;
static double RANGE_OBJ = 1.0;


void HeightEstimator::setColor(double z_max, double z_min, double z, int x, int y, cv::Mat img)
{
	double rangeZMid = (z_max - z_min) * 0.5;
	double stepColor = 400 / rangeZMid;

	/*
	* UPDATE FOR PROJECT 3
	* DON'T WRITE ON (x,y), BUT MARGIN AWAY FROM IT
	*/
	double centerX, centerY;// *************************WARNING***********************************
	int u,v;
	double d = hypot(x - centerX, y - centerY);
	double h = (d != 0) ? (1 - MARGIN / d) : 1;
	int u = floor(centerX + h*(x-centerX));
	int v = floor(centerY + h*(y-centerY));	

	/*
	* END UPDATE
	*/
	
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
	}
}

HeightEstimator::HeightEstimator(): n("~") 
{
	n.param("base_frame",base_frame_,std::string("/world"));
	n.param("max_range",max_range_,5.0);
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

	sub = n.subscribe("scans", 1, &HeightEstimator::splitPointsCallback, this);
	marker_plane_pub_ = n.advertise<visualization_msgs::MarkerArray>("floor_plane",1);

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
	setColor(0.7, -0.7, z, u, v, imgResHeight);		
}

void HeightEstimator::splitPointsCallback(const sensor_msgs::PointCloud2ConstPtr msg){

	// GET A POINT IN THE LAKE (FIRST POSITION OF THE ROBOT)
	if (cpt == 0)
	{
		geometry_msgs::PointStamped dummy, center;
		dummy.header.frame_id = "/BubbleRob";
		dummy.header.stamp = ros::Time::now();
		dummy.point.x = 0;
		dummy.point.y = 0;
		dummy.point.z = 0;
		transformPoint("/world", dummy.header.frame_id, dummy, center);
		centerX = center.point.x;
		centerY = center.point.y;
		cpt++; 
	}

	pcl::PointCloud<pcl::PointXYZ> temp;
	pcl::fromROSMsg(*msg, temp);
	listener_.waitForTransform(base_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
	pcl_ros::transformPointCloud(base_frame_,msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);
	//pcl_ros::transformPointCloud("/world", msg->header.stamp, temp, msg->header.frame_id, lastpcBBR_, listener_);

	unsigned int n = temp.size();
	std::vector<size_t> pidx;
	
	double newCellData[6]; // z_mean, z_biais, z_min, z_max, mu_tm1, sigma_tm1

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

		/*******************************************************/
		/**********  Update properties of this cell  **********/
		/*******************************************************/
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
		setColorImgHeight(imgResHeight, u, v, mu_t);						
	
		tab[{u,v}].clear();
	}
	
	// GET A LINE TO SEE IN FRONT OF THE ROBOT
	geometry_msgs::PointStamped dummy, p1, p2;
	dummy.header.frame_id = "/VSV/platform";
	dummy.header.stamp = ros::Time::now();
	dummy.point.x = 0;
	dummy.point.y = 0;
	dummy.point.z = 0;
	transformPoint("/world", dummy.header.frame_id, dummy, p1);
	dummy.point.x = 0.1;
	transformPoint("/world", dummy.header.frame_id, dummy, p2);

	double a = (p1.point.y - p2.point.y) / (p1.point.x - p2.point.x);
	int b = p1.point.y - a*p1.point.x;
	int cpt = 0;
	do
	{
		int u = (int) floor((cpt - minX)/stepX);
		int v = (int) floor((a*cpt + b - minY)/stepY);
		if(hypot(cpt - p1.point.x, a*cpt + b - p1.point.x) < RANGE)
			if((img.at<cv::Vec3b>(u, v))[1] != 0) break;
			else continue;
	}
	while((u < maxX && v < maxY))

	if(!(u < maxX && v < maxY))
	{


		vector<pcl::PointXY> Tang;
		double tangCoeff[2];
		for(int cptX = 0 ; cptX < SQUARE_SIZE ; cptX++)
		{
			for(int cptY = 0 ; cptY < SQUARE_SIZE ; cptY++)
			{
				if(u + cptX >= maxX || u - cptX <= minX || v + cptY >= maxY || v - cptY <= minY)
					continue;
				if((img.at<cv::Vec3b>(u + cptX, v + cptY))[1] != 0)
					Tang.push_back(PointXY(u + cptX, v - cptY));
				if((img.at<cv::Vec3b>(u + cptX, v + cptY))[1] != 0)
					Tang.push_back(PointXY(u + cptX, v - cptY));
				if((img.at<cv::Vec3b>(u - cptX, v + cptY))[1] != 0)
					Tang.push_back(PointXY(u - cptX, v + cptY));
				if((img.at<cv::Vec3b>(u - cptX, v - cptY))[1] != 0)
					Tang.push_back(PointXY(u - cptX, v - cptY));
			}
		}
		floorLinearRegression(Tang, tangCoeff);
	}


	Vector3Stamped tangDir, tangDirWorld;
	tangDirWorld.header.frame_id = "/world";
	tangDirWorld.header.stamp = ros::Time::now();
	tangDirWorld.vector.x = tangCoeff[0];
	tangDirWorld.vector.y = tangCoeff[1];
	tangDirWorld.vector.z = 0;
	transformVector("/VSV/platform", tangDirWorld.header.frame_id, tangDirWorld, tangDir);
	double alpha = acos(tangDir.vector / tangDir.vector.norm());

	










	Vector3Stamped vectDir, vectDirWorld;
	vectDirWorld.header.frame_id = "/VSV/platform";
	vectDirWorld.header.stamp = ros::Time::now();
	vectDirWorld.vector.x = 0;
	vectDirWorld.vector.y = RANGE_OBJ;
	vectDirWorld.vector.z = 0;
	transformVector("/world", vectDirWorld.header.frame_id, vectDirWorld, vectDir);

	Eigen::Vector3f vectNormal;
	vectNormal << -vectDir.vector.y, vectDir.vector.x, 0.0;
	vectNormal /= vectNormal.norm();
	double a = -vectNormal.y;
	double b =  vectNormal.x;

	













	geometry_msgs::Twist cmd;
	cmd.twist.linear.x = v_Robot;				
	cmd.twist.angulat.z = K_1*tan + K_2*alpha;		
	armTwist_cmd_pub.publish(cmd)


	//Display the image
	cv::imshow("Mapping of the Height", imgResHeight);
	cv::waitKey(1);

	//Build ROS message
	// imgResROS = cv_bridge::CvImage(std_msgs::Header(), "rgba8", imgResTrav).toImageMsg();
	imgResROS2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgResHeight).toImageMsg();
	imgPub2.publish(imgResROS2);
}


