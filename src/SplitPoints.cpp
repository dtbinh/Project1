#include "../include/SplitPoints.h"

static float H_THRES = 0.05;
static int W_OBST_MAX = 30;

MatrixAccumulator::MatrixAccumulator(): n("~") {

	n.param("base_frame",base_frame_,std::string("/world"));
	n.param("max_range",max_range_,5.0);
	n.param("nb_points_est", nb_points_est_, 100);
	n.param("W_OBST_MAX2", W_OBST_MAX2, 0.7);
	
	n.param("distance_secu", distance_secu, 0.2);
	
	n.param("stepX", stepX, 0.1);
	n.param("stepY", stepY, 0.1);
	nbX = (int)floor((maxX - minX)/stepX);
	nbY = (int)floor((maxY - minY)/stepY);
	
	// Make sure TF is ready
	ros::Duration(1).sleep();

	//~ sub = n.subscribe("/vrep/depthSensor", 1, &MatrixAccumulator::splitPointsCallback, this);
	sub = n.subscribe("/vrep/hokuyoSensor", 1, &MatrixAccumulator::splitPointsCallback, this);
	marker_plane_pub_ = n.advertise<visualization_msgs::MarkerArray>("floor_plane",1);
	marker_cylinder_pub_ = n.advertise<visualization_msgs::MarkerArray>("floor_cylinder",1);
	
	// Arm command publisher
	arm_cmd_pub = n.advertise<geometry_msgs::PointStamped>("/arm_ik/position_stamped", 1000);
	armPos_cmd_pub = n.advertise<geometry_msgs::Point>("/arm_ik/position", 1000);
	armTwist_cmd_pub = n.advertise<geometry_msgs::Twist>("/arm_ik/twist", 1000);

	ROS_INFO("-------------- Ready ----------------");
}


MatrixAccumulator::~MatrixAccumulator(){}

double MatrixAccumulator::normV(pcl::PointXYZ pt1, pcl::PointXYZ pt2 ){
	
	return sqrt( (pt1.x - pt2.x)*(pt1.x - pt2.x) + (pt1.y - pt2.y)*(pt1.y - pt2.y));
	
	}

void MatrixAccumulator::splitPointsCallback(const sensor_msgs::PointCloud2ConstPtr msg){
		
	pcl::PointCloud<pcl::PointXYZ> temp;
	pcl::fromROSMsg(*msg, temp);
	// Make sure the point cloud is in the base-frame
	listener_.waitForTransform(base_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
	pcl_ros::transformPointCloud(base_frame_,msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);
	pcl_ros::transformPointCloud("/VSV/ArmPan", msg->header.stamp, temp, msg->header.frame_id, lastpcBBR_, listener_);
	unsigned int n = temp.size();

	for (int i=n-1; i>=0; i--) {	
		if(lastpc_[i].z > H_THRES)
		{
			int zmax = i;
			float h_to_avoid = 0.0;
			float h_to_avoid2 = -10.0;
			int count = 0;
			ROS_INFO("----------------------------------------");
			ROS_INFO("Begin of potential obstacle at %d", zmax);
			while (lastpc_[i].z > 0.0 && i > 0 && count < W_OBST_MAX) //normV(lastpc_[zmax],lastpc_[i]) < W_OBST_MAX2) 
			{
				i--;
				count ++;
				h_to_avoid = std::max(h_to_avoid, lastpc_[i].z);
				h_to_avoid2 = std::max(h_to_avoid2, lastpcBBR_[i].z);
			}
			ROS_INFO("Number of point tested %d", zmax-i);
			if (count == W_OBST_MAX) //normV(lastpc_[zmax],lastpc_[i]) >= W_OBST_MAX2) //
			{
				if(i == 0 || abs(lastpc_[zmax].x) > maxX || abs(lastpc_[zmax].y) > maxY) 
				{
					ROS_INFO("Only water this time !");
					break;
				}
				
				ROS_INFO("Was not an obstacle, publishing command");

				// Some info about the point in different tf's
				ROS_INFO("%0.2f %0.2f %0.2f %d", lastpc_[zmax].x, lastpc_[zmax].y, lastpc_[zmax].z, zmax);
				//ROS_INFO("%0.2f %0.2f %0.2f %d", temp[zmax].x, temp[zmax].y, temp[zmax].z, zmax);
				//ROS_INFO("%0.2f %0.2f %0.2f %d", lastpcBBR_[zmax].x, lastpcBBR_[zmax].y, lastpcBBR_[zmax].z, zmax);

				geometry_msgs::PointStamped cmd;
				cmd.header.frame_id = "/VSV/ArmPan";
				cmd.header.stamp = ros::Time::now();
				cmd.point.x = 0;
				cmd.point.y = lastpcBBR_[zmax].y;
				cmd.point.z = h_to_avoid2 + 0.2;//lastpc_[zmax].z+0.2;
				arm_cmd_pub.publish(cmd);


				//~ geometry_msgs::Point cmd;
				//~ cmd.x = lastpcBBR_[zmax].x;
				//~ cmd.y = 0;
				//~ cmd.z = h_to_avoid;		
				//~ armPos_cmd_pub.publish(cmd);
				
				
				//~ float y_goalTo = lastpcBBR_[zmax].y;
				//~ 
				//~ geometry_msgs::Twist cmd;
				//~ cmd.twist.linear.x = lastpc_[zmax].x;				
				//~ cmd.twist.angulat.z = h_to_avoid;		
				//~ armTwist_cmd_pub.publish(cmd)

				break;
			}
			else 
			{
				ROS_INFO("End of obstacle found at %d", i);				
				continue;
			}
		}
	}
	
}


