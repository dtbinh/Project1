#include "../include/SplitPoints.h"

static float H_THRES = 0.05;
static int W_OBST_MAX = 30;

MatrixAccumulator::MatrixAccumulator(): n("~") {
	n.param("base_frame",base_frame_,std::string("/world"));
	n.param("max_range",max_range_,5.0);
	n.param("nb_points_est", nb_points_est_, 100);
	
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

	ROS_INFO("-------------- Ready ----------------");
}


MatrixAccumulator::~MatrixAccumulator(){}

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
			int count = 0;
			ROS_INFO("----------------------------------------");
			ROS_INFO("Begin of potential obstacle at %d", zmax);
			while (lastpc_[i].z > 0.0 && i > 0 && count < W_OBST_MAX) 
			{
				i--;
				count ++;
				h_to_avoid = std::max(h_to_avoid, lastpc_[i].z);
			}
			if (count == W_OBST_MAX) 
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
				cmd.header.frame_id = "/world";
				cmd.point.x = lastpc_[zmax].x;
				cmd.point.y = lastpc_[zmax].y;
				cmd.point.z = lastpc_[zmax].z;
				arm_cmd_pub.publish(cmd);

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


