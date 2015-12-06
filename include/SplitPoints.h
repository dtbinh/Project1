#include "supraInclude.h"

class MatrixAccumulator{
    public:
        MatrixAccumulator();
        virtual ~MatrixAccumulator();	
		void splitPointsCallback(const sensor_msgs::PointCloud2ConstPtr);
		void DetectorCallback(const std_msgs::Float32 msg);

		double normV(pcl::PointXYZ pt1, pcl::PointXYZ pt2 );
		
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
	
	// Detector
	pcl::PointXYZ currentPosition;
	int nbMine = 0;
	double thresholdDetector;
	
	
	ros::Subscriber sub;
	ros::Subscriber subDetector;
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

