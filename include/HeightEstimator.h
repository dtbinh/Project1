#include "supraInclude.h"

class HeightEstimator{
    public:
        HeightEstimator(void);
        virtual ~HeightEstimator();
	double p_Zt_Xt(int Zt, int Xt);
	void setColorImgHeight( cv::Mat imgResHeight, int u, int v, double z );
	void setColorImgSigma( cv::Mat imgResSigma, int u, int v, double z );
	void setColorImgError( cv::Mat imgResError, int u, int v, double z );
	void splitPointsCallback(const sensor_msgs::PointCloud2ConstPtr);

    protected:
    private:	
	std::string base_frame_;
	double max_range_;
	int nb_points_est_;

	// Parameters Estimation Height
	double A_t, B_t, C_t; // State
	double R_t, Q_t; // Noise
	double sigma_t0; // Initial value of the gaussian variable of the height
	
	double stepX, stepY;
	int nbX, nbY; 
	
	ros::Subscriber sub;
	ros::Publisher marker_plane_pub_;
	
	tf::TransformListener listener_;
	pcl::PointCloud<pcl::PointXYZ> lastpc_;
	pcl::PointCloud<pcl::PointXYZ> lastpcBBR_;
	
	std::map< std::pair<int, int>, std::vector<pcl::PointXYZ> > tab;

	ros::NodeHandle n;

	sensor_msgs::ImagePtr imgResROS2;
	image_transport::Publisher imgPub2;
	
	
	// For the map of height	
	std::map< std::pair<int, int>, double[7] > mapHeight; // z_mean, z_biais, z_min, z_max, mu_tm1, sigma_tm1, errorZ
	std::map< std::pair<int, int>, double > mapConfidence;
	cv::Mat imgResHeight;	
};

