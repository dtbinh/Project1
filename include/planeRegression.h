#include<ros/ros.h>
#include <Eigen/Dense>
#include<vector>
#include<pcl/point_types.h>


//Ransac parameters
#define N_SAMPLES 100
#define TOLERANCE 0.001
#define CIRCLE_TOLERANCE 0.08

using namespace std;  

void floorRansac(const vector<pcl::PointXYZ>, double*);

void floorHeight(const vector<pcl::PointXYZ>, double*);

void floorLinearRegression(const vector<pcl::PointXY>, double*);

void circleRansac(const vector<pcl::PointXYZ>, double*);
