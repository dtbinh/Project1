#ifndef ACCUMULATOR_H
#define ACCUMULATOR_H


#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <vector>

class Accumulator{
    public:
        Accumulator();
        virtual ~Accumulator();
	double getBias() const;
	void setBias(double bias);
	int getNbPoints();
	void setNbPoints(int nbPoints);
	void incNbPoints();
	std::vector<pcl::PointXYZ> getPoints() const;
	void setPoints(std::vector<pcl::PointXYZ> points);
	void addPoints(pcl::PointXYZ point);
	void clearPoints() ;
	const double* getPlaneCoeff() const;

	std::vector<pcl::PointXYZ> points;
        int nbPoints;
        double bias;
        //a, b, c and d
        double planeCoeff[4] = {0,0,0,0};

    protected:
    private:	

};

#endif // ACCUMULATOR_H
