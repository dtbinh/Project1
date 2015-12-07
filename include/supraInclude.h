#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float32.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
//~ #include <Eigen/Core>
#include <vector>
#include <map>


#define minY -20
#define maxY 20
#define minX -20
#define maxX 20

#define TRAV_COLOR 64
#define NONTRAV_COLOR 255

#define OBSTACLE 3
#define TRAVERSABLE 1
#define NOT_TRAVERSABLE 2 
#define UNKNOWN 0

#define MAX_RANGE 5
