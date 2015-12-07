#include "../include/planeRegression.h"

void floorLinearRegression(const vector<pcl::PointXY> msg, double* lineCoeff){
	size_t n = msg.size();
	
	// Eigen is a matrix library. The line below create a 3x3 matrix A,
	// and a 3x1 vector B
	Eigen::MatrixXf A(n,2);
	Eigen::MatrixXf B(n,1);
	for (unsigned int i=0;i<n;i++) {
		// Assign x,y,z to the coordinates of the point we are
		// considering.
		A(i,0) = msg[i].x;
		A(i,1) = 1;
		B(i,0) = msg[i].y;		
	}
	// Eigen operation on matrices are very natural:
	Eigen::MatrixXf X = A.transpose() * A;
	X = X.inverse();
	X = X * A.transpose();
	X = X * B;
	// ROS_INFO("Extracted floor plane: z = %.2fx + %.2fy + %.2f", X(0),X(1),X(2));
	 
	// ax-y+b=0
	lineCoeff[0] = X(0); //a
	lineCoeff[1] = X(1); //b	
}



