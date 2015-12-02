#include "../include/planeRegression.h"

void floorLinearRegression(const vector<pcl::PointXYZ> msg, const double* planeCoeff){
/*	size_t n = msg.size();
	
	// Eigen is a matrix library. The line below create a 3x3 matrix A,
	// and a 3x1 vector B
	Eigen::MatrixXf A(n,3);
	Eigen::MatrixXf B(n,1);
	for (unsigned int i=0;i<n;i++) {
		// Assign x,y,z to the coordinates of the point we are
		// considering.
		A(i,0) = msg[i].x;
		A(i,1) = msg[i].y;
		A(i,2) = 1;

		B(i,0) = msg[i].z;		
	}
	// Eigen operation on matrices are very natural:
	Eigen::MatrixXf X = (A.transpose() * A).inverse() * A.transpose() * B;
	// ROS_INFO("Extracted floor plane: z = %.2fx + %.2fy + %.2f", X(0),X(1),X(2));
	 
	planeCoeff[0] = X(0); //a
	planeCoeff[1] = X(1); //b
	planeCoeff[2] = -1;	  //c
	planeCoeff[3] = X(2); //d	
	
	
	
	// Compute de bias
	double bias = 0;
	Eigen::Vector3f n_vect; n_vect << planeCoeff[0], planeCoeff[1], planeCoeff[2];
	Eigen::Vector3f P;
	if(planeCoeff[0] != 0) P << -planeCoeff[3]/planeCoeff[0], 0, 0;	//-d/a
	else if(planeCoeff[1] != 0) P << 0, -planeCoeff[3]/planeCoeff[1], 0; //-d/b
	else if(planeCoeff[1] != 0) P << 0, 0, -planeCoeff[3]/planeCoeff[2]; //-d/c
	else P << 0, 0, 0;
		
	for(unsigned int i=0 ; i < n ; i++){
		Eigen::Vector3f M; M << msg[i].x, msg[i].y, msg[i].z;
		double dis = n_vect.dot(M-P);
		bias += dis*dis;
	}
	bias /= n;
	
	planeCoeff[4] = bias;
	planeCoeff[5] = n;	*/
}



