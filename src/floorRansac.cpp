#include "../include/planeRegression.h"
 
void floorRansac(const vector<pcl::PointXYZ> msg, double* planeCoeff){
	size_t n = msg.size();
	size_t best = 0;
	double X[3] = {0,0,0};
 
	if(n > 0){
		for (unsigned int i=0 ; i < (unsigned) N_SAMPLES ; i++){
			int j = rand() % n;
			Eigen::Vector3f P;
			P << msg[j].x, msg[j].y,msg[j].z;

			int k = 0;
			do{
				k = rand() % n;
			}while ( j == k );
			Eigen::Vector3f Q;
			Q << msg[k].x, msg[k].y, msg[k].z;

			int l = 0;
			do{
				l = rand() % n;
			}while (l == j || l == k) ;
			Eigen::Vector3f R;
			R << msg[l].x, msg[l].y, msg[l].z;


			// Find the equation of the plane
			Eigen::Vector3f n_vect = (P-Q).cross(P-R);
			n_vect = n_vect / n_vect.norm();


			// Count the number of points near of the plane
			size_t countPoint = 0;
			for (unsigned int p=0; p<n; p++){
				Eigen::Vector3f M; M << msg[p].x, msg[p].y, msg[p].z;
				if(fabs(n_vect.dot(M-P)) < TOLERANCE ) countPoint ++;
			}

			// Check if is bigger than the max -> Save if yes
			if( best < countPoint){
				best = countPoint;
				double d = - n_vect.dot(R);
				planeCoeff[0] = n_vect[0];
				planeCoeff[1] = n_vect[1];
				planeCoeff[2] = n_vect[2];
				planeCoeff[3] = d;
			}
		}
		
		// Compute de bias
		double bias = 0;
		Eigen::Vector3f n_vect; n_vect << planeCoeff[0], planeCoeff[1], planeCoeff[2];
		Eigen::Vector3f P;
		if(planeCoeff[0] != 0) P << -planeCoeff[3]/planeCoeff[0], 0, 0;	//-d/a
		else if(planeCoeff[1] != 0) P << 0, -planeCoeff[3]/planeCoeff[1], 0; //-d/b
		else if(planeCoeff[1] != 0) P << 0, 0, -planeCoeff[3]/planeCoeff[2]; //-d/c
		else P << 0, 0, 0;
		
		for (unsigned int i=0 ; i < n ; i++){
			Eigen::Vector3f M; M << msg[i].x, msg[i].y, msg[i].z;
			double dis = n_vect.dot(M-P);
			bias += dis*dis;
		}
		bias /= n;
		
		planeCoeff[4] = bias;
		planeCoeff[5] = n;
	}
};


