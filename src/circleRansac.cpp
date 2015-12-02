#include "../include/planeRegression.h"

void circleRansac(const vector<pcl::PointXYZ> msg, double* circleCoeff)
{
	size_t n = msg.size();
	size_t best = 0;	
	double X[3] = {0,0,0};

	if(n > 0){
		double z_min = msg[0].z;
		double z_max = msg[0].z;
		
		for (unsigned int i=0 ; i < (unsigned) N_SAMPLES ; i++)	{
			// Select 3 points to find a circle
			int j = rand() % n;
			Eigen::Vector3f P;
			P << msg[j].x, msg[j].y,msg[j].z;

			int k = 0;
			do{
				k = rand() % n;
			}while ( j == k );
			Eigen::Vector3f Q;
			Q << msg[k].x, msg[k].y, msg[k].z;

			/*int l = 0;
			do{
				l = rand() % n;
			}while (l == j || l == k || msg[l].y == 0) ;
			Eigen::Vector3f R;
			R << msg[l].x, msg[l].y, msg[l].z;*/
			
			//Calculate middle of [PQ]
			Eigen::Vector3f Mid;
			Mid = (P+Q)/2;
			
			//Build random third point
			double xR, yR, zR;
			Eigen::Vector3f R;
			double mR = (rand()-0.5)*2;
			// R = mR * (Mid + (Mid-P)); // False : R = mR*Q ....
			R << P(0) - Q(0), -(P(1) - Q(1)), P(2) - Q(2);
			R = Mid + (mR * R);

			// Find the equation of the circle. May need to draw the figure ...
			double h = (R-Mid).norm();
			double l = (P-Q).norm();
			double r_minus_h = ( l*l + 2*h*h )/ (4*h); // False : (need + ) double x = l*l / 8*h - h / 2; 
			Eigen::Vector3f O = - r_minus_h*(R-Mid)/((R-Mid).norm());
			double r = r_minus_h + h;
			
			double x = O(0);
			double y = O(1);
			// Count the number of points near of the plane
			size_t countPoint = 0;
			for (unsigned int p=0; p<n; p++){
				Eigen::Vector3f M; M << msg[p].x, msg[p].y, msg[p].z;
				if(fabs(r - sqrt((M[0] - x)*(M[0] - x) + (M[1] - y)*(M[1] - y))) < CIRCLE_TOLERANCE ) countPoint ++;
				if(M[2] < z_min){
					z_min = M[2];
				}
				if(M[2] > z_max){				
					z_max = M[2];
				}
			}

			// Check if is bigger than the max -> Save if yes
			if( best < countPoint){
				best = countPoint;
				circleCoeff[0] = x;
				circleCoeff[1] = y;
				circleCoeff[2] = r;
				circleCoeff[3] = z_min;
				circleCoeff[4] = z_max;
			}
		}
		
		// Compute de bias
		double bias = 0;
		for (unsigned int i=0 ; i < n ; i++){
			Eigen::Vector3f M; M << msg[i].x, msg[i].y, msg[i].z;
			double dis = fabs(circleCoeff[2] - sqrt((M[0] - circleCoeff[0])*(M[0] - circleCoeff[0]) + (M[1] - circleCoeff[1])*(M[1] - circleCoeff[1])));
			bias += dis*dis;
		}
		bias /= n;
		
		circleCoeff[5] = bias;
		circleCoeff[6] = n;
	}
}


