#include "../include/planeRegression.h"

void floorHeight(const vector<pcl::PointXYZ> msg, double* planeData){	
	size_t n = msg.size();	
 
	if(n > 0){
		double z_mean = 0.0;
		double z_dev = 0.0;
		double z_min = msg[0].z;
		double z_max = msg[0].z;
		
		// Compute the Z mean
		for (unsigned int i=0 ; i < (unsigned) n ; i++){
			z_mean += msg[i].z;
			
			z_max = z_max > msg[i].z ? z_max : msg[i].z;
			z_min = z_min < msg[i].z ? z_min : msg[i].z;
		}
		z_mean /= (double)n;
		
		// Compute the Z biais
		for (unsigned int i=0 ; i < (unsigned) n ; i++){
			z_dev += (z_mean - msg[i].z);
		}
		z_dev /= (double)n;
		
		
		// Saved data for this set of sample
		planeData[0] = z_mean;
		planeData[1] = z_dev;
		planeData[2] = z_min;
		planeData[3] = z_max;
		
	}
}
