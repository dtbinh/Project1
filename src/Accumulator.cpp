#include "../include/Accumulator.h"

Accumulator::Accumulator(){
    //ctor
	this->nbPoints = 0;
	this->bias = 0;
}

Accumulator::~Accumulator(){
    //dtor
	this->nbPoints = 0;
	this->bias = 0;
}

double Accumulator::getBias() const {
	return bias;
}

void Accumulator::setBias(double bias) {
	this->bias = bias;
}

int Accumulator::getNbPoints(){
	return nbPoints;
}

void Accumulator::setNbPoints(int nbPoints) {
	this->nbPoints = nbPoints;
}

void Accumulator::incNbPoints() {
	this->nbPoints = this->nbPoints + 1;
}

std::vector<pcl::PointXYZ> Accumulator::getPoints() const {
	return points;
}

void Accumulator::setPoints(std::vector<pcl::PointXYZ> points) {
	this->points = points;
}

void Accumulator::addPoints(pcl::PointXYZ point){
	this->incNbPoints();
	(this->points).push_back(point);
	//ROS_INFO(" Warning 2-1-1 capacity : %lu",(this->points).capacity());
}

void Accumulator::clearPoints(){
	(this->points).clear() ;
	this->nbPoints = 0;
}

const double* Accumulator::getPlaneCoeff() const {
	return planeCoeff;
}

