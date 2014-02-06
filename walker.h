#ifndef WALKER_H
#define WALKER_H

#include "Eigen/Dense"
#include <vector>

typedef std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > Point2DVector;

using namespace Eigen;

void getSearchAreas(int numVehicles, Point2DVector vehicleStartingPoints, Point2DVector regionOfInterest, double errorAllowed, int direction, Point2DVector & output, VectorXi & number_of_outputs);
double calculateArea(Point2DVector regionOfInterest); 
bool findSearchArea(double errorAllowed, double goalArea, Vector2d start, Vector2d next, Point2DVector corners, Vector2d & polygon);

#endif