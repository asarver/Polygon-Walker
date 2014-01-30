#ifndef WALKER_H
#define WALKER_H

#include "Eigen/Dense"

using namespace Eigen;

void getSearchAreas(int numVehicles, Matrix<Vector2d, Dynamic, 1> vehicleStartingPoints, Matrix<Vector2d, Dynamic, 1> regionOfInterest, double errorAllowed, int direction, Matrix<Vector2d, Dynamic, Dynamic> & output, VectorXi & number_of_outputs);
double calculateArea(Matrix<Vector2d, Dynamic, 1> regionOfInterest); 
bool findSearchArea(double errorAllowed, double goalArea, Vector2d start, Vector2d next, Matrix<Vector2d, Dynamic, 1> corners, Vector2d & polygon);

#endif