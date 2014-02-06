#include "walker.h"
#include <stdio.h>
#include <math.h>


using namespace std;
using namespace Eigen;

// points need to be in order
// two points need to be given for the line the copters are starting on
// direction is -1 if you want to look counter clock wise (traverse the region of interest in reverse order)
// 1 if you want to look clock wise (traverse the region of interest in given order - default if invalid value is given)
// so if the first quadcopter is to the right of the others you want to search counter clock wise
// if the first quadcopter is to the left of the others you want to search clock wise
void getSearchAreas(int numVehicles, Point2DVector vehicleStartingPoints,
                    Point2DVector regionOfInterest, double errorAllowed, int direction, Point2DVector & output, VectorXi & number_of_outputs) {
  Point2DVector startingPoints;
  Point2DVector corners;
  if (regionOfInterest.size() < 3) {
    // not a polygon
    return;
  }
  
  int output_row_index=0, output_col_index=0;
  
  double goalArea = fabs(calculateArea(regionOfInterest)/numVehicles);
  double distanceX = (vehicleStartingPoints[0].x() - vehicleStartingPoints[1].x())/numVehicles;
  double distanceY = (vehicleStartingPoints[0].y() - vehicleStartingPoints[1].y())/numVehicles;
  
  for(int i = 0; i < numVehicles; i++) {
    startingPoints.push_back(Vector2d(vehicleStartingPoints[1].x() + (i+1)*distanceX, vehicleStartingPoints[1].y() + (i+1)*distanceY));
  }
  
  //only for forward direction
  int i = 1;
  corners.push_back(Vector2d(regionOfInterest[0].x(), regionOfInterest[0].y()));
  
  for (int vehicleIndex = 0; vehicleIndex < numVehicles; vehicleIndex++) {
    if (direction == -1) {
    
    } else {
      while (i < regionOfInterest.size()) {
        Vector2d boundary(0,0);
        bool found = findSearchArea(errorAllowed, goalArea, startingPoints[vehicleIndex], regionOfInterest[i], corners, boundary);
        
        if(found) {
          
          
          int numResults = 0;
          
          for (output_col_index = 0; output_col_index < corners.size(); output_col_index++) {
            
            numResults++;
            output.push_back(Vector2d(corners[output_col_index].x(), corners[output_col_index].y()));
          }
          
          // covers last case
          if ((corners[output_col_index-1].x() != boundary.x() || corners[output_col_index-1].y() != boundary.y()) &&
              (startingPoints[vehicleIndex].x() != boundary.x() || startingPoints[vehicleIndex].y() != boundary.y())) {
            output.push_back(Vector2d(boundary.x(), boundary.y()));
          numResults++;
          }
          
          output.push_back(Vector2d(startingPoints[vehicleIndex].x(), startingPoints[vehicleIndex].y()));
          number_of_outputs(output_row_index) = numResults + 1;
          output_row_index++;
          
          corners.clear();
          corners.push_back(Vector2d(startingPoints[vehicleIndex].x(), startingPoints[vehicleIndex].y()));
          corners.push_back(Vector2d(boundary.x(), boundary.y()));

          break;
        } else {
          if (corners[corners.size()-1].x() != regionOfInterest[i].x() || corners[corners.size()-1].y() != regionOfInterest[i].y()) {
            corners.push_back(Vector2d(regionOfInterest[i].x(), regionOfInterest[i].y()));
          }
        }
        i++;
      }
    }
    
  }
}

double calculateArea(Point2DVector regionOfInterest) {
  int numOfPoints = regionOfInterest.size();
  if (numOfPoints < 3) {
    return 0;
  }
  double sum = 0;
  for (int i = 0; i < numOfPoints-1; i++) {
    sum += regionOfInterest[i].x() * regionOfInterest[i+1].y() -
    regionOfInterest[i].y() * regionOfInterest[i+1].x();
  }
  
  sum += regionOfInterest[numOfPoints-1].x() * regionOfInterest[0].y() -
  regionOfInterest[numOfPoints-1].y() * regionOfInterest[0].x();
  
  return fabs(sum/2);
}

bool findSearchArea(double errorAllowed, double goalArea, Vector2d start, Vector2d next_corner, Point2DVector corners, Vector2d & boundary) {
  Vector2d next = Vector2d(next_corner.x(), next_corner.y());
  double slopeX = next.x() - corners[corners.size()-1].x();
  double slopeY = next.y() - corners[corners.size()-1].y();
  
  if (slopeX == 0 && slopeY == 0) {
    return false;
  }
  
  slopeX /= 20;
  slopeY /= 20;

  Point2DVector polygonPoints;
  Vector2d current = Vector2d(corners[corners.size()-1].x(), corners[corners.size()-1].y());

  int i;

  for (i = 0; i < corners.size(); i++) {
    polygonPoints.push_back(Vector2d(corners[i].x(), corners[i].y()));
  }
  polygonPoints.push_back(Vector2d(current.x(), current.y()));
  polygonPoints.push_back(Vector2d(start.x(), start.y()));
  i = polygonPoints.size()-2;

  double currentArea = calculateArea(polygonPoints);

  while ((next.x() >= current.x() || (slopeX < 0 && -current.x() <= next.x())) &&
          ((next.y() >= current.y()) || (slopeY < 0 && -current.y() <= next.y())) && currentArea < goalArea ) {
    current = Vector2d(current.x()+slopeX, current.y()+slopeY);
    polygonPoints[i] = Vector2d(current.x(), current.y());
    currentArea = calculateArea(polygonPoints);
  }
  
  if(currentArea <= goalArea*(1+errorAllowed) && currentArea >= goalArea*(1-errorAllowed)) {
    boundary = Vector2d(current.x(), current.y());
    return true;
  }
  return false;
}