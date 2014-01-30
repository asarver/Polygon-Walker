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
void getSearchAreas(int numVehicles, Matrix<Vector2d, Dynamic, 1> vehicleStartingPoints,
                    Matrix<Vector2d, Dynamic, 1> regionOfInterest, double errorAllowed, int direction, Matrix<Vector2d, Dynamic, Dynamic> & output, VectorXi & number_of_outputs) {
  Matrix<Vector2d, Dynamic, 1> startingPoints(numVehicles);
  Matrix<Vector2d, Dynamic, 1> corners(1);
  if (regionOfInterest.rows() < 3) {
    // not a polygon
    return;
  }
  
  int output_row_index=0, output_col_index=0;
  
  double goalArea = fabs(calculateArea(regionOfInterest)/numVehicles);
  double distanceX = (vehicleStartingPoints(0).x() - vehicleStartingPoints(1).x())/numVehicles;
  double distanceY = (vehicleStartingPoints(0).y() - vehicleStartingPoints(1).y())/numVehicles;
  
  for(int i = 0; i < numVehicles; i++) {
    startingPoints(i) = Vector2d(vehicleStartingPoints(1).x() + (i+1)*distanceX,
                                 vehicleStartingPoints(1).y() + (i+1)*distanceY);
    //cout << "points: " << startingPoints(i).x() << ", " << startingPoints(i).y() << "\n";
  }
  //only for forward direction
  int i = 1;
  corners(0) = Vector2d(regionOfInterest(0).x(), regionOfInterest(0).y());
  for (int vehicleIndex = 0; vehicleIndex < numVehicles; vehicleIndex++) {
    if (direction == -1) {
      corners(0) = Vector2d(regionOfInterest(regionOfInterest.rows()-1).x(), regionOfInterest(regionOfInterest.rows()-1).y());
      for (int i = regionOfInterest.rows()-2; i >= 0; i--) {
        Vector2d boundary(0,0);
        bool found = findSearchArea(errorAllowed, goalArea, startingPoints(vehicleIndex), regionOfInterest(i), corners, boundary);
      
      }
    
    } else {
      while (i < regionOfInterest.rows()) {
        Vector2d boundary(0,0);
        //cout << "vehicle loc: " << startingPoints(vehicleIndex).x() << ", " << startingPoints(vehicleIndex).y() << "\n";
        //cout << "region of interest: " << regionOfInterest(i).x() << ", " << regionOfInterest(i).y() << "\n";
        bool found = findSearchArea(errorAllowed, goalArea, startingPoints(vehicleIndex), regionOfInterest(i), corners, boundary);

        if(found) {
          int numResults = 0;
          
          for (output_col_index = 0; output_col_index < corners.rows(); output_col_index++) {
            //cout << "in output storing: " << output_col_index << "\n";
            //cout << corners(output_col_index).x() << "," << corners(output_col_index).y() << "\n";
            
            numResults++;
            output(output_row_index, output_col_index) = Vector2d(corners(output_col_index).x(), corners(output_col_index).y());
          }
          
          // covers last case
          if ((corners(output_col_index-1).x() == regionOfInterest(i).x() && corners(output_col_index-1).y() == regionOfInterest(i).y()) ||
              (startingPoints(vehicleIndex).x() == regionOfInterest(i).x() && startingPoints(vehicleIndex).y() == regionOfInterest(i).y())) {
            //cout << "Why-tf am I not going in here?\n";
            numResults--;
            output_col_index--;
          } else {
            output(output_row_index, output_col_index) = Vector2d(boundary.x(), boundary.y());
          }
          
          output(output_row_index, output_col_index+1) = Vector2d(startingPoints(vehicleIndex).x(), startingPoints(vehicleIndex).y());
          number_of_outputs(output_row_index) = numResults + 2;
          
          corners = Matrix<Vector2d, Dynamic, 1>(2);
          corners(0) = Vector2d(startingPoints(vehicleIndex).x(), startingPoints(vehicleIndex).y());
          corners(1) = Vector2d(boundary.x(), boundary.y());
          
          //cout << "checking shit: " << startingPoints(vehicleIndex).x() << ", " << startingPoints(vehicleIndex).y() << " - " << regionOfInterest(i).x() << ", " << regionOfInterest(i).y() << "\n";
          
          //cout << "output col index: " << output_col_index << "\n";
          
          //cout << "frick\n";
          break;
        } else {
          Matrix<Vector2d, Dynamic, 1> temp(corners.rows()+1);
          
          if (corners(corners.rows()-1).x() != regionOfInterest(i).x() || corners(corners.rows()-1).y() != regionOfInterest(i).y()) {
            for(int x = 0; x < corners.rows(); x++) {
              temp(x) = Vector2d(corners(x).x(), corners(x).y());
            }
            corners = Matrix<Vector2d, Dynamic, 1>(temp.rows());
            for(int x = 0; x < temp.rows()-1; x++) {
              corners(x) = Vector2d(temp(x).x(), temp(x).y());
            }
            corners(corners.rows()-1) = Vector2d(regionOfInterest(i).x(), regionOfInterest(i).y());
            //cout << "adding region of interest: " << regionOfInterest(i).x() << ", " << regionOfInterest(i).y();
            //cout << "corners: ";
          }
          
          for (int l = 0; l < corners.rows(); l++) {
            //cout << "(" << corners(l).x() << ", " << corners(l).y() << ") ";
          }
          //cout << "\n";
          
        }
        i++;
      }
    }
    output_row_index++;
  }
  //cout << "output:\n";
  for (int row= 0; row < output.rows(); row++) {
    for(int col = 0; col < output.cols(); col++) {
      //cout << "(" << output(row, col).x() << "," << output(row, col).y() << ") ";
    }
    //cout << "\n";
  }
  
  //cout << "corners:\n";
  for(int r = 0; r < corners.rows(); r++) {
    //cout << "(" << corners(r).x() << "," << corners(r).y() << ") ";
  }
  
  //cout << "output numbers:\n";
  for (int i = 0; i < numVehicles; i++) {
    //cout << number_of_outputs(i) << "\n";
  }
}

double calculateArea(Matrix<Vector2d, Dynamic, 1> regionOfInterest) {
  int numOfPoints = regionOfInterest.rows();
  if (numOfPoints < 3) {
    return 0;
  }
  double sum = 0;
  for(int i = 0; i < numOfPoints-1; i++) {
    sum += regionOfInterest(i).x() * regionOfInterest(i+1).y() -
    regionOfInterest(i).y() * regionOfInterest(i+1).x();
  }
  
  sum += regionOfInterest(numOfPoints-1).x() * regionOfInterest(0).y() -
  regionOfInterest(numOfPoints-1).y() * regionOfInterest(0).x();
  
  return fabs(sum/2);
}

bool findSearchArea(double errorAllowed, double goalArea, Vector2d start, Vector2d next_corner, Matrix<Vector2d, Dynamic, 1> corners, Vector2d & boundary) {
  Vector2d next = Vector2d(next_corner.x(), next_corner.y());
  double slopeX = next.x() - corners(corners.rows()-1).x();
  double slopeY = next.y() - corners(corners.rows()-1).y();
  
  if (slopeX == 0 && slopeY == 0) {
    return false;
  }
  
  slopeX /= 20;
  slopeY /= 20;
  
  Matrix<Vector2d, Dynamic, 1> polygonPoints(corners.rows()+2);
  Vector2d current = Vector2d(corners(corners.rows()-1).x(), corners(corners.rows()-1).y());
  //cout << "current: " << current.x() << ", " << current.y() << "\n";
  //cout << "next: " << next.x() << ", " << next.y() << "\n";
  //cout << "slopeX: " << slopeX << "\n";
  //cout << "slopeY: " << slopeY << "\n";

  int i;
  for (i = 0; i < corners.rows(); i++) {
    polygonPoints(i) = Vector2d(corners(i).x(), corners(i).y());
  }
  i = corners.rows();
  polygonPoints(i) = Vector2d(start.x(), start.y());
  polygonPoints(i+1) = Vector2d(start.x(), start.y());

  for(int m = 0; m < polygonPoints.rows(); m++) {
    //cout << "polygon points: " << polygonPoints(m).x() << " " << polygonPoints(m).y() << "\n";
  }
  double currentArea = calculateArea(polygonPoints);
  
  while ((next.x() >= current.x() || (slopeX < 0 && -current.x() <= next.x())) &&
          ((next.y() >= current.y()) || (slopeY < 0 && -current.y() <= next.y())) && currentArea < goalArea ) {
    current = Vector2d(current.x()+slopeX, current.y()+slopeY);
    polygonPoints(i) = Vector2d(current.x(), current.y());
    for(int m = 0; m < polygonPoints.rows(); m++) {
      //cout << "in loop: polygon points: " << polygonPoints(m).x() << " " << polygonPoints(m).y() << "\n";
    }
    currentArea = calculateArea(polygonPoints);
    //cout << "in loop: current (x,y): " << current.x() << ", " << current.y() << "\n";
    //cout << "in loop: current area: " << currentArea << "\n";
    //cout << "slope X " << slopeX << ", slopeY " << slopeY << "\n";
  }
  
  if(currentArea <= goalArea*(1+errorAllowed) && currentArea >= goalArea*(1-errorAllowed)) {
    //cout << "finished loop: current (x,y): " << current.x() << ", " << current.y() << "\n";
    boundary = Vector2d(current.x(), current.y());
    return true;
  }
  
  //cout << "here\n";
  return false;
}