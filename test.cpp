#include "walker.h"
#include <assert.h>
#include <stdio.h>

using namespace std;
using namespace Eigen;

void validateTestCase(int numVehicles, double error, Matrix<Vector2d, Dynamic, 1> regionOfInterest, Matrix<Vector2d, Dynamic, Dynamic> output, VectorXi number_of_outputs) {
  
  assert(number_of_outputs.rows() == numVehicles);
  double vehicleArea = calculateArea(regionOfInterest);
  printf("\tArea: %f\n", vehicleArea);
  for (int row = 0; row < numVehicles; row++) { 
    double goalArea = vehicleArea / numVehicles;
    printf ("\tVehicle: %d\n",row);
    printf("\t\t");
    Matrix<Vector2d, Dynamic, 1> ROI(number_of_outputs(row));
    for (int col = 0; col < number_of_outputs(row); col++) {
      double x = output(row,col).x();
      double y = output(row,col).y();
      ROI(col) = Vector2d(x,y);
      printf("(%f, %f) ", x, y); 
    }
    printf("\n\t\t");
    double ROIArea = calculateArea(ROI);
    printf("Area: %f\n", ROIArea);
    assert(goalArea*(1-error) <= ROIArea && ROIArea <= goalArea*(1+error));
  }
  for(int i = 0; i < 20; i++) {
    printf("-");
  }
  printf("\n");
}

void runDefaultTestCase() {
  
  int numVehicles=2;
  double error = .10;
  
  Matrix<Vector2d, Dynamic, 1> regionOfInterest(3); 
  Matrix<Vector2d, Dynamic, 1> vehicleStartingPoints(2);
  Matrix<Vector2d, Dynamic, Dynamic> output(numVehicles, regionOfInterest.rows()+2);
  VectorXi number_of_outputs(numVehicles);
  
  // the two points the vehicles are starting along: { (5,0) (0,0) }
  // it does not matter what order the two points are in, however, it might be more efficient
  // if traversing the region of interest forward, the "greater" point needs to be listed first
  // and the "lesser" point needs to start the region of interest and
  // if traversing the region of interest backwards, the "lesser" point needs to be listed first
  // and the "greater" point needs to be at the end of the list
  vehicleStartingPoints(0) = Vector2d(5,0);
  vehicleStartingPoints(1) = Vector2d(0,0);
  
  //---- Test Case 1, Triangle { (0,0) (5,5) (0,5) }
  //---- Starting line: { (5,0) (0,0) }
  //---- Number of Vehicles: 2
  //---- Error: 10%
  //---- forward direction
  printf("Testing 3 sided polygon with 2 vehicles");
  regionOfInterest(0) = Vector2d(0,0);
  regionOfInterest(1) = Vector2d(5,5);
  regionOfInterest(2) = Vector2d(5,0);
  
  getSearchAreas(numVehicles, vehicleStartingPoints, regionOfInterest, error, 1, output, number_of_outputs);
  validateTestCase(numVehicles, error, regionOfInterest, output, number_of_outputs);
  
  //---- Test Case 2, Triangle { (0,0) (5,5) (0,5) }
  //---- Starting line: { (5,0) (0,0) }
  //---- Number of Vehicles: 3
  //---- Error: 10%
  //---- forward direction
  printf("Testing 3 sided polygon with 3 vehicles");
  numVehicles = 3;
  output = Matrix<Vector2d, Dynamic, Dynamic>(numVehicles, regionOfInterest.rows()+2);
  number_of_outputs = VectorXi(numVehicles);
  getSearchAreas(numVehicles, vehicleStartingPoints, regionOfInterest, error, 1, output, number_of_outputs);
  validateTestCase(numVehicles, error, regionOfInterest, output, number_of_outputs);
  
  //---- Test Case 3, Square { (0,0) (0,5) (5,5) (5,0) }
  //---- Starting line: { (5,0) (0,0) }
  //---- Number of Vehicles: 2
  //---- Error: 10%
  //---- forward direction
  printf("Testing 4 sided polygon with 2 vehicles");
  numVehicles = 2;
  regionOfInterest = Matrix<Vector2d, Dynamic, 1>(4);
  output = Matrix<Vector2d, Dynamic, Dynamic>(numVehicles, regionOfInterest.rows()+2);
  number_of_outputs = VectorXi(numVehicles);
  
  regionOfInterest(0) = Vector2d(0,0);
  regionOfInterest(1) = Vector2d(0,5);
  regionOfInterest(2) = Vector2d(5,5);
  regionOfInterest(3) = Vector2d(5,0);
  
  getSearchAreas(numVehicles, vehicleStartingPoints, regionOfInterest, error, 1, output, number_of_outputs);
  validateTestCase(numVehicles, error, regionOfInterest, output, number_of_outputs);
  
  //---- Test Case 4, Square { (0,0) (0,5) (5,5) (5,0) }
  //---- Starting line: { (5,0) (0,0) }
  //---- Number of Vehicles: 3
  //---- Error: 10%
  //---- forward direction
  printf("Testing 4 sided polygon with 3 vehicles");
  numVehicles = 3;
  output = Matrix<Vector2d, Dynamic, Dynamic>(numVehicles, regionOfInterest.rows()+2);
  number_of_outputs = VectorXi(numVehicles);
  
  getSearchAreas(numVehicles, vehicleStartingPoints, regionOfInterest, error, 1, output, number_of_outputs);
  validateTestCase(numVehicles, error, regionOfInterest, output, number_of_outputs);
  
  //---- Test Case 5, Pentagon { (0,0) (0,5) (5,10) (10,5) (10,0) }
  //---- Starting line: { (10,0) (0,0) }
  //---- Number of Vehicles: 2
  //---- Error: 10%
  //---- forward direction
  printf("Testing 5 sided polygon with 2 vehicles");
  numVehicles = 2;
  regionOfInterest = Matrix<Vector2d, Dynamic, 1>(5);
  vehicleStartingPoints(0) = Vector2d(10,0);
  vehicleStartingPoints(1) = Vector2d(0,0);
  output = Matrix<Vector2d, Dynamic, Dynamic>(numVehicles, regionOfInterest.rows()+2);
  number_of_outputs = VectorXi(numVehicles);
  
  regionOfInterest(0) = Vector2d(0,0);
  regionOfInterest(1) = Vector2d(0,5);
  regionOfInterest(2) = Vector2d(5,10);
  regionOfInterest(3) = Vector2d(10,5);
  regionOfInterest(4) = Vector2d(10,0);
  
  getSearchAreas(numVehicles, vehicleStartingPoints, regionOfInterest, error, 1, output, number_of_outputs);
  validateTestCase(numVehicles, error, regionOfInterest, output, number_of_outputs);
  
  //---- Test Case 5, Pentagon { (0,0) (0,5) (5,10) (10,5) (10,0) }
  //---- Starting line: { (10,0) (0,0) }
  //---- Number of Vehicles: 3
  //---- Error: 10%
  //---- forward direction
  printf("Testing 5 sided polygon with 3 vehicles");
  numVehicles = 3;
  output = Matrix<Vector2d, Dynamic, Dynamic>(numVehicles, regionOfInterest.rows()+2);
  number_of_outputs = VectorXi(numVehicles);
  
  getSearchAreas(numVehicles, vehicleStartingPoints, regionOfInterest, error, 1, output, number_of_outputs);
  validateTestCase(numVehicles, error, regionOfInterest, output, number_of_outputs);
  
  //---- Test Case 6, Hexagon { (0,0) (0,5) (5,10) (10,10) (15,5) (0,15) }
  //---- Starting line: { (15,0) (0,0) }
  //---- Number of Vehicles: 2
  //---- Error: 10%
  //---- forward direction
  printf("Testing 6 sided polygon with 2 vehicles");
  numVehicles = 2;
  vehicleStartingPoints(0) = Vector2d(15,0);
  vehicleStartingPoints(1) = Vector2d(0,0);
  regionOfInterest = Matrix<Vector2d, Dynamic, 1>(6);
  output = Matrix<Vector2d, Dynamic, Dynamic>(numVehicles, regionOfInterest.rows()+2);
  number_of_outputs = VectorXi(numVehicles);
  
  regionOfInterest(0) = Vector2d(0,0);
  regionOfInterest(1) = Vector2d(0,5);
  regionOfInterest(2) = Vector2d(5,10);
  regionOfInterest(3) = Vector2d(10,10);
  regionOfInterest(4) = Vector2d(15,5);
  regionOfInterest(5) = Vector2d(15,0);
  
  getSearchAreas(numVehicles, vehicleStartingPoints, regionOfInterest, error, 1, output, number_of_outputs);
  validateTestCase(numVehicles, error, regionOfInterest, output, number_of_outputs);
  
  //---- Test Case 7, Hexagon { (0,0) (0,5) (5,10) (10,10) (15,5) (0,15) }
  //---- Starting line: { (10,0) (0,0) }
  //---- Number of Vehicles: 4
  //---- Error: 10%
  //---- forward direction
  printf("Testing 6 sided polygon with 4 vehicles");
  numVehicles = 4;
  output = Matrix<Vector2d, Dynamic, Dynamic>(numVehicles, regionOfInterest.rows()+2);
  number_of_outputs = VectorXi(numVehicles);
  
  getSearchAreas(numVehicles, vehicleStartingPoints, regionOfInterest, error, 1, output, number_of_outputs);
  validateTestCase(numVehicles, error, regionOfInterest, output, number_of_outputs);
  
  //---- Test Case 8, Hexagon { (0,0) (0,5) (5,10) (10,10) (15,5) (0,15) }
  //---- Starting line: { (10,0) (0,0) }
  //---- Number of Vehicles: 5
  //---- Error: 10%
  //---- forward direction
  printf("Testing 6 sided polygon with 5 vehicles");
  numVehicles = 5;
  output = Matrix<Vector2d, Dynamic, Dynamic>(numVehicles, regionOfInterest.rows()+2);
  number_of_outputs = VectorXi(numVehicles);
  
  getSearchAreas(numVehicles, vehicleStartingPoints, regionOfInterest, error, 1, output, number_of_outputs);
  validateTestCase(numVehicles, error, regionOfInterest, output, number_of_outputs);
  
  //---- Test Case 9, Octagon { (0,0) (0,5) (0,10) (5,15) (10,15) (15,10) (15,5) (10,0) }
  //---- Starting line: { (10,0) (0,0) }
  //---- Number of Vehicles: 2
  //---- Error: 10%
  //---- forward direction
  printf("Testing 8 sided polygon with 2 vehicles");
  numVehicles = 2;
  regionOfInterest = Matrix<Vector2d, Dynamic, 1>(8);
  output = Matrix<Vector2d, Dynamic, Dynamic>(numVehicles, regionOfInterest.rows()+2);
  number_of_outputs = VectorXi(numVehicles);
  
  regionOfInterest(0) = Vector2d(0,0);
  regionOfInterest(1) = Vector2d(0,5);
  regionOfInterest(2) = Vector2d(0,10);
  regionOfInterest(3) = Vector2d(5,15);
  regionOfInterest(4) = Vector2d(10,15);
  regionOfInterest(5) = Vector2d(15,10);
  regionOfInterest(6) = Vector2d(15,5);
  regionOfInterest(7) = Vector2d(10,0);
  
  getSearchAreas(numVehicles, vehicleStartingPoints, regionOfInterest, error, 1, output, number_of_outputs);
  validateTestCase(numVehicles, error, regionOfInterest, output, number_of_outputs);
  
  //---- Test Case 10, dodecagon { (15,0) (10,5) (5,10) (0,15) (5,20) (10,25) (15,30) (20,25), (25,20) (30,15), (25,10) (20,5) }
  //---- Starting line: { 20,5) (15,0) }
  //---- Number of Vehicles: 2
  //---- Error: 10%
  //---- forward direction
  printf("Testing 12 sided polygon with 2 vehicles");
  numVehicles = 2;
  vehicleStartingPoints(0) = Vector2d(20,5);
  vehicleStartingPoints(1) = Vector2d(15,0);
  regionOfInterest = Matrix<Vector2d, Dynamic, 1>(12);
  output = Matrix<Vector2d, Dynamic, Dynamic>(numVehicles, regionOfInterest.rows()+2);
  number_of_outputs = VectorXi(numVehicles);
  
  regionOfInterest(0) = Vector2d(15,0);
  regionOfInterest(1) = Vector2d(10,5);
  regionOfInterest(2) = Vector2d(5,10);
  regionOfInterest(3) = Vector2d(0,15);
  regionOfInterest(4) = Vector2d(5,20);
  regionOfInterest(5) = Vector2d(10,25);
  regionOfInterest(6) = Vector2d(15,30);
  regionOfInterest(7) = Vector2d(20,25);
  regionOfInterest(8) = Vector2d(25,20);
  regionOfInterest(9) = Vector2d(30,15);
  regionOfInterest(10) = Vector2d(25,10);
  regionOfInterest(11) = Vector2d(20,5);
  
  getSearchAreas(numVehicles, vehicleStartingPoints, regionOfInterest, error, 1, output, number_of_outputs);
  validateTestCase(numVehicles, error, regionOfInterest, output, number_of_outputs);
}

void displayHelp() {
  printf("./walker\n");
  printf("The polygon walker splits an n-sided two-dimensional polygon into x many equal areas along a specified starting line.\n\n");
  printf("-s n, --segments=n Specifies the points of the polygon in x y coordinates. The points must be listed in counter clock wise or clock wise order.\n");
  printf("-p x0,y0,x1,y1,..., --polygon=x0,y0,x1,y1,... Specifies two points where the polygon will be split. The two points must be on an existing edge.\n");
  printf("-s x0,y0,x1,y1, --start=x0,y0,x1,y1 Specifies two points where the polygon will be split. The two points must be on an existing edge.\n");
  printf("-e err, --error=err Specifies a percentage error of how equal the segments shoudl be. err must be between 0 and 1.\n");
  printf("--demo Starts the default test cases.\n");
  printf("--help  Displays the help menu. The help menu is also displayed when there are no arguments.\n\n");
}

int main(int argc, char* argv[]) {
  int numVehicles = 0;
  int numSides = 0;
  double error = 0;
  Matrix<Vector2d, Dynamic, 1> regionOfInterest(numSides);
  Matrix<Vector2d, 2, 1> vehicleStartingPoints;
  bool haveStartingPoint = false;
  bool haveROI = false;
  
  // run help menu if help
  if (argc == 2) {
    if (string(argv[1]) == "help") {
      displayHelp();
    }
  }
  
  if (argc == 1) {
    displayHelp();
  }
  
  // if running own case, at a minimum need number of vehicles, 
  for (int i = 1; i < argc; i++) {
    if ((string(argv[i]) == "-v" || string(argv[i]) == "--vehicles") && i+1 < argc) {
      try {
        numVehicles = atoi(argv[i+1]);
      } catch (exception) {
        numVehicles=0;
      }
    } else if ((string(argv[i]) == "-p" || string(argv[i]) == "--polygon") && i+1 < argc) {
      try {
        numSides = atoi(argv[i+1]);
      } catch (exception) {
        numSides=0;
      }
      try {
        if (numSides*2 + i < argc && numSides > 2) {
          regionOfInterest = Matrix<Vector2d, Dynamic, 1>(numSides);
          for (int j = 0; j < numSides*2; j+=2) {
            double x = atof(string(argv[j+i+2]).c_str());
            double y = atof(string(argv[j+i+3]).c_str());
            regionOfInterest(j/2) = Vector2d(x,y);
            haveROI = true;
          }
        }
      } catch (exception) {
        haveROI = false;
      }
    } else if ((string(argv[i]) == "-s" || string(argv[i]) == "--start") && i+4 < argc) {
      try {
        for (int j = 0; j <4; j+=2) {
          double x = atof(string(argv[j+i+1]).c_str());
          double y = atof(string(argv[j+i+2]).c_str());
          vehicleStartingPoints(j/2) = Vector2d(x,y);
        }
        haveStartingPoint = true;
      } catch (exception) {
        haveStartingPoint = false;
          continue;
      }
    } else if ((string(argv[i]) == "-e" || string(argv[i]) == "--error") && i+1 < argc) {
      try {
        error = atof(string(argv[i+1]).c_str());
      } catch (exception) {
        error = 0;
      }
    } else if ((string(argv[i]) == "demo")) {
      runDefaultTestCase();
    }
  }

  if (numSides != 0 && haveROI && haveStartingPoint && error >= 0 && error < 1) {
    printf("Testing %d sided polygon with %d vehicles\n", numSides, numVehicles);
    Matrix<Vector2d, Dynamic, Dynamic> output(numVehicles, regionOfInterest.rows()+2);
    VectorXi number_of_outputs(numVehicles);
    getSearchAreas(numVehicles, vehicleStartingPoints, regionOfInterest, error, 1, output, number_of_outputs);
    validateTestCase(numVehicles, error, regionOfInterest, output, number_of_outputs);
  }
  
  
}