#include "walker.h"
#include <assert.h>
#include <stdio.h>
#include <sstream>


using namespace std;
using namespace Eigen;

void validateTestCase(int numVehicles, double error, Point2DVector regionOfInterest, Point2DVector output, VectorXi number_of_outputs) {
  
  assert(number_of_outputs.rows() == numVehicles);
  Point2DVector ROI;
  double vehicleArea = calculateArea(regionOfInterest);
  int start = 0;
  printf("\tArea: %f\n", vehicleArea);
  for (int row = 0; row < numVehicles; row++) { 
    double goalArea = vehicleArea / numVehicles;
    printf ("\tVehicle: %d\n",row);
    printf("\t\t");
    ROI.clear();
    if (row == 0) {
      start = 0;
    } else {
      start += number_of_outputs(row-1);
    }
    for (int col = start; col < number_of_outputs(row)+start; col++) {
      double x = output[col].x();
      double y = output[col].y();
      ROI.push_back(Vector2d(x,y));
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
  
  Point2DVector regionOfInterest; 
  Point2DVector vehicleStartingPoints;
  Point2DVector output;
  VectorXi number_of_outputs(numVehicles);
  
  // the two points the vehicles are starting along: { (5,0) (0,0) }
  // it does not matter what order the two points are in, however, it might be more efficient
  // if traversing the region of interest forward, the "greater" point needs to be listed first
  // and the "lesser" point needs to start the region of interest and
  // if traversing the region of interest backwards, the "lesser" point needs to be listed first
  // and the "greater" point needs to be at the end of the list
  vehicleStartingPoints.push_back(Vector2d(5,0));
  vehicleStartingPoints.push_back(Vector2d(0,0));
  
  //---- Test Case 1, Triangle { (0,0) (5,5) (5,0) }
  //---- Starting line: { (5,0) (0,0) }
  //---- Number of Vehicles: 2
  //---- Error: 10%
  //---- forward direction
  printf("Testing 3 sided polygon with 2 vehicles");
  regionOfInterest.push_back(Vector2d(0,0));
  regionOfInterest.push_back(Vector2d(5,5));
  regionOfInterest.push_back(Vector2d(5,0));
  
  getSearchAreas(numVehicles, vehicleStartingPoints, regionOfInterest, error, 1, output, number_of_outputs);
  validateTestCase(numVehicles, error, regionOfInterest, output, number_of_outputs);
  
  //---- Test Case 2, Triangle { (0,0) (5,5) (5,0) }
  //---- Starting line: { (5,0) (0,0) }
  //---- Number of Vehicles: 3
  //---- Error: 10%
  //---- forward direction
  printf("Testing 3 sided polygon with 3 vehicles");
  numVehicles = 3;
  output.clear();
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
  regionOfInterest.clear();
  output.clear();
  number_of_outputs = VectorXi(numVehicles);
  
  regionOfInterest.push_back(Vector2d(0,0));
  regionOfInterest.push_back(Vector2d(0,5));
  regionOfInterest.push_back(Vector2d(5,5));
  regionOfInterest.push_back(Vector2d(5,0));
  
  getSearchAreas(numVehicles, vehicleStartingPoints, regionOfInterest, error, 1, output, number_of_outputs);
  validateTestCase(numVehicles, error, regionOfInterest, output, number_of_outputs);
  
  //---- Test Case 4, Square { (0,0) (0,5) (5,5) (5,0) }
  //---- Starting line: { (5,0) (0,0) }
  //---- Number of Vehicles: 3
  //---- Error: 10%
  //---- forward direction
  printf("Testing 4 sided polygon with 3 vehicles");
  numVehicles = 3;
  output.clear();
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
  regionOfInterest.clear();
  vehicleStartingPoints.clear();
  output.clear();
  vehicleStartingPoints.push_back(Vector2d(10,0));
  vehicleStartingPoints.push_back(Vector2d(0,0));
  number_of_outputs = VectorXi(numVehicles);
  
  regionOfInterest.push_back(Vector2d(0,0));
  regionOfInterest.push_back(Vector2d(0,5));
  regionOfInterest.push_back(Vector2d(5,10));
  regionOfInterest.push_back(Vector2d(10,5));
  regionOfInterest.push_back(Vector2d(10,0));
  
  getSearchAreas(numVehicles, vehicleStartingPoints, regionOfInterest, error, 1, output, number_of_outputs);
  validateTestCase(numVehicles, error, regionOfInterest, output, number_of_outputs);
  
  //---- Test Case 5, Pentagon { (0,0) (0,5) (5,10) (10,5) (10,0) }
  //---- Starting line: { (10,0) (0,0) }
  //---- Number of Vehicles: 3
  //---- Error: 10%
  //---- forward direction
  printf("Testing 5 sided polygon with 3 vehicles");
  numVehicles = 3;
  output.clear();
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
  vehicleStartingPoints.clear();
  regionOfInterest.clear();
  output.clear();
  vehicleStartingPoints.push_back(Vector2d(15,0));
  vehicleStartingPoints.push_back(Vector2d(0,0));
  number_of_outputs = VectorXi(numVehicles);
  
  regionOfInterest.push_back(Vector2d(0,0));
  regionOfInterest.push_back(Vector2d(0,5));
  regionOfInterest.push_back(Vector2d(5,10));
  regionOfInterest.push_back(Vector2d(10,10));
  regionOfInterest.push_back(Vector2d(15,5));
  regionOfInterest.push_back(Vector2d(15,0));
  
  getSearchAreas(numVehicles, vehicleStartingPoints, regionOfInterest, error, 1, output, number_of_outputs);
  validateTestCase(numVehicles, error, regionOfInterest, output, number_of_outputs);
  
  //---- Test Case 7, Hexagon { (0,0) (0,5) (5,10) (10,10) (15,5) (0,15) }
  //---- Starting line: { (10,0) (0,0) }
  //---- Number of Vehicles: 4
  //---- Error: 10%
  //---- forward direction
  printf("Testing 6 sided polygon with 4 vehicles");
  numVehicles = 4;
  output.clear();
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
  output.clear();
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
  regionOfInterest.clear();
  output.clear();
  number_of_outputs = VectorXi(numVehicles);
  
  regionOfInterest.push_back(Vector2d(0,0));
  regionOfInterest.push_back(Vector2d(0,5));
  regionOfInterest.push_back(Vector2d(0,10));
  regionOfInterest.push_back(Vector2d(5,15));
  regionOfInterest.push_back(Vector2d(10,15));
  regionOfInterest.push_back(Vector2d(15,10));
  regionOfInterest.push_back(Vector2d(15,5));
  regionOfInterest.push_back(Vector2d(10,0));
  
  getSearchAreas(numVehicles, vehicleStartingPoints, regionOfInterest, error, 1, output, number_of_outputs);
  validateTestCase(numVehicles, error, regionOfInterest, output, number_of_outputs);
  
  //---- Test Case 10, dodecagon { (15,0) (10,5) (5,10) (0,15) (5,20) (10,25) (15,30) (20,25), (25,20) (30,15), (25,10) (20,5) }
  //---- Starting line: { 20,5) (15,0) }
  //---- Number of Vehicles: 2
  //---- Error: 10%
  //---- forward direction
  printf("Testing 12 sided polygon with 2 vehicles");
  numVehicles = 2;
  vehicleStartingPoints.clear();
  vehicleStartingPoints.push_back(Vector2d(20,5));
  vehicleStartingPoints.push_back(Vector2d(15,0));
  regionOfInterest.clear();
  output.clear();
  number_of_outputs = VectorXi(numVehicles);
  
  regionOfInterest.push_back(Vector2d(15,0));
  regionOfInterest.push_back(Vector2d(10,5));
  regionOfInterest.push_back(Vector2d(5,10));
  regionOfInterest.push_back(Vector2d(0,15));
  regionOfInterest.push_back(Vector2d(5,20));
  regionOfInterest.push_back(Vector2d(10,25));
  regionOfInterest.push_back(Vector2d(15,30));
  regionOfInterest.push_back(Vector2d(20,25));
  regionOfInterest.push_back(Vector2d(25,20));
  regionOfInterest.push_back(Vector2d(30,15));
  regionOfInterest.push_back(Vector2d(25,10));
  regionOfInterest.push_back(Vector2d(20,5));
  
  getSearchAreas(numVehicles, vehicleStartingPoints, regionOfInterest, error, 1, output, number_of_outputs);
  validateTestCase(numVehicles, error, regionOfInterest, output, number_of_outputs);
}

void displayHelp() {
  printf("./walker\n");
  printf("The polygon walker splits an n-sided two-dimensional polygon into x many equal areas along a specified starting line.\n\n");
  printf("-s n, --segments=n Specifies the points of the polygon in x y coordinates. The points must be listed in counter clock wise or clock wise order.\n");
  printf("-p n,x0,y0,x1,y1,..., --polygon=n,x0,y0,x1,y1,... Specifies n sides followed by the different points that make up the polygon seperated by a comma.\n");
  printf("-st x0,y0,x1,y1, --start=x0,y0,x1,y1 Specifies two points where the polygon will be split. The two points must be on an existing edge.\n");
  printf("-e err, --error=err Specifies a percentage error of how equal the segments shoudl be. err must be between 0 and 1.\n");
  printf("--demo Starts the default test cases.\n");
  printf("--help  Displays the help menu. The help menu is also displayed when there are no arguments.\n\n");
}

int main(int argc, char* argv[]) {
  int numVehicles = 0;
  int numSides = 0;
  double error = 0;
  Point2DVector regionOfInterest;
  Point2DVector vehicleStartingPoints;
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
  
  // if running own case, at a minimum need number of segments, 
  for (int i = 1; i < argc; i++) {
    if (string(argv[i]) == "-s" || string(argv[i]).find("--segments=") != string::npos) {
      try {
        if (string(argv[i]) == "-s" && i + 1 < argc) {
          numVehicles = atoi(argv[i+1]);
        } else {
          string parsed, input = string(argv[i]);
          istringstream input_stream(input);
          // throw away first part of argument
          getline(input_stream, parsed, '=');
          getline(input_stream, parsed, '=');
          numVehicles = atoi(parsed.c_str()); 
        }
        printf("vehicles %d\n", numVehicles);
      } catch (exception) {
        numVehicles=0;
      }
    } else if (string(argv[i]) == "-p" || string(argv[i]).find("--polygon=") != string::npos) {
      try {
        string parsed, input;
        if (string(argv[i]) == "-p" && i + 1 < argc) {
          input = string(argv[i+1]);
        } else {
          input = string(argv[i]);
          istringstream input_stream(input);
          getline(input_stream, parsed, '=');
          getline(input_stream, parsed, '=');
          input = parsed;
        }
        istringstream input_stream(input);
        getline(input_stream, parsed, ',');
        // grab number of sides
        numSides = atoi(parsed.c_str());
        printf("numSides %d\n", numSides);
        double x,y;
        int j = 0;
        
        while (getline(input_stream, parsed, ','))
        {
          if (j % 2 == 0) {
            x = atof(parsed.c_str());
            printf("x: %f ", x);
          } else {
            y = atof(parsed.c_str());
            printf("y: %f\n", y);
            regionOfInterest.push_back(Vector2d(x,y));
          }
          j++;
        }
        
        haveROI = true;
        numSides = (j+1)/2;
        printf("haveROI %d, numSides %d\n", haveROI, numSides);
      } catch (exception) {
        numSides = 0;
        haveROI = false;
      }
    } // should be the first and last points, needs to be changed to do this automatically
    else if (string(argv[i]) == "-st" || string(argv[i]).find("--start=") != string::npos) {
      try {
        string parsed, input;
        if (string(argv[i]) == "-st" && i + 1 < argc) {
          input = string(argv[i+1]);
        } else {
          input = string(argv[i]);
          istringstream input_stream(input);
          getline(input_stream, parsed, '=');
          getline(input_stream, parsed, '=');
          input = parsed;
        }
        
        istringstream input_stream(input);
        double x,y;
        int j = 0;
        
        while (getline(input_stream, parsed, ','))
        {
          if (j % 2 == 0) {
            x = atof(parsed.c_str());
            printf("x: %f ", x);
          } else {
            y = atof(parsed.c_str());
            printf("y: %f\n", y);
            vehicleStartingPoints.push_back(Vector2d(x,y));
          }
          j++;
        }
        haveStartingPoint = true;
      } catch (exception) {
        haveStartingPoint = false;
          continue;
      }
    } else if (string(argv[i]) == "-e" || string(argv[i]).find("--error=") != string::npos) {
      try {
        if (string(argv[i]) == "-e" && i + 1 < argc) {
          error = atof(string(argv[i+1]).c_str());
        } else {
          string parsed, input = string(argv[i]);
          istringstream input_stream(input);
          getline(input_stream, parsed, '=');
          getline(input_stream, parsed, '=');
          error = atof(parsed.c_str());
        }
        printf("error %f\n", error);
      } catch (exception) {
        error = 0;
      }
    } else if ((string(argv[i]) == "--demo")) {
      runDefaultTestCase();
    }
  }

  if (numSides > 2 && haveROI && haveStartingPoint && error >= 0 && error < 1) {
    printf("Testing %d sided polygon with %d vehicles\n", numSides, numVehicles);
    Point2DVector output;
    VectorXi number_of_outputs(numVehicles);
    getSearchAreas(numVehicles, vehicleStartingPoints, regionOfInterest, error, 1, output, number_of_outputs);
    validateTestCase(numVehicles, error, regionOfInterest, output, number_of_outputs);
  }
  
  
}