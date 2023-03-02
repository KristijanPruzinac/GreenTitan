#ifndef Algorithm_H
#define Algorithm_H
 
#include <Arduino.h>

#include <List.hpp>

#include "Point.h"
 
class Algorithm {
public:
  Algorithm();
  ~Algorithm();

  //Settings
  void SetMowerDiameter(double);
  void SetMowerOverlap(double);

  //Setup
  void AddOutline(List<Point>);

  //Functions
  String GenerateGCODE();
    void CreateInfill();
      void FindBoundaries();
private:
  //SETTINGS
  double mowerDiameter; //Diameter of the cutting surface, in meters
  double mowerOverlap; //Percentage of overlap when generating GCODE, initially set to 80%

  List<List<Point>> outlines;
  List<List<Point>> connectedOutlines;

  String GCODE;

  //Outline boundaries
  double minX, maxX;
  double minY, maxY;
};
 
#endif