#ifndef Algorithm_H
#define Algorithm_H
 
#include <Arduino.h>

#include <List.hpp>

#include "Point.h"
 
class Algorithm {
public:
  Algorithm();
  ~Algorithm();
  int AddOutline(List<Point>);
  String GenerateGCODE();
private:
  List<List<Point>> outlines;
  String GCODE;
};
 
#endif