#ifndef Point_H
#define Point_H
 
#include <Arduino.h>
 
class Point {
public:
  Point(double, double);
  ~Point();
  double x();
  double y();

private:
  double x_;
  double y_;
};
 
#endif