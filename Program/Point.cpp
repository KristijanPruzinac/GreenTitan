#include "Point.h"

Point::Point(double x, double y){
  x_ = x;
  y_ = y;
}

double Point::x(){
  return x_;
}

double Point::y(){
  return y_;
}

Point::~Point(){}