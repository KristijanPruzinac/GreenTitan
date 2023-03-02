#include "Algorithm.h"

Algorithm::Algorithm(){
  SetMowerOverlap(0.8);
}

Algorithm::~Algorithm(){}

void Algorithm::SetMowerDiameter(double mowerDiameter){
  this -> mowerDiameter = mowerDiameter;
}

void Algorithm::SetMowerOverlap(double mowerOverlap){
  this -> mowerOverlap = mowerOverlap;
}

void Algorithm::AddOutline(List<Point> outline){
  outlines.add(outline);
}

void Algorithm::FindBoundaries(){
  //Find outline boundaries
  maxX = -999999;
  maxY = -999999;

  minX = 999999;
  minY = 999999;

  for (int i = 0; i < outlines[0].getSize(); i++){
    Point p = outlines[0][i];

    if (p.x() > maxX){
      maxX = p.x();
    }
    if (p.x() < minX){
      minX = p.x();
    }

    if (p.y() > maxY){
      maxY = p.y();
    }
    if (p.y() < minY){
      minY = p.y();
    }
  }
}

void Algorithm::CreateInfill(){
  //Find outline boundaries
  FindBoundaries();

  //Divide with lines that are parallel to the X axis
  int numOfLines = ceil((maxY - minY) / (mowerDiameter * mowerOverlap));
}

String Algorithm::GenerateGCODE(){

}