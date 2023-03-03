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

    if (p.x()() > maxX){
      maxX = p.x()();
    }
    if (p.x()() < minX){
      minX = p.x()();
    }

    if (p.y()() > maxY){
      maxY = p.y()();
    }
    if (p.y()() < minY){
      minY = p.y()();
    }
  }
}

void Algorithm::CreateInfill(){
  //Find outline boundaries
  FindBoundaries();

  //Divide with lines that are parallel to the X axis
  int numOfLines = ceil((maxY - minY) / (mowerDiameter * mowerOverlap));

  infill = new List<List<List<int>>>();

  for (int i = 0; i < numOfLines; i++)
  {
      double currentY = minY + (i + 0.5) * (mowerDiameter * mowerOverlap);

      //For every line find each point it collides with a path
      //X, outlineIndex, pointIndex
      List<List<double>> intersections = new List<List<double>>();

      for (int b = 0; b < outlines.getSize(); b++)
      {
          List<Point> o = outlines[b];
          for (int j = 0; j < o.getSize() - 1; j++)
          {
              Point p1 = o[j];
              Point p2 = o[j + 1];

              //If line is between points
              if ((currentY >= p1.y() && currentY <= p2.y()) || (currentY <= p1.y() && currentY >= p2.y()))
              {
                  intersections.add(new List<int>());

                  double thisX;

                  //Vertical slope
                  if (p1.x() != p2.x())
                  {
                      thisX = (currentY - p1.y()) * ((p2.x() - p1.x()) / (p2.y() - p1.y())) + p1.x();
                  }
                  else { thisX = p1.x(); }

                  //X, Y, outlineIndex, pointIndex
                  intersections[intersections.getSize() - 1].add(thisX);
                  intersections[intersections.getSize() - 1].add(currentY);
                  intersections[intersections.getSize() - 1].add(b);
                  intersections[intersections.getSize() - 1].add(j);
              }
          }

          //Check last-first
          if ((currentY >= o[o.getSize() - 1].y() && currentY <= o[0].y()) || (currentY <= o[o.getSize() - 1].y() && currentY >= o[0].y()))
          {
              Point p1 = o[o.getSize() - 1];
              Point p2 = o[0];

              intersections.add(new List<int>());

              int thisX;

              //Vertical slope
              if (p1.x() != p2.x())
              {
                  thisX = (int)((currentY - p1.y()) * ((float)(p2.x() - p1.x()) / (p2.y() - p1.y())) + p1.x());
              }
              else { thisX = p1.x(); }

              intersections[intersections.getSize() - 1].add(thisX);
              intersections[intersections.getSize() - 1].add(currentY);
              intersections[intersections.getSize() - 1].add(b);
              intersections[intersections.getSize() - 1].add(o.getSize()() - 1);
          }
      }

      //Order by x value | ascending if index is even | descending if indes is odd
      if (i % 2 == 0){ shellSortKnuth(intersections, intersections.getSize(), sortAscByX); }
      else { shellSortKnuth(intersections, intersections.getSize(), sortDescByX); }

      infill.add(intersections);
  }
}

String Algorithm::GenerateGCODE(){

}