#include "Algorithm.h"

//Charging station
int prevPointLon = 0;
int prevPointLat = 0;

//Target
int targetPointLon = BASE_LON;
int targetPointLat = BASE_LAT;

//Globals
int numOfLines;

//Algorithm
String algorithmTarget = "FORWARD"; // FORWARD, BASE
String algorithmMode = "OUTLINE";
// FORWARD: OUTLINE, SEEK, INFILL
// BASE: INFILL, SEEK

bool algorithmAbortFull = false;

/*
Explanation:
OUTLINE - Mows outer outline
SEEK - Searches for point aint outer outline where infill line starts
INFILL - Mows infill line and inside outlines
*/

//Only in INFILL mode
int algorithmInfillIndex;
int algorithmInfillPoint;
int algorithmInfillDirection;

int algorithmCurrentOutline = 0;
int algorithmCurrentPoint = 0;

//Outlines - First point is the charging station exit point
std::vector<std::vector<std::vector<int>>> outlines;
std::vector<std::vector<std::vector<int>>> extOutlines; //Outlines with intersection points inserted
std::vector<std::vector<std::vector<int>>> intersectionPaths; //Collections of points where infill intersects terrain and outer outline
std::vector<std::vector<int>> intersections;

//MAX 50M HEIGHT

int terrainMinX;
int terrainMaxX;
int terrainMinY;
int terrainMaxY;

void FindTerrainBounds(){
  //Find terrain bounds
  terrainMinX = outlines.at(0).at(0).at(0);
  terrainMaxX = outlines.at(0).at(0).at(0);
  terrainMinY = outlines.at(0).at(0).at(1);
  terrainMaxY = outlines.at(0).at(0).at(1);
  
  for (int i = 0; i < outlines.size(); i++){
    for (int j = 0; j < outlines.at(i).size(); j++){
      int xVal = outlines.at(i).at(j).at(0);
      int yVal = outlines.at(i).at(j).at(1);
      
      if (xVal < terrainMinX){terrainMinX = xVal;}
      if (yVal < terrainMinY){terrainMinY = yVal;}
      
      if (xVal > terrainMaxX){terrainMaxX = xVal;}
      if (yVal > terrainMaxY){terrainMaxY = yVal;}
    }
  }
  
  numOfLines = ceil(abs(terrainMaxY - terrainMinY) / (MOWER_OVERLAP * 1.109)); //1.109 is gps latitude scaling factor
}

void ClearInterference(){
  for (int i = 0; i < numOfLines; i++)
  {
    int currentY = (int) (terrainMinY + abs(terrainMaxY - terrainMinY) / (float)(numOfLines) * i + abs(terrainMaxY - terrainMinY) / (float)(numOfLines) / 2.0);
    
    for (int o = 0; o < outlines.size(); o++){
      for (int p = 0; p < outlines.at(o).size(); p++){
        if (outlines.at(o).at(p).at(1) == currentY){
          outlines.at(o).at(p).at(1) = outlines.at(o).at(p).at(1) + 1;
        }
      }
    }
  }
}

bool FindOutlineIntersections(){
  //Extend outlines with intersection points
  for (int i = 0; i < outlines.size(); i++){
    extOutlines.push_back( std::vector<std::vector<int>>());

    for (int j = 0; j < outlines.at(i).size(); j++){
      extOutlines.at(i).push_back( std::vector<int>());
      
      for (int k = 0; k < outlines.at(i).at(j).size(); k++){
        extOutlines.at(i).at(j).push_back(outlines.at(i).at(j).at(k));
      }
    }
  }

  for (int i = 0; i < numOfLines; i++)
  {
    int currentY = (int) (terrainMinY + abs(terrainMaxY - terrainMinY) / (float)(numOfLines) * i + abs(terrainMaxY - terrainMinY) / (float)(numOfLines) / 2.0);
    
    //Traverse outlines and extend outlines with intersections
    for (int o = 0; o < extOutlines.size(); o++){
      
      intersections.clear();
      // X1, Y1, NX, NY
      
      //Find intersections for current outline
      for (int p = 0; p < extOutlines.at(o).size(); p++){
        int p1X = extOutlines.at(o).at(p).at(0);
        int p1Y = extOutlines.at(o).at(p).at(1);
        
        int p2X, p2Y;
        
        //First-last case
        if (p == extOutlines.at(o).size() - 1){
          p2X = extOutlines.at(o).at(0).at(0);
          p2Y = extOutlines.at(o).at(0).at(1);
        }
        else {
          p2X = extOutlines.at(o).at(p + 1).at(0);
          p2Y = extOutlines.at(o).at(p + 1).at(1);
        }
        
        //Check if lines intersect, and add intersection
        if ((currentY >= p1Y && currentY <= p2Y) || (currentY <= p1Y && currentY >= p2Y))
        {
            intersections.push_back( std::vector<int>());

            int thisX;

            if (p1X != p2X)
            {
                thisX = (int)((currentY - p1Y) / (float)(p2Y - p1Y) * (p2X - p1X) + p1X);
            }
            else { thisX = p1X; } //Vertical slope

            // X1, Y1, NX, NY
            intersections.at(intersections.size() - 1).push_back(p1X);
            intersections.at(intersections.size() - 1).push_back(p1Y);
            intersections.at(intersections.size() - 1).push_back(thisX);
            intersections.at(intersections.size() - 1).push_back(currentY);
        }
      }
      
      //Insert intersections into list
      int p = 0;
      while (p < extOutlines.at(o).size()){
        int p1X = extOutlines.at(o).at(p).at(0);
        int p1Y = extOutlines.at(o).at(p).at(1);
        
        /*
        int p2X, p2Y;
        
        //First-last case
        if (p == extOutlines.at(o).size() - 1){
          p2X = extOutlines.at(o).at(0).at(0);
          p2Y = extOutlines.at(o).at(0).at(1);
        }
        else {
          p2X = extOutlines.at(o).at(p + 1).at(0);
          p2Y = extOutlines.at(o).at(p + 1).at(1);
        }
        */
        
        //Check corresponding intersections
        for (int it = 0; it < intersections.size(); it++){
          
          //If point is between these points, insert it into list
          if ((intersections.at(it).at(0) == p1X) &&
              (intersections.at(it).at(1) == p1Y))
          {
            std::vector<int> appendList;
            appendList.push_back(intersections.at(it).at(2));
            appendList.push_back(intersections.at(it).at(3));
            
            extOutlines.at(o).insert(extOutlines[o].begin() + p + 1, appendList);

            p++;
          }
        }
        
        p++;
      }
    }
  }

  return true;
}

bool GeneratePaths(){
  //Scan and find intersections
  for (int i = 0; i < numOfLines; i++)
  {
    int currentY = (int) (terrainMinY + abs(terrainMaxY - terrainMinY) / (float)(numOfLines) * i + abs(terrainMaxY - terrainMinY) / (float)(numOfLines) / 2.0);

    intersectionPaths.push_back( std::vector<std::vector<int>>());
    
    for (int o = 0; o < extOutlines.size(); o++){
      for (int p = 0; p < extOutlines.at(o).size(); p++){
        if (extOutlines.at(o).at(p).at(1) == currentY){
          //O, P, NX
          intersectionPaths.at(intersectionPaths.size() - 1).push_back( std::vector<int>());
          intersectionPaths.at(intersectionPaths.size() - 1).at(intersectionPaths.at(intersectionPaths.size() - 1).size() - 1).push_back((int) o);
          intersectionPaths.at(intersectionPaths.size() - 1).at(intersectionPaths.at(intersectionPaths.size() - 1).size() - 1).push_back((int) p);
          intersectionPaths.at(intersectionPaths.size() - 1).at(intersectionPaths.at(intersectionPaths.size() - 1).size() - 1).push_back((int) extOutlines.at(o).at(p).at(0));
        }
      }
    }
  }
  
  //Sort by intitude
  for (int d = 0; d < intersectionPaths.size(); d++){
    ace_sorting::quickSortMiddle(intersectionPaths.at(d).data(), (uint16_t) intersectionPaths.at(d).size(), [](std::vector<int> o1, std::vector<int> o2) { return o1.at(2) < o2.at(2); });
  }

  return true;
}

bool GenerateGcode(){
  //Error conditions
  if (outlines.size() < 1){return false;}
  if (outlines.at(0).size() <= 2){return false;}
  
  FindTerrainBounds();
  ClearInterference(); //Makes sure no point in outlines is directly on infill Y (Patch for intersection check in FindOutlineIntersections)
  if (!FindOutlineIntersections()){return false;}
  if (!GeneratePaths()){return false;}

  return true;
}

//negative - decreasing index
//positive - increasing index

//WARNING:
/*
OUTLINE MUST EXIST.
OUTLINE NEEDS TO HAVE AT LEAST 2 POINTS.
POINTS MUST BE IN OUTLINE
*/
int ShortestOutlinePath(int outline_index, int current_point, int target_point){
  float path_length_inc = 0;
  float path_length_dec = 0;
  
  int tmp_index;
  
  //Increasing
  tmp_index = current_point;
  
  while (tmp_index != target_point) {
    
    //End of list
    if (tmp_index == extOutlines.at(outline_index).size() - 1){
      
      path_length_inc += (float) sqrt(pow(extOutlines.at(outline_index).at(extOutlines.at(outline_index).size() - 1).at(0) - extOutlines.at(outline_index).at(0).at(0), 2) + pow(extOutlines.at(outline_index).at(extOutlines.at(outline_index).size() - 1).at(1) - extOutlines.at(outline_index).at(0).at(1), 2));
      
      tmp_index = 0;
      continue;
    }
    
    path_length_inc += (float) sqrt(pow(extOutlines.at(outline_index).at(tmp_index).at(0) - extOutlines.at(outline_index).at(tmp_index + 1).at(0), 2) + pow(extOutlines.at(outline_index).at(tmp_index).at(1) - extOutlines.at(outline_index).at(tmp_index + 1).at(1), 2));
    tmp_index++;
  }
  
  //Decreasing
  tmp_index = current_point;
  
  while (tmp_index != target_point) {
    
    //End of list
    if (tmp_index == 0){
      
      path_length_dec += (float) sqrt(pow(extOutlines.at(outline_index).at(extOutlines.at(outline_index).size() - 1).at(0) - extOutlines.at(outline_index).at(0).at(0), 2) + pow(extOutlines.at(outline_index).at(extOutlines.at(outline_index).size() - 1).at(1) - extOutlines.at(outline_index).at(0).at(1), 2));
      
      tmp_index = extOutlines.at(outline_index).size() - 1;
      continue;
    }
    
    path_length_dec += (float) sqrt(pow(extOutlines.at(outline_index).at(tmp_index).at(0) - extOutlines.at(outline_index).at(tmp_index - 1).at(0), 2) + pow(extOutlines.at(outline_index).at(tmp_index).at(1) - extOutlines.at(outline_index).at(tmp_index - 1).at(1), 2));
    tmp_index--;
  }
  
  if (path_length_inc < path_length_dec){
    return (int) ceil(path_length_inc);
  }
  else {
    return (int) -ceil(path_length_dec);
  }
}

int OutlineTraverseInc(int outline_index, int current_point, int amount){
  for (int i = 0; i < amount; i++){
    current_point++;
    
    if (current_point >= extOutlines.at(outline_index).size()){current_point = 0;}
  }
  
  return current_point;
}

int OutlineTraverseDec(int outline_index, int current_point, int amount){
  for (int i = 0; i < amount; i++){
    current_point--;
    
    if (current_point < 0){current_point = extOutlines.at(outline_index).size() - 1;}
  }
  
  return current_point;
}

std::vector<int> AlgorithmNextPoint(){
  int NextX = 0;
  int NextY = 0;
  
  if (algorithmTarget == "FORWARD"){
    if (algorithmMode == "INFILL"){
      MotorMainOn();
      
      //End of infill
      if (algorithmInfillDirection == 1 && algorithmInfillPoint == intersectionPaths.at(algorithmInfillIndex).size() - 1){
        algorithmMode = "SEEK";
        
        algorithmCurrentOutline = int(intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(0));
        algorithmCurrentPoint = int(intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(1));
        
        algorithmInfillIndex++;
        
        if (algorithmInfillIndex >= intersectionPaths.size()){
          algorithmTarget = "BASE";
          algorithmMode = "SEEK";
          
          algorithmCurrentOutline = int(intersectionPaths.at(algorithmInfillIndex - 1).at(algorithmInfillPoint).at(0));
          algorithmCurrentPoint = int(intersectionPaths.at(algorithmInfillIndex - 1).at(algorithmInfillPoint).at(1));
        }
      }
      else if (algorithmInfillDirection == 0 && algorithmInfillPoint == 0){
        algorithmMode = "SEEK";
        
        algorithmCurrentOutline = int(intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(0));
        algorithmCurrentPoint = int(intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(1));
        
        algorithmInfillIndex++;
        
        if (algorithmInfillIndex >= intersectionPaths.size()){
          algorithmTarget = "BASE";
          algorithmMode = "SEEK";
          
          algorithmCurrentOutline = int(intersectionPaths.at(algorithmInfillIndex - 1).at(algorithmInfillPoint).at(0));
          algorithmCurrentPoint = int(intersectionPaths.at(algorithmInfillIndex - 1).at(algorithmInfillPoint).at(1));
        }
      }
      else {
        //Jump outlines
        if ((algorithmInfillDirection == 1 && algorithmInfillPoint % 2 == 0) || (algorithmInfillDirection == 0 && algorithmInfillPoint % 2 == 1)){
          if (algorithmInfillDirection == 1){
            algorithmInfillPoint++;
          }
          else {
            algorithmInfillPoint--;
          }
          
          algorithmCurrentOutline = int(intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(0));
          algorithmCurrentPoint = int(intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(1));
        }
        //Traverse outline
        else {
          //Calculate distances
          int shortestPath;
          int nextInfillPoint;
          
          //Find next infill point
          if (algorithmInfillDirection == 1){
            nextInfillPoint = algorithmInfillPoint + 1;
          }
          else {
            nextInfillPoint = algorithmInfillPoint - 1;
          }
          
          //Find shortest path to the next infill point
          shortestPath = ShortestOutlinePath(algorithmCurrentOutline, algorithmCurrentPoint, int(intersectionPaths.at(algorithmInfillIndex).at(nextInfillPoint).at(1)));
          
          //Traverse in shortest direction
          if (shortestPath > 0){
            algorithmCurrentPoint = OutlineTraverseInc(algorithmCurrentOutline, algorithmCurrentPoint, 1);
          }
          else {
            algorithmCurrentPoint = OutlineTraverseDec(algorithmCurrentOutline, algorithmCurrentPoint, 1);
          }
          
          //Reached continuation of infill
          if (algorithmCurrentPoint == int(intersectionPaths.at(algorithmInfillIndex).at(nextInfillPoint).at(1))){
            if (algorithmInfillDirection == 1){
              algorithmInfillPoint++;
            }
            else {
              algorithmInfillPoint--;
            }
            
            algorithmCurrentOutline = int(intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(0));
            algorithmCurrentPoint = int(intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(1));
          }
        }
      }
      
      NextX = extOutlines.at(algorithmCurrentOutline).at(algorithmCurrentPoint).at(0);
      NextY = extOutlines.at(algorithmCurrentOutline).at(algorithmCurrentPoint).at(1);
    }
    if (algorithmTarget == "FORWARD" && algorithmMode == "SEEK"){
      MotorMainOff();
      
      //Calculate distances
      int distInfillA = ShortestOutlinePath(algorithmCurrentOutline, algorithmCurrentPoint, int(intersectionPaths.at(algorithmInfillIndex).at(0).at(1)));
      int distInfillB = ShortestOutlinePath(algorithmCurrentOutline, algorithmCurrentPoint, int(intersectionPaths.at(algorithmInfillIndex).at(intersectionPaths.at(algorithmInfillIndex).size() - 1).at(1)));
      
      int shortestPath;
      
      //Choose shortest path
      if (abs(distInfillA) < abs(distInfillB)){
        shortestPath = distInfillA;
      }
      else {
        shortestPath = distInfillB;
      }
      
      //Traverse in shortest direction
      if (shortestPath > 0){
        algorithmCurrentPoint = OutlineTraverseInc(algorithmCurrentOutline, algorithmCurrentPoint, 1);
      }
      else {
        algorithmCurrentPoint = OutlineTraverseDec(algorithmCurrentOutline, algorithmCurrentPoint, 1);
      }
      
      //If reached infill, switch mode
      if (algorithmCurrentPoint == int(intersectionPaths.at(algorithmInfillIndex).at(0).at(1))){
        algorithmMode = "INFILL";
        algorithmInfillPoint = 0;
        algorithmInfillDirection = 1;
      }
      else if (algorithmCurrentPoint == int(intersectionPaths.at(algorithmInfillIndex).at(intersectionPaths.at(algorithmInfillIndex).size() - 1).at(1))){
        algorithmMode = "INFILL";
        algorithmInfillPoint = intersectionPaths.at(algorithmInfillIndex).size() - 1;
        algorithmInfillDirection = 0;
      }
      
      NextX = extOutlines.at(algorithmCurrentOutline).at(algorithmCurrentPoint).at(0);
      NextY = extOutlines.at(algorithmCurrentOutline).at(algorithmCurrentPoint).at(1);
    }
    if (algorithmMode == "OUTLINE"){
      MotorMainOn();
      
      algorithmCurrentPoint++;
      
      if (algorithmCurrentPoint >= extOutlines.at(0).size()){
        algorithmCurrentPoint = 0;
        
        //Start seeking first infill start
        algorithmMode = "SEEK";
        algorithmInfillIndex = 0;
      }
      
      NextX = extOutlines.at(algorithmCurrentOutline).at(algorithmCurrentPoint).at(0);
      NextY = extOutlines.at(algorithmCurrentOutline).at(algorithmCurrentPoint).at(1);
    }
  }
  if (algorithmTarget == "BASE"){
    MotorMainOff();
    
    if (algorithmMode == "INFILL"){
      //Jump outlines
      if ((algorithmInfillDirection == 0 && algorithmInfillPoint % 2 == 0 && algorithmCurrentPoint == intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(1)) || (algorithmInfillDirection == 1 && algorithmInfillPoint % 2 == 1 && algorithmCurrentPoint == intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(1))){
        if (algorithmInfillDirection == 1){
          algorithmInfillPoint--;
        }
        else {
          algorithmInfillPoint++;
        }
        
        algorithmCurrentOutline = int(intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(0));
        algorithmCurrentPoint = int(intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(1));
      }
      //Traverse outline
      else {
        //Jump outline skip
        if ((algorithmInfillDirection == 0 && algorithmInfillPoint % 2 == 1 && algorithmCurrentPoint == intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(1)) || (algorithmInfillDirection == 1 && algorithmInfillPoint % 2 == 0 && algorithmCurrentPoint == intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(1))){
          if (algorithmInfillDirection == 1){
            algorithmInfillPoint--;
          }
          else {
            algorithmInfillPoint++;
          }
        }
        
        //Reached continuation of infill
        if (algorithmCurrentPoint == int(intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(1))){
          if (algorithmInfillDirection == 1){
            algorithmInfillPoint--;
          }
          else {
            algorithmInfillPoint++;
          }
          
          algorithmCurrentOutline = int(intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(0));
          algorithmCurrentPoint = int(intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(1));
        }
        
        //Calculate distances
        int shortestPath;
        
        //Find shortest path to the next infill point
        shortestPath = ShortestOutlinePath(algorithmCurrentOutline, algorithmCurrentPoint, int(intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(1)));
        
        //Traverse in shortest direction
        if (shortestPath > 0){
          algorithmCurrentPoint = OutlineTraverseInc(algorithmCurrentOutline, algorithmCurrentPoint, 1);
        }
        else {
          algorithmCurrentPoint = OutlineTraverseDec(algorithmCurrentOutline, algorithmCurrentPoint, 1);
        }
      }
      
      //End of infill
      if (algorithmInfillDirection == 0 && algorithmInfillPoint == intersectionPaths.at(algorithmInfillIndex).size() - 1){
        if (!algorithmAbortFull){
          algorithmTarget = "FORWARD";
        }
        
        algorithmMode = "SEEK";
        
        algorithmCurrentOutline = int(intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(0));
        algorithmCurrentPoint = int(intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(1));
        
        algorithmInfillIndex++;
        
        if (algorithmInfillIndex >= intersectionPaths.size()){
          algorithmAbortFull = true;
        }
      }
      else if (algorithmInfillDirection == 1 && algorithmInfillPoint == 0){
        if (!algorithmAbortFull){
          algorithmTarget = "FORWARD";
        }
        
        algorithmMode = "SEEK";
        
        algorithmCurrentOutline = int(intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(0));
        algorithmCurrentPoint = int(intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(1));
        
        algorithmInfillIndex++;
        
        if (algorithmInfillIndex >= intersectionPaths.size()){
          algorithmAbortFull = true;
        }
      }
      
      NextX = extOutlines.at(algorithmCurrentOutline).at(algorithmCurrentPoint).at(0);
      NextY = extOutlines.at(algorithmCurrentOutline).at(algorithmCurrentPoint).at(1);
    }
    else if (algorithmMode == "SEEK"){
      //If reached charging station exit point
      if (algorithmCurrentPoint == 0){
        MainChargingStart();
      }
      
      //Find shortest path to the charging station exit point
      int shortestPath = ShortestOutlinePath(algorithmCurrentOutline, algorithmCurrentPoint, 0);
      
      //Traverse in shortest direction
      if (shortestPath > 0){
        algorithmCurrentPoint = OutlineTraverseInc(algorithmCurrentOutline, algorithmCurrentPoint, 1);
      }
      else {
        algorithmCurrentPoint = OutlineTraverseDec(algorithmCurrentOutline, algorithmCurrentPoint, 1);
      }
      
      NextX = extOutlines.at(algorithmCurrentOutline).at(algorithmCurrentPoint).at(0);
      NextY = extOutlines.at(algorithmCurrentOutline).at(algorithmCurrentPoint).at(1);
    }
  }
  
  prevPointLon = targetPointLon;
  prevPointLat = targetPointLat;
  
  std::vector<int> returnList;
  returnList.push_back(NextX); returnList.push_back(NextY);
  return returnList;
}

void AlgorithmAbort(bool full_abort){
  if (algorithmTarget != "BASE"){
    if (full_abort){
      algorithmAbortFull = true;
    }
    else {
      algorithmAbortFull = false;
    }
    
    algorithmTarget = "BASE";
    
    if (algorithmMode == "OUTLINE"){
      algorithmMode = "SEEK";
    }
    else if (algorithmMode == "INFILL"){
      //End of infill
      if (algorithmInfillDirection == 0 && algorithmInfillPoint == intersectionPaths.at(algorithmInfillIndex).size() - 1){
        if (!algorithmAbortFull){
          algorithmTarget = "FORWARD";
        }
        
        algorithmMode = "SEEK";
        
        algorithmCurrentOutline = int(intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(0));
        algorithmCurrentPoint = int(intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(1));
        
        algorithmInfillIndex++;
        
        if (algorithmInfillIndex >= intersectionPaths.size()){
          algorithmAbortFull = true;
        }
      }
      else if (algorithmInfillDirection == 1 && algorithmInfillPoint == 0){
        if (!algorithmAbortFull){
          algorithmTarget = "FORWARD";
        }
        
        algorithmMode = "SEEK";
        
        algorithmCurrentOutline = int(intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(0));
        algorithmCurrentPoint = int(intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(1));
        
        algorithmInfillIndex++;
        
        if (algorithmInfillIndex >= intersectionPaths.size()){
          algorithmAbortFull = true;
        }
      }
    }
    
    //TODO: Mower reverse for a bit from obstacle
    std::vector<int> targetPoint = AlgorithmNextPoint();
    targetPointLon = targetPoint.at(0);
    targetPointLat = targetPoint.at(1);
    
    MotionRotateToTarget();
  }
}

void AlgorithmCaptureStart(){
  outlines.clear();

  AlgorithmCaptureNewOutline();
}

void AlgorithmCaptureNewOutline(){
  outlines.push_back( std::vector<std::vector<int>>());
}
void AlgorithmCaptureNewPoint(){
  outlines.back().push_back( std::vector<int>());

  outlines.back().back().push_back(GpsGetLon());
  outlines.back().back().push_back(GpsGetLat());
}
void AlgorithmCaptureSetNewPoint(int lon, int lat){
  outlines.back().push_back( std::vector<int>());

  outlines.back().back().push_back(lon);
  outlines.back().back().push_back(lat);
}
bool AlgorithmCaptureRemoveOutline(){
  //Index out of bounds, failed
  if (outlines.size() == 0){return false;}

  outlines.pop_back();
  
  return true;
}

bool AlgorithmCaptureRemovePoint(){
  //Index out of bounds, stop algorithm
  if (outlines.back().size() == 0){return false;} 

  outlines.back().pop_back();
  
  return true;
}

bool AlgorithmCaptureEnd(){
  return GenerateGcode();
}