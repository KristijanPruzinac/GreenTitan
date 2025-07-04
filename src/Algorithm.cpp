#include "Algorithm.h"

void MainChargingStart(){
  
}

//Charging station
long long prevPointLon = 0;
long long prevPointLat = 0;

//Target
long long targetPointLon = BASE_LON;
long long targetPointLat = BASE_LAT;

//Globals
int numOfLines = 1;

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
std::vector<std::vector<std::vector<long long>>> outlines;
std::vector<std::vector<std::vector<long long>>> extOutlines; //Outlines with intersection points inserted
std::vector<std::vector<std::vector<long long>>> intersectionPaths; //Collections of points where infill intersects terrain and outer outline

long long terrainMinX;
long long terrainMaxX;
long long terrainMinY;
long long terrainMaxY;

void ClearOutlines(){
  extOutlines.clear();
  intersectionPaths.clear();
}

void FindTerrainBounds(){
  //Find terrain bounds
  terrainMinX = outlines.at(0).at(0).at(0);
  terrainMaxX = outlines.at(0).at(0).at(0);
  terrainMinY = outlines.at(0).at(0).at(1);
  terrainMaxY = outlines.at(0).at(0).at(1);
  
  for (int i = 0; i < outlines.size(); i++){
    for (int j = 0; j < outlines.at(i).size(); j++){
      long long xVal = outlines.at(i).at(j).at(0);
      long long yVal = outlines.at(i).at(j).at(1);
      
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
    long long currentY = (long long) (terrainMinY + abs(terrainMaxY - terrainMinY) / (double)(numOfLines) * i + abs(terrainMaxY - terrainMinY) / (double)(numOfLines) / 2.0);
    
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
    extOutlines.push_back( std::vector<std::vector<long long>>());

    for (int j = 0; j < outlines.at(i).size(); j++){
      extOutlines.back().push_back( std::vector<long long>());
      
      for (int k = 0; k < outlines.at(i).at(j).size(); k++){
        extOutlines.back().back().push_back(outlines.at(i).at(j).at(k));
      }
    }
  }

  for (int i = 0; i < numOfLines; i++)
  {
    long long currentY = (long long) (terrainMinY + abs(terrainMaxY - terrainMinY) / (double)(numOfLines) * i + abs(terrainMaxY - terrainMinY) / (double)(numOfLines) / 2.0);
    
    //Traverse outlines and extend outlines with intersections
    for (int o = 0; o < extOutlines.size(); o++){
      
      std::vector<std::vector<long long>> intersections;
      // X1, Y1, NX, NY

      //Find intersections for current outline
      for (int p = 0; p < extOutlines.at(o).size(); p++){
        long long p1X = extOutlines.at(o).at(p).at(0);
        long long p1Y = extOutlines.at(o).at(p).at(1);

        long long p2X, p2Y;
        
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
            intersections.push_back( std::vector<long long>());

            long long thisX;

            if (p1X != p2X)
            {
                thisX = (long long)((currentY - p1Y) / (double)(p2Y - p1Y) * (p2X - p1X) + p1X);
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
        long long p1X = extOutlines.at(o).at(p).at(0);
        long long p1Y = extOutlines.at(o).at(p).at(1);
        
        //Check corresponding intersections
        for (int it = 0; it < intersections.size(); it++){
          
          //If point is between these points, insert it into list
          if ((intersections.at(it).at(0) == p1X) &&
              (intersections.at(it).at(1) == p1Y))
          {
            std::vector<long long> appendList;
            appendList.push_back(intersections.at(it).at(2));
            appendList.push_back(intersections.at(it).at(3));

            extOutlines.at(o).insert(extOutlines.at(o).begin() + p + 1, appendList);

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
    long long currentY = (long long) (terrainMinY + abs(terrainMaxY - terrainMinY) / (double)(numOfLines) * i + abs(terrainMaxY - terrainMinY) / (double)(numOfLines) / 2.0);

    intersectionPaths.push_back( std::vector<std::vector<long long>>());
    
    for (int o = 0; o < extOutlines.size(); o++){
      for (int p = 0; p < extOutlines.at(o).size(); p++){
        if (extOutlines.at(o).at(p).at(1) == currentY){
          //O, P, NX
          intersectionPaths.at(intersectionPaths.size() - 1).push_back( std::vector<long long>());
          intersectionPaths.at(intersectionPaths.size() - 1).at(intersectionPaths.at(intersectionPaths.size() - 1).size() - 1).push_back((long long) o);
          intersectionPaths.at(intersectionPaths.size() - 1).at(intersectionPaths.at(intersectionPaths.size() - 1).size() - 1).push_back((long long) p);
          intersectionPaths.at(intersectionPaths.size() - 1).at(intersectionPaths.at(intersectionPaths.size() - 1).size() - 1).push_back((long long) extOutlines.at(o).at(p).at(0));
        }
      }
    }
  }
  
  //Sort by intitude
  for (int d = 0; d < intersectionPaths.size(); d++){
    ace_sorting::quickSortMiddle(intersectionPaths.at(d).data(), (uint16_t) intersectionPaths.at(d).size(), [](std::vector<long long> o1, std::vector<long long> o2) { return o1.at(2) < o2.at(2); });
  }

  return true;
}

bool GenerateGcode(){
  //Clean up
  ClearOutlines();
  //Error conditions
  if (outlines.size() < 1){return false;}
  if (outlines.at(0).size() <= 2){return false;}
  
  FindTerrainBounds();

  //Here it gets stuck
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
  double path_length_inc = 0;
  double path_length_dec = 0;
  
  int tmp_index;
  
  //Increasing
  tmp_index = current_point;
  
  while (tmp_index != target_point) {
    
    //End of list
    if (tmp_index == extOutlines.at(outline_index).size() - 1){
      
      path_length_inc += (double) sqrt(pow(extOutlines.at(outline_index).at(extOutlines.at(outline_index).size() - 1).at(0) - extOutlines.at(outline_index).at(0).at(0), 2) + pow(extOutlines.at(outline_index).at(extOutlines.at(outline_index).size() - 1).at(1) - extOutlines.at(outline_index).at(0).at(1), 2));
      
      tmp_index = 0;
      continue;
    }
    
    path_length_inc += (double) sqrt(pow(extOutlines.at(outline_index).at(tmp_index).at(0) - extOutlines.at(outline_index).at(tmp_index + 1).at(0), 2) + pow(extOutlines.at(outline_index).at(tmp_index).at(1) - extOutlines.at(outline_index).at(tmp_index + 1).at(1), 2));
    tmp_index++;
  }
  
  //Decreasing
  tmp_index = current_point;
  
  while (tmp_index != target_point) {
    
    //End of list
    if (tmp_index == 0){
      
      path_length_dec += (double) sqrt(pow(extOutlines.at(outline_index).at(extOutlines.at(outline_index).size() - 1).at(0) - extOutlines.at(outline_index).at(0).at(0), 2) + pow(extOutlines.at(outline_index).at(extOutlines.at(outline_index).size() - 1).at(1) - extOutlines.at(outline_index).at(0).at(1), 2));
      
      tmp_index = extOutlines.at(outline_index).size() - 1;
      continue;
    }
    
    path_length_dec += (double) sqrt(pow(extOutlines.at(outline_index).at(tmp_index).at(0) - extOutlines.at(outline_index).at(tmp_index - 1).at(0), 2) + pow(extOutlines.at(outline_index).at(tmp_index).at(1) - extOutlines.at(outline_index).at(tmp_index - 1).at(1), 2));
    tmp_index--;
  }
  
  if (path_length_inc < path_length_dec){
    return (long long) ceil(path_length_inc);
  }
  else {
    return (long long) -ceil(path_length_dec);
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

std::vector<long long> AlgorithmNextPoint(){
  long long NextX = 0;
  long long NextY = 0;
  
  if (algorithmTarget == "FORWARD"){
    if (algorithmMode == "INFILL"){
      MotorMainOn();
      
      //End of infill
      if (algorithmInfillDirection == 1 && algorithmInfillPoint == intersectionPaths.at(algorithmInfillIndex).size() - 1){
        algorithmMode = "SEEK";
        
        algorithmCurrentOutline = (long long) (intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(0));
        algorithmCurrentPoint = (long long) (intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(1));
        
        algorithmInfillIndex++;
        
        if (algorithmInfillIndex >= intersectionPaths.size()){
          algorithmTarget = "BASE";
          algorithmMode = "SEEK";
          
          algorithmCurrentOutline = (long long) (intersectionPaths.at(algorithmInfillIndex - 1).at(algorithmInfillPoint).at(0));
          algorithmCurrentPoint = (long long) (intersectionPaths.at(algorithmInfillIndex - 1).at(algorithmInfillPoint).at(1));
        }
      }
      else if (algorithmInfillDirection == 0 && algorithmInfillPoint == 0){
        algorithmMode = "SEEK";
        
        algorithmCurrentOutline = (long long) (intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(0));
        algorithmCurrentPoint = (long long) (intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(1));
        
        algorithmInfillIndex++;
        
        if (algorithmInfillIndex >= intersectionPaths.size()){
          algorithmTarget = "BASE";
          algorithmMode = "SEEK";
          
          algorithmCurrentOutline = (long long) (intersectionPaths.at(algorithmInfillIndex - 1).at(algorithmInfillPoint).at(0));
          algorithmCurrentPoint = (long long) (intersectionPaths.at(algorithmInfillIndex - 1).at(algorithmInfillPoint).at(1));
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
          
          algorithmCurrentOutline = (long long) (intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(0));
          algorithmCurrentPoint = (long long) (intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(1));
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
          shortestPath = ShortestOutlinePath(algorithmCurrentOutline, algorithmCurrentPoint, (long long) (intersectionPaths.at(algorithmInfillIndex).at(nextInfillPoint).at(1)));
          
          //Traverse in shortest direction
          if (shortestPath > 0){
            algorithmCurrentPoint = OutlineTraverseInc(algorithmCurrentOutline, algorithmCurrentPoint, 1);
          }
          else {
            algorithmCurrentPoint = OutlineTraverseDec(algorithmCurrentOutline, algorithmCurrentPoint, 1);
          }
          
          //Reached continuation of infill
          if (algorithmCurrentPoint == (long long) (intersectionPaths.at(algorithmInfillIndex).at(nextInfillPoint).at(1))){
            if (algorithmInfillDirection == 1){
              algorithmInfillPoint++;
            }
            else {
              algorithmInfillPoint--;
            }
            
            algorithmCurrentOutline = (long long) (intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(0));
            algorithmCurrentPoint = (long long) (intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(1));
          }
        }
      }
      
      NextX = extOutlines.at(algorithmCurrentOutline).at(algorithmCurrentPoint).at(0);
      NextY = extOutlines.at(algorithmCurrentOutline).at(algorithmCurrentPoint).at(1);
    }
    if (algorithmTarget == "FORWARD" && algorithmMode == "SEEK"){
      MotorMainOff();
      
      //Calculate distances
      int distInfillA = ShortestOutlinePath(algorithmCurrentOutline, algorithmCurrentPoint, (long long) (intersectionPaths.at(algorithmInfillIndex).at(0).at(1)));
      int distInfillB = ShortestOutlinePath(algorithmCurrentOutline, algorithmCurrentPoint, (long long) (intersectionPaths.at(algorithmInfillIndex).at(intersectionPaths.at(algorithmInfillIndex).size() - 1).at(1)));
      
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
      if (algorithmCurrentPoint == (long long) (intersectionPaths.at(algorithmInfillIndex).at(0).at(1))){
        algorithmMode = "INFILL";
        algorithmInfillPoint = 0;
        algorithmInfillDirection = 1;
      }
      else if (algorithmCurrentPoint == (long long) (intersectionPaths.at(algorithmInfillIndex).at(intersectionPaths.at(algorithmInfillIndex).size() - 1).at(1))){
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
        
        algorithmCurrentOutline = (long long) (intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(0));
        algorithmCurrentPoint = (long long) (intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(1));
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
        if (algorithmCurrentPoint == (long long) (intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(1))){
          if (algorithmInfillDirection == 1){
            algorithmInfillPoint--;
          }
          else {
            algorithmInfillPoint++;
          }
          
          algorithmCurrentOutline = (long long) (intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(0));
          algorithmCurrentPoint = (long long) (intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(1));
        }
        
        //Calculate distances
        int shortestPath;
        
        //Find shortest path to the next infill point
        shortestPath = ShortestOutlinePath(algorithmCurrentOutline, algorithmCurrentPoint, (long long) (intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(1)));
        
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
        
        algorithmCurrentOutline = (long long) (intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(0));
        algorithmCurrentPoint = (long long) (intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(1));
        
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
        
        algorithmCurrentOutline = (long long) (intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(0));
        algorithmCurrentPoint = (long long) (intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(1));
        
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
  
  std::vector<long long> returnList;
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
        
        algorithmCurrentOutline = (long long) (intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(0));
        algorithmCurrentPoint = (long long) (intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(1));
        
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
        
        algorithmCurrentOutline = (long long) (intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(0));
        algorithmCurrentPoint = (long long) (intersectionPaths.at(algorithmInfillIndex).at(algorithmInfillPoint).at(1));
        
        algorithmInfillIndex++;
        
        if (algorithmInfillIndex >= intersectionPaths.size()){
          algorithmAbortFull = true;
        }
      }
    }
    
    //Mower reverse for a bit from obstacle
    //TODO: Make into a function
    MotorDriveAngle(0, false, 1.0);
    delay(750);
    
    std::vector<long long> targetPoint = AlgorithmNextPoint();
    targetPointLon = targetPoint.at(0);
    targetPointLat = targetPoint.at(1);
    
    MotionSetTargetPoint(targetPointLon, targetPointLat);
  }
}

void AlgorithmCaptureStart(){
  outlines.clear();
}

void AlgorithmCaptureNewOutline(){
  outlines.push_back( std::vector<std::vector<long long>>());
}
void AlgorithmCaptureBasePoint(){
  BASE_LON = GetLon();
  BASE_LAT = GetLat();
}
void AlgorithmCaptureBaseExitPoint(){
  BASE_EXIT_LON = GetLon();
  BASE_EXIT_LAT = GetLat();
}
void AlgorithmCaptureNewPoint(){
  outlines.back().push_back( std::vector<long long>());

  outlines.back().back().push_back(GetLon());
  outlines.back().back().push_back(GetLat());
}
void AlgorithmCaptureSetNewPoint(long long lon, long long lat){
  outlines.back().push_back( std::vector<long long>());

  outlines.back().back().push_back(lon);
  outlines.back().back().push_back(lat);

  //TODO: Remove
  //Serial.println(">point:" + String(lon) + ":" + String(lat) + "|xy");
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

String AlgorithmGetPathString(){
  String PathString = "";

  for (int i = 0; i < outlines.size(); i++){
    PathString += "O\n";

    for (int j = 0; j < outlines.at(i).size(); j++){
      PathString += String((int) outlines.at(i).at(j).at(0)) + " " + String((int) outlines.at(i).at(j).at(1)) + "\n";
    }
  }

  return PathString;
}

bool AlgorithmPopulatePathFromString(String& readData){
  outlines.clear();

  int newlineIndex = 0;
  while ((newlineIndex = readData.indexOf('\n')) != -1) {
        String line = readData.substring(0, newlineIndex);
        readData = readData.substring(newlineIndex + 1);

        if (line[0] == 'O'){
          AlgorithmCaptureNewOutline();
        }
        else {
          // Split each line by space and assign values to variables
          int spaceIndex = line.indexOf(' ');
          if (spaceIndex != -1) {
              String variableValue1 = line.substring(0, spaceIndex);
              String variableValue2 = line.substring(spaceIndex + 1);

              AlgorithmCaptureSetNewPoint(variableValue1.toInt(), variableValue2.toInt());
          }
        }
    }

    return AlgorithmCaptureEnd();
}