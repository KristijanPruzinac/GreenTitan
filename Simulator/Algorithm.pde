import java.io.*; //<>//
import java.util.*;

//Charging station
long baseLon = -5672328; //TODO: REMOVE HARDCODED VALUE (Filled at power on)
long baseLat = 3376391; //TODO: REMOVE HARDCODED VALUE (Filled at power on)

long prevPointLon = 0;
long prevPointLat = 0;

//Globals
int numOfLines;

//Algorithm
String algorithmTarget = "FORWARD"; // FORWARD, BASE
String algorithmMode = "OUTLINE";
// FORWARD: OUTLINE, SEEK, INFILL
// BASE: INFILL, SEEK

boolean algorithmAbortFull = false;

/*
Explanation:
OUTLINE - Mows whole outline
SEEK - Searches for point along outer outline
INFILL - Mows infill
*/

//Only in INFILL mode
int algorithmInfillIndex;
int algorithmInfillPoint;
int algorithmInfillDirection;

int algorithmCurrentOutline = 0;
int algorithmCurrentPoint = 0;

//Outlines - First point is the charging station exit point
ArrayList<ArrayList<ArrayList<Long>>> outlines = new ArrayList<ArrayList<ArrayList<Long>>>();
ArrayList<ArrayList<ArrayList<Long>>> extOutlines = new ArrayList<ArrayList<ArrayList<Long>>>(); //Outlines with intersection points inserted

ArrayList<ArrayList<ArrayList<Long>>> intersectionPaths = new ArrayList<ArrayList<ArrayList<Long>>>(); //Collections of points where infill intersects terrain

long terrainMinX;
long terrainMaxX;
long terrainMinY;
long terrainMaxY;

void FindTerrainBounds(){
  //Find terrain bounds
  terrainMinX = outlines.get(0).get(0).get(0);
  terrainMaxX = outlines.get(0).get(0).get(0);
  terrainMinY = outlines.get(0).get(0).get(1);
  terrainMaxY = outlines.get(0).get(0).get(1);
  
  for (int i = 0; i < outlines.size(); i++){
    for (int j = 0; j < outlines.get(i).size(); j++){
      long xVal = outlines.get(i).get(j).get(0);
      long yVal = outlines.get(i).get(j).get(1);
      
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
    long currentY = (long) (terrainMinY + abs(terrainMaxY - terrainMinY) / (float)(numOfLines) * i + abs(terrainMaxY - terrainMinY) / (float)(numOfLines) / 2.0);
    
    for (int o = 0; o < outlines.size(); o++){
      for (int p = 0; p < outlines.get(o).size(); p++){
        if (outlines.get(o).get(p).get(1) == currentY){
          outlines.get(o).get(p).set(1, outlines.get(o).get(p).get(1) + 1);
        }
      }
    }
  }
}

void FindOutlineIntersections(){
  //Extend outlines with intersection points
  extOutlines = new ArrayList<ArrayList<ArrayList<Long>>>();
  for (int i = 0; i < outlines.size(); i++){
    extOutlines.add(new ArrayList<ArrayList<Long>>());
    
    for (int j = 0; j < outlines.get(i).size(); j++){
      extOutlines.get(i).add(new ArrayList<Long>());
      
      for (int k = 0; k < outlines.get(i).get(j).size(); k++){
        extOutlines.get(i).get(j).add(outlines.get(i).get(j).get(k));
      }
    }
  }

  for (int i = 0; i < numOfLines; i++)
  {
    long currentY = (long) (terrainMinY + abs(terrainMaxY - terrainMinY) / (float)(numOfLines) * i + abs(terrainMaxY - terrainMinY) / (float)(numOfLines) / 2.0);
    
    //Traverse outlines and extend outlines with intersections
    for (int o = 0; o < extOutlines.size(); o++){
      
      ArrayList<ArrayList<Long>> intersections = new ArrayList<ArrayList<Long>>();
      // X1, Y1, NX, NY
      
      //Find intersections for current outline
      for (int p = 0; p < extOutlines.get(o).size(); p++){
        long p1X = extOutlines.get(o).get(p).get(0);
        long p1Y = extOutlines.get(o).get(p).get(1);
        
        long p2X, p2Y;
        
        //First-last case
        if (p == extOutlines.get(o).size() - 1){
          p2X = extOutlines.get(o).get(0).get(0);
          p2Y = extOutlines.get(o).get(0).get(1);
        }
        else {
          p2X = extOutlines.get(o).get(p + 1).get(0);
          p2Y = extOutlines.get(o).get(p + 1).get(1);
        }
        
        //Check if lines intersect, and add intersection
        if ((currentY >= p1Y && currentY <= p2Y) || (currentY <= p1Y && currentY >= p2Y))
        {
            intersections.add(new ArrayList<Long>());

            long thisX;

            if (p1X != p2X)
            {
                thisX = (long)((currentY - p1Y) / (float)(p2Y - p1Y) * (p2X - p1X) + p1X);
            }
            else { thisX = p1X; } //Vertical slope

            // X1, Y1, X2, Y2, NX, NY
            intersections.get(intersections.size() - 1).add(p1X);
            intersections.get(intersections.size() - 1).add(p1Y);
            intersections.get(intersections.size() - 1).add(thisX);
            intersections.get(intersections.size() - 1).add(currentY);
        }
      }
      
      //Insert intersections into list
      int p = 0;
      while (p < extOutlines.get(o).size()){
        long p1X = extOutlines.get(o).get(p).get(0);
        long p1Y = extOutlines.get(o).get(p).get(1);
        
        /*
        long p2X, p2Y;
        
        //First-last case
        if (p == extOutlines.get(o).size() - 1){
          p2X = extOutlines.get(o).get(0).get(0);
          p2Y = extOutlines.get(o).get(0).get(1);
        }
        else {
          p2X = extOutlines.get(o).get(p + 1).get(0);
          p2Y = extOutlines.get(o).get(p + 1).get(1);
        }
        */
        
        //Check corresponding intersections
        for (int it = 0; it < intersections.size(); it++){
          
          //If point is between these points, insert it into list
          if ((intersections.get(it).get(0) == p1X) &&
              (intersections.get(it).get(1) == p1Y))
          {
            ArrayList<Long> appendList = new ArrayList<Long>();
            appendList.add(intersections.get(it).get(2));
            appendList.add(intersections.get(it).get(3));
            
            extOutlines.get(o).add(p + 1, appendList);
            p++;
          }
        }
        
        p++;
      }
    }
  }
}

void GeneratePaths(){
  intersectionPaths = new ArrayList<ArrayList<ArrayList<Long>>>();
  
  //Scan and find intersections
  for (int i = 0; i < numOfLines; i++)
  {
    long currentY = (long) (terrainMinY + abs(terrainMaxY - terrainMinY) / (float)(numOfLines) * i + abs(terrainMaxY - terrainMinY) / (float)(numOfLines) / 2.0);
      
    intersectionPaths.add(new ArrayList<ArrayList<Long>>());
    
    for (int o = 0; o < extOutlines.size(); o++){
      for (int p = 0; p < extOutlines.get(o).size(); p++){
        if (extOutlines.get(o).get(p).get(1) == currentY){
          //O, P, NX
          intersectionPaths.get(intersectionPaths.size() - 1).add(new ArrayList<Long>());
          intersectionPaths.get(intersectionPaths.size() - 1).get(intersectionPaths.get(intersectionPaths.size() - 1).size() - 1).add((long) o);
          intersectionPaths.get(intersectionPaths.size() - 1).get(intersectionPaths.get(intersectionPaths.size() - 1).size() - 1).add((long) p);
          intersectionPaths.get(intersectionPaths.size() - 1).get(intersectionPaths.get(intersectionPaths.size() - 1).size() - 1).add((long) extOutlines.get(o).get(p).get(0));
        }
      }
    }
  }
  
  //Sort by longitude
  for (int d = 0; d < intersectionPaths.size(); d++){
    Collections.sort(intersectionPaths.get(d), new Comparator<ArrayList<Long>>() {
      @Override
      public int compare(ArrayList<Long> o1, ArrayList<Long> o2) {
          return o1.get(2).compareTo(o2.get(2));
      }         
    });
  }
}

void GenerateGcode(){
  //Error conditions
  if (outlines.size() < 1){return;}
  if (outlines.get(0).size() <= 2){return;}
  
  FindTerrainBounds();
  ClearInterference(); //Makes sure no point in outlines is directly on infill Y (Patch for intersection check in FindOutlineIntersections)
  FindOutlineIntersections();
  GeneratePaths();
}

//negative - decreasing index
//positive - increasing index

//WARNING:
/*
OUTLINE MUST EXIST.
OUTLINE NEEDS TO HAVE AT LEAST 2 POINTS.
POINTS MUST BE IN OUTLINE
*/
long ShortestOutlinePath(int outline_index, int current_point, int target_point){
  float path_length_inc = 0;
  float path_length_dec = 0;
  
  int tmp_index;
  
  //Increasing
  tmp_index = current_point;
  
  while (tmp_index != target_point) {
    
    //End of list
    if (tmp_index == extOutlines.get(outline_index).size() - 1){
      
      path_length_inc += (float) sqrt(pow(extOutlines.get(outline_index).get(extOutlines.get(outline_index).size() - 1).get(0) - extOutlines.get(outline_index).get(0).get(0), 2) + pow(extOutlines.get(outline_index).get(extOutlines.get(outline_index).size() - 1).get(1) - extOutlines.get(outline_index).get(0).get(1), 2));
      
      tmp_index = 0;
      continue;
    }
    
    path_length_inc += (float) sqrt(pow(extOutlines.get(outline_index).get(tmp_index).get(0) - extOutlines.get(outline_index).get(tmp_index + 1).get(0), 2) + pow(extOutlines.get(outline_index).get(tmp_index).get(1) - extOutlines.get(outline_index).get(tmp_index + 1).get(1), 2));
    tmp_index++;
  }
  
  //Decreasing
  tmp_index = current_point;
  
  while (tmp_index != target_point) {
    
    //End of list
    if (tmp_index == 0){
      
      path_length_dec += (float) sqrt(pow(extOutlines.get(outline_index).get(extOutlines.get(outline_index).size() - 1).get(0) - extOutlines.get(outline_index).get(0).get(0), 2) + pow(extOutlines.get(outline_index).get(extOutlines.get(outline_index).size() - 1).get(1) - extOutlines.get(outline_index).get(0).get(1), 2));
      
      tmp_index = extOutlines.get(outline_index).size() - 1;
      continue;
    }
    
    path_length_dec += (float) sqrt(pow(extOutlines.get(outline_index).get(tmp_index).get(0) - extOutlines.get(outline_index).get(tmp_index - 1).get(0), 2) + pow(extOutlines.get(outline_index).get(tmp_index).get(1) - extOutlines.get(outline_index).get(tmp_index - 1).get(1), 2));
    tmp_index--;
  }
  
  if (path_length_inc < path_length_dec){
    return (long) ceil(path_length_inc);
  }
  else {
    return (long) -ceil(path_length_dec);
  }
}

int OutlineTraverseInc(int outline_index, int current_point, int amount){
  for (int i = 0; i < amount; i++){
    current_point++;
    
    if (current_point >= extOutlines.get(outline_index).size()){current_point = 0;}
  }
  
  return current_point;
}

int OutlineTraverseDec(int outline_index, int current_point, int amount){
  for (int i = 0; i < amount; i++){
    current_point--;
    
    if (current_point < 0){current_point = extOutlines.get(outline_index).size() - 1;}
  }
  
  return current_point;
}

ArrayList<Long> AlgorithmNextPoint(){
  long NextX = 0;
  long NextY = 0;
  
  if (algorithmTarget == "FORWARD"){
    if (algorithmMode == "INFILL"){
      MotorMainOn();
      
      //End of infill
      if (algorithmInfillDirection == 1 && algorithmInfillPoint == intersectionPaths.get(algorithmInfillIndex).size() - 1){
        algorithmMode = "SEEK";
        
        algorithmCurrentOutline = int(intersectionPaths.get(algorithmInfillIndex).get(algorithmInfillPoint).get(0));
        algorithmCurrentPoint = int(intersectionPaths.get(algorithmInfillIndex).get(algorithmInfillPoint).get(1));
        
        algorithmInfillIndex++;
        
        if (algorithmInfillIndex >= intersectionPaths.size()){
          algorithmTarget = "BASE"; //<>//
          algorithmMode = "SEEK";
          
          algorithmCurrentOutline = int(intersectionPaths.get(algorithmInfillIndex - 1).get(algorithmInfillPoint).get(0));
          algorithmCurrentPoint = int(intersectionPaths.get(algorithmInfillIndex - 1).get(algorithmInfillPoint).get(1));
        }
      }
      else if (algorithmInfillDirection == 0 && algorithmInfillPoint == 0){
        algorithmMode = "SEEK";
        
        algorithmCurrentOutline = int(intersectionPaths.get(algorithmInfillIndex).get(algorithmInfillPoint).get(0));
        algorithmCurrentPoint = int(intersectionPaths.get(algorithmInfillIndex).get(algorithmInfillPoint).get(1));
        
        algorithmInfillIndex++;
        
        if (algorithmInfillIndex >= intersectionPaths.size()){
          algorithmTarget = "BASE"; //<>//
          algorithmMode = "SEEK";
          
          algorithmCurrentOutline = int(intersectionPaths.get(algorithmInfillIndex - 1).get(algorithmInfillPoint).get(0));
          algorithmCurrentPoint = int(intersectionPaths.get(algorithmInfillIndex - 1).get(algorithmInfillPoint).get(1));
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
          
          algorithmCurrentOutline = int(intersectionPaths.get(algorithmInfillIndex).get(algorithmInfillPoint).get(0));
          algorithmCurrentPoint = int(intersectionPaths.get(algorithmInfillIndex).get(algorithmInfillPoint).get(1));
        }
        //Traverse outline
        else {
          //Calculate distances
          long shortestPath;
          int nextInfillPoint;
          
          //Find next infill point
          if (algorithmInfillDirection == 1){
            nextInfillPoint = algorithmInfillPoint + 1;
          }
          else {
            nextInfillPoint = algorithmInfillPoint - 1;
          }
          
          //Find shortest path to the next infill point
          shortestPath = ShortestOutlinePath(algorithmCurrentOutline, algorithmCurrentPoint, int(intersectionPaths.get(algorithmInfillIndex).get(nextInfillPoint).get(1)));
          
          //Traverse in shortest direction
          if (shortestPath > 0){
            algorithmCurrentPoint = OutlineTraverseInc(algorithmCurrentOutline, algorithmCurrentPoint, 1);
          }
          else {
            algorithmCurrentPoint = OutlineTraverseDec(algorithmCurrentOutline, algorithmCurrentPoint, 1);
          }
          
          //Reached continuation of infill
          if (algorithmCurrentPoint == int(intersectionPaths.get(algorithmInfillIndex).get(nextInfillPoint).get(1))){
            if (algorithmInfillDirection == 1){
              algorithmInfillPoint++;
            }
            else {
              algorithmInfillPoint--;
            }
            
            algorithmCurrentOutline = int(intersectionPaths.get(algorithmInfillIndex).get(algorithmInfillPoint).get(0));
            algorithmCurrentPoint = int(intersectionPaths.get(algorithmInfillIndex).get(algorithmInfillPoint).get(1));
          }
        }
      }
      
      NextX = extOutlines.get(algorithmCurrentOutline).get(algorithmCurrentPoint).get(0);
      NextY = extOutlines.get(algorithmCurrentOutline).get(algorithmCurrentPoint).get(1);
    }
    if (algorithmTarget == "FORWARD" && algorithmMode == "SEEK"){
      MotorMainOff();
      
      //Calculate distances
      long distInfillA = ShortestOutlinePath(algorithmCurrentOutline, algorithmCurrentPoint, int(intersectionPaths.get(algorithmInfillIndex).get(0).get(1)));
      long distInfillB = ShortestOutlinePath(algorithmCurrentOutline, algorithmCurrentPoint, int(intersectionPaths.get(algorithmInfillIndex).get(intersectionPaths.get(algorithmInfillIndex).size() - 1).get(1)));
      
      long shortestPath;
      
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
      if (algorithmCurrentPoint == int(intersectionPaths.get(algorithmInfillIndex).get(0).get(1))){
        algorithmMode = "INFILL";
        algorithmInfillPoint = 0;
        algorithmInfillDirection = 1;
      }
      else if (algorithmCurrentPoint == int(intersectionPaths.get(algorithmInfillIndex).get(intersectionPaths.get(algorithmInfillIndex).size() - 1).get(1))){
        algorithmMode = "INFILL";
        algorithmInfillPoint = intersectionPaths.get(algorithmInfillIndex).size() - 1;
        algorithmInfillDirection = 0;
      }
      
      NextX = extOutlines.get(algorithmCurrentOutline).get(algorithmCurrentPoint).get(0);
      NextY = extOutlines.get(algorithmCurrentOutline).get(algorithmCurrentPoint).get(1);
    }
    if (algorithmMode == "OUTLINE"){
      MotorMainOn();
      
      algorithmCurrentPoint++;
      
      if (algorithmCurrentPoint >= extOutlines.get(0).size()){
        algorithmCurrentPoint = 0;
        
        //Start seeking first infill start
        algorithmMode = "SEEK";
        algorithmInfillIndex = 0;
      }
      
      NextX = extOutlines.get(algorithmCurrentOutline).get(algorithmCurrentPoint).get(0);
      NextY = extOutlines.get(algorithmCurrentOutline).get(algorithmCurrentPoint).get(1);
    }
  }
  if (algorithmTarget == "BASE"){
    MotorMainOff();
    
    if (algorithmMode == "INFILL"){
      //Jump outlines
      if ((algorithmInfillDirection == 0 && algorithmInfillPoint % 2 == 0 && algorithmCurrentPoint == intersectionPaths.get(algorithmInfillIndex).get(algorithmInfillPoint).get(1)) || (algorithmInfillDirection == 1 && algorithmInfillPoint % 2 == 1 && algorithmCurrentPoint == intersectionPaths.get(algorithmInfillIndex).get(algorithmInfillPoint).get(1))){ //<>//
        if (algorithmInfillDirection == 1){
          algorithmInfillPoint--;
        }
        else {
          algorithmInfillPoint++;
        }
        
        algorithmCurrentOutline = int(intersectionPaths.get(algorithmInfillIndex).get(algorithmInfillPoint).get(0));
        algorithmCurrentPoint = int(intersectionPaths.get(algorithmInfillIndex).get(algorithmInfillPoint).get(1));
      }
      //Traverse outline
      else {
        //Jump outline skip
        if ((algorithmInfillDirection == 0 && algorithmInfillPoint % 2 == 1 && algorithmCurrentPoint == intersectionPaths.get(algorithmInfillIndex).get(algorithmInfillPoint).get(1)) || (algorithmInfillDirection == 1 && algorithmInfillPoint % 2 == 0 && algorithmCurrentPoint == intersectionPaths.get(algorithmInfillIndex).get(algorithmInfillPoint).get(1))){
          if (algorithmInfillDirection == 1){
            algorithmInfillPoint--;
          }
          else {
            algorithmInfillPoint++;
          }
        }
        
        //Reached continuation of infill
        if (algorithmCurrentPoint == int(intersectionPaths.get(algorithmInfillIndex).get(algorithmInfillPoint).get(1))){
          if (algorithmInfillDirection == 1){
            algorithmInfillPoint--;
          }
          else {
            algorithmInfillPoint++;
          }
          
          algorithmCurrentOutline = int(intersectionPaths.get(algorithmInfillIndex).get(algorithmInfillPoint).get(0));
          algorithmCurrentPoint = int(intersectionPaths.get(algorithmInfillIndex).get(algorithmInfillPoint).get(1));
        }
        
        //Calculate distances
        long shortestPath;
        
        //Find shortest path to the next infill point
        shortestPath = ShortestOutlinePath(algorithmCurrentOutline, algorithmCurrentPoint, int(intersectionPaths.get(algorithmInfillIndex).get(algorithmInfillPoint).get(1)));
        
        //Traverse in shortest direction
        if (shortestPath > 0){
          algorithmCurrentPoint = OutlineTraverseInc(algorithmCurrentOutline, algorithmCurrentPoint, 1);
        }
        else {
          algorithmCurrentPoint = OutlineTraverseDec(algorithmCurrentOutline, algorithmCurrentPoint, 1);
        }
      }
      
      //End of infill
      if (algorithmInfillDirection == 0 && algorithmInfillPoint == intersectionPaths.get(algorithmInfillIndex).size() - 1){
        if (!algorithmAbortFull){
          algorithmTarget = "FORWARD";
        }
        
        algorithmMode = "SEEK";
        
        algorithmCurrentOutline = int(intersectionPaths.get(algorithmInfillIndex).get(algorithmInfillPoint).get(0));
        algorithmCurrentPoint = int(intersectionPaths.get(algorithmInfillIndex).get(algorithmInfillPoint).get(1));
        
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
        
        algorithmCurrentOutline = int(intersectionPaths.get(algorithmInfillIndex).get(algorithmInfillPoint).get(0));
        algorithmCurrentPoint = int(intersectionPaths.get(algorithmInfillIndex).get(algorithmInfillPoint).get(1));
        
        algorithmInfillIndex++;
        
        if (algorithmInfillIndex >= intersectionPaths.size()){
          algorithmAbortFull = true;
        }
      }
      
      NextX = extOutlines.get(algorithmCurrentOutline).get(algorithmCurrentPoint).get(0);
      NextY = extOutlines.get(algorithmCurrentOutline).get(algorithmCurrentPoint).get(1);
    }
    else if (algorithmMode == "SEEK"){
      //If reached charging station exit point
      if (algorithmCurrentPoint == 0){
        MainChargingStart();
      }
      
      //Find shortest path to the charging station exit point
      long shortestPath = ShortestOutlinePath(algorithmCurrentOutline, algorithmCurrentPoint, 0);
      
      //Traverse in shortest direction
      if (shortestPath > 0){
        algorithmCurrentPoint = OutlineTraverseInc(algorithmCurrentOutline, algorithmCurrentPoint, 1);
      }
      else {
        algorithmCurrentPoint = OutlineTraverseDec(algorithmCurrentOutline, algorithmCurrentPoint, 1);
      }
      
      NextX = extOutlines.get(algorithmCurrentOutline).get(algorithmCurrentPoint).get(0);
      NextY = extOutlines.get(algorithmCurrentOutline).get(algorithmCurrentPoint).get(1);
    }
  }
  
  prevPointLon = targetPointLon;
  prevPointLat = targetPointLat;
  
  ArrayList<Long> returnList = new ArrayList<Long>();
  returnList.add(NextX); returnList.add(NextY);
  return returnList;
}

void AlgorithmAbort(boolean full_abort){
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
      if (algorithmInfillDirection == 0 && algorithmInfillPoint == intersectionPaths.get(algorithmInfillIndex).size() - 1){
        if (!algorithmAbortFull){
          algorithmTarget = "FORWARD";
        }
        
        algorithmMode = "SEEK";
        
        algorithmCurrentOutline = int(intersectionPaths.get(algorithmInfillIndex).get(algorithmInfillPoint).get(0));
        algorithmCurrentPoint = int(intersectionPaths.get(algorithmInfillIndex).get(algorithmInfillPoint).get(1));
        
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
        
        algorithmCurrentOutline = int(intersectionPaths.get(algorithmInfillIndex).get(algorithmInfillPoint).get(0));
        algorithmCurrentPoint = int(intersectionPaths.get(algorithmInfillIndex).get(algorithmInfillPoint).get(1));
        
        algorithmInfillIndex++;
        
        if (algorithmInfillIndex >= intersectionPaths.size()){
          algorithmAbortFull = true;
        }
      }
    }
    
    //TODO: Mower reverse for a bit from obstacle
    ArrayList<Long> targetPoint = AlgorithmNextPoint();
    targetPointLon = targetPoint.get(0);
    targetPointLat = targetPoint.get(1);
    
    MotionRotateToTarget();
  }
}
