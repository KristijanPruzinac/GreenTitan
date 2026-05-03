#include "algorithm.h"

// Position tracking
static double current_lon = 0;
static double current_lat = 0;
static double current_yaw = 0;

static constexpr double GPS_SCALE = 1e7;
static inline long long gpsToLL(double v)   { return (long long)(v * GPS_SCALE); }
static inline double    llToGPS(long long v) { return (double)v / GPS_SCALE; }

//Prev
static long long prevPointLon = 0;
static long long prevPointLat = 0;

static double prevPointLonActual = 0;
static double prevPointLatActual = 0;

//Target
static long long targetPointLon = 0;
static long long targetPointLat = 0;

// -------------------------------------------------------- HELPERS ---------------------------------------------------------------

static double GetLon() {
    return current_lon;
}

static double GetLat() {
    return current_lat;
}

static void MotorMainOn() {
    motor_data_t msg = { MOTOR_MAIN_ON, 0.0f, 0.0f };
    dds_result_t result = DDS_PUBLISH("/motor", msg);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("[ALGO] Failed to send motor main on: %d\r\n", result);
    }
}

static void MotorMainOff() {
    motor_data_t msg = { MOTOR_MAIN_OFF, 0.0f, 0.0f };
    dds_result_t result = DDS_PUBLISH("/motor", msg);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("[ALGO] Failed to send motor main off: %d\r\n", result);
    }
}

static void MotorDriveAngle(float angle, bool forward, float speedFactor) {
    // Convert angle and direction to motor velocities
    // This is a simple implementation - adjust based on your needs
    float linear_vel = forward ? 0.3f * speedFactor : -0.3f * speedFactor;
    float angular_vel = angle * 0.5f;  // Scale factor for turning
    
    motor_data_t msg = { MOTOR_MOVE, linear_vel, angular_vel };
    dds_result_t result = DDS_PUBLISH("/motor", msg);
    if (result != DDS_SUCCESS) {
        SerialDebug.printf("[ALGO] Failed to send motor drive command: %d\r\n", result);
    }
}

void AlgorithmMotionSetTargetPoint(double tLon, double tLat) {
  if (prevPointLonActual == 0 || prevPointLatActual == 0) {
    prevPointLonActual = current_lon;
    prevPointLatActual = current_lat;

    SerialDebug.printf("[ALGO] MotionSetTargetPoint: No previous point, using current position\r\n");
  }

  motion_command_t cmd = { MOVING, llToGPS(prevPointLonActual), llToGPS(prevPointLatActual),
                            llToGPS(tLon), llToGPS(tLat) };
  dds_result_t result = DDS_PUBLISH("/motion/command", cmd);
  if (result != DDS_SUCCESS) {
      SerialDebug.printf("[ALGO] Failed to send motion command: %d\r\n", result);
  }
}

static void MainChargingStart(){
  controller_signal_t cmd = { SIGNAL_MAIN_CHARGING_START };
  dds_result_t result = DDS_PUBLISH("/controller/signal", cmd);
  if (result != DDS_SUCCESS) {
    SerialDebug.printf("[ALGO] Failed to send controller command: %d\r\n", result);
  }
}

//Globals
static int numOfLines = 1;
//Algorithm
static String algorithmTarget = "FORWARD"; // FORWARD, BASE
static String algorithmMode = "OUTLINE";
// FORWARD: OUTLINE, SEEK, INFILL
// BASE: INFILL, SEEK

static bool algorithmAbortFull = false;
/*
Explanation:
OUTLINE - Mows outer outline
SEEK - Searches for point aint outer outline where infill line starts
INFILL - Mows infill line and inside outlines
*/

//Only in INFILL mode
static int algorithmInfillIndex;
static int algorithmInfillPoint;
static int algorithmInfillDirection;

static int algorithmCurrentOutline = 0;
static int algorithmCurrentPoint = 0;

//Outlines - First point is the charging station exit point
static std::vector<std::vector<std::vector<long long>>> outlines;
static std::vector<std::vector<std::vector<long long>>> extOutlines; //Outlines with intersection points inserted
static std::vector<std::vector<std::vector<long long>>> intersectionPaths; //Collections of points where infill intersects terrain and outer outline

static long long terrainMinX;
static long long terrainMaxX;
static long long terrainMinY;
static long long terrainMaxY;

static void ClearOutlines(){
  extOutlines.clear();
  intersectionPaths.clear();
}

static void FindTerrainBounds(){
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

static void ClearInterference(){
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

static bool FindOutlineIntersections(){
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

static bool GeneratePaths(){
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

static bool GenerateGcode(){
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
static int ShortestOutlinePath(int outline_index, int current_point, int target_point){
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

static int OutlineTraverseInc(int outline_index, int current_point, int amount){
  for (int i = 0; i < amount; i++){
    current_point++;
    
    if (current_point >= extOutlines.at(outline_index).size()){current_point = 0;}
  }
  
  return current_point;
}

static int OutlineTraverseDec(int outline_index, int current_point, int amount){
  for (int i = 0; i < amount; i++){
    current_point--;
    
    if (current_point < 0){current_point = extOutlines.at(outline_index).size() - 1;}
  }
  
  return current_point;
}

std::vector<double> AlgorithmNextPoint(){
  long long NextX = 0;
  long long NextY = 0;

  prevPointLonActual = prevPointLon;
  prevPointLatActual = prevPointLat;
  
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
  
  std::vector<double> returnList;
  returnList.push_back(llToGPS(NextX));
  returnList.push_back(llToGPS(NextY));
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
    
    std::vector<double> targetPoint = AlgorithmNextPoint();
    AlgorithmMotionSetTargetPoint(targetPoint.at(0), targetPoint.at(1));
  }
}

static void AlgorithmCaptureStart(){
  outlines.clear();
}

static void AlgorithmCaptureNewOutline(){
  outlines.push_back( std::vector<std::vector<long long>>());
}
static void AlgorithmCaptureBasePoint(){
  BASE_LON = GetLon();
  BASE_LAT = GetLat();
}
static void AlgorithmCaptureBaseExitPoint(){
  BASE_EXIT_LON = GetLon();
  BASE_EXIT_LAT = GetLat();
}
static void AlgorithmCaptureNewPoint(){
  outlines.back().push_back( std::vector<long long>());

  outlines.back().push_back({ gpsToLL(GetLon()), gpsToLL(GetLat()) });
}
static void AlgorithmCaptureSetNewPoint(double lon, double lat){
  outlines.back().push_back( std::vector<long long>());

  outlines.back().back().push_back(gpsToLL(lon));
  outlines.back().back().push_back(gpsToLL(lat));
}
static bool AlgorithmCaptureRemoveOutline(){
  //Index out of bounds, failed
  if (outlines.size() == 0){return false;}

  outlines.pop_back();
  
  return true;
}

static bool AlgorithmCaptureRemovePoint(){
  //Index out of bounds, stop algorithm
  if (outlines.back().size() == 0){return false;} 

  outlines.back().pop_back();
  
  return true;
}

static bool AlgorithmCaptureEnd(){
  return GenerateGcode();
}

//TODO: Rewrite to work with doubles
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

//TODO: Rewrite to work with doubles
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

              AlgorithmCaptureSetNewPoint(variableValue1.toDouble(), variableValue2.toDouble());
          }
        }
    }

    return AlgorithmCaptureEnd();
}

static void algorithm_command_callback(dds_callback_context_t* context) {
    algorithm_command_t* cmd = (algorithm_command_t*)context->message_data.data;
    
    switch (cmd->command) {
        case CMD_ALGO_START_MOWING:
            // Start full mowing algorithm
            if (outlines.size() > 0 && extOutlines.size() > 0) {
                algorithmAbortFull = false;
                algorithmTarget = "FORWARD";
                algorithmMode = "OUTLINE";
                algorithmCurrentOutline = 0;
                algorithmCurrentPoint = 0;
                algorithmInfillIndex = 0;
                
                // Start at base exit point if defined
                if (BASE_EXIT_LON != 0 || BASE_EXIT_LAT != 0) {
                    targetPointLon = gpsToLL(BASE_EXIT_LON);
                    targetPointLat = gpsToLL(BASE_EXIT_LAT);
                } else {
                    targetPointLon = extOutlines.at(0).at(0).at(0);
                    targetPointLat = extOutlines.at(0).at(0).at(1);
                }
                
                MotorMainOn();
                AlgorithmMotionSetTargetPoint(llToGPS(targetPointLon), llToGPS(targetPointLat));
                SerialDebug.printf("[ALGO] Started mowing algorithm\r\n");
            } else {
                SerialDebug.printf("[ALGO] Cannot start mowing - no outlines defined\r\n");
            }
            break;
            
        case CMD_ALGO_PAUSE:
            // Pause algorithm
            MotorMainOff();
            algorithmAbortFull = true; // Use abort flag to pause
            SerialDebug.printf("[ALGO] Algorithm paused\r\n");
            break;
            
        case CMD_ALGO_RESUME:
            // Resume algorithm
            if (algorithmAbortFull && algorithmTarget != "BASE") {
                algorithmAbortFull = false;
                MotorMainOn();
                SerialDebug.printf("[ALGO] Algorithm resumed\r\n");
            } else {
                SerialDebug.printf("[ALGO] Cannot resume - algorithm not paused or returning to base\r\n");
            }
            break;
            
        case CMD_ALGO_ABORT:
            // Abort current operation (full or line only)
            AlgorithmAbort(cmd->full_abort); // Assuming cmd has full_abort field
            SerialDebug.printf("[ALGO] Aborted operation (full: %d)\r\n", cmd->full_abort);
            break;
            
        case CMD_ALGO_START_RECORDING:
            // Start recording outline
            AlgorithmCaptureStart();
            AlgorithmCaptureNewOutline(); // Start first outline
            SerialDebug.printf("[ALGO] Started recording outline\r\n");
            break;
            
        case CMD_ALGO_CAPTURE_BASE:
            // Capture charging station location
            AlgorithmCaptureBasePoint();
            SerialDebug.printf("[ALGO] Captured base position: (%.6f, %.6f)\r\n", GetLon(), GetLat());
            break;
            
        case CMD_ALGO_CAPTURE_BASE_EXIT:
            // Capture base exit point
            AlgorithmCaptureBaseExitPoint();
            SerialDebug.printf("[ALGO] Captured base exit point: (%.6f, %.6f)\r\n", GetLon(), GetLat());
            break;
            
        case CMD_ALGO_NEW_OUTLINE:
            // Start new outline
            AlgorithmCaptureNewOutline();
            SerialDebug.printf("[ALGO] Started new outline (total: %d)\r\n", (int)outlines.size());
            break;
            
        case CMD_ALGO_CAPTURE_POINT:
            // Capture current position as point
            AlgorithmCaptureNewPoint();
            SerialDebug.printf("[ALGO] Captured point %d in outline %d: (%.6f, %.6f)\r\n", 
                             (int)outlines.back().size(), (int)outlines.size() - 1, GetLon(), GetLat());
            break;
            
        case CMD_ALGO_REMOVE_OUTLINE:
            // Remove last outline
            if (AlgorithmCaptureRemoveOutline()) {
                SerialDebug.printf("[ALGO] Removed last outline (remaining: %d)\r\n", (int)outlines.size());
            } else {
                SerialDebug.printf("[ALGO] Failed to remove outline - no outlines exist\r\n");
            }
            break;
            
        case CMD_ALGO_REMOVE_POINT:
            // Remove last point
            if (AlgorithmCaptureRemovePoint()) {
                SerialDebug.printf("[ALGO] Removed last point from outline %d (remaining: %d)\r\n", 
                                 (int)outlines.size() - 1, (int)outlines.back().size());
            } else {
                SerialDebug.printf("[ALGO] Failed to remove point - outline is empty\r\n");
            }
            break;
            
        case CMD_ALGO_END_RECORDING:
            // Finish recording and generate paths
            if (AlgorithmCaptureEnd()) {
                SerialDebug.printf("[ALGO] Recording finished - paths generated successfully\r\n");
                SerialDebug.printf("[ALGO] Terrain bounds: X[%.2f,%.2f] Y[%.2f,%.2f]\r\n",
                                 llToGPS(terrainMinX), llToGPS(terrainMaxX),
                                 llToGPS(terrainMinY), llToGPS(terrainMaxY));
                SerialDebug.printf("[ALGO] Number of infill lines: %d\r\n", numOfLines);
            } else {
                SerialDebug.printf("[ALGO] Failed to generate paths - invalid outline data\r\n");
            }
            break;
            
        case CMD_ALGO_CLEAR_ALL:
            // Clear all outlines
            outlines.clear();
            extOutlines.clear();
            intersectionPaths.clear();
            algorithmAbortFull = false;
            algorithmTarget = "FORWARD";
            algorithmMode = "OUTLINE";
            SerialDebug.printf("[ALGO] Cleared all outlines and paths\r\n");
            break;
            /* TODO: Implement
        case CMD_ALGO_GET_PATH_STRING:
            // Request path string (for saving)
            {
                String pathStr = AlgorithmGetPathString();
                // Send back via DDS or store in response buffer
                // Assuming there's a response topic
                path_string_response_t response;
                strncpy(response.path_string, pathStr.c_str(), sizeof(response.path_string) - 1);
                response.path_string[sizeof(response.path_string) - 1] = '\0';
                DDS_PUBLISH("/algorithm/path_string_response", response);
                SerialDebug.printf("[ALGO] Path string generated (length: %d)\r\n", pathStr.length());
            }
            break;
             
        case CMD_ALGO_LOAD_PATH_STRING:
            // Load path from string
            {
                String pathStr(cmd->path_string); // Assuming cmd has path_string field
                if (AlgorithmPopulatePathFromString(pathStr)) {
                    SerialDebug.printf("[ALGO] Path loaded successfully\r\n");
                } else {
                    SerialDebug.printf("[ALGO] Failed to load path - invalid data\r\n");
                }
            }
            break;
            */
        default:
            SerialDebug.printf("[ALGO] Unknown command: %d\r\n", cmd->command);
            break;
    }
}

static void fused_pose_callback(dds_callback_context_t* context) {
    fused_pose_data_t* data = (fused_pose_data_t*)context->message_data.data;
    
    current_lon = gpsToLL(data->x);
    current_lat = gpsToLL(data->y);
    current_yaw = data->yaw;
}

static dds_thread_context_t thread_context;
static void thread_timer_callback(void* arg) { xTaskNotify(thread_context.task, THREAD_NOTIFY_BIT, eSetBits); }
void algorithm_task(void* parameter) {
    thread_context.task = xTaskGetCurrentTaskHandle();
    thread_context.queue = xQueueCreate(5, sizeof(dds_callback_context_t));
    thread_context.sync_mutex = xSemaphoreCreateMutex();
    
    esp_timer_create_args_t timer_args = {
        .callback = &thread_timer_callback,
        .arg = NULL,
    };
    esp_timer_create(&timer_args, &(thread_context.timer));
    esp_timer_start_periodic(thread_context.timer, 200 * 1000); // 200 ms

    // ------- THREAD SETUP CODE START -------

    dds_result_t result;
    result = DDS_SUBSCRIBE("/algorithm/command", algorithm_command_callback, &thread_context);
    if (result != DDS_SUCCESS) {
        Serial.printf("Topic subscribe failed: %s\n", DDS_RESULT_TO_STRING(result));
    }

    result = DDS_SUBSCRIBE("/fused_pose", fused_pose_callback, &thread_context);
    if (result != DDS_SUCCESS) {
        Serial.printf("Fused pose subscribe failed: %s\n", DDS_RESULT_TO_STRING(result));
    }

    // ------- THREAD SETUP CODE END -------

    vTaskDelay(500);
    
    while(1) {
        // Wait for any notification (message or timer)
        uint32_t notification_value;
        xTaskNotifyWait(0x00, 0xFF, &notification_value, portMAX_DELAY);
        
        if (notification_value & DDS_NOTIFY_BIT) { // DDS message notification
            DDS_TAKE_MUTEX(&thread_context);
            DDS_PROCESS_THREAD_MESSAGES(&thread_context);
            DDS_GIVE_MUTEX(&thread_context);
        }
        if (notification_value & THREAD_NOTIFY_BIT) { // Timer tick notification
            DDS_TAKE_MUTEX(&thread_context);

            // ------- THREAD LOOP CODE START -------

            // ------- THREAD LOOP CODE END -------

            DDS_PROCESS_THREAD_MESSAGES(&thread_context);
            DDS_GIVE_MUTEX(&thread_context);
        }
    }
}