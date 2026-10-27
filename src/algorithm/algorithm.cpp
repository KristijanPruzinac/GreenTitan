#include "algorithm.h"

// Position tracking
static long long current_lon = 0;
static long long current_lat = 0;
static long long current_yaw = 0;

//Prev
static long long prevPointLon = 0;
static long long prevPointLat = 0;

//Target
static long long targetPointLon = 0;
static long long targetPointLat = 0;

bool algorithmDone = false;

static long long GetLon(){
  return current_lon;
}

static long long GetLat(){
  return current_lat;
}

// Blade state for the point being handed out. Points go to the motion queue ahead of the
// mower, so the blade switches when the mower starts that leg (motion task), not here.
static uint8_t nextTool = NAV_TOOL_KEEP;
// Outline point handed out last, to tell legs that run along an outline (see AlgorithmNextPoint)
static long long lastOutline = -1;
static long long lastPoint = -1;

void AlgorithmMotionSetTargetPoint(double target_x_m, double target_y_m, uint8_t tool, int8_t edge) {
  motion_command_t cmd = {
    MOVING,
    (double)prevPointLon / 100.0,
    (double)prevPointLat / 100.0,
    target_x_m,
    target_y_m,
    tool,
    edge
  };
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
std::vector<std::vector<std::vector<long long>>> planOutlines; //Outlines moved in by MOWER_EDGE_INSET: what the paths are planned on

// Polygon offset by d cm, positive = to the left of the edges (inside for a counter-clockwise
// polygon). Mitred corners; a vertex whose neighbouring offset edge would flip direction (the
// edge collapses) is dropped and the offset redone. Fine for the gentle concave corners of a
// lawn; a narrow neck thinner than 2 d may still fold.
static std::vector<std::vector<long long>> OffsetPolygon(std::vector<std::vector<long long>> poly, double d){
  for (int attempt = 0; attempt < 64 && poly.size() >= 3; attempt++){
    size_t n = poly.size();
    std::vector<std::vector<long long>> out(n, std::vector<long long>(2));
    for (size_t i = 0; i < n; i++){
      const std::vector<long long>& a = poly[(i + n - 1) % n];
      const std::vector<long long>& b = poly[i];
      const std::vector<long long>& c = poly[(i + 1) % n];
      double e1x = b[0] - a[0], e1y = b[1] - a[1], l1 = hypot(e1x, e1y);
      double e2x = c[0] - b[0], e2y = c[1] - b[1], l2 = hypot(e2x, e2y);
      if (l1 < 1e-9 || l2 < 1e-9){ out[i] = b; continue; }
      double n1x = -e1y / l1, n1y = e1x / l1, n2x = -e2y / l2, n2y = e2x / l2;   // left normals
      double den = 1.0 + n1x * n2x + n1y * n2y;
      double mx = n1x + n2x, my = n1y + n2y;
      if (den < 0.1){ mx = n2x; my = n2y; den = 1.0; }   // near reversal: no mitre spike
      out[i][0] = (long long)llround(b[0] + d * mx / den);
      out[i][1] = (long long)llround(b[1] + d * my / den);
    }
    int collapsed = -1;
    for (size_t i = 0; i < n; i++){
      size_t j = (i + 1) % n;
      double ox = poly[j][0] - poly[i][0], oy = poly[j][1] - poly[i][1];
      double nx = out[j][0] - out[i][0], ny = out[j][1] - out[i][1];
      if (ox * nx + oy * ny <= 0){ collapsed = (int)j; break; }
    }
    if (collapsed < 0) return out;
    poly.erase(poly.begin() + collapsed);
  }
  return std::vector<std::vector<long long>>();
}

static double SignedArea(const std::vector<std::vector<long long>>& poly){
  double a = 0;
  for (size_t i = 0; i < poly.size(); i++){
    const std::vector<long long>& p = poly[i];
    const std::vector<long long>& q = poly[(i + 1) % poly.size()];
    a += (double)p[0] * q[1] - (double)q[0] * p[1];
  }
  return a / 2.0;
}

// The outlines are where the mower must not go (a wall, a bed). The planned (axle) path keeps
// MOWER_EDGE_INSET from them: the outer outline moves in, the inner ones (obstacles) move out.
static bool BuildPlanOutlines(){
  planOutlines.clear();
  double d = MOWER_EDGE_INSET * 100.0;
  for (size_t o = 0; o < outlines.size(); o++){
    double inward = SignedArea(outlines[o]) > 0 ? d : -d;     // left = inside when counter-clockwise
    std::vector<std::vector<long long>> off = OffsetPolygon(outlines[o], o == 0 ? inward : -inward);
    if (off.size() < 3){
      if (o == 0){
        SerialDebug.printf("[ALGO] Outline too small for the %.2f m edge clearance\r\n", MOWER_EDGE_INSET);
        return false;
      }
      SerialDebug.printf("[ALGO] Inner outline %u vanished in the offset, skipped\r\n", (unsigned)o);
      continue;
    }
    planOutlines.push_back(off);
  }
  return true;
}
std::vector<std::vector<std::vector<long long>>> intersectionPaths; //Collections of points where infill intersects terrain and outer outline

long long terrainMinX;
long long terrainMaxX;
long long terrainMinY;
long long terrainMaxY;

void ClearOutlines(){
  planOutlines.clear();
  extOutlines.clear();
  intersectionPaths.clear();
}

void FindTerrainBounds(){
  //Find terrain bounds
  terrainMinX = planOutlines.at(0).at(0).at(0);
  terrainMaxX = planOutlines.at(0).at(0).at(0);
  terrainMinY = planOutlines.at(0).at(0).at(1);
  terrainMaxY = planOutlines.at(0).at(0).at(1);
  
  for (int i = 0; i < planOutlines.size(); i++){
    for (int j = 0; j < planOutlines.at(i).size(); j++){
      long long xVal = planOutlines.at(i).at(j).at(0);
      long long yVal = planOutlines.at(i).at(j).at(1);
      
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
    
    for (int o = 0; o < planOutlines.size(); o++){
      for (int p = 0; p < planOutlines.at(o).size(); p++){
        if (planOutlines.at(o).at(p).at(1) == currentY){
          planOutlines.at(o).at(p).at(1) = planOutlines.at(o).at(p).at(1) + 1;
        }
      }
    }
  }
}

bool FindOutlineIntersections(){
  //Extend outlines with intersection points
  for (int i = 0; i < planOutlines.size(); i++){
    extOutlines.push_back( std::vector<std::vector<long long>>());

    for (int j = 0; j < planOutlines.at(i).size(); j++){
      extOutlines.back().push_back( std::vector<long long>());
      
      for (int k = 0; k < planOutlines.at(i).at(j).size(); k++){
        extOutlines.back().back().push_back(planOutlines.at(i).at(j).at(k));
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

  // The infill walks each scan line in enter/leave pairs; a line without points or with an
  // odd count (it grazed a vertex) would index past its end, so it is skipped.
  for (int d = (int)intersectionPaths.size() - 1; d >= 0; d--){
    size_t n = intersectionPaths.at(d).size();
    if (n < 2 || n % 2 != 0){
      SerialDebug.printf("[ALGO] Scan line %d skipped (%u crossings)\r\n", d, (unsigned)n);
      intersectionPaths.erase(intersectionPaths.begin() + d);
    }
  }

  // Skip-row order: going straight to the neighbouring row would need a U-turn tighter than the
  // tool can curve (radius = half the row spacing). Rows k apart are at least two minimum radii
  // apart, so each pass goes up every k-th row. Passes start at rows 0, k-1, ..., 1, so the jump
  // from the top of one pass back to the start of the next is long too (k = 3: 0,3,6,2,5,1,4).
  double spacing_cm = abs(terrainMaxY - terrainMinY) / (double)numOfLines;
  int k = spacing_cm > 0 ? (int)ceil(2.0 * MOTION_MIN_CORNER_RADIUS * 100.0 / spacing_cm) : 1;
  if (k > 1 && (int)intersectionPaths.size() > 1){
    std::vector<std::vector<std::vector<long long>>> ordered;
    int n = (int)intersectionPaths.size();
    for (int pass = 0; pass < k; pass++){
      int first = pass == 0 ? 0 : k - pass;
      for (int row = first; row < n; row += k) ordered.push_back(intersectionPaths.at(row));
    }
    intersectionPaths.swap(ordered);
  }

  return true;
}

bool GenerateGcode(){
  //Clean up
  ClearOutlines();
  //Error conditions
  if (outlines.size() < 1){return false;}
  if (outlines.at(0).size() <= 2){return false;}

  // Validate all outlines have at least 3 points
  for (size_t i = 0; i < outlines.size(); i++) {
    if (outlines[i].size() < 3) {
      SerialDebug.println("[ALGO] Error: In GenerateGCode, outline with less than 3 points detected");
      return false;
    }
  }
  
  if (!BuildPlanOutlines()){return false;}
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

std::vector<double> AlgorithmNextPoint(){
  if (algorithmDone) {
    return std::vector<double>();  // signals end-of-mow to controller
  }

  long long NextX = 0;
  long long NextY = 0;
  
  if (algorithmTarget == "FORWARD"){
    if (algorithmMode == "INFILL"){
      nextTool = NAV_TOOL_ON;
      
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
      nextTool = NAV_TOOL_OFF;
      
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
      nextTool = NAV_TOOL_ON;
      
      algorithmCurrentPoint++;
      
      if (algorithmCurrentPoint >= extOutlines.at(0).size()){
        algorithmCurrentPoint = 0;
        
        //Start seeking first infill start
        algorithmMode = "SEEK";
        algorithmInfillIndex = 0;
        if (intersectionPaths.empty()){
          algorithmTarget = "BASE";  // nothing to infill (area narrower than one lane)
        }
      }
      
      NextX = extOutlines.at(algorithmCurrentOutline).at(algorithmCurrentPoint).at(0);
      NextY = extOutlines.at(algorithmCurrentOutline).at(algorithmCurrentPoint).at(1);
    }
  }
  if (algorithmTarget == "BASE"){
    nextTool = NAV_TOOL_OFF;
    
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
      //If reached charging station exit point (only meaningful when returning to base)
      if (algorithmTarget == "BASE" && algorithmCurrentPoint == 0){
        MainChargingStart();
        algorithmDone = true;
        return std::vector<double>();  // bail immediately
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

  targetPointLon = NextX;
  targetPointLat = NextY;
  
  // A leg between neighbouring points of one outline runs along it: the mower must not swing
  // out on the outline's side. Counter-clockwise, the inside is on the left going up the
  // indices; outside the outer outline and inside an island is where the mower must not go.
  int edge = 0;
  if (lastOutline == algorithmCurrentOutline && lastPoint >= 0){
    long long n = extOutlines.at(algorithmCurrentOutline).size();
    int dir = algorithmCurrentPoint == (lastPoint + 1) % n ? 1 : lastPoint == (algorithmCurrentPoint + 1) % n ? -1 : 0;
    int ccw = SignedArea(extOutlines.at(algorithmCurrentOutline)) > 0 ? 1 : -1;
    edge = dir * ccw * (algorithmCurrentOutline == 0 ? -1 : 1);
  }
  lastOutline = algorithmCurrentOutline;
  lastPoint = algorithmCurrentPoint;

  std::vector<double> returnList;
  returnList.push_back((double)NextX / 100.0);
  returnList.push_back((double)NextY / 100.0);
  returnList.push_back((double)nextTool);
  returnList.push_back((double)edge);
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
    
    //TODO: Implement mower reverse for a bit from obstacle
  }
  // The caller asks AlgorithmNextPoint() for the next target. (Before 2026-10-27 this function
  // did that itself, crashed on the empty "done" result and sent cm values as metres.)
}

void AlgorithmSetCurrentPosition(long long x_cm, long long y_cm) {
  current_lon = x_cm;
  current_lat = y_cm;
}

void AlgorithmReset() {
  algorithmTarget = "FORWARD";
  algorithmMode = "OUTLINE";
  algorithmAbortFull = false;
  algorithmDone = false;

  algorithmCurrentOutline = 0;
  algorithmCurrentPoint = 0;

  algorithmInfillIndex = 0;
  algorithmInfillPoint = 0;
  algorithmInfillDirection = 0;

  prevPointLon = 0;
  prevPointLat = 0;
  targetPointLon = 0;
  targetPointLat = 0;
  nextTool = NAV_TOOL_KEEP;
  lastOutline = -1;
  lastPoint = -1;
}

void AlgorithmCaptureStart(){
  AlgorithmReset();
  outlines.clear();
}

void AlgorithmCaptureNewOutline(){
  outlines.push_back( std::vector<std::vector<long long>>());
}
void AlgorithmCaptureBaseExitPoint(int64_t x_cm, int64_t y_cm){
  BASE_EXIT_X_CM = x_cm;
  BASE_EXIT_Y_CM = y_cm;
}
void AlgorithmCaptureNewPoint(){
  if (outlines.empty()) {
    AlgorithmCaptureNewOutline();
  }

  outlines.back().push_back( std::vector<long long>());

  outlines.back().back().push_back(GetLon());
  outlines.back().back().push_back(GetLat());
}
void AlgorithmCaptureSetNewPoint(long long lon, long long lat){
  if (outlines.empty()) {
    AlgorithmCaptureNewOutline();
  }

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
  if (outlines.empty() || outlines.back().size() == 0){return false;}

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
  AlgorithmReset();
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

void AlgorithmSeedStartPosition() {
    // Seed prevPoint and targetPoint to current pose so the first segment's
    // start = robot's actual position. Subsequent segments will use the
    // intended (planned) previous target.
    prevPointLon = current_lon;
    prevPointLat = current_lat;
    targetPointLon = current_lon;
    targetPointLat = current_lat;
}

const std::vector<std::vector<std::vector<long long>>>& AlgorithmGetExtOutlines() {
    return extOutlines;
}
const std::vector<std::vector<std::vector<long long>>>& AlgorithmGetIntersectionPaths() {
    return intersectionPaths;
}