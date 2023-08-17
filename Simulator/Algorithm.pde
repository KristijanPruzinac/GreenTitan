//Main task

long mowerLon = -5672328;
long mowerLat = 3376391;

float mowerAzimuth = 270.0 + 45.0;

//Charging station
long baseLon = -5672328; //REMOVE
long baseLat = 3376391; //REMOVE

int numOfLines;

//Outlines - First point is the charging station exit point
//FILL ON POWER UP FROM CONFIGURATION FILE
ArrayList<ArrayList<ArrayList<Long>>> outlines = new ArrayList<ArrayList<ArrayList<Long>>>();
ArrayList<ArrayList<Long>> gcode = new ArrayList<ArrayList<Long>>();

long terrainMinX;
long terrainMaxX;
long terrainMinY;
long terrainMaxY;

void GenerateGcode(){
  //Error conditions
  if (outlines.size() < 1){return;}
  if (outlines.get(0).size() <= 2){return;}
  
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
  
  //Infill
  numOfLines = ceil(abs(terrainMaxY - terrainMinY) / MOWER_OVERLAP);
  
  //long[][][] pointsToInsert = new long[MAX_OUTLINE_COUNT][MAX_POINT_COUNT * 2][4]; // OUTLINE | INSERT_NUM | POINT_INDEX_X, POINT_INDEX_Y, NEW_POINT_X, NEW_POINT_Y
  
  for (int i = 1; i < numOfLines; i++){
    long currentY = (long) (terrainMinY + ((float)(terrainMaxY - terrainMinY)) / numOfLines * i);
  }
  
  //long[][][] outlinesFilled = new long[MAX_OUTLINE_COUNT][MAX_POINT_COUNT * 2][2];
}
