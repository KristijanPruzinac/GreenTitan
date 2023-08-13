//Main task

long mowerLon = -5672328;
long mowerLat = 3376391;

float mowerAzimuth = 270.0 + 45.0;

//Charging station
long baseLon = -5672328; //REMOVE
long baseLat = 3376391; //REMOVE

//Outlines - First point is the charging station exit point
//FILL ON POWER UP FROM CONFIGURATION FILE
long[][][] outlines = new long[][][]{
{
  {-5672428, 3376291},
  {-5672337, 3376280},
  {-5672243, 3376319},
  {-5672130, 3376305},
  {-5672130, 3376240},
  {-5672184, 3376188},
  {-5672143, 3375989},
  {-5672291, 3375864},
  {-5672455, 3375851},
  {-5672405, 3375956},
  {-5672501, 3375991},
  {-5672613, 3375940},
  {-5672609, 3376093},
  {-5672548, 3376230}
},
{
  {-5672355, 3376011},
  {-5672368, 3375913},
  {-5672262, 3375942},
  {-5672215, 3376036},
  {-5672315, 3376075}
},
{
  {-5672408, 3376080},
  {-5672564, 3376021},
  {-5672496, 3376222},
  {-5672460, 3376164},
  {-5672371, 3376208},
  {-5672340, 3376148}
}
}; //20 50 2

char[][] gcode;

long terrainMinX;
long terrainMaxX;
long terrainMinY;
long terrainMaxY;

void GenerateGcode(){
  //Gcode
  gcode = new char[MAX_OUTLINE_COUNT * MAX_POINT_COUNT * 2][2];
  
  //Error conditions
  if (outlines[0].length <= 2){return;}
  
  //Find terrain bounds
  terrainMinX = outlines[0][0][0]; terrainMaxX = outlines[0][0][0];
  terrainMinY = outlines[0][0][1]; terrainMaxY = outlines[0][0][1];
  
  for (int i = 0; i < outlines.length; i++){
    for (int j = 0; j < outlines[i].length; j++){
      long xVal = outlines[i][j][0];
      long yVal = outlines[i][j][1];
      
      if (xVal < terrainMinX){terrainMinX = xVal;}
      if (yVal < terrainMinY){terrainMinY = yVal;}
      
      if (xVal > terrainMaxX){terrainMaxX = xVal;}
      if (yVal > terrainMaxY){terrainMaxY = yVal;}
    }
  }
  
  //Infill
  int numOfLines = ceil(abs(terrainMaxY - terrainMinY) / MOWER_OVERLAP);
  
  //long[][][] outlinesFilled = new long[MAX_OUTLINE_COUNT][MAX_POINT_COUNT * 2][2];
}
