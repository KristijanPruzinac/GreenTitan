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
ArrayList<ArrayList<ArrayList<Long>>> extOutlines; //Outlines with intersection points inserted

ArrayList<ArrayList<Long>> gcode = new ArrayList<ArrayList<Long>>();

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
  
  numOfLines = ceil(abs(terrainMaxY - terrainMinY) / (MOWER_OVERLAP * 1.109)); //1.109 is gps latitude scaling factor

  for (int i = 0; i < numOfLines; i++)
  {
    long currentY = (long) (terrainMinY + abs(terrainMaxY - terrainMinY) / (float)(numOfLines) * i + abs(terrainMaxY - terrainMinY) / (float)(numOfLines) / 2.0);
    
    //Traverse outlines and extend outlines with intersections
    for (int o = 0; o < outlines.size(); o++){
      
      ArrayList<ArrayList<Long>> intersections = new ArrayList<ArrayList<Long>>();
      // X1, Y1, NX, NY
      
      //Find intersections for current outline
      for (int p = 0; p < outlines.get(o).size(); p++){
        long p1X = outlines.get(o).get(p).get(0);
        long p1Y = outlines.get(o).get(p).get(1);
        
        long p2X, p2Y;
        
        //First-last case
        if (p == outlines.get(o).size() - 1){
          p2X = outlines.get(o).get(0).get(0);
          p2Y = outlines.get(o).get(0).get(1);
        }
        else {
          p2X = outlines.get(o).get(p + 1).get(0);
          p2Y = outlines.get(o).get(p + 1).get(1);
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
        
        //Check corresponding intersections
        for (int it = 0; it < intersections.size(); it++){
          
          //If point is between these points, insert it into list
          if ((intersections.get(it).get(0) == p1X) &&
              (intersections.get(it).get(1) == p1Y))
          {
            ArrayList<Long> appendList = new ArrayList<Long>();
            appendList.add(intersections.get(it).get(2));
            appendList.add(intersections.get(it).get(3));
            
            extOutlines.get(o).add(p, appendList);
            p++;
          }
        }
        
        p++;
      }
    }
  }
}

void GeneratePaths(){
  
}

void GenerateGcode(){
  //Error conditions
  if (outlines.size() < 1){return;}
  if (outlines.get(0).size() <= 2){return;}
  
  FindTerrainBounds();
  FindOutlineIntersections();
  GeneratePaths();
}
