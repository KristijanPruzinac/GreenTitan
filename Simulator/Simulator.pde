//Adapt to Arduino IDE
String String(long val){return str(val);}
String String(int val){return str(val);}
String String(float val){return str(val);}

int translateYVal;

void setup(){
  size(800, 800);
  
  GenerateGcode();
  
  translateYVal = height * 6 / 7;
}

long adjustX(long in){
  return in - baseLon;
}
long adjustY(long in){
  return in - baseLat;
}

void mousePressed(){
  println("{" + str(int(mouseX - width / 2 + mowerLon)) + ", " + str(int(mouseY - height * 6 / 7 + mowerLat)) + "},");
}

void keyPressed(){
  //Test different coordinates
  if (key == 'q'){
    for (int i = 0; i < outlines.length; i++){
      for (int j = 0; j < outlines[i].length; j++){
        outlines[i][j][0] = -outlines[i][j][0];
      }
    }
    baseLon = -baseLon;
    mowerLon = - mowerLon;
    
    GenerateGcode();
  }
  if (key == 'w'){
    for (int i = 0; i < outlines.length; i++){
      for (int j = 0; j < outlines[i].length; j++){
        outlines[i][j][1] = -outlines[i][j][1];
      }
    }
    
    baseLat = -baseLat;
    mowerLat = -mowerLat;
    
    translateYVal = height - translateYVal;
    
    GenerateGcode();
  }
}

void draw(){
  background(255);
  translate(width / 2, translateYVal);
  
  //Boundaries
  stroke(220);
  strokeWeight(1);
  line(adjustX(terrainMinX), adjustY(terrainMinY), adjustX(terrainMaxX), adjustY(terrainMinY));
  line(adjustX(terrainMinX), adjustY(terrainMaxY), adjustX(terrainMaxX), adjustY(terrainMaxY));
  
  line(adjustX(terrainMinX), adjustY(terrainMinY), adjustX(terrainMinX), adjustY(terrainMaxY));
  line(adjustX(terrainMaxX), adjustY(terrainMinY), adjustX(terrainMaxX), adjustY(terrainMaxY));
  
  //Points
  noStroke();
  fill(200);
  for (int i = 0; i < outlines.length; i++){
    for (int j = 0; j < outlines[i].length; j++){
      circle(adjustX(outlines[i][j][0]), adjustY(outlines[i][j][1]), 15);
    }
  }
  
  //Lines
  stroke(200);
  strokeWeight(1);
  for (int i = 0; i < outlines.length; i++){
    for (int j = 0; j < outlines[i].length - 1; j++){
      line(adjustX(outlines[i][j][0]), adjustY(outlines[i][j][1]), adjustX(outlines[i][j + 1][0]), adjustY(outlines[i][j + 1][1]));
    }
    if (outlines[i].length > 1){
      line(adjustX(outlines[i][0][0]), adjustY(outlines[i][0][1]), adjustX(outlines[i][outlines[i].length - 1][0]), adjustY(outlines[i][outlines[i].length - 1][1]));
    }
  }
  
  //Base station
  noStroke();
  fill(100, 100, 200);
  circle(0, 0, 15);
  
  noStroke();
  fill(100, 150, 200);
  circle(adjustX(outlines[0][0][0]), adjustY(outlines[0][0][1]), 15);
  
  //Mower
  noStroke();
  fill(100, 200, 100);
  circle(adjustX(mowerLon), adjustY(mowerLat), 15);
  
  stroke(100, 200, 100);
  pushMatrix();
  translate(adjustX(mowerLon), adjustY(mowerLat));
  rotate(radians(mowerAzimuth));
  line(0, 0, 0, -30);
  popMatrix();
  
  noStroke();
  fill(0, 0, 0);
  text(mode, width - 100, height - 50);
  
}
