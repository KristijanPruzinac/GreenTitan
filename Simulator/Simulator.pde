
String String(long val){return str(val);}
String String(int val){return str(val);}
String String(float val){return str(val);}

int translateYVal;

boolean MirrorX = false;
boolean MirrorY = true;

String MotorEmulation = "ROTATING";
float TurnAmountLeft = 0;
float TurnAmountRight = 0;
boolean MotorMowing = false;

void setup(){
  size(1440, 810);
  frameRate(60);
  
  translateYVal = height * 6 / 7;
  
  //Program start
  MainPowerOn();
  
  //Mirroring
  if (MirrorX){
    for (int i = 0; i < outlines.size(); i++){
      for (int j = 0; j < outlines.get(i).size(); j++){
        outlines.get(i).get(j).set(0, -outlines.get(i).get(j).get(0));
      }
    }
    baseLon = -baseLon;
    mowerLon = - mowerLon;
    
    GenerateGcode();
  }
  if (MirrorY){
    for (int i = 0; i < outlines.size(); i++){
      for (int j = 0; j < outlines.get(i).size(); j++){
        outlines.get(i).get(j).set(1, -outlines.get(i).get(j).get(1));
      }
    }
    
    baseLat = -baseLat;
    mowerLat = -mowerLat;
    
    translateYVal = height - translateYVal;
    
    GenerateGcode();
  }
  
  mowerAzimuth = angleBetweenPoints(mowerLon, mowerLat, outlines.get(0).get(0).get(0), outlines.get(0).get(0).get(1));
}

long adjustX(long in){
  return in;
}
long adjustY(long in){
  return in;
}

void mousePressed(){
  ArrayList<Long> targetPoint = AlgorithmNextPoint();
  targetPointLon = targetPoint.get(0);
  targetPointLat = targetPoint.get(1);
  
  MotionMoveToTarget();
  
  print(targetPointLon); print(" "); println(targetPointLat);
  
  //println("{" + str(int(mouseX - width / 2 + mowerLon)) + ", " + str(int(mouseY - height * 6 / 7 + mowerLat)) + "},");
}

void keyPressed(){
  if (keyCode == RIGHT){
    for (int i = 0; i < 5; i++){
      ArrayList<Long> mowerPoint = AlgorithmNextPoint();
      targetPointLon = mowerPoint.get(0); mowerLon = targetPointLon;
      targetPointLat = mowerPoint.get(1); mowerLat = targetPointLat;
    }
  }
  else if (keyCode == LEFT){
    AlgorithmAbort(false);
  }
}

void draw(){
  if (MotorEmulation == "MOVING"){
    if (frameCount % 5 == 0){
      MotionMoveToTarget();
      mowerAzimuth = NormalizeAngle(mowerAzimuth - (TurnAmountRight - TurnAmountLeft) * 5);
    
      mowerLon += cos(radians(NormalizeAngle(mowerAzimuth - 90))) * 5;
      mowerLat += sin(radians(NormalizeAngle(mowerAzimuth - 90))) * 5;
    }
  }
  else if (MotorEmulation == "ROTATING"){
    MotionRotateToTarget();
    mowerAzimuth = NormalizeAngle(mowerAzimuth - (TurnAmountRight - TurnAmountLeft) * 5);
  }
  
  background(255);
  pushMatrix();
  translate(width / 2 - (terrainMaxX - terrainMinX) / 2, height / 2 - (terrainMaxY - terrainMinY) / 2);
  
  //Boundaries
  stroke(220);
  strokeWeight(1);
  line(adjustX(terrainMinX), adjustY(terrainMinY), adjustX(terrainMaxX), adjustY(terrainMinY));
  line(adjustX(terrainMinX), adjustY(terrainMaxY), adjustX(terrainMaxX), adjustY(terrainMaxY));
  
  line(adjustX(terrainMinX), adjustY(terrainMinY), adjustX(terrainMinX), adjustY(terrainMaxY));
  line(adjustX(terrainMaxX), adjustY(terrainMinY), adjustX(terrainMaxX), adjustY(terrainMaxY));
  
  //Infill lines
  stroke(220, 0, 0);
  strokeWeight(1);
  for (int i = 0; i < numOfLines; i++){
    line(adjustX(terrainMinX), adjustY((long) (terrainMinY + abs(terrainMaxY - terrainMinY) / (float)(numOfLines) * i + abs(terrainMaxY - terrainMinY) / (float)(numOfLines) / 2.0)), adjustX(terrainMaxX), adjustY((long) (terrainMinY + abs(terrainMaxY - terrainMinY) / (float)(numOfLines) * i + abs(terrainMaxY - terrainMinY) / (float)(numOfLines) / 2.0)));
  }
  
  //Points
  noStroke();
  fill(200, 0, 0);
  for (int i = 0; i < extOutlines.size(); i++){
    for (int j = 0; j < extOutlines.get(i).size(); j++){
      circle(adjustX(extOutlines.get(i).get(j).get(0)), adjustY(extOutlines.get(i).get(j).get(1)), 10);
      //text(str(i) + ", " + str(j), adjustX(extOutlines.get(i).get(j).get(0)) + 10, adjustY(extOutlines.get(i).get(j).get(1)) - 10);
    }
  }
  noStroke();
  fill(200);
  for (int i = 0; i < outlines.size(); i++){
    for (int j = 0; j < outlines.get(i).size(); j++){
      circle(adjustX(outlines.get(i).get(j).get(0)), adjustY(outlines.get(i).get(j).get(1)), 10);
    }
  }
  
  //Lines
  stroke(200);
  strokeWeight(1);
  for (int i = 0; i < outlines.size(); i++){
    for (int j = 0; j < outlines.get(i).size() - 1; j++){
      line(adjustX(outlines.get(i).get(j).get(0)), adjustY(outlines.get(i).get(j).get(1)), adjustX(outlines.get(i).get(j + 1).get(0)), adjustY(outlines.get(i).get(j + 1).get(1)));
    }
    if (outlines.get(i).size() > 1){
      line(adjustX(outlines.get(i).get(0).get(0)), adjustY(outlines.get(i).get(0).get(1)), adjustX(outlines.get(i).get(outlines.get(i).size() - 1).get(0)), adjustY(outlines.get(i).get(outlines.get(i).size() - 1).get(1)));
    }
  }
  
  //Base station
  noStroke();
  fill(100, 100, 200);
  circle(0, 0, 15);
  
  noStroke();
  fill(100, 150, 200);
  circle(adjustX(outlines.get(0).get(0).get(0)), adjustY(outlines.get(0).get(0).get(1)), 15);
  
  //Mower
  noStroke();
  if (MotorMowing){
    fill(100, 230, 100);
  }
  else {
    fill(100, 150, 100);
  }
  circle(adjustX(mowerLon), adjustY(mowerLat), 15);
  
  stroke(100, 200, 100);
  pushMatrix();
  translate(adjustX(mowerLon), adjustY(mowerLat));
  rotate(radians(mowerAzimuth));
  line(0, 0, 0, -30);
  popMatrix();
  
  //Target
  noStroke();
  fill(150, 100, 100);
  circle(adjustX(targetPointLon), adjustY(targetPointLat), 15);
  
  noStroke();
  fill(0, 0, 0);
  text(Mode, width - 100, height - 50);
  
  //circle(adjustX(extOutlines.get(2).get((int) (frameCount / 60)).get(0)), adjustY(extOutlines.get(2).get((int) (frameCount / 60)).get(1)), 20);
  
  popMatrix();
  
  fill(0);
  text(algorithmInfillPoint, width - 100, height - 100);
  text(algorithmCurrentPoint, width - 100, height - 75);
  text(algorithmTarget, width - 100, height - 50);
  text(algorithmMode, width - 100, height - 25);
  
  if (frameCount == 0){
    delay(5000);
  }
}
