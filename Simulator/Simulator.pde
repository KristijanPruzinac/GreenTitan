//Adapt to Arduino IDE
String String(long val){return str(val);}
String String(int val){return str(val);}
String String(float val){return str(val);}

void setup(){
  size(800, 800);
}

void mousePressed(){
  println("{" + str(int(mouseX - width / 2 + mowerLon)) + ", " + str(int(mouseY - height * 6 / 7 + mowerLat)) + "},");
}

void draw(){
  background(255);
  translate(width / 2, height * 6 / 7);
  
  //Points
  noStroke();
  fill(200);
  for (int i = 0; i < outlines.length; i++){
    for (int j = 0; j < outlines[i].length; j++){
      circle(outlines[i][j][0] - baseLon, outlines[i][j][1] - baseLat, 15);
    }
  }
  
  //Lines
  stroke(200);
  strokeWeight(1);
  for (int i = 0; i < outlines.length; i++){
    for (int j = 0; j < outlines[i].length - 1; j++){
      line(outlines[i][j][0] - baseLon, outlines[i][j][1] - baseLat, outlines[i][j + 1][0] - baseLon, outlines[i][j + 1][1] - baseLat);
    }
    if (outlines[i].length > 1){
      line(outlines[i][0][0] - baseLon, outlines[i][0][1] - baseLat, outlines[i][outlines[i].length - 1][0] - baseLon, outlines[i][outlines[i].length - 1][1] - baseLat);
    }
  }
  
  //Base station
  noStroke();
  fill(100, 100, 200);
  circle(0, 0, 15);
  
  noStroke();
  fill(100, 150, 200);
  circle(outlines[0][0][0] - baseLon, outlines[0][0][1] - baseLat, 15);
  
  //Mower
  noStroke();
  fill(100, 200, 100);
  circle(mowerLon - baseLon, mowerLat - baseLat, 15);
  
  stroke(100, 200, 100);
  pushMatrix();
  translate(mowerLon - baseLon, mowerLat - baseLat);
  rotate(radians(mowerAzimuth));
  line(0, 0, 0, -30);
  popMatrix();
  
  noStroke();
  fill(0, 0, 0);
  text(mode, width - 100, height - 50);
  
}
