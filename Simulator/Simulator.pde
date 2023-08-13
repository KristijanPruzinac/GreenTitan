void setup(){
  size(800, 800);
}

void draw(){
  background(255);
  translate(width / 2, height * 6 / 7);
  
  //Base station
  noStroke();
  fill(100, 100, 200);
  circle(0, 0, 15);
  
  noStroke();
  fill(100, 150, 200);
  circle(baseExitLon - baseLon, baseExitLat - baseLat, 15);
  
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
