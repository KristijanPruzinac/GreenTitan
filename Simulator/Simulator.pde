String mode = "CHARGING";
/* CHARGING STOP PAUSE RUNNING */

long mowerLon = -5672328;
long mowerLat = 3376391;

float mowerAzimuth = 270.0 + 45.0;

//Charging station
long baseLon = -5672328;
long baseLat = 3376391;

long baseExitLon = -5672428;
long baseExitLat = 3376291;

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
  
}
