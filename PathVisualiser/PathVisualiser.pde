String[] lines;
int[] gpsX, gpsY;
float[] azimuth;
float minX, maxX, minY, maxY;
int currentLine = 0;
int frameRate = 5;

void setup() {
  size(800, 600);
  frameRate(frameRate);
  
  String[] data = loadStrings("TEST1NNN.txt");
  lines = new String[data.length];
  gpsX = new int[data.length];
  gpsY = new int[data.length];
  azimuth = new float[data.length];

  for (int i = 0; i < data.length; i++) {
    lines[i] = data[i];
    String[] values = split(data[i], ' ');
    gpsX[i] = int(values[0]) % 1000000;
    gpsY[i] = int(values[1]) % 1000000;
    azimuth[i] = float(values[2]);

    // Update min and max values
    if (i == 0) {
      minX = maxX = gpsX[i];
      minY = maxY = gpsY[i];
    } else {
      minX = min(minX, gpsX[i]);
      maxX = max(maxX, gpsX[i]);
      minY = min(minY, gpsY[i]);
      maxY = max(maxY, gpsY[i]);
    }
  }
}

void draw() {
  background(255);

  // Calculate center of data
  float centerX = (minX + maxX) / 2;
  float centerY = (minY + maxY) / 2;

  // Translate and scale the view
  float scaleFactor = min(width / (maxX - minX), height / (maxY - minY)); scaleFactor = 1;
  translate(width / 2 - centerX * scaleFactor, height / 2 - centerY * scaleFactor);
  scale(scaleFactor);

  // Draw data points
  int i = max(currentLine - 1, 0);
  background(255);
  float x = gpsX[i];
  float y = gpsY[i];
  float angle = radians(azimuth[i]);

  stroke(0);
  
  pushMatrix();
  translate(x, y);
  rotate(angle);
  line(0, 0, 0, -50);
  popMatrix();
  
  if (i > 0){
    float angleRad = atan2(gpsY[i] - gpsY[i - 1], gpsX[i] - gpsX[i - 1]);

    stroke(255, 0, 0);
    pushMatrix();
    translate(x, y);
    rotate(radians((angleRad * 180 / PI) + 90.0));
    line(0, 0, 0, -50);
    popMatrix();
  }

  // Draw a point at the GPS coordinates
  fill(0);
  ellipseMode(CENTER);
  ellipse(x, y, 5, 5);

  // Update current line to be drawn in the next frame
  currentLine = min(currentLine + 1, lines.length);
}

void keyPressed() {
  // Pause or resume animation on spacebar press
  if (key == ' ') {
    if (frameRate == 0) {
      frameRate(5);
    } else {
      frameRate(0);
    }
  }
}
