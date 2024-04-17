String[] lines; // Array to store lines from the file
float minX, maxX, minY, maxY;

float[] angles = new float[72];

void setup() {
  size(800, 600);  // Set the size of the canvas
  background(255); // Set the background color to white
  lines = loadStrings("data.txt");  // Load data from the file
  drawXYPlot();    // Call the function to draw the XY plot
  
  findMinMax();
}

void findMinMax() {
  // Initialize min and max values with the first data point
  String[] values = split(lines[0], ' ');
  minX = maxX = float(values[0]);
  minY = maxY = float(values[1]);

  // Find min and max values for X and Y coordinates
  for (int i = 1; i < lines.length; i++) {
    values = split(lines[i], ' ');
    float x = float(values[0]);
    float y = float(values[1]);

    minX = min(minX, x);
    maxX = max(maxX, x);
    minY = min(minY, y);
    maxY = max(maxY, y);
  }
  
  println((maxX + minX) / 2);
  println((maxY + minY) / 2);
  
  println(maxX-minX);
  println(maxY-minY);
}

void drawXYPlot() {
  // Draw X and Y axes at the center of the canvas
  pushMatrix();
  translate(width / 2, height / 2);
  scale(3);
  stroke(0);  // Set stroke color to black
  line(-width / 2, 0, width / 2, 0);  // X-axis
  line(0, -height / 2, 0, height / 2);  // Y-axis

  // Plot the data points in real scale on both axes
  for (int i = 0; i < lines.length; i++) {
    String[] values = split(lines[i], ' ');
    float x = float(values[0]);
    float y = float(values[1]);
    ellipse(x, -(y), 2, 2);  // Invert y-coordinate to match the screen coordinate system
  }
  popMatrix();
}
