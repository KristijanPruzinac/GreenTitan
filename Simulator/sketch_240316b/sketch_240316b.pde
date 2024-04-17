float[] angles;  // Array to store angles
int n = 200; // Number of entries to display

void setup() {
  size(1600, 600);
  smooth();
  loadData();  // Load data from file
  plotGraph(); // Plot the graph
}

void loadData() {
  String[] lines = loadStrings("data.txt");  // Load data from file
  angles = new float[lines.length];          // Initialize array to store angles
  
  for (int i = 0; i < lines.length; i++) {
    angles[i] = Float.parseFloat(lines[i]);  // Parse angle from string to float
  }
}

void plotGraph() {
  background(255);  // Set background to white
  
  float centerX = width / 2;  // X-coordinate of the center of the graph
  float centerY = height / 2; // Y-coordinate of the center of the graph
  
  // Draw X-axis
  stroke(0);
  line(0, centerY, width, centerY);
  
  // Draw Y-axis
  line(centerX, 0, centerX, height);
  
  // Set y-axis limits
  float yMin = -180;
  float yMax = 180;
  float yRange = yMax - yMin;
  
  // Plot data points
  stroke(255, 0, 0);  // Set color to red
  float prevX = 0;
  float prevY = 0;
  
  // Limit number of entries to display
  for (int i = 0; i < n; i++) {
    if (i >= angles.length)
      break;
      
    float x = map(i, 0, n-1, 0, width);  // Map index to X-coordinate
    float y = map(angles[i], yMin, yMax, height, 0);    // Map angle to Y-coordinate
    
    if (i > 0) {
      line(prevX, prevY, x, y);  // Draw line segment between consecutive points
    }
    
    prevX = x;
    prevY = y;
  }
  
  fill(0);
  // Display y-axis labels every 10 units
  for (int i = 30; i <= yMax; i += 30) {
    textAlign(RIGHT, CENTER);
    text(i, centerX - 5, map(i, yMin, yMax, height, 0));
  }
  for (int i = 30; i >= yMin; i -= 30) {
    textAlign(RIGHT, CENTER);
    text(i, centerX - 5, map(i, yMin, yMax, height, 0));
  }
  
  // Display x-axis labels representing seconds every 5 data points
  for (int i = 0; i <= n; i += 5) {
    float x = map(i, 0, n - 1, 0, width);
    textAlign(CENTER, CENTER);
    text(i / 5, x, centerY + 15);
  }
}
