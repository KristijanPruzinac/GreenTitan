float RobotPosition = 200;
float RobotSpeed = 100;
float RobotAcceleration = 0;

//Controller controlled by Position and Speed
class Controller {
  private float Position;
  private float Target;
  
  private float Speed;
  private float MaxSpeed;
  
  private float Acceleration;
  private float MaxAcceleration;
  private float NormalAccelerationFactor;
  
  private float Reaction;
  
  Controller(float Position, float Target, float MaxSpeed, float MaxAcceleration, float NormalAccelerationFactor, float Reaction){
    this.Position = Position;
    this.Target = Target;
    
    this.MaxSpeed = MaxSpeed;
    
    this.MaxAcceleration = MaxAcceleration;
    this.NormalAccelerationFactor = NormalAccelerationFactor;
    
    this.Reaction = Reaction;
    
    this.Acceleration = 0;
  }
  
  //Calculations
  void Update(float Position, float Speed, float TimeElapsed){
    this.Position = Position;
    this.Acceleration = (Speed - this.Speed) / TimeElapsed;
    this.Speed = Speed;
    
    //Minimum path needed to stop
    float StoppingAcceleration = MaxAcceleration * NormalAccelerationFactor;
    float StoppingSpeed = MaxSpeed;
    float StoppingPath = pow(StoppingSpeed, 2) / (2 * StoppingAcceleration);
    
    //If in range of stopping path, adjust speed linearly
    float TargetSpeed;
    float DistanceToTarget = Target - Position;
    if (abs(DistanceToTarget) < StoppingPath){
      TargetSpeed = ((Target - Position) / StoppingPath) * MaxSpeed;
    }
    //Else speed up to MaxSpeed
    else {
      TargetSpeed = (Target - Position) / abs(Target - Position) * MaxSpeed;
    }
    
    //Match speed
    this.Acceleration = constrain(((TargetSpeed - Speed) / MaxSpeed) * MaxAcceleration * Reaction, -MaxAcceleration, MaxAcceleration);
  }
  
  float Acceleration(){
    return this.Acceleration;
  }
}

Controller Robot;

void setup(){
  Robot = new Controller(0, width / 2, 100, 100, 0.5, 2);
  
  size(800, 800);
  frameRate(60);
}

void mousePressed(){
  RobotPosition = mouseX;
}

void draw(){
  background(255);
  
  strokeWeight(1);
  stroke(150);
  line(0, height / 2, width, height / 2);
  
  //Acceleration
  line(0, 50, abs(RobotAcceleration) / 100.0 * width, 50);
  
  noStroke();
  fill(70);
  ellipseMode(CENTER);
  ellipse(RobotPosition, height / 2, 20, 20);

  
  //Update robot
  Robot.Update(RobotPosition, RobotSpeed, 1.0 / frameRate);
  
  //Simulation
  RobotAcceleration = Robot.Acceleration();// + constrain((width / 2 - mouseX) / 4, -50, 50);
  RobotSpeed += RobotAcceleration / (float) frameRate;
  RobotPosition += RobotSpeed / (float) frameRate;
  
  println(abs(RobotPosition - 400), RobotSpeed);
}
