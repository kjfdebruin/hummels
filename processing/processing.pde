/**
 * 
*/

import processing.serial.*;

Serial myPort;  // Create object from Serial class

boolean firstSample = true;

float [] RwAcc = new float[3];         //projection of normalized gravitation force vector on x/y/z axis, as measured by accelerometer
float [] Gyro = new float[3];          //Gyro readings
float [] RwGyro = new float[3];        //Rw obtained from last estimated value and gyro movement
float [] Awz = new float[2];           //angles between projection of R on XZ/YZ plane and Z axis (deg)
float [] RwEst = new float[3];


int lastTime = 0;
int interval = 0;
float wGyro = 10.0;

int lf = 10; // 10 is '\n' in ASCII
byte[] inBuffer = new byte[255];

PFont font;
final int VIEW_SIZE_X = 800, VIEW_SIZE_Y = 800;


void setup() 
{
  size(600, 600, P3D);
  myPort = new Serial(this, "COM7", 9600);  
  
  // The font must be located in the sketch's "data" directory to load successfully
  font = loadFont("CourierNew36.vlw"); 
  print("Setup complete.");
}


void readSensors() {
  if(myPort.available() > 0) {
    if (myPort.readBytesUntil(lf, inBuffer) > 0) {
      String inputString = new String(inBuffer);
      println("in:", inputString);
      String [] inputStringArr = split(inputString, ',');

     if (inputStringArr.length == 6) {
        // convert raw readings to G
        RwAcc[0] = float(trim(inputStringArr[0]));
        RwAcc[1] = float(trim(inputStringArr[1]));
        RwAcc[2] = float(trim(inputStringArr[2]));
        
        // convert raw readings to degrees/sec
        Gyro[0] = float(trim(inputStringArr[3]));
        Gyro[1] = float(trim(inputStringArr[4]));
        Gyro[2] = float(trim(inputStringArr[5]));
        println("out:", RwAcc[0], ",", RwAcc[1], ",", RwAcc[2], ",", Gyro[0], ",", Gyro[1], ",",Gyro[2]);
     }
      
    }
  }
}


void normalize3DVec(float [] vector) {
  float R;
  R = sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2]);
  vector[0] /= R;
  vector[1] /= R;  
  vector[2] /= R;  
}


float squared(float x){
  return x*x;
}


void buildBoxShape() {
  //box(60, 10, 40);
  noStroke();
  beginShape(QUADS);
  
  //Z+ (to the drawing area)
  fill(#00ff00);
  vertex(-30, -5, 20);
  vertex(30, -5, 20);
  vertex(30, 5, 20);
  vertex(-30, 5, 20);
  
  //Z-
  fill(#0000ff);
  vertex(-30, -5, -20);
  vertex(30, -5, -20);
  vertex(30, 5, -20);
  vertex(-30, 5, -20);
  
  //X-
  fill(#ff0000);
  vertex(-30, -5, -20);
  vertex(-30, -5, 20);
  vertex(-30, 5, 20);
  vertex(-30, 5, -20);
  
  //X+
  fill(#ffff00);
  vertex(30, -5, -20);
  vertex(30, -5, 20);
  vertex(30, 5, 20);
  vertex(30, 5, -20);
  
  //Y-
  fill(#ff00ff);
  vertex(-30, -5, -20);
  vertex(30, -5, -20);
  vertex(30, -5, 20);
  vertex(-30, -5, 20);
  
  //Y+
  fill(#00ffff);
  vertex(-30, 5, -20);
  vertex(30, 5, -20);
  vertex(30, 5, 20);
  vertex(-30, 5, 20);
  
  endShape();
}


void drawCube() {  
  pushMatrix();
    translate(300, 450, 0);
    scale(4,4,4);
    
    rotateX(HALF_PI * -RwEst[0]);
    rotateZ(HALF_PI * RwEst[1]);
    
    buildBoxShape();
    
  popMatrix();
}


void getInclination() {
  int w = 0;
  float tmpf = 0.0;
  int currentTime, signRzGyro;
  
  
  readSensors();
  normalize3DVec(RwAcc);
  
  currentTime = millis();
  interval = currentTime - lastTime;
  lastTime = currentTime;
  
  if (firstSample || Float.isNaN(RwEst[0])) { // the NaN check is used to wait for good data from the Arduino
    for(w=0;w<=2;w++) {
      RwEst[w] = RwAcc[w];    //initialize with accelerometer readings
    }
  }
  else{
    //evaluate RwGyro vector
    if(abs(RwEst[2]) < 0.1) {
      //Rz is too small and because it is used as reference for computing Axz, Ayz it's error fluctuations will amplify leading to bad results
      //in this case skip the gyro data and just use previous estimate
      for(w=0;w<=2;w++) {
        RwGyro[w] = RwEst[w];
      }
    }
    else {
      //get angles between projection of R on ZX/ZY plane and Z axis, based on last RwEst
      for(w=0;w<=1;w++){
        tmpf = Gyro[w];                        //get current gyro rate in deg/s
        tmpf *= interval / 1000.0f;                     //get angle change in deg
        Awz[w] = atan2(RwEst[w],RwEst[2]) * 180 / PI;   //get angle and convert to degrees 
        Awz[w] += tmpf;             //get updated angle according to gyro movement
      }
      
      //estimate sign of RzGyro by looking in what qudrant the angle Axz is, 
      //RzGyro is pozitive if  Axz in range -90 ..90 => cos(Awz) >= 0
      signRzGyro = ( cos(Awz[0] * PI / 180) >=0 ) ? 1 : -1;
      
      //reverse calculation of RwGyro from Awz angles, for formulas deductions see  http://starlino.com/imu_guide.html
      for(w=0;w<=1;w++){
        RwGyro[0] = sin(Awz[0] * PI / 180);
        RwGyro[0] /= sqrt( 1 + squared(cos(Awz[0] * PI / 180)) * squared(tan(Awz[1] * PI / 180)) );
        RwGyro[1] = sin(Awz[1] * PI / 180);
        RwGyro[1] /= sqrt( 1 + squared(cos(Awz[1] * PI / 180)) * squared(tan(Awz[0] * PI / 180)) );        
      }
      RwGyro[2] = signRzGyro * sqrt(1 - squared(RwGyro[0]) - squared(RwGyro[1]));
    }
    
    //combine Accelerometer and gyro readings
    for(w=0;w<=2;w++) RwEst[w] = (RwAcc[w] + wGyro * RwGyro[w]) / (1 + wGyro);

    normalize3DVec(RwEst);
  }
  
  firstSample = false;
}


void draw() {  
  getInclination();
  
  background(#000000);
  fill(#ffffff);
  
  textFont(font, 20);
  //float temp_decoded = 35.0 + ((float) (temp + 13200)) / 280;
  //text("temp:\n" + temp_decoded + " C", 350, 250);
  text("RwAcc (G):\n" + RwAcc[0] + "\n" + RwAcc[1] + "\n" + RwAcc[2] + "\ninterval: " + interval, 20, 50);
  text("Gyro (°/s):\n" + Gyro[0] + "\n" + Gyro[1] + "\n" + Gyro[2], 220, 50);
  text("Awz (°):\n" + Awz[0] + "\n" + Awz[1], 420, 50);
  text("RwGyro (°/s):\n" + RwGyro[0] + "\n" + RwGyro[1] + "\n" + RwGyro[2], 20, 180);
  text("RwEst :\n" + RwEst[0] + "\n" + RwEst[1] + "\n" + RwEst[2], 220, 180);
  
  // display axes
  pushMatrix();
    translate(450, 250, 0);
    stroke(#ffffff);
    scale(100, 100, 100);
    line(0,0,0,1,0,0);
    line(0,0,0,0,-1,0);
    line(0,0,0,0,0,1);
    line(0,0,0, -RwEst[0], RwEst[1], RwEst[2]);
  popMatrix();
  
  drawCube();
}
