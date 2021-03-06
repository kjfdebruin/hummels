#include "Volume3.h"

// Motors & PWM Speed Control：
int E1 = 5;
int M1 = 4; // Right motors
int E2 = 6;
int M2 = 7; // Left motors

// Obstacle detector inputs
int OBSTACLE_LEFT = 8;
int OBSTACLE_MIDDLE = 11;
int OBSTACLE_RIGHT = 10;
int HUMAN = 12;

// Movement defintions
int FORWARD = 1;
int BACKWARD = 0;
int MAX_SPEED = 200; // 0 to 255, where 255 is the fastest

int TURN_QUARTER = 45;
int TURN_HALF = 90;
int TURN_THREE_QUARTERS = 135;
int TURN_FULL = 180;

int currentDirection = FORWARD;
int currentSpeed = 0;
int hasObstacleLeft = false;
int hasObstacleMiddle = false;
int hasObstacleRight = false;
int hasHuman = false;

int timeOutCounter = 0;
bool isOnTimeout = false;
int soundToMake = 0;

#define SPEAKER 9

void setup()
{
    //Motors
    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);

    // Obtacle detectors
    pinMode(OBSTACLE_LEFT, INPUT);
    pinMode(OBSTACLE_MIDDLE, INPUT);
    pinMode(OBSTACLE_RIGHT, INPUT);
    pinMode(HUMAN, INPUT);

    // Sound
    pinMode(SPEAKER, OUTPUT);
    
    stopAll();

    gameboy();

    Serial.begin(9600);
}

void getObstacles() 
{
  // Not sure why, but pins are inverted
  hasObstacleLeft = !digitalRead(OBSTACLE_LEFT);
  hasObstacleMiddle = !digitalRead(OBSTACLE_MIDDLE); 
  hasObstacleRight = !digitalRead(OBSTACLE_RIGHT);
  hasHuman = digitalRead(HUMAN);

  if (hasObstacleLeft)
    Serial.println("OBSTACLE LEFT");
  if (hasObstacleMiddle)
    Serial.println("OBSTACLE MIDDLE");
  if (hasObstacleRight)
    Serial.println("OBSTACLE RIGHT");
  if (hasHuman)
    Serial.println("HUMAN!");
  
}

void accelerate()
{
  if (currentSpeed != MAX_SPEED) 
  {
    int value;
    for(value = 0 ; value < MAX_SPEED; value+=25)
    {
      analogWrite(E1, value);   //PWM Speed Control
      analogWrite(E2, value);   //PWM Speed Control
      delay(30);
    }
    currentSpeed = MAX_SPEED;
  }
}

void decelerate()
{
  if (currentSpeed != 0) 
  {
    int value;
    for(value = MAX_SPEED ; value > 0; value-=25)
    {
      analogWrite(E1, value);   //PWM Speed Control
      analogWrite(E2, value);   //PWM Speed Control
      delay(30);
    }
    currentSpeed = 0;
  }
}

// Stop all motors. Also initialise both left and right motors to forward
void stopAll()
{
  analogWrite(E1, 0);
  analogWrite(E2, 0);
  setMotorDirection(FORWARD, FORWARD);
  delay(1000);
}

void setMotorDirection(int directionOfMovementLeft, int directionOfMovementRight)
{
  digitalWrite(M1, directionOfMovementRight);
  digitalWrite(M2, directionOfMovementLeft);
}

// Move either directionOfMovement = FORWARD or BACKWARD for duration (in ms)
void go(int directionOfMovement, int duration)
{
  if (currentDirection == directionOfMovement) {
    // Keep going in the same direction. If we aren't moving, accelerate
    if (currentSpeed == 0) {
      accelerate();
    }
  }
  else {
    // If we're changing direction, first decelerate
    decelerate();
    setMotorDirection(directionOfMovement, directionOfMovement);
    accelerate();
  }
  currentDirection = directionOfMovement;
  delay(duration);
}

void turnLeft(int angle)
{
  int timeToTurn = 0;
  if (angle == TURN_QUARTER) {
    timeToTurn = 600;
  }
  if (angle == TURN_HALF) {
    timeToTurn = 1200;
  }
  else if (angle == TURN_THREE_QUARTERS) {
    timeToTurn = 1800;
  }
  else if (angle == TURN_FULL) {
    timeToTurn = 2400;
  }
  decelerate();
  setMotorDirection(BACKWARD, FORWARD);
  accelerate();
  delay(timeToTurn);
  decelerate();
  setMotorDirection(FORWARD, FORWARD);
}

void turnRight(int angle)
{
  int timeToTurn = 0;
  if (angle == TURN_QUARTER) {
    timeToTurn = 600;
  }
  if (angle == TURN_HALF) {
    timeToTurn = 1200;
  }
  else if (angle == TURN_THREE_QUARTERS) {
    timeToTurn = 1800;
  }
  else if (angle == TURN_FULL) {
    timeToTurn = 2400;
  }
  decelerate();
  setMotorDirection(FORWARD, BACKWARD);
  accelerate();
  delay(timeToTurn);
  decelerate();
  setMotorDirection(FORWARD, FORWARD);
}

void jiggle()
{
  MAX_SPEED = 255;
  turnRight(TURN_QUARTER);
  turnLeft(TURN_QUARTER);
}

void avoidObstacles()
{
  if (hasObstacleLeft || hasObstacleMiddle || hasObstacleRight) {
    decelerate();
    // If there is an obstacle, try and "intelligently" avoid it
    if (!hasObstacleRight && (hasObstacleLeft || (hasObstacleLeft && hasObstacleMiddle))) {
      turnRight(TURN_QUARTER);
    }
    else if (!hasObstacleLeft && (hasObstacleRight || (hasObstacleRight && hasObstacleMiddle))) {
      turnLeft(TURN_QUARTER);
    }
    else if (!hasObstacleLeft && !hasObstacleRight && hasObstacleMiddle) {
      turnLeft(TURN_HALF);
    }
    else if (hasObstacleLeft && hasObstacleMiddle) {
      turnLeft(TURN_FULL);
    }
  }
}

void gameboy(){
  vol.tone(SPEAKER,1025,1023); // pa
  delay(70);
  uint16_t v = 1000;
  while(v > 0){
    vol.tone(SPEAKER, 2090, v); // ting!
    delay(10);
    v-=10;
  }
}

void wolfWhistle() {
  int f = 122; // starting frequency
  int v = 0;   // starting volume
  while (f < 4000) {  // slide up to 4000Hz
    vol.tone(SPEAKER, f, v);
    v = 1023 * (f / 4000.00);
    f += 25;
    delay(1);
  }
  vol.noTone();
  delay(100); // wait a moment
  f = 122; // starting frequency
  v = 0;   // starting volume
  while (f < 3000) { // slide up to 3000Hz
    vol.tone(SPEAKER, f, v);
    v = 1023 * (f / 4000.00); 
    f += 25;
    delay(2);
  }
  while (f > 125) { // slide down to 125Hz
    vol.tone(SPEAKER, f, v);
    v = 1023 * (f / 4000.00);
    f -= 25;
    delay(2);
  }
  vol.noTone(); // end tone production
}

void R2D2() {
  int beeps[] = {1933, 2156, 1863, 1505, 1816, 1933, 1729, 2291};
  int buzzVols[] = {144, 180, 216, 252, 252, 252, 252, 216, 180, 144};

  int i = 9;
  while (i >= 0) {
    vol.tone(SPEAKER, 1050, buzzVols[i]*4);
    delayMicroseconds(20*64);
    vol.tone(SPEAKER, 1050, buzzVols[i] / 8*4);
    delayMicroseconds(40*64);
    i--;
  }

  delay(35);

  i = 0;
  while (i < 8) {
    int v = 0;
    while (v < 250) { // 12.5 mS fade up time
      vol.tone(SPEAKER, beeps[i], v*4);
      v += 10;
      delayMicroseconds(2*64);
    }
    delay(20);
    v = 250;
    while (v > 0) { // 12.5 mS fade down time
      vol.tone(SPEAKER, beeps[i], v*4);
      v -= 10;
      delayMicroseconds(5*64);
    }
    vol.noTone();
    delay(25);
    i++;
  }

  int f = 2466;
  while (f < 2825) {
    vol.tone(SPEAKER, f, 1023);
    f += 3;
    delay(1);
  }  
  f = 2825;
  int v = 255;
  while (f > 2000) {
    vol.tone(SPEAKER, f, v*4);
    f -= 6;
    v -= 1;
    delay(1);
  }
  vol.noTone();
  delay(35);

  i = 10;
  while (i > 0) {
    vol.tone(SPEAKER, 1050, buzzVols[i]*4);
    delayMicroseconds(20*64);
    vol.tone(SPEAKER, 1050, buzzVols[i] / 8*4);
    delayMicroseconds(40*64);
    i--;
  }
  vol.noTone();
}

void beCute() 
{
  if (hasHuman && (!isOnTimeout)) {
    stopAll();
    if (soundToMake == 0) {
      wolfWhistle();
    }
    else if (soundToMake == 1) {
      R2D2();
    }

    jiggle();
    
    soundToMake++;
    if (soundToMake > 1) {
      soundToMake = 0;
    }

    isOnTimeout = true;
  }
}

void loop()
{
  getObstacles();
  beCute();
  avoidObstacles();
  if (!hasObstacleLeft && !hasObstacleMiddle & !hasObstacleRight) {
    //go(FORWARD, 10);
  }
  timeOutCounter++;
  if (timeOutCounter > 2000) {
    timeOutCounter = 0;
    isOnTimeout = false;
  }
  delay(10);
}
