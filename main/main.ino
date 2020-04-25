// Motors & PWM Speed Control：
int E1 = 5;
int M1 = 4; // Right motors
int E2 = 6;
int M2 = 7; // Left motors

// Obstacle detector inputs
int OBSTACLE_LEFT = 8;
int OBSTACLE_MIDDLE = 9;
int OBSTACLE_RIGHT = 10;

// Movement defintions
int FORWARD = 1;
int BACKWARD = 0;
int MAX_SPEED = 250; // 0 to 255, where 255 is the fastest

int TURN_QUARTER = 45;
int TURN_HALF = 90;
int TURN_THREE_QUARTERS = 135;
int TURN_FULL = 180;

int currentDirection = FORWARD;
int currentSpeed = 0;
int hasObstacleLeft = false;
int hasObstacleMiddle = false;
int hasObstacleRight = false;

void setup()
{
    //Motors
    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);

    // Obtacle detectors
    pinMode(OBSTACLE_LEFT, INPUT);
    pinMode(OBSTACLE_MIDDLE, INPUT);
    pinMode(OBSTACLE_RIGHT, INPUT);

    stopAll();

    Serial.begin(9600);
}

void getObstacles() 
{
  // Not sure why, but pins are inverted
  hasObstacleLeft = !digitalRead(OBSTACLE_LEFT);
  hasObstacleMiddle = !digitalRead(OBSTACLE_MIDDLE); 
  hasObstacleRight = !digitalRead(OBSTACLE_RIGHT);

  if (hasObstacleLeft)
    Serial.println("OBSTACLE LEFT");
  if (hasObstacleMiddle)
    Serial.println("OBSTACLE MIDDLE");
  if (hasObstacleRight)
    Serial.println("OBSTACLE RIGHT");
  
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

void loop()
{
  getObstacles();
  avoidObstacles();
  if (!hasObstacleLeft && !hasObstacleMiddle & !hasObstacleRight) {
    go(FORWARD, 50);
  }
}
