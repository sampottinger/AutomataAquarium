/**
 * Name: cont_rot_servo.ino
 * Desc: Logic for controlling and calibrating continuous rotation servos
 * Auth: Sam Pottinger, DJ Sutton, Jessica Ebert
**/

#define NUM_CONTINUOUS_ROT_SERVOS 1

#define DEFAULT_ROUGH_ZERO_VAL 1500
#define REQUIRED_NUM_ZERO_VALS 200
#define RUNAWAY_CALIBRATION_TOLLERANCE 20

#define INCREASING 1
#define NEUTRAL 0
#define DECREASING -1

#define TIME_STEP 100
#define ROTATION_STEPS 1024

#define MIN_TRUSTED_SERVO_VAL 500
#define MAX_TRUSTED_SERVO_VAL 700

#define POSITION_TOLLERANCE 200

#include <Servo.h>

Servo globalServo;

// Abstraction and instances of continuous rotation servos

struct ContinuousRotationServo
{
  long currentPos;
  long targetPos;
  int velocity;
  int zeroVal;
  int velMultiplier;
  int controlLine;
  int potLine;
  int targetVelocity;
  boolean paused;
};

ContinuousRotationServo contRotServos[NUM_CONTINUOUS_ROT_SERVOS];

// Logic for continuous rotation servo

/**
 * Name: crs_getInstance(int id)
 * Desc: Get an instance of the ContinuousRotationServo
 * Para: id, The unique id of the servo to retrieve
**/
struct ContinuousRotationServo * crs_getInstance(int id)
{
  return &(contRotServos[id]);
}

/**
 * Name: crs_init(int id)
 * Desc: Constructor (with calibration) for continuous rotation servo
 * Para: id, The unique intenger id of the servo to construct
**/
void crs_init(int id, int controlLine, int potLine)
{
  // Set default values
  ContinuousRotationServo * target = crs_getInstance(id);
  target->currentPos = 0;
  target->targetPos = 0;
  target->velocity = 0;
  target->zeroVal = DEFAULT_ROUGH_ZERO_VAL;
  target->velMultiplier = 1;
  target->controlLine = controlLine;
  target->potLine = potLine;
  target->targetVelocity = 0;
  target->paused=false;
  
  // Set up for calibration
  int stepsTaken = 0;
  int velocityOffset = 0;
  int numZeroVals = 0;
  int previousPotRead = analogRead(potLine);
  int newPotRead;
  int delta;
  int runawayVals;
  int runningDirection = NEUTRAL;
  
  // Start finding zero position
  while(numZeroVals < REQUIRED_NUM_ZERO_VALS)
  {
    // Get the latest pot reading
    newPotRead = analogRead(potLine);
    
    // Determine in which direction to travel
    if(newPotRead > previousPotRead)
    {
      numZeroVals = 0;
      velocityOffset--;
      
      if(runningDirection == DECREASING)
        runawayVals++;
      else
        runawayVals=0;
    }
    else if(newPotRead < previousPotRead)
    {
      numZeroVals = 0;
      velocityOffset++;
      
      if(runningDirection == INCREASING)
        runawayVals++;
      else
        runawayVals=0;
    }
    else
    {
      numZeroVals++;
    }
    
    // Update velocity and turn around if necessary
    crs_setVelocity(id, velocityOffset);
    
    if(runawayVals >= RUNAWAY_CALIBRATION_TOLLERANCE)
    {
      runawayVals = 0;
      target->velMultiplier = target->velMultiplier * -1;
    }
    
    // Update step count and expire previous pot reading
    stepsTaken = previousPotRead - newPotRead;
    previousPotRead = newPotRead;
  }
  
  // Apply offset and update position
  target->zeroVal = target->zeroVal + velocityOffset;
  target->currentPos = stepsTaken;
  
  // Set down velocity
  crs_setVelocity(id, 0);
  target->paused=true;
}

/**
 * Name: crs_convertVelToRaw(int servoNum, int val)
 * Desc: Converts the given human readable velocity
 *       to a velocity that can be given to the servos
 * Para: val, The velocity (steps / sec) desired
 * Retr: Raw value to use with servos
**/
int crs_convertVelToRaw(int servoNum, int val)
{
  ContinuousRotationServo * target = crs_getInstance(servoNum);
  int zeroVal = target->zeroVal;
  float retVal = val * target->velMultiplier + zeroVal;
  return (int)retVal;
}

/**
 * Name: crs_setVelocity(int servoNum, int val)
 * Desc: Sets the velocity of a continuous rotation servo
 * Para: servoNum, the number of the servo to set the velocity of
 *       val, The velocity to set the servo to (steps per sec)
**/
void crs_setVelocity(int servoNum, int val)
{
  ContinuousRotationServo * target = crs_getInstance(servoNum);
  globalServo.attach(target->controlLine);
  globalServo.write(crs_convertVelToRaw(servoNum, val));
  target->velocity = val;
}

/**
 * Name: crs_setTargetVelocity(int servoNum, int val)
 * Desc: Sets the velocity the given servo should use to get to a
 *       goal
 * Para: servoNum, The id of the servo whose velocity is to be set
 *       val, The velocity this servo should use to get to goal
 *            positions
**/
void crs_setTargetVelocity(int servoNum, int val)
{
  ContinuousRotationServo * target = crs_getInstance(servoNum);
  target->targetVelocity = val;
}

/**
 * Name: crs_startMovingTo(int servoNum, int targetPosition)
 * Desc: Tells this servo to start moving to a given position
 * Para: servoNum, The id of the servo whose goal position is to be set
 *       targetPosition, the position to which this servo should travel
**/
void crs_startMovingTo(int servoNum, long targetPosition)
{
  ContinuousRotationServo * target = crs_getInstance(servoNum);
  target->targetPos = targetPosition;
  target->paused = false;
}

/**
 * Name: crs_setp(int servoNum)
 * Desc: Updates this servo's velocity, stopping if necessary
 * Para: servoNum, The servo to check on
**/
void crs_step(int servoNum)
{
  int velocity;
  long currentPos;
  long targetPos;
  boolean finished;
  int inRotationPos;
  int stepAdj;
  int potRotationPos;
  ContinuousRotationServo * target;
 
  // Parse vals
  target = crs_getInstance(servoNum);
  velocity = target->velocity;
  currentPos = target->currentPos;
  targetPos = target->targetPos;
  
  // Update position 
  // TODO: A little kludgey
  currentPos = currentPos + velocity;
  potRotationPos = analogRead(target->potLine);
  if(potRotationPos >= MIN_TRUSTED_SERVO_VAL || potRotationPos <= MAX_TRUSTED_SERVO_VAL)
  {
    inRotationPos = abs(currentPos) % ROTATION_STEPS;
    stepAdj = potRotationPos - inRotationPos;
    currentPos = currentPos + stepAdj; // TODO: is this right?
  }
  target->currentPos = currentPos;
  
  // If we are still, make sure we are at goal position
  if(velocity == 0 && targetPos != currentPos)
  {
    if(targetPos > currentPos)
      crs_setVelocity(servoNum, target->targetVelocity);
    else
      crs_setVelocity(servoNum, -target->targetVelocity);
  }
  
  // Otherwise make sure we have not overstepped goal
  else
  {
    finished = velocity < 0 && targetPos >= currentPos;
    finished = finished || (velocity > 0 && targetPos <= currentPos);
    if(finished)
    {
      crs_setVelocity(servoNum, 0);
      target->targetPos = currentPos;
      target->paused = true;
    }
  }
}

void setup()
{
  Serial.begin(9600);
  crs_init(0, 9, 0);
  crs_setTargetVelocity(0, 100);
  crs_startMovingTo(0, -2000);
}

void loop()
{
  long time;
  long startTime;
  int potval;
  int startPotVal;
  /*long waitTime;
  int delta;
  int adjustment;
  long startTime = millis();
  crs_step(0);
  delta = millis() - startTime;
  adjustment = TIME_STEP - delta;
  if(adjustment > 0)
    delay(adjustment);*/
  int velocity;
  velocity = 15;
  
  potval = analogRead(0);
  
  Serial.print("Starting!\n");
  
  
  /*crs_setVelocity(0, velocity);
  while(1)
  {
    Serial.print(analogRead(0));
    Serial.print("\n");
  }*/
  
  while(velocity <= 1000)
  {
    // Get to start of rotation
    crs_setVelocity(0, velocity);
    while(potval > 25)
      potval = analogRead(0);
    while(potval < 1000)
      potval = analogRead(0);
    while(potval > 512)
    {
      potval = analogRead(0);
      time = millis();
    }
    startPotVal = potval;
    startTime = time;
    for(int i = 0; i < 10; i++)
    {
      crs_setVelocity(0, velocity);
      while(potval > 25)
        potval = analogRead(0);
      while(potval < 1000)
        potval = analogRead(0);
      while(potval > 512)
      {
        potval = analogRead(0);
        time = millis();
        //Serial.print(potval);
        //Serial.print("\n");
      }
      Serial.print(startPotVal);
      Serial.print(" => ");
      Serial.print(potval);
      Serial.print(" in ");
      Serial.print(time - startTime);
      Serial.print(" at ");
      Serial.print(velocity);
      Serial.print("\n");
      startPotVal = potval;
      startTime = time;
    }
    velocity++;
  }
}
