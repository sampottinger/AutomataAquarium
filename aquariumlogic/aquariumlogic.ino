#include "aquariumlogic.h"
#include <Servo.h>
#include <EEPROM.h>

Servo globalServos[NUM_LIM_ROT_SERVOS + NUM_CONT_ROT_SERVOS]; // Shared limited resource servo instance

ContinuousRotationServo contRotServos[NUM_CONT_ROT_SERVOS];
LimitedRotationServo limitedRotationServos[NUM_LIM_ROT_SERVOS];
PiezoSensor piezoSensors[NUM_PIEZO_SENSORS];
LightSensor lightSensors[NUM_LIGHT_SENSORS];
LEDAbstraction leds[NUM_LED];
Jellyfish jellyfish[NUM_JELLYFISH];
Fish fish[NUM_FISH];
PiezoSensorGroup piezoSensorGroups[NUM_PIEZO_SENSOR_GROUPS];
Aquarium aquariums[NUM_AQUARIUMS];

int globalStep;

void setup()
{
  int i;
  ContinuousRotationServo * crs;

  Serial.begin(9600);

  for(i=0; i<=13; i++)
  {
    pinMode(i, OUTPUT);
    digitalWrite(i, 0);
  }
  
  ls_init(0, 12);

  //crs_init(0, 4, 4, true);
  crs_init(0, 4, 4, false);
  //crs_init(1, 5, 5, true);
  crs_init(1, 5, 5, false);
  //crs_init(2, 6, 6, true);
  crs_init(2, 6, 6, false);
  //crs_init(3, 7, 7, true);
  //`crs_init(3, 7, 7, false);

  Serial.print("Finished initalization\n");
  //crs_setVelocity_(3, 100);

  //crs_setTargetVelocity(0, 5000);
  //crs_setTargetVelocity(1, 5000);
  //crs_setTargetVelocity(2, 5000);
  //crs_setTargetVelocity(3, 1000);
  fish_init(0, 0, 1, 2, 3);
  
  piezo_init(0, 0);
  piezo_init(1, 1);
  piezo_init(2, 13);
  piezo_init(3, 11);
  
  psg_init(0, 4);
  psg_addToSensorList(0, 0, SOUTHWEST);
  psg_addToSensorList(0, 1, NORTHWEST);
  psg_addToSensorList(0, 2, NORTHEAST);
  psg_addToSensorList(0, 3, SOUTHEAST);

  //crs_startMovingTo(0, 5000000);
  //crs_startMovingTo(1, 5000000);
  //crs_startMovingTo(2, 5000000);

  lrs_init(0, 9);
  led_init(0, 12);
  jellyfish_init(0, 0, 0);

  fish_setVelocity(0, 5000);
  fish_goTo(0, 50000000, 5000000, 5000000);

  aquarium_init(0, 0, 0, 0, 0);
}

void loop()
{
  delay(1);
  aquarium_tick(0, millis());

  //fish_step(0, 100);

  //crs_step(0, 100);
  //crs_step(1, 100);
  //crs_step(2, 100);
  //crs_step(3, 100);
}

ContinuousRotationServo * crs_getInstance(int id)
{
  return &(contRotServos[id]);
}

void crs_stop(int id)
{
  crs_setVelocity_(id, 0);
  crs_setTargetVelocity(id, 0);
}

void crs_init(int id, byte controlLine, byte potLine, boolean calibrate)
{
  ContinuousRotationServo * target = crs_getInstance(id);

  target->controlLine = controlLine;
  target->potLine = potLine;
  target->zeroValue = PRE_CALIBRATION_ZERO_VAL;
  target->position = PRE_CALIBRATION_POSITION;
  target->targetPosition = PRE_CALIBRATION_POSITION;
  target->targetVel = STARTING_TARGET_VELOCITY;
  target->velocitySlope = DEFAULT_VELOCITY_SLOPE;
  target->inTrustedArea = false;
  target->numMatchingVals = 0;
  target->correctionLastVal = analogRead(potLine);
  target->ownerID = NONE;
  target->ownerType = NONE;
  target->selfID = id;

  globalServos[NUM_LIM_ROT_SERVOS + id].attach(target->controlLine);

  if(calibrate)
  {
    crs_calibrate_(id);
    crs_saveCalibration_(id);
  }
  else
  {
    crs_loadCalibration_(id);
  }

  crs_setVelocity_(id, 0);
}

void crs_setOwner(int id, int type, int ownerID)
{
  ContinuousRotationServo * target = crs_getInstance(id);
  target->ownerID = ownerID;
  target->ownerType = type;
}

void crs_startMovingTo(int id, long targetPosition)
{
  ContinuousRotationServo * target = crs_getInstance(id);
  target->targetPosition = targetPosition;
  if(targetPosition > target->position)
  {
    target->decreasing = false;
    crs_setVelocity_(id, target->targetVel);
    /*Serial.print("Setting velocity to ");
     Serial.print(target->targetVel);
     Serial.print(" on ");
     Serial.print(id);
     Serial.print("\n");*/
  }
  else
  {
    target->decreasing = true;
    crs_setVelocity_(id, -target->targetVel);
    /*Serial.print("Setting velocity to ");
     Serial.print(-target->targetVel);
     Serial.print(" on ");
     Serial.print(id);
     Serial.print("\n");*/
  }
}

void crs_startMovingToAngle(int id, double angle)
{
  crs_startMovingTo(id, NUM_STEPS_PER_RAD * angle);
}

void crs_step(int id, long ms)
{
  long delta;
  boolean decreasing;
  ContinuousRotationServo * target;

  target = crs_getInstance(id);
  decreasing = target->decreasing;

  // Update expected position
  if(decreasing)
    target->position -= ms * target->targetVel;
  else
    target->position += ms * target->targetVel;

  // Attempt to correct with pot
  if(target->targetVel != 0)
    crs_correctPos_(id);

  // Check on delta
  delta = target->targetPosition - target->position;
  /*Serial.print("Delta: ");
   Serial.print(delta);
   Serial.print(" and velocity: ");
   Serial.print(target->targetVel);
   Serial.print("\n");*/
  if(decreasing && delta >= 0)
    crs_onGoalReached(id);
  else if(!decreasing && delta <= 0)
    crs_onGoalReached(id);
}

void crs_setTargetVelocity(int id, int targetVelocity)
{
  ContinuousRotationServo * target = crs_getInstance(id);
  target->targetVel = targetVelocity;
}

void crs_onGoalReached(int id)
{
  ContinuousRotationServo * target;
  target = crs_getInstance(id);

  /*Serial.print("Reached goal! Stop?");
   Serial.print("\n");*/
  crs_setVelocity_(id, 0);

  // Inform owner
  switch(target->ownerType)
  {
  case FISH_OWNER:
    fish_onServoGoalReached(target->ownerID, target->selfID);
    break;
  }
}

long crs_getPos(int id)
{
  ContinuousRotationServo * target = crs_getInstance(id);
  return target->position;
}

int crs_convertVelocityToRaw_(int id, float vel)
{
  ContinuousRotationServo * target = crs_getInstance(id);
  return vel * target->velocitySlope + target->zeroValue;
}

void crs_loadCalibration_(int id)
{
  int i;
  long position;
  CrsDto dto;
  byte * dtoPtr;
  ContinuousRotationServo * target;

  target = crs_getInstance(id);

  dtoPtr = (byte *)&dto;

  for(i = 0; i<sizeof(CrsDto); i++)
  {
    dtoPtr[i] = EEPROM.read(id * sizeof(CrsDto) + i);
  }
  target->position = dto.position;
  target->zeroValue = dto.zeroValue;
  target->velocitySlope = dto.velocitySlope;
}

void crs_saveCalibration_(int id)
{
  int i;
  long position;
  byte * dtoPtr;
  ContinuousRotationServo * target;
  CrsDto dto;

  target = crs_getInstance(id);
  dto.position = target->position;
  dto.zeroValue = target->zeroValue;
  dto.velocitySlope = target->velocitySlope;
  dtoPtr = (byte*)&dto;

  for(i = 0; i<sizeof(CrsDto); i++)
  {
    EEPROM.write(id * sizeof(CrsDto) + i, dtoPtr[i]);
  }
}

/*void crs_goToMatchingSection_(int id, int minVal, int maxVal, int reqNumReadings)
 {
 int numMatchingVals;
 int potVal;
 int potLine;
 ContinuousRotationServo * target;
 
 // Get common information loaded
 target = crs_getInstance(id);
 potLine = target->potLine;
 
 // Find section
 numMatchingVals = 0;
 while(numMatchingVals < reqNumReadings)
 {
 potVal = analogRead(potLine);
 if(minVal <= potVal && potVal <= maxVal)
 numMatchingVals++;
 else
 numMatchingVals = 0;
 delay(SHORT_CALIBRATION_DUR);
 }
 }*/

void crs_exhaustMatchingSection_(int id, int minVal, int maxVal)
{
  int potVal;
  int potLine;
  ContinuousRotationServo * target;

  // Get common information loaded
  target = crs_getInstance(id);
  potLine = target->potLine;

  // Exhaust section
  do
  {
    potVal = analogRead(potLine);
    delay(SHORT_CALIBRATION_DUR);
  }
  while(minVal <= potVal && potVal <= maxVal);
}

void crs_goToTrustedSection_(int id)
{
  int numMatchingVals;
  int potVal;
  int lastVal;
  int potLine;
  boolean increasing;
  boolean consistent;
  ContinuousRotationServo * target;

  // Get background information
  target = crs_getInstance(id);
  increasing = true;
  potLine = target->potLine;
  numMatchingVals = 0;

  // Run to linear section
  numMatchingVals = 0;
  lastVal = analogRead(potLine);
  increasing = true;
  while(numMatchingVals < REQUIRED_NUM_MATCHING_VALS_LOOSE)
  {
    delay(SHORT_CALIBRATION_DUR);
    potVal = analogRead(potLine);
    consistent = (increasing && potVal >= lastVal) || (!increasing && potVal <= lastVal);
    if(MIN_TRUSTED_VALUE <= potVal && potVal <= MAX_TRUSTED_VALUE && consistent)
      numMatchingVals++;
    else
    {
      numMatchingVals -= abs(potVal - lastVal)/2;
      if(numMatchingVals < 0)
        numMatchingVals = 0;
      increasing = potVal > lastVal;
    }
    lastVal = potVal;
  }
}

void crs_calibrate_(int id)
{
  long lastPos;
  int deltaPos;
  int potLine;
  int potVal;
  int lastVal;
  int currentVel;
  int numMatchingVals;
  //boolean consistent;
  //boolean increasing;
  boolean finished;
  float raw1;
  float speed1;
  float raw2;
  float speed2;
  int potVal1;
  int potVal2;
  float estimatedSlope;
  int estimatedZeroVal;
  float deltaSpeed;
  float estimatedVelocity;
  int i;
  ContinuousRotationServo * target;

  // Get common information loaded
  target = crs_getInstance(id);
  potLine = target->potLine;

  // Set small starting velocity
  lastPos = analogRead(potLine);
  currentVel = START_CALIBRATION_VEL;
  do
  {
    currentVel++;
    crs_setVelocity_(id, currentVel);
    delay(SHORT_CALIBRATION_DUR);
    deltaPos = analogRead(potLine) - lastPos;
  }
  while(abs(deltaPos) < 10);

  crs_goToTrustedSection_(id);

  // Newton's Method
  //avgSlope = 0;
  //numSlopesInAvg = 0;
  estimatedSlope = 1;
  estimatedZeroVal = 1500;
  estimatedVelocity = estimatedSlope * currentVel + estimatedZeroVal;

  // Observe at first speed
  potVal1 = analogRead(potLine);
  delay(100);
  potVal2 = analogRead(potLine);
  deltaPos = potVal2 - potVal1;
  speed1 = currentVel;
  raw1 = deltaPos; // / 300.0; // TODO: constant for slope find delay time (50)

  currentVel -= 3; // TODO: Constant

  finished = false;
  while(!finished)
  {
    // TODO: take care of duplicated code 
    crs_setVelocity_(id, currentVel);
    potVal1 = analogRead(potLine);
    delay(100);
    potVal2 = analogRead(potLine);
    deltaPos = potVal2 - potVal1;
    speed2 = currentVel;
    raw2 = deltaPos; // / 300.0; // TODO: constant for slope find delay time (50)

    estimatedSlope = (raw2 - raw1) / (speed2 - speed1);
    deltaSpeed = raw2 * CALIBRATION_CAUTIOUS_FACTOR / estimatedSlope;
    finished = abs(deltaSpeed) < 1;

    //avgSlope = (avgSlope * numSlopesInAvg + estimatedSlope) / (numSlopesInAvg + 1);
    //numSlopesInAvg++;

    if(finished)
      break;
    else
    {
      estimatedZeroVal = speed2 - deltaSpeed;
      raw1 = raw2;
      speed1 = speed2;
      currentVel = estimatedZeroVal;
    }
  }

  Serial.print("Newtons done?\n");

  // Finish with hill climbing
  // Change speed until delta position = 0
  // within clean section
  numMatchingVals = 0;
  while(numMatchingVals < REQUIRED_NUM_MATCHING_VALS)
  {
    lastVal = analogRead(potLine);
    delay(10);
    potVal = analogRead(potLine);
    deltaPos = potVal - lastVal;
    Serial.print(deltaPos);
    Serial.print("\n");

    if(deltaPos == 0)
    {
      numMatchingVals++;
    }
    else
    {
      if(deltaPos < 0)
        currentVel++;
      else
        currentVel--;
      if(currentVel > 40)
        currentVel = 40;
      else if(currentVel < -40)
        currentVel = -40;

      numMatchingVals = 0;

      crs_setVelocity_(id, currentVel);
    }
  }

  // Update zero value
  target->zeroValue += target->velocitySlope * currentVel;

  // Determine velocity conversion slope
  crs_setVelocity_(id, SLOPE_FINDING_VEL_1);
  crs_exhaustMatchingSection_(id, MIN_TRUSTED_VALUE, MAX_TRUSTED_VALUE);
  crs_goToTrustedSection_(id);

  potVal1 = analogRead(potLine);
  delay(SLOPE_FINDING_DUR);
  potVal2 = analogRead(potLine);
  deltaPos = potVal2 - potVal1;
  speed1 = SLOPE_FINDING_VEL_1;
  raw1 = deltaPos / (float)SLOPE_FINDING_DUR;

  crs_setVelocity_(id, SLOPE_FINDING_VEL_2);
  potVal1 = analogRead(potLine);
  delay(SLOPE_FINDING_DUR);
  potVal2 = analogRead(potLine);
  deltaPos = potVal2 - potVal1;
  speed2 = SLOPE_FINDING_VEL_2;
  raw2 = deltaPos / (float)SLOPE_FINDING_DUR;

  estimatedSlope = (raw2 - raw1) / (speed2 - speed1);
  target->velocitySlope = estimatedSlope;

  // Stop
  crs_setVelocity_(id, 0);
  crs_setTargetVelocity(id, 0);
}

void crs_setVelocity_(int id, int velocity)
{
  int convertedVelocity;

  ContinuousRotationServo * target = crs_getInstance(id);

  convertedVelocity = crs_convertVelocityToRaw_(id, velocity);
  globalServos[NUM_LIM_ROT_SERVOS + id].writeMicroseconds(convertedVelocity);
}

void crs_correctPos_(int id)
{
  ContinuousRotationServo * target = crs_getInstance(id);
  int currentVal = analogRead(target->potLine);
  int lastVal = target->correctionLastVal;
  boolean increasing = !target->decreasing;
  int numMatchingVals = target->numMatchingVals;
  boolean consistent;
  int numStepsIntoRot;
  int deltaSteps;

  // If in trusted zone, make sure we are still there and correct pos
  if(target->inTrustedArea)
  {
    Serial.print("Here!");
    Serial.print("\n");

    // Make sure we are still in trusted range
    if(currentVal >= MIN_TRUSTED_VALUE && currentVal <= MAX_TRUSTED_VALUE)
    {
      numStepsIntoRot = target->position % NUM_STEPS_ROT;
      deltaSteps = currentVal - numStepsIntoRot;
      target->position += deltaSteps;
    }
    else
    {
      target->inTrustedArea = false;
    }
  }
  // Otherwise, see if we are getting there
  else
  {
    // See if we have a matching value
    consistent = (increasing && currentVal >= lastVal) || (!increasing && currentVal <= lastVal);
    if(MIN_TRUSTED_VALUE <= currentVal && currentVal <= MAX_TRUSTED_VALUE && consistent)
    {
      numMatchingVals++;
      Serial.print("Num matching vals:");
      Serial.print(numMatchingVals);
      Serial.print("\n");
    }
    else
    {
      numMatchingVals -= abs(currentVal - lastVal)/2;
      if(numMatchingVals < 0)
        numMatchingVals = 0;
    }
    target->correctionLastVal = currentVal;

    // Test to see if we made it
    target->inTrustedArea = numMatchingVals > REQUIRED_NUM_MATCHING_VALS_LOOSE;
    target->numMatchingVals = numMatchingVals;
  }
}

LimitedRotationServo * lrs_getInstance(int id)
{
  return &(limitedRotationServos[id]);
}

void lrs_init(int id, byte controlLine)
{
  LimitedRotationServo * target = lrs_getInstance(id);
  globalServos[id].attach(controlLine);
  target->controlLine = controlLine;
}

void lrs_setAngle(int id, int angle)
{
  LimitedRotationServo * target = lrs_getInstance(id);
  int line = target->controlLine;

  globalServos[id].write(angle);
}

void lrs_step(int id, long ms)
{

}

PiezoSensor * piezo_getInstance(int id)
{
  return &(piezoSensors[id]);
}

void piezo_init(int id, byte line)
{
  PiezoSensor * target = piezo_getInstance(id);
  target->line = line;
  target->fired = NO_TAP;
}

int piezo_isFired(int id)
{
  PiezoSensor * target = piezo_getInstance(id);
  
  int ret_val = target->fired;
  target->fired = NO_TAP;
  return ret_val;
}

void piezo_onTick(int id)
{
  PiezoSensor * target = piezo_getInstance(id);
  int line = target->line;

  int val = analogRead(line);

  //keep track of max fired val in target->fired
  //if(val >= PIEZO_MIN_TAP_VAL && val > target->fired)
  //  target->fired = val;
}

LightSensor * ls_getInstance(int id)
{
  return &(lightSensors[id]);
}

void ls_init(int id, byte line)
{
  LightSensor * target = ls_getInstance(id);
  target->line = line;
  target->isLight = analogRead(target->line) > MIN_LIGHT_VAL;
}

boolean ls_isLight(int id)
{
  boolean isLight;
  
  LightSensor * target = ls_getInstance(id);
  if(target->isLight)
    isLight = analogRead(target->line) > MIN_LIGHT_VAL;
  else
    isLight = analogRead(target->line) > MIN_LIGHT_RECOVERY_VAL;
  
  target->isLight = isLight;
  return isLight;
}

LEDAbstraction * led_getInstance(int id)
{
  return &(leds[id]);
}

void led_init(int id, byte line)
{
  LEDAbstraction * target = led_getInstance(id);
  target->line = line;
  /*Serial.print("Initing with ");
   Serial.print(target->line);
   Serial.print("\n");*/
  pinMode(line, OUTPUT);
}

void led_turnOn(int id)
{
  int line;
  LEDAbstraction * target;

  target = led_getInstance(id);
  line = target->line;
  //Serial.print(line);
  digitalWrite(line, 1);
}

void led_turnOff(int id)
{
  int line;
  LEDAbstraction * target;

  target = led_getInstance(id);
  line = target->line;
  //Serial.print(line);
  digitalWrite(line, 0);
}

Jellyfish * jellyfish_getInstance(int id)
{
  return &(jellyfish[id]);
}

void jellyfish_init(int id, int servoNum, int ledNum)
{
  // Save properties
  Jellyfish * jellyfish = jellyfish_getInstance(id);
  jellyfish->servoNum = servoNum;
  jellyfish->ledNum = ledNum;
  jellyfish_raise(id);
}

void jellyfish_lower(int id)
{
  Jellyfish * jellyfish = jellyfish_getInstance(id);
  lrs_setAngle(jellyfish->servoNum, JELLYFISH_LOWERED_ANGLE);
  led_turnOn(jellyfish->ledNum);
}

void jellyfish_raise(int id)
{
  Jellyfish * jellyfish = jellyfish_getInstance(id);
  lrs_setAngle(jellyfish->servoNum, JELLYFISH_RAISED_ANGLE);
  led_turnOff(jellyfish->ledNum);
}

void jellyfish_step(int id, long ms)
{
  Jellyfish * jellyfish = jellyfish_getInstance(id);
  lrs_step(jellyfish->servoNum, ms);
}

Fish * fish_getInstance(int id)
{
  return &(fish[id]);
}

void fish_stop(int id)
{
  Fish * targetFish = fish_getInstance(id);

  // Save servo nums
  crs_stop(targetFish->xServo);
  crs_stop(targetFish->yServo);
  crs_stop(targetFish->zServo);
}

void fish_init(int id, int xServoNum, int yServoNum, int zServoNum, int thetaServo)
{
  Fish * targetFish = fish_getInstance(id);

  // Save servo nums
  targetFish->xServo = xServoNum;
  crs_setOwner(xServoNum, FISH_OWNER, id);
  targetFish->yServo = yServoNum;
  crs_setOwner(yServoNum, FISH_OWNER, id);
  targetFish->zServo = zServoNum;
  crs_setOwner(zServoNum, FISH_OWNER, id);
  targetFish->thetaServo = thetaServo;
  targetFish->numWaitingServos = 0;

  // Start going home
  fish_goTo(id, 0, 0, 0);
}

void fish_goTo(long id, long targetX, long targetY, long targetZ)
{
  long currentX;
  long currentY;
  long currentZ;
  long deltaX;
  long deltaY;
  long deltaZ;
  double targetTheta;
  byte limitingAxis;
  float limitingAxisDistance;

  Fish * target = fish_getInstance(id);

  // Update target position
  /*target->targetX = targetX;
   target->targetY = targetY;
   target->targetZ = targetZ;
   
   // Get current position
   currentX = crs_getPos(target->xServo);
   currentY = crs_getPos(target->yServo);
   currentZ = crs_getPos(target->zServo);
   
   // Find vector
   deltaX = targetX - currentX;
   deltaY = targetY - currentY;
   deltaZ = targetZ - currentZ;
   
   // Determine delta
   targetTheta = atan(deltaY / deltaX);
   
   // Adjust for second and third quadrants
   if(deltaX < 0)
   {
   if(deltaY > 0) // Second quadrant
   targetTheta += M_PI / 2;
   else if(deltaY < 0) // Third quadrant
   targetTheta += M_PI;
   }
   
   // Change orientation as quickly as possible
   crs_startMovingToAngle(target->thetaServo, targetTheta);
   
   // Determine limiting axis for this goal
   if(deltaX > deltaY)
   limitingAxisDistance = deltaX;
   else
   limitingAxisDistance = deltaY;
   
   if(limitingAxisDistance < deltaZ)
   limitingAxisDistance = deltaZ;
   
   // Determine ratios
   target->xSpeedPortion = deltaX / limitingAxisDistance;
   target->ySpeedPortion = deltaY / limitingAxisDistance;
   target->zSpeedPortion = deltaZ / limitingAxisDistance;
   
   // Reset substeps and determine substep duration
   target->subStepsLeftToGoal = FISH_SUB_STEPS_TO_GOAL;
   
   // Save starting position and time
   target->startX = currentX;
   target->startY = currentY;
   target->startZ = currentZ;
   target->startMS = millis();
   
   // Save angle
   target->moveAngle = targetTheta;*/

  // Start off to first positional subgoal
  //fish_goToNextInternalGoal_(id); // TODO: Cos wiggle
  crs_setTargetVelocity(target->xServo, target->velocity);
  crs_startMovingTo(target->xServo, targetX);
  crs_setTargetVelocity(target->yServo, target->velocity);
  crs_startMovingTo(target->yServo, targetY);
  crs_setTargetVelocity(target->zServo, target->velocity);
  crs_startMovingTo(target->zServo, targetZ);

  // We are waiting on a few servos
  target->numWaitingServos = 4;
}

void fish_onServoGoalReached(int id, int servoID)
{
  Fish * target = fish_getInstance(id);
  target->numWaitingServos--;
  if(target->numWaitingServos == 0)
    fish_onGoalReached(id);
}

void fish_onGoalReached(int id)
{
  Fish * target = fish_getInstance(id);

  Serial.print("Here :(\n");

  // Determine if subgoal or actual goal
  target->subStepsLeftToGoal--;
  //if(target->subStepsLeftToGoal <= 0)
  aquarium_onFishReachedGoal(AQUARIUM_ID, id); // Tell system
  /*else 
   fish_goToNextInternalGoal_(id); // Next subgoal*/
}

void fish_goToNextInternalGoal_(int id)
{
  Fish * target;
  long curTime;
  long nextTime;
  double nextTimeSec;
  double wiggleOffset;
  double xWiggleOffset;
  double yWiggleOffset;
  double velAngle;
  long newGoalGeneral;
  long newGoalX;
  long newGoalY;
  long newGoalZ;
  int overallVelocity;

  // Get target, time, and overall velocity
  target = fish_getInstance(id);
  curTime = millis() - target->startMS;
  nextTime = curTime + LONG_TIME_STEP;
  nextTimeSec = nextTime / 1000.0;
  overallVelocity = target->velocity;

  // Determine wiggle offsets
  velAngle = target->moveAngle;
  wiggleOffset = WIGGLE_AMPLITUDE * sin(nextTime * WIGGLE_SPEED);
  xWiggleOffset = cos(velAngle) * wiggleOffset;
  yWiggleOffset = sin(velAngle) * wiggleOffset;

  // Common computation
  newGoalGeneral = (long)(overallVelocity * nextTimeSec);

  // Update x goal
  newGoalX = (long)(newGoalGeneral * target->xSpeedPortion  + target->startX);
  newGoalX += xWiggleOffset;
  crs_startMovingTo(target->xServo, newGoalX);

  // Update y goal
  newGoalY = (long)(newGoalGeneral * target->ySpeedPortion + target->startY);
  newGoalY += yWiggleOffset;
  crs_startMovingTo(target->yServo, newGoalY);

  // Update z goal
  newGoalZ = (long)(newGoalGeneral * target->zSpeedPortion + target->startZ);
  crs_startMovingTo(target->zServo, newGoalZ);
}

void fish_step(int id, long ms)
{
  Fish * target = fish_getInstance(id);
  crs_step(target->xServo, ms);
  crs_step(target->yServo, ms);
  crs_step(target->zServo, ms);
  //crs_step(target->thetaServo, ms);
}

void fish_setVelocity(int id, float velocity)
{
  Fish * target = fish_getInstance(id);
  target->velocity = velocity;
  /*crs_setTargetVelocity(target->xServo, velocity);
   crs_setTargetVelocity(target->yServo, velocity);
   crs_setTargetVelocity(target->zServo, velocity);*/
}

PiezoSensorGroup * psg_getInstance(int id)
{
  return &(piezoSensorGroups[id]);
}

void psg_init(int id, int numSensors)
{
  PiezoSensorGroup * target = psg_getInstance(id);
  target->numSensors = numSensors;
  target->nextElementIndex = 0;
  target->sensorNums = (SensorGroupMembershipRecord *)(malloc(numSensors * sizeof(SensorGroupMembershipRecord)));
}

void psg_dest(int id)
{
  PiezoSensorGroup * target = psg_getInstance(id);
  free(target->sensorNums);
}

void psg_addToSensorList(int id, int sensorID, int sensorHighLevelID)
{
  PiezoSensorGroup * target = psg_getInstance(id);
  int nextElementIndex = target->nextElementIndex;

  SensorGroupMembershipRecord * record = &(target->sensorNums[nextElementIndex]);
  record->sensorID = sensorID;
  record->highLevelID = sensorHighLevelID;

  target->nextElementIndex++;
}

int psg_getTapped(int id)
{
  int i;
  int sensorID;
  int sensorVal;
  int maxSensorRecordIndex;
  int maxSensorVal;
  PiezoSensorGroup * target = psg_getInstance(id);

  maxSensorRecordIndex = NONE;
  maxSensorVal = 0;
  for(i = 0; i < target->nextElementIndex; i++)
  {
    sensorID = target->sensorNums[i].sensorID;
    sensorVal = piezo_isFired(sensorID);
    if(sensorVal > 0 && sensorVal > maxSensorVal)
    {
      maxSensorVal = sensorVal;
      maxSensorRecordIndex = i;
    }
  }

  if(maxSensorRecordIndex == NONE)
    return NONE;
  else
    return target->sensorNums[maxSensorRecordIndex].highLevelID;
}

void psg_onTick(int id)
{
  int i;
  int sensorID;
  
  PiezoSensorGroup * target = psg_getInstance(id);
  
  for(i = 0; i < target->nextElementIndex; i++)
  {
    sensorID = target->sensorNums[i].sensorID;
    piezo_onTick(sensorID);
  }
}

Aquarium * aquarium_getInstance(int id)
{
  return &(aquariums[id]);
}

void aquarium_init(int id, int fishNum, int jellyfishNum, int lightSensorNum,
int piezoSensorGroupNum)
{
  Aquarium * target = aquarium_getInstance(id);

  globalStep = 0;

  // Save simple attributes
  target->fishNum = fishNum;
  target->jellyfishNum = jellyfishNum;
  target->lightSensorNum = lightSensorNum;
  target->piezoSensorGroupNum = piezoSensorGroupNum;
  target->isLight = true;

  // Setup necessary state for time keeping
  target->lastMS = millis();
  target->shortMSRemain = SHORT_TIME_STEP;
  target->longMSRemain = LONG_TIME_STEP;
}

void aquarium_shortStep(int id, long ms)
{
  int tappedSensor;
  bool curLight;
  Aquarium * target;
  PiezoSensorGroup * piezoGroup;
  LightSensor * lightSensor;

  // Get necessary instances
  target = aquarium_getInstance(id);

  // Check sensors
  tappedSensor = psg_getTapped(target->piezoSensorGroupNum);
  curLight = ls_isLight(target->lightSensorNum);
  Serial.print("isLight:");
  Serial.print(target->lightSensorNum);
  Serial.print("\n");

  // Respond to tap
  if(tappedSensor != NONE)
      aquarium_runFishToOpposingSide_(id, tappedSensor);

  // Repond to light
  if(curLight != target->isLight) // If the light sensor state has changed
  {
    target->isLight = curLight;

    if(curLight)
      aquarium_transitionToFishState_(id);
    else
      aquarium_transitionToJellyfishState_(id);
  }
}

void aquarium_longStep(int id, long ms)
{
  Aquarium * target = aquarium_getInstance(id);

  jellyfish_step(target->jellyfishNum, ms);
  fish_step(target->fishNum, ms);
}

/*void aquarium_shortStep(int id, long ms)
 {
 boolean isLight;
 Aquarium * target = aquarium_getInstance(id);
 isLight = ls_isLight(target->lightSensorNum);
 
 if(isLight != target->isLight)
 {
 if(isLight)
 aquarium_transitionToJellyfishState(id);
 else
 aquarium_transitionToFishState(id);
 target->isLight = isLight;
 }
 }*/

void aquarium_transitionToFishState_(int id)
{
  Aquarium * target = aquarium_getInstance(id);
  jellyfish_raise(target->jellyfishNum);
  aquarium_onFishReachedGoal(id, target->fishNum);
}

void aquarium_transitionToJellyfishState_(int id)
{
  Serial.print("Jellyfish?\n");
  Aquarium * target = aquarium_getInstance(id);
  fish_goTo(0, 0, 0, 1000000000);
  jellyfish_lower(target->jellyfishNum);
}

void aquarium_tick(int id, long newMS)
{
  Aquarium * target;
  long deltaMS;
  long newShortMSRemain;
  long newLongMSRemain;

  target = aquarium_getInstance(id);
  deltaMS = newMS - target->lastMS;
  newShortMSRemain = target->shortMSRemain - deltaMS;
  newLongMSRemain = target->longMSRemain - deltaMS;

  // Check if long step was fired
  if(newLongMSRemain < 0)
  {
    Serial.print("Long step \n");
    aquarium_longStep(id, LONG_TIME_STEP - newLongMSRemain);
    newLongMSRemain = LONG_TIME_STEP;
  }
  target->longMSRemain = newLongMSRemain;

  // Check if short step was fired
  if(newShortMSRemain < 0)
  {
    Serial.print("Short step \n");
    aquarium_shortStep(id, SHORT_TIME_STEP - newShortMSRemain);
    newShortMSRemain = SHORT_TIME_STEP;
  }
  target->shortMSRemain = newShortMSRemain;

  target->lastMS = newMS;
  
  //perform events for every tick
  psg_onTick(target->piezoSensorGroupNum);
}

void aquarium_onFishReachedGoal(int id, int fishID)
{
  Aquarium * target;
  target = aquarium_getInstance(id);
  if(target->isLight)
  {
    globalStep++;
    globalStep %= 3;
    fish_setVelocity(target->fishNum, 5000);
    switch(globalStep)
    {
      case 0:
        fish_goTo(target->fishNum, 0, 0, 0);
        break;
      case 1:
        fish_goTo(target->fishNum, 50000000, 5000000, 5000000);
        break;
      case 2:
        fish_goTo(target->fishNum, 100000000, 0, 0);
        break;
    }
  }
  else
  {
    fish_stop(target->fishNum);
  }
}

void aquarium_runFishToOpposingSide_(int id, int tappedSensor)
{
  Aquarium * target;
  target = aquarium_getInstance(id);
  
  fish_setVelocity(0, 10000);
  switch(tappedSensor)
  {
    case NORTHEAST:
    case SOUTHEAST:
      fish_goTo(target->fishNum, -100000000, 0, 0);
      break;
    
    case SOUTHWEST:
    case NORTHWEST:
      fish_goTo(target->fishNum, 100000000, 0, 0);
      break;
  }
}

