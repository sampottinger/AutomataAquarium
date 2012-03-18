#include "aquariumlogic.h"
#include <Servo.h>
#include <EEPROM.h>

Servo globalServo; // Shared limited resource servo instance

ContinuousRotationServo contRotServos[NUM_CONT_ROT_SERVOS];
LimitedRotationServo limitedRotationServos[NUM_LIM_ROT_SERVOS];
PiezoSensor piezoSensors[NUM_PIEZO_SENSORS];
LightSensor lightSensors[NUM_LIGHT_SENSORS];
LEDAbstraction leds[NUM_LED];
Jellyfish jellyfish[NUM_JELLYFISH];
Fish fish[NUM_FISH];
PiezoSensorGroup piezoSensorGroups[NUM_PIEZO_SENSOR_GROUPS];
Aquarium aquariums[NUM_AQUARIUMS];

void setup()
{
  Serial.begin(9600);
  crs_init(0, 9, 0, true);
}

void loop()
{
}

ContinuousRotationServo * crs_getInstance(int id)
{
  return &(contRotServos[id]);
}

void crs_init(int id, byte controlLine, byte potLine, boolean zero)
{
  ContinuousRotationServo * target = crs_getInstance(id);

  target->controlLine = controlLine;
  target->potLine = potLine;
  target->zeroValue = PRE_CALIBRATION_ZERO_VAL;
  target->position = PRE_CALIBRATION_POSITION;
  target->targetPosition = PRE_CALIBRATION_POSITION;
  target->targetVel = STARTING_TARGET_VELOCITY;
  target->velocitySlope = DEFAULT_VELOCITY_SLOPE;

  if(!zero)
    crs_loadPosition_(id);

  crs_calibrate_(id);
}

void crs_startMovingTo(int id, long targetPosition)
{
  ContinuousRotationServo * target = crs_getInstance(id);
  target->targetPosition = targetPosition;
  if(targetPosition > target->position)
  {
    target->decreasing = false;
    crs_setVelocity_(id, -target->targetVel);
  }
  else
  {
    target->decreasing = true;
    crs_setVelocity_(id, target->targetVel);
  }
}

void crs_startMovingToAngle(int id, double angle)
{
  crs_startMovingTo(id, NUM_STEPS_PER_RAD * angle);
}

void crs_step(int id, long ms)
{
  long delta;
  ContinuousRotationServo * target;

  //crs_correct_pos_(id);

  target = crs_getInstance(id);
  delta = target->targetPosition - target->position;

  if(target->decreasing && delta >= 0)
    crs_onGoalReached(id);
  else if(!target->decreasing && delta <= 0)
    crs_onGoalReached(id);
}

void crs_setTargetVelocity(int id, int targetVelocity)
{
  ContinuousRotationServo * target = crs_getInstance(id);
  target->targetVel = targetVelocity;
}

void crs_onGoalReached(int id)
{
  crs_setVelocity_(id, 0);
}

long crs_getPos(int id)
{
  ContinuousRotationServo * target = crs_getInstance(id);
  return target->position;
}

int crs_convertVelocityToRaw_(int id, float vel)
{
  ContinuousRotationServo * target = crs_getInstance(id);
  Serial.print("Velocity slope: ");
  Serial.print(target->velocitySlope);
  Serial.print("\n");
  return vel * target->velocitySlope + target->zeroValue;
}

void crs_loadPosition_(int id)
{
  int i;
  long position;
  byte * positionPtr;
  ContinuousRotationServo * target;

  target = crs_getInstance(id);

  positionPtr = (byte*)&position;

  for(i = 0; i<SIZE_OF_POSITION_VAL; i++)
    positionPtr[i] = EEPROM.read(id * SIZE_OF_POSITION_VAL + i);

  target->position = position;
}

void crs_savePosition_(int id)
{
  int i;
  long position;
  byte * positionPtr;
  ContinuousRotationServo * target;

  target = crs_getInstance(id);
  position = target->position;
  positionPtr = (byte*)&position;

  for(i = 0; i<SIZE_OF_POSITION_VAL; i++)
    EEPROM.write(id * SIZE_OF_POSITION_VAL + i, positionPtr[i]);
}

void crs_goToMatchingSection_(int id, int minVal, int maxVal, int reqNumReadings)
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
}

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
  while(MIN_HIGH_VAL <= potVal && potVal <= MAX_HIGH_VAL);
}

void crs_calibrate_(int id)
{
  long lastPos;
  int deltaPos;
  int potLine;
  int potVal;
  int deltaTotals;
  int lastVal;
  int currentVel;
  int numMatchingVals;
  byte multiplier;
  boolean consistent;
  boolean increasing;
  boolean inReverse;
  int raw1;
  int speed1;
  int raw2;
  int speed2;
  int i;
  ContinuousRotationServo * target;
  
  Serial.print("here!\n");

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
  while(deltaPos == 0);
  
  Serial.print("Found starting velocity.\n");

  // Wait for reliable segment
  numMatchingVals = 0;
  lastVal = analogRead(potLine);
  increasing = true;
  while(numMatchingVals < REQUIRED_NUM_MATCHING_VALS_LOOSE)
  {
    delay(SHORT_CALIBRATION_DUR);
    potVal = analogRead(potLine);
    Serial.print(potVal);
    Serial.print("\n");
    consistent = (increasing && potVal >= lastVal) || (!increasing && potVal <= lastVal);
    if(MIN_TRUSTED_VALUE <= potVal && potVal <= MAX_TRUSTED_VALUE && consistent)
      numMatchingVals++;
    else
    {
      numMatchingVals = 0;
      increasing = potVal > lastVal;
    }
    lastVal = potVal;
  }
    
  Serial.print("Found linear segment.\n");
  
  // Adjust for backwards
  if(increasing)
  {
    target->velocitySlope *= -1;
    crs_setVelocity_(id, currentVel);
  }
  
  Serial.print("Starting speed observation.\n");

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
      Serial.print("High level velocity value: ");
      Serial.print(currentVel);
      Serial.print("\n");
      crs_setVelocity_(id, currentVel);
    }
  }
  
  // Update zero value
  target->zeroValue += target->velocitySlope * currentVel;

  // Record zero and values for x1 and y1
  raw1 = 0;
  speed1 = 0;

  // Try another velocity for x2 and y2
  //crs_setVelocity_(id, CALIBRATION_VEL_SLOPE_SPEED);
  
  // Calculate slope

  // Go back to zero
  
  // Stop
  crs_setVelocity_(id, 0);
  crs_setTargetVelocity(id, 0);
}

void crs_setVelocity_(int id, int velocity)
{
  int convertedVelocity;
  
  ContinuousRotationServo * target = crs_getInstance(id);
  globalServo.attach(target->controlLine);
  
  convertedVelocity = crs_convertVelocityToRaw_(id, velocity);
  Serial.print("Setting velocity to ");
  Serial.print(convertedVelocity);
  Serial.print(".\n");
  globalServo.writeMicroseconds(convertedVelocity);
}

void crs_correctPos_(int id)
{

}

LimitedRotationServo * lrs_getInstance(int id)
{
  return &(limitedRotationServos[id]);
}

void lrs_init(int id, byte controlLine)
{
  LimitedRotationServo * target = lrs_getInstance(id);
  target->controlLine = controlLine;
}

void lrs_setAngle(int id, int angle)
{
  LimitedRotationServo * target = lrs_getInstance(id);
  int line = target->controlLine;

  globalServo.attach(line);
  globalServo.write(angle);
}

void lrs_step(int id)
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
}

int piezo_isFired(int id)
{
  PiezoSensor * target = piezo_getInstance(id);
  int line = target->line;

  int val = analogRead(line);

  if(val < PIEZO_MIN_TAP_VAL)
    return NO_TAP;
  else
    return val;
}

LightSensor * ls_getInstance(int id)
{
  return &(lightSensors[id]);
}

void ls_init(int id, byte line)
{
  LightSensor * target = ls_getInstance(id);
  target->line = line;
}

boolean ls_isLight(int id)
{
  LightSensor * target = ls_getInstance(id);
  return analogRead(target->line) > MIN_LIGHT_VAL;
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

void fish_init(int id, int xServoNum, int yServoNum, int zServoNum, int thetaServo)
{
  Fish * targetFish = fish_getInstance(id);

  // Save servo nums
  targetFish->xServo = xServoNum;
  targetFish->yServo = yServoNum;
  targetFish->zServo = zServoNum;
  targetFish->thetaServo = thetaServo;

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
  target->targetX = targetX;
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
  target->moveAngle = targetTheta;

  // Start off to first positional subgoal
  fish_goToNextInternalGoal_(id);
}

void fish_onGoalReached(int id)
{
  Fish * target = fish_getInstance(id);

  // Determine if subgoal or actual goal
  if(target->subStepsLeftToGoal == 0)
    aquarium_onFishReachedGoal(id); // Tell system
  else 
    fish_goToNextInternalGoal_(id); // Next subgoal
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
    crs_step(target->thetaServo, ms);
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

  nextElementIndex++;
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

  if(maxSensorRecordIndex != NONE)
    return NONE;
  else
    return target->sensorNums[maxSensorRecordIndex].highLevelID;
}

Aquarium * aquarium_getInstance(int id)
{
  return &(aquariums[id]);
}

void aquarium_init(int id, int fishNum, int jellyfishNum, int lightSensorNum,
                   int piezoSensorGroupNum)
{
  Aquarium * target = aquarium_getInstance(id);
    
  // Save simple attributes
  target->fishNum = fishNum;
  target->jellyfishNum = jellyfishNum;
  target->lightSensorNum = lightSensorNum;
  target->piezoSensorGroupNum = piezoSensorGroupNum;

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

void aquarium_tick(int id, long newMS)
{
  Aquarium * target;
  long deltaMS;
  long newShortMSRemain;
  long newLongMSRemain;

  target = aquarium_getInstance(id);
  deltaMS = newMS - target->lastMS;
  newShortMSRemain = target->shortMSRemain + deltaMS;
  newLongMSRemain = target->longMSRemain + deltaMS;
  
  // Check if long step was fired
  if(newLongMSRemain < 0)
  {
    aquarium_longStep(id, LONG_TIME_STEP - newLongMSRemain);
    newLongMSRemain = LONG_TIME_STEP;
  }
  target->longMSRemain = newLongMSRemain;

  // Check if short step was fired
  if(newShortMSRemain < 0)
  {
    aquarium_shortStep(id, SHORT_TIME_STEP - newShortMSRemain);
    newShortMSRemain = SHORT_TIME_STEP;
  }
  target->shortMSRemain = newShortMSRemain;

  target->lastMS = newMS;
}

void aquarium_onFishReachedGoal(int id, int fishID)
{
  
}
