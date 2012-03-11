#include "aquariumlogic.h"
#include <Servo.h>

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
  lrs_init(0, 9);
  led_init(0, 3);
  jellyfish_init(0, 0, 0);
}

void loop()
{
  jellyfish_raise(0);
  delay(3000);
  jellyfish_lower(0);
  delay(3000);
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
    return 0;
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
