/**
 * Name: digital_aquarium_sketch.ino
 * Desc: Logic for the DigitalAquarium
 * Auth: Jessica Ebert, Sam Pottinger, DJ Sutton
**/

// Directional constants
#define NORTH 1
#define NORTHEAST 2
#define EAST 3
#define SOUTHEAST 4
#define SOUTH 5
#define SOUTHWEST 6
#define WEST 7
#define NORTHWEST 8

// Axis identification constants
#define X_AXIS 1
#define Y_AXIS 2
#define Z_AXIS 3

// Component listing (how many of each component?)
#define NUM_CONT_ROT_SERVOS 4
#define NUM_LIM_ROT_SERVOS 1
#define NUM_PIEZO_SENSORS 4
#define NUM_LIGHT_SENSORS 1
#define NUM_LED 2
#define NUM_JELLYFISH 1
#define NUM_FISH 1
#define NUM_PIEZO_SENSOR_GROUPS 1
#define NUM_AQUARIUMS 1

// Sensor tollerance levels
#define MIN_LIGHT_VAL 400
#define PIEZO_MIN_TAP_VAL 50
#define NO_TAP -1

// Generic multi-purpose NONE value
#define NONE -1

// Jellyfish behavorial constants
#define JELLYFISH_RAISED_ANGLE 180
#define JELLYFISH_LOWERED_ANGLE 0

// Fish behavorial constants
// NOTE: Location and speed constraints below
#define FISH_SUB_STEPS_TO_GOAL 1
#define WIGGLE_AMPLITUDE 10
#define WIGGLE_SPEED 3.14159 // rad / sec
#define FISH_OWNER 1

// Aquarium behavior constants
#define LONG_TIME_STEP 100
#define SHORT_TIME_STEP 10

// Calibration constants
#define PRE_CALIBRATION_ZERO_VAL 1500
#define PRE_CALIBRATION_POSITION 0
#define STARTING_TARGET_VELOCITY 100
#define DEFAULT_VELOCITY_SLOPE -1
#define SIZE_OF_POSITION_VAL 4
#define CALIBRATION_CAUTIOUS_FACTOR 0.75

#define REQUIRED_NUM_MATCHING_VALS 15
#define REQUIRED_NUM_MATCHING_VALS_MED 10
#define REQUIRED_NUM_MATCHING_VALS_LOOSE 4
#define MIN_HIGH_VAL 1000 // depricated
#define MAX_HIGH_VAL 1024 // deprectaed
#define MIN_TRUSTED_VALUE 500
#define MAX_TRUSTED_VALUE 900
#define MAX_LOW_VAL 24
#define MIN_LOW_VAL 0
#define SHORT_CALIBRATION_DUR 10
#define START_CALIBRATION_VEL 10

#define SLOPE_FINDING_VEL_1 100
#define SLOPE_FINDING_VEL_2 200
#define SLOPE_FINDING_DUR 500

// Location and speed constraints
#define MIN_FISH_SPEED 0
#define MAX_FISH_SPEED 50
#define MIN_X_VAL -102400
#define MAX_X_VAL 102400
#define MIN_Y_VAL -102400
#define MAX_Y_VAL 102400
#define MIN_Z_VAL -102400
#define MAX_Z_VAL 102400
#define CENTRAL_X_VAL 0
#define CENTRAL_Y_VAL 0
#define FISH_HIDE_X 0
#define FISH_HIDE_Y 0
#define FISH_HIDE_Z 0

// Piezo sensor ids
#define NE_SENSOR_ID 0
#define SE_SENSOR_ID 1
#define SW_SENSOR_ID 2
#define NW_SENSOR_ID 3
#define NUM_DIR_PIEZO_SENSORS 4

// AIN ports
#define NE_AIN_PORT 0
#define SE_AIN_PORT 1
#define SW_AIN_PORT 2
#define NW_AIN_PORT 3

#define MS_PER_SEC 1000

#define NUM_STEPS_PER_RAD 0.00306796158
#define NUM_STEPS_ROT 2048

#define AQUARIUM_ID 0

#define RAND_AIN_PORT 0

#include <Arduino.h>
#include <math.h>
#include <stdarg.h>

// Abstraction for continuous rotation servos

typedef struct
{
  long position; // Zerored at calibration
  int zeroValue; // From calibration
  double velocitySlope; // From calibration
} CrsDto;

typedef struct
{
  int controlLine;
  int potLine; 
  int zeroValue; // From calibration
  long position; // Zerored at calibration
  long targetPosition;
  boolean decreasing;
  boolean inTrustedArea;
  int numMatchingVals;
  int correctionLastVal;
  int targetVel;
  double velocitySlope; // From calibration
  int ownerType;
  int ownerID;
  int selfID;
} ContinuousRotationServo;

// Continuous rotation servo behavior

/**
 * Name: crs_getInstance(int id)
 * Desc: Get the continuous rotation servo instance with the given id
 * Para: id, The unique numerical id of the desired servo
**/
ContinuousRotationServo * crs_getInstance(int id);

/**
 * Name: crs_init(int id, byte controlLine, byte potLinem boolean calibrate)
 * Desc: Calibrates and enumerates a cont. rotation servo
 * Para: id, The unique id of the servo to operate on
 *       controlLine, Which line to use for PWM to control the device
 *       potLine, The line where the potentiometer is installed
 *       calibrate, If true, servo's position is reset. If false, loaded from EEPROM
**/
void crs_init(int id, byte controlLine, byte potLine, boolean callibrate);

void crs_stop(int id);

/**
 * Name: crs_setOwner(int id, int type, int ownerID);
 * Desc: Sets the id of the owner of this servo
 * Para: id, The id of the servo to set the owner of
 *       type, The type of the object that owns this servo
 *       ownerID, The unique numerical id of the object that owns this servo
**/
void crs_setOwner(int id, int type, int ownerID);

/**
 * Name: crs_startMovingTo(int id, long targetPosition)
 * Desc: Moves the given servo to the given position
 * Para: id, The unique numerical id of the servo to operate on
 *       targetPosition, The position the servo should go to
**/
void crs_startMovingTo(int id, long targetPosition);

/**
 * Name: crs_startMovingToAngle(int id, double angle)
 * Desc: Has this servo start moving to a given angle
 * Para: id, The unique numerical id of the servo to operate on
 *       angle, The angle to move to in radians (zero due right)
**/
void crs_startMovingToAngle(int id, double angle);

/**
 * Name: crs_step(int id, long ms)
 * Desc: Update this servo's state and check to see if its goal was reached
 * Para: id, The id of the servo to operate on
 *       ms, The time (milliseconds) since this was last called
**/
void crs_step(int id, long ms);

/**
 * Name: crs_setTargetVelocity(int id, int targetVelocity)
 * Desc: Sets the velocity this servo should use to reach its goal point
 * Para: id, The unique numerical id of the servo to operate on
 *       targetVelocity, The velocity this servo should use
**/
void crs_setTargetVelocity(int id, int targetVelocity);

/**
 * Name: crs_onGoalReached(int id)
 * Desc: The function called when a continuous rotation servo hits its
 *       goal positoin
 * Para: id, The unique id of the servo this should operate on
**/
void crs_onGoalReached(int id);

/**
 * Name: crs_getPos(int id)
 * Desc: Get the current position of this servo
 * Para: id, The unique id of the servo to get the position for
 * Retr: Current estimated position of this servo
**/
long crs_getPos(int id);

/**
 * Name: crs_convertToVelocityRaw(float vel)
 * Desc: Converts the given velocity to a raw value that can be sent
 *       to the servo
 * Para: id, The id of the servo to make the conversion for
 *       vel, The velocity to convert (steps / sec)
 * Note: Should be treated as private member of ContinuousRotationServo
**/
int crs_convertVelocityToRaw_(int id, float vel);

/**
 * Name: crs_loadCalConstants_(int id)
 * Desc: Load position information from EEPROM
 * Para: id, The id of the servo to operate on
 * Note: Should be treated as private member of ContinuousRotationServo
**/
void crs_loadPosition_(int id);

/**
 * Name:crsh_save_(int id)
 * Desc: Saves this servo's position information to EEPROM
 * Para: id, The unique numerical id of the servo to save to mem
 * Note: Should be treated as private member of ContinuousRotationServo
**/
void crs_savePosition_(int id);

/**
 * Name: crs_calibrate_(int id)
 * Desc: Generate calibration constants and zero position
 * Para: id, The id of the servo to operate on
 * Note: Should be treated as private member of ContinuousRotationServo
**/
void crs_calibrate_(int id);

/**
 * Name: crs_setVelocity_(int id, int velocity)
 * Desc: Sets the actual velocity the servo should use
 * Para: id, The id of the servo to set the velocity for
 *       velocity, The velocity to set this servo to use
**/
void crs_setVelocity_(int id, int velocity);

/**
 * Name: crs_correctPos_(int id)
 * Desc: Attempts to correct this servo's position using its pot reading
 * Para: id, The id of the servo to operate on
**/
void crs_correctPos_(int id);

/**
 * Name: crs_goToLinearSection_(int id)
 * Desc: Have the servo rotate out to the trusted linear section on its pot
 * Para: id, The servo to get there
**/
void crs_goToTrustedSection_(int id);

/**
 * Name: crs_exhaustMatchingSection_(int id, int minVal, int maxVal)
 * Desc: Wait until the servo's pot registers a value outside of the given range
 * Para: id, The id of the servo to watch
 *       minVal, The minimum value in the range to exhaust
 *       maxVal, The maximum value in the range to exhaust
**/
void crs_exhaustMatchingSection_(int id, int minVal, int maxVal);

// Logic for simple limited rotation servos

typedef struct
{
  byte controlLine;
} LimitedRotationServo;

// Limited rotation servo behavior

/**
 * Name: lrs_getInstance(int id)
 * Desc: Get the limited rotation servo instance with the given id
 * Para: id, The unique numerical id of the desired servo
**/
LimitedRotationServo * lrs_getInstance(int id);

/**
 * Name: lrs_init(int id, byte controlLine)
 * Desc: Initializes this limited rotation servo with starting values
 * Para: id, The id of the servo to operate on
 *       controlLine, The line to which this servo is attached
**/
void lrs_init(int id, byte controlLine);

/**
 * Name: lrs_setAngle(int id, int angle)
 * Desc: Makes this servo rotate to the given angle
**/
void lrs_setAngle(int id, int angle);

/**
 * Name: lrs_step(int id, long ms)
 * Desc: Has this limited rotation servo update interal state
 * Para: id, The unique id of the servo to operate on
 *       ms, The number of milliseconds since this was last called
**/
void lrs_step(int id, long ms);

// Piezo sensor abstraction
typedef struct
{
  byte line;
} PiezoSensor;

// Piezo sensor behavior

/**
 * Name: piezo_getInstance(int id)
 * Desc: Get the piezo instance with the given id
 * Para: id, The unique numerical id of the desired piezo sensor
**/
PiezoSensor * piezo_getInstance(int id);

/**
 * Name: piezo_init(int id, byte line)
 * Desc: Creates internal state for piezo sensor
 * Para: id, The unique id of the piezo sensor to initalize
 *       line, The line number (analog) to which this sensor
 *             is attached
**/
void piezo_init(int id, byte line);

/**
 * Name: piezo_isFired(int id)
 * Desc: Determines if the given piezo sensor reigsters a "tapped" state
 * Para: id, The unique id of the piezeo sensor to check
 * Retr: 0 if not reigstering a tap (below threshold) and raw non-zero value
 *       positive reading otherwise
**/
int piezo_isFired(int id);

// Light sensor abstraction
typedef struct
{
  byte line;
} LightSensor;

// Light sensor behavior

/**
 * Name: ls_getInstance(int id)
 * Desc: Gets the light sensor instance that cooresponds to the given id
 * Para: id, The unique numerical id of the light sensor to get
**/
LightSensor * ls_getInstance(int id);

/**
 * Name: ls_init(int id, byte line)
 * Desc: Initalizes this light sensors' state
 * Para: id, The unique numerical id for this light sensor
 *       line, The line this sensor is attached to
**/
void ls_init(int id, byte line);

/**
 * Name: ls_isLight(int id)
 * Desc: Determines if this light sensor indicates that the 
 *       environment is light or dark
 * Para: id, The unique numerical id of the light sensor to
 *           check
 * Retr: True if light and False if dark
**/
boolean ls_isLight(int id);

// LED abstraction

typedef struct
{
  byte line;
} LEDAbstraction;

// LED abstraction behavior

/**
 * Name: led_getInstance(int id)
 * Desc: Gets the structure instance corresponding to
 *       the given id
 * Para: id, The unique numerical id of the instance to
 *           get
 * Retr: Pointer to instance
**/
LEDAbstraction * led_getInstance(int id);

/**
 * Name: led_init(int id, byte line)
 * Desc: Creates internal state for given LED
 * Para: id, The unique numerical id of the LED to initalize
 *       port, Which line this led is installed on
**/
void led_init(int id, byte line);

/**
 * Name: led_turnOn(int id)
 * Desc: Turns on the given LED
 * Para: id, The unique numerical id of the LED to turn on
**/
void led_turnOn(int id);

/**
 * Name: led_turnOff(int id)
 * Desc: Turns off the given LED
 * Para: id, The unique numerical id of the LED to turn off
**/
void led_turnOff(int id);

// Jellyfish abstraction

typedef struct
{
  int servoNum;
  int ledNum;
} Jellyfish;

/**
 * Name: jellyfish_getInstance(int id)
 * Desc: Gets the jellyfish instance corresponding to
 *       the given id
 * Para: id, The unique numerical id of the instance to
 *           get
 * Retr: Pointer to instance
**/
Jellyfish * jellyfish_getInstance(int id);

/**
 * Name: jellyfish_init(int id, int servoNum, int ledNum)
 * Desc: Initalize state of given jellyfish
 * Para: id, The unqiue numerical id of the jellyfish to initalize
 *       servoNum, The unique numerical id of the limited roatation
 *                 servo controlling this jellyfish
 *       ledNum, The unique numerical id of the LED inside of this
 *               jellyfish
**/
void jellyfish_init(int id, int servoNum, int ledNum);

/**
 * Name: jellyfish_lower(int id)
 * Desc: Lower this jellyfish into view and turn on its led
 * Para: id, The unique numerical id of the jellyfish to lower
**/
void jellyfish_lower(int id);

/**
 * Name: jellyfish_raise(int id)
 * Desc: Raise this jellyfish out of view and turn of its led
 * Para: id, The unique numerical id of the jellyfish to raise
**/
void jellyfish_raise(int id);

/**
 * Name: jellyfish_step(int id, long ms)
 * Desc: Propogate the on step event to this jellyfish and its servos
 * Para: id, The unique numerical id of the jellyfish to propogate events to
 *       ms, The number of milliseconds since this function was last called
**/
void jellyfish_step(int id, long ms);

// Fish abstraction

typedef struct
{
  int xServo;
  int yServo;
  int zServo;
  int thetaServo;
  int velocity;
  long targetX;
  long targetY;
  long targetZ;
  long startX;
  long startY;
  long startZ;
  long startMS;
  float xSpeedPortion;
  float ySpeedPortion;
  float zSpeedPortion;
  int subStepsLeftToGoal;
  double moveAngle;
  int numWaitingServos;
} Fish;

/**
 * Name: jellyfish_getInstance(int id)
 * Desc: Gets the fish instance corresponding to
 *       the given id
 * Para: id, The unique numerical id of the instance to
 *           get
 * Retr: Pointer to instance
**/
Fish * fish_getInstance(int id);

/**
 * Name: fish_init(int id, int xServoNum, int yServoNum, int zServoNum, int thetaServo, boolean calibrate)
 * Desc: Initalizes the state of a fish
 * Para: id, The unique numerical id of the instance to initalize
 *       xServoNum, ID of continuous rotation servo on x axis
 *       yServoNum, ID of continuous rotation servo on y axis
 *       zServoNum, ID of continuous rotation servo on z axis
 *       thetaServo, ID of the servo rotating this fish
**/
void fish_init(int id, int xServoNum, int yServoNum, int zServoNum, int thetaServo);

/**
 * Name: fish_goTo(long id, long x, long y, long z)
 * Desc: Has the fish go to the given position
 * Para: id, The id of the fish to move
 *       x, The x position to take this fish to
 *       y, The y position to take this fish to
 *       z, The z position to take this fish to
**/
void fish_goTo(long id, long x, long y, long z);

/**
 * Name: fish_onGoalReached(int id)
 * Desc: Event handler for when a fish reaches its goal position
 * Para: id, The id of the fish that reached its goal position
**/
void fish_onGoalReached(int id);

void fish_stop(int id);

/**
 * Name: fish_onServoGoalReached(int id, int servoID)
 * Desc: Event handler for when a servo for a fish reaches its goal
 * Para: id, The id of the fish whose servo has reached a goal
 *       servoID, The id of the servo that reached its goal
**/
void fish_onServoGoalReached(int id, int servoID);

/**
 * Name: fish_goToNextInternalGoal_(int id)
 * Desc: Has this fish go to the next sub goal in its larger goal position
 * Para: id, The unique numerical id of the fish to operate on
 * Note: Should be treated as a private member of Fish
**/
void fish_goToNextInternalGoal_(int id);

/**
 * Name: fish_step(int id, long ms)
 * Desc: Propogates this step event to this fish and its servos
 * Para: id, The id of the fish to propogate the event to
 *       ms, The number of milliseconds since this was last called
**/
void fish_step(int id, long ms);

/**
 * Name: fish_setVelocity(int id, float velocity)
 * Desc: Set the target (single axis max) velocity for this fish
 * Para: id, The unique numerical id of the fish to set the target velocity for
 *       velocity, The max velocity to use for this fish
**/
void fish_setVelocity(int id, float velocity);

// Piezo sensor group abstraction
typedef struct
{
  int sensorID;
  int highLevelID;
} SensorGroupMembershipRecord;

typedef struct
{
  int numSensors;
  int nextElementIndex;
  SensorGroupMembershipRecord * sensorNums;
} PiezoSensorGroup;

/**
 * Name: psg_getInstance(int id)
 * Desc: Gets the piezeo sensor group instance corresponding to
 *       the given id
 * Para: id, The unique numerical id of the instance to
 *           get
 * Retr: Pointer to instance
**/
PiezoSensorGroup * psg_getInstance(int id);

/**
 * Name: psg_init(int id, int numSensors)
 * Desc: Initialize this sensor group's internal state
 * Para: id, The unique numerical id of this sensor group
 *       numSensors, The number of sensors in this group
**/
void psg_init(int id, int numSensors);

/**
 * Name: psg_dest(int id)
 * Desc: Destructs support structures for the given piezo sensor group
 * Para: id, The unique numerical id of the sensor group to deallocate
**/
void psg_dest(int id);

/**
 * Name: psg_addToSensorList(int id, int sensorID)
 * Desc: Adds a new piezo sensor to this group
 * Para: id, The unique numerical id of the group to operate on
 *       sensorID, The id of the sensor to add to this group
 *       sensorHighLevelID, The int to return when this sensor is fired
**/
void psg_addToSensorList(int id, int sensorID, int sensorHighLevelID);

/**
 * Name: psg_getTapped(int id)
 * Desc: Get the sensor that shows a "tapped state"
 * Para: id, The unique numerical id of the group to operate on
 * Retr: NONE or high level id of sensor that indicated a tap
**/
int psg_getTapped(int id);

// Aquarium abstraction

typedef struct
{
  int fishNum;
  int jellyfishNum;
  int lightSensorNum;
  int piezoSensorGroupNum;
  bool isLight;
  long lastMS;
  long shortMSRemain;
  long longMSRemain;
} Aquarium;

/**
 * Name: psg_getInstance(int id)
 * Desc: Gets the aquarium instance corresponding to
 *       the given id
 * Para: id, The unique numerical id of the instance to
 *           get
 * Retr: Pointer to instance
**/
Aquarium * aquarium_getInstance(int id);

/**
 * Name: aquarium_init(int id, int fishNum, int jellyfishNum, 
 *                     int lightSensorNum, piezeoSensorGroupNum)
 * Desc: Initalizes state of the given aquarium
 * Para: id, The unique numerical id of this aquarium
 *       jellyfishNum, The unique numerical id of the jellyfish in
 *                     use in this aquarium
 *       lightSensorNum, The unique numerical id of the light sensor 
 *                       for this aquarium
 *       piezoSensorGroupNum, The unique numerical id of the piezo 
 *                            sensor group for this aquarium
**/
void aquarium_init(int id, int fishNum, int jellyfishNum, int lightSensorNum, 
                   int piezeoSensorGroupNum);

/**
 * Name: aquarium_shortStep(int id, long ms)
 * Desc: Executes the low latency loop operations (excuted more frequently)
 * Para: id, The unique numerical id of this aquarium
 *       ms, The number of milliseconds since this was last called
**/
void aquarium_shortStep(int id, long ms);

/**
 * Name: aquarium_longStep(int id, long ms)
 * Desc: Executes the high latency loop operations (excuted less frequently)
 * Para: id, The unique numerical id of this aquarium
 *       ms, The number of milliseconds since this was last called
**/
void aquarium_longStep(int id, long ms);

/**
 * Name: aquarium_tick(int id, long ms)
 * Desc: Response to a "program level" tick, an event fired at regular intervals
 *       that propogates to drive the step event
 * Para: id, The unique id of the aquarium to respond to the tick
 *       ms, The global system millisecond count
**/
void aquarium_tick(int id, long ms);

/**
 * Name: aquarium_onFishReachedGoal(int fishID)
 * Desc: Event handler for when a fish reaches its goal position
 * Para: fishID, The id of the fish that reached its goal
**/
void aquarium_onFishReachedGoal(int id, int fishID);

/**
 * Name: aquarium_onFishReachedGoal_(int id, int fishID)
 * Desc: Event handler for when a fish reaches its goal position
 * Para: id, The id of the aquarium who is responding to this event
 *       fishID, The id of the fish taht reached its goal
 * Note: Should be treated as private member of Aquarium
**/
void aquarium_onFishReachedGoal_(int id, int fishID);

/**
 * Name: aquarium_runFishToOpposingSide_(int id, int tappedSensor);
 * Desc: Move the fish for the given aquarium to the side opposing the tapped
 *       sensor id
 * Para: id, The id of the aquarium to operate on
 *       tappedSensor, The high level id of the sensor that was fired
**/
void aquarium_runFishToOpposingSide_(int id, int tappedSensor);

/**
 * Name: aquarium_transitionToFishState_(int id)
 * Desc: Hide the jellyfish and show / move the fish
 * Para: id, The id of the aquarium to operate on
**/
void aquarium_transitionToFishState_(int id);

/**
 * Name: aquarium_transitionToJellyfishState_(int id)
 * Desc: Show the jellyfish and hide the fish, stopping its continued motion
 * Para: id, The id of the aquarium to operate on
**/
void aquarium_transitionToJellyfishState_(int id);

/**
 * Name: aquarium_getOpposingDirection_(int direction)
 * Desc: Get the cardinal direction that opposes the given direction
 * Para: id, the unique id of the aquarium to calculate this direction in
 *       dir, The direction for which the opposite direction is desired
 * Retr: Constant corresponding to the given direction
**/
int aquarium_getOpposingDirection_(int id, int dir);

/**
 * Name: aquarium_getXBoundInDirection_(int id, int direction)
 * Desc: Get the farthest x value in the given direction
 * Para: id, The unique id of the aquarium to calculate this x position in
 *       direction, The direction in which the extreme x value is requested
 * Retr: Extreme x position in direction
**/
long aquarium_getXBoundInDirection_(int id, int direction);

/**
 * Name: aquarium_getYBoundInDirection_(int id, int direction)
 * Desc: Get the farthest y value in the given direction
 * Para: id, The unique id of the aquarium to calculate this y position in
 *       direction, The direction in which the extreme y value is requested
 * Retr: Extreme y position in direction
**/
long aquarium_getYBoundInDirection_(int id, int direction);
