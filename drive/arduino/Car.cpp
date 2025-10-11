#include "Freenove_4WD_Car_for_Arduino.h"
#include "Servo.h"   //include servo library
#include "Sonar.cpp"
#include "PidController.cpp"
#include <MPU6050.h>
//#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define AC_NO_TURN 0
#define AC_TURNED_RIGHT 1
#define AC_TURNED_LEFT 2

// For sonar
#define MAX_DISTANCE    300   //cm
#define SONIC_TIMEOUT   (MAX_DISTANCE*60)
#define SOUND_VELOCITY    340   //soundVelocity: 340m/s

// For LED strip
#define I2C_ADDRESS  0x20 
#define LEDS_COUNT   10  //it defines number of lEDs. 

extern Servo servo2;             //create servo object
extern byte servoOffset;

const int MPU_addr=0x68;

struct MotionModel {
  int power;
  int adjust;
};

struct Power {
  int pwrL;
  int pwrR;
};

struct Gyroscope {
  float accl_x = 0;
  float accl_y = 0;
  float accl_z = 0;
  float gyro_x = 0;
  float gyro_y = 0;
  float gyro_z = 0;
  float gz_rest = 0;
  unsigned char calibrateCount = 0;
  
  float accAngleX = 0;
  float accAngleY = 0;
  float accAngleZ = 0;
  float gyroAngleX = 0;
  float gyroAngleY = 0;
  float gyroAngleZ = 0;
  
  float roll = 0;
  float pitch = 0;
  float yaw = 0;
};

class Car {
  private:
  int axleLength = 0.1016; // meters, or 4 inches
  float maxSpeed = 0.65; // m/sec
  float curSpeed = 0; // m/sec
  float actualSpeed = 0; // m/sec
  int power_left = 0;
  int power_right = 0;
  long int prevTime = 0;
  long int prevTimeGyro = 0;
  float distance = 0; // distance travelled - meters
  int distanceL = 0;
  int distanceR = 0;
  int orientation = 0;
  u8 trackingSensorVal = 0;
  Sonar sonar;
  byte state = 0;
  Controller pid; // create PID controller
  float x;
  float y;
  float heading;
  float wayPoints[1][2]; 
  byte wpLen = 0; // length of wayPoints array
  byte wpIdx = 0; // index into wayPoints array

  //Adafruit_MPU6050 mpu;
  MPU6050 mpu;
  Gyroscope gyro;

  bool bInitialized = false;
                           
  public:
  // Car's possible states
  const byte STOPPED = 0;
  const byte GOING_STRAIGHT_FRONT = 1;
  const byte GOING_STRAIGHT_BACK = 2;
  const byte GOING_LEFT_FRONT = 3;
  const byte GOING_RIGHT_FRONT = 4;
  const byte GOING_LEFT_BACK = 5;
  const byte GOING_RIGHT_BACK = 6;

  Car() {
    
  }

  void init() {
    if(!bInitialized) {
      Wire.begin();
  
      // initialize device
      //Serial.println("Initializing I2C devices...");
      mpu.initialize();
  
      // verify connection
      //Serial.println("Testing device connections...");
      //Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
      bInitialized = true;
    }
  }
  
  void reset() {
    prevTime = 0;
    
  }
  
  // speed in m/sec
  void move(float speed) {
  }

  void setMovePowers(int left, int right) {
    power_left = left;
    power_right = right;
  }

  Power getPower() {
    Power pwr;
    pwr.pwrL = power_left;
    pwr.pwrR = power_right;
    return pwr;
  }
  
  void move() {
    motorRun(power_left, power_right);
    //Serial.print("pwr: ");
    //Serial.print(power_left);
    //Serial.print(",");
    //Serial.println(power_right);
    long int currTime = millis();
    if(prevTime != 0) {
      float dt = (currTime - prevTime)/1000;

      /*
       *  Calculate vel and distance from the LR model of each wheel/motor
        V (LR) y = 0.0057x - 0.1423
        V (LF) y = 0.0063x - 0.1755
        V (RF) y = 0.0055x - 0.1261
        V (RR) y = 0.0052x - 0.1029
       */
       
      distanceL = dt * ((0.0057 * power_left - 0.1423) + (0.0063 * power_left - 0.1755))/2;
      distanceR = dt * ((0.0055 * power_right - 0.1261) + (.0052 * power_right - 0.1029))/2;
    }
    
    if(power_left == 0 && power_right == 0) {
      prevTime = 0;
    } else {
      prevTime = currTime;
    }
  }
  
  // speed in terms of left motor power level and right motor power level
  void move(int speedL, int speedR) {
    motorRun(speedL, speedR);
    
    long int currTime = millis();
    if(prevTime != 0) {
      float dt = (currTime - prevTime)/1000;

      /*
       *  Calculate vel and distance from the LR model of each wheel/motor
        V (LR) y = 0.0057x - 0.1423
        V (LF) y = 0.0063x - 0.1755
        V (RF) y = 0.0055x - 0.1261
        V (RR) y = 0.0052x - 0.1029
       */
       
      distanceL = dt * ((0.0057 * power_left - 0.1423) + (0.0063 * power_left - 0.1755))/2;
      distanceR = dt * ((0.0055 * power_right - 0.1261) + (.0052 * power_right - 0.1029))/2;
    }
    
    if(speedL == 0 && speedR == 0) {
      prevTime = 0;
    } else {
      prevTime = currTime;
    }

    power_left = speedL;
    power_right = speedR;
  }

  void initPID(float p, float i, float d) {
    pid.init(0.6, 0, 0);
  }

  void setX(float x) {
    this->x = x;
  }

  void setY(float y) {
    this->y = y;
  }

  void setHeading(float heading) {
    this->heading = heading;
  }

  float getX() {
    return x;
  }

  float getY() {
    return y;
  }

  float getHeading() {
    return heading;
  }

/*
  void passObjectFromRight(Position pos, float distance, byte incomingByte) {
    float rad = pos.angle * PI/180;
    float x = pos.distance * cos(rad);
    float y = pos.distance * sin(rad);
    int adjust = (int)pid.genericController(x + distance, incomingByte);
    move(80 + adjust, 80 - adjust);
    state = GOING_RIGHT_FRONT;
  }

  void passObjectFromLeft(Position pos, float distance, byte incomingByte) {
    float rad = pos.angle * PI/180;
    float x = pos.distance * cos(rad);
    float y = pos.distance * sin(rad);
    int adjust = (int)pid.genericController(x - distance, incomingByte);
    move(80 + adjust, 80 - adjust);
    state = GOING_LEFT_FRONT;
  }
 */

  MotionModel controller(float err, byte incomingByte) {
    int adjust = (int)pid.genericController(err, incomingByte);
    move(80 + adjust, 80 - adjust);
    MotionModel m = {80, adjust};
    return m;
  }

  void addWayPoint(float x, float y) {
    if(wpLen < 5) {
      wayPoints[wpLen][0] = x;
      wayPoints[wpLen][1] = y;
      wpLen++;
    }
  }

  float* getWayPoint(byte idx) {
    return wayPoints[idx];
  }

  float* getNextWayPoint() {
    // if the car's Y coordinate is very close to the current target waypoint,
    // then set the next waypoint as target
    if(abs(y - wayPoints[wpIdx][1]) < 5) { 

      // but first check if we are out of waypoints
      if(wpIdx == wpLen-1) {  // last index
        return NULL; // return NULL to stop the car, because what else can we do?
      } else {
        wpIdx++; // increment index to set it to the next waypoint
      }
    }
    
    return wayPoints[wpIdx];
  }

  byte getWayPointsLen() {
    return wpLen;
  }

  byte initWayPoints() {
    wpIdx = 0;
    wpLen = 0;
    // No need to empty out the array since resetting the above two should achieve 
    // the desired outcome. As long as we don't try to access the array directly.
  }

  void incrementWayPointIndex() {
    wpIdx++;
  }

  byte getState() {
    return state;
  }

  //when black line on one side is detected, the value of the side will be 0, or the value is 1  
  // Yaser: The above comment seems opposite of what's actually happening. The sensor returns
  // 1 when black (or dark color) is detected and 0 when white (or light color).
  u8 getTrackingSensorVal() {
    return (digitalRead(PIN_TRACKING_LEFT) == 1 ? 1 : 0) << 2 | (digitalRead(PIN_TRACKING_CENTER) == 1 ? 1 : 0) << 1 | (digitalRead(PIN_TRACKING_RIGHT) == 1 ? 1 : 0) << 0;
  }

  Sonar& getSonar() {
    return this->sonar;
  }

  Gyroscope& readGyroscope() {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    gyro.accl_x = ax / 16384.0 + 0.135;
    gyro.accl_y = ay / 16384.0 + 0.045;
    gyro.accl_z = az / 16384.0 - 0.08;
    gyro.gyro_x = gx / 131.0;
    gyro.gyro_y = gy / 131.0;
    gyro.gyro_z = gz / 131.0 + 0.8834;

    long int currentTime = millis();
    float elapsedTime = 0;
    
    if(prevTimeGyro != 0) {
      elapsedTime = (currentTime - prevTimeGyro) / 1000; // Divide by 1000 to get seconds
    }
    
    gyro.yaw = gyro.yaw + gyro.gyro_z * elapsedTime;
    prevTimeGyro = currentTime;

    return gyro;
  }
  
  void setLEDColor(int color) {
  }

  void printState() {
    Serial.println();
    Serial.print("dist travelled: ");
    Serial.print(this->distance);
    Serial.print(" orient: ");
    Serial.print(this->orientation);
    Serial.print(" speed: ");
    Serial.print(this->curSpeed);
    Serial.print(" power (l, r): ");
    Serial.print(this->power_left);
    Serial.print(",");
    Serial.print(this->power_right);
    
    Serial.println();
  }
};
