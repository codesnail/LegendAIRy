#include "Freenove_4WD_Car_for_Arduino.h"
#include "Servo.h"   //include servo library
#include "Sonar.cpp"

#define AC_NO_TURN 0
#define AC_TURNED_RIGHT 1
#define AC_TURNED_LEFT 2

#define MAX_DISTANCE    300   //cm
#define SONIC_TIMEOUT   (MAX_DISTANCE*60)
#define SOUND_VELOCITY    340   //soundVelocity: 340m/s

extern Servo servo2;             //create servo object
extern byte servoOffset;

class Car {
  private:
  int axleLength = 0.1016; // meters, or 4 inches
  float maxSpeed = 0.65; // m/sec
  float curSpeed = 0; // m/sec
  float actualSpeed = 0; // m/sec
  int power_left = 0;
  int power_right = 0;
  long int prevTime = 0;
  long int prevSenseTime = 0;
  float distance = 0; // distance travelled - meters
  float orientation = 0;
  u8 trackingSensorVal = 0;
  Sonar sonar;
                           
  public:

  void move(float speed) {
  }

  void turnLeft(float factor) {
  }

  void turnRight(float factor) {
  }

  void updateDistance() {
  }

  float getDistance() {
  }

  void moveDistance(float meters) {
  }

  //when black line on one side is detected, the value of the side will be 0, or the value is 1  
  // Yaser: The above comment seems opposite of what's actually happening. The sensor returns
  // 1 when black (or dark color) is detected and 0 when white (or light color).
  void updateTrackingSensorVal() {
  }

  u8 getTrackingSensorVal() {
    return this->trackingSensorVal;
  }

  Sonar& getSonar() {
    return this->sonar;
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
