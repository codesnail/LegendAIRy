#include "Freenove_4WD_Car_for_Arduino.h"
#include "Servo.h"   //include servo library

#define MAX_DISTANCE    300   //cm
#define SONIC_TIMEOUT   (MAX_DISTANCE*60)
#define SOUND_VELOCITY    340   //soundVelocity: 340m/s

extern Servo servo2;             //create servo object
extern byte servoOffset;

struct Position {
  int angle;
  float distance;
};

struct ObjectPosition {
  Position rightEnd;
  Position leftEnd;
};

class Sonar {
    private:
    // Ultrasonic sensor radar parameters
    int scanDelay = 4;         // Delay between each servo movement (in milliseconds)
    int servoMinAngle = 0;      // Minimum angle for servo rotation
    int servoMaxAngle = 180;    // Maximum angle for servo rotation
    byte servoStep = 1;         // Angle step size for servo movement
    int trackedAngle = servoMinAngle;  // Angle where the object is detected
    bool reverseDirection = false;     // Flag to reverse servo direction
    long maxTrackedDistance = 70; // Only last object seen within this distance will be tracked
        
    public:
    void init(int iMinAngle) {
      servoMinAngle = iMinAngle;
      init(); 
    }   
    
    void init() {
        servo2.write(servoMinAngle);
        delay(500);
        trackedAngle = servoMinAngle;
        reverseDirection = false;
    }

    void setMaxTrackedDistance(long maxTrackedDistance) {
      this->maxTrackedDistance = maxTrackedDistance;
    }

    long getMaxTrackedDistance() {
      return maxTrackedDistance;
    }

    void move(int angle) {
        // Move servo to input angle
        servo2.write(angle);
        delay(500);
        trackedAngle = angle;
    }

    void setScanDelay(int scanDelay) {
      this->scanDelay = scanDelay;
    }
    
    void move() {
        // Move the servo to the tracked angle
        servo2.write(trackedAngle);
        delay(scanDelay);
        
        // Move the tracked angle to the next position
        if (!reverseDirection) {
          trackedAngle += servoStep;
          if (trackedAngle > servoMaxAngle) {
            trackedAngle = servoMaxAngle;
            reverseDirection = true;
          }
        } else {
          trackedAngle -= servoStep;
          if (trackedAngle < servoMinAngle) {
            trackedAngle = servoMinAngle;
            reverseDirection = false;
          }
        }
    }

    float getDistance() {
        unsigned long pingTime;
        float distance;
        digitalWrite(PIN_SONIC_TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(PIN_SONIC_TRIG, HIGH); // make trigPin output high level lasting for 10μs to triger HC_SR04,
        delayMicroseconds(10);
        digitalWrite(PIN_SONIC_TRIG, LOW);
        pingTime = pulseIn(PIN_SONIC_ECHO, HIGH, SONIC_TIMEOUT); // Wait HC-SR04 returning to the high level and measure out this waiting time
        if (pingTime != 0)
          distance = (float)pingTime * SOUND_VELOCITY / 2 / 10000; // calculate the distance according to the time
        else
          distance = MAX_DISTANCE;
        
        return distance; // return the distance value
    }

    int getAngle() {
      return trackedAngle;
    }

    void setServoStep(byte stepSize) {
      servoStep = stepSize;
      scanDelay = (int)ceil(stepSize * 3.33); // estimating about 3.33 milliseconds per degree
    }

    bool getReverseDirection() {
      return reverseDirection;
    }
};
