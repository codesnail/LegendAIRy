#include "Freenove_4WD_Car_for_Arduino.h"
#include "Servo.h"   //include servo library

#define MAX_DISTANCE    300   //cm
#define SONIC_TIMEOUT   (MAX_DISTANCE*60)
#define SOUND_VELOCITY    340   //soundVelocity: 340m/s

extern Servo servo2;             //create servo object
extern byte servoOffset;

class Sonar {
    private:
    // Ultrasonic sensor radar parameters
    const int scanDelay = 10;        // Delay between each servo movement (in milliseconds)
    const int servoMinAngle = 0;     // Minimum angle for servo rotation
    const int servoMaxAngle = 180;   // Maximum angle for servo rotation
    const int servoStep = 1;         // Angle step size for servo movement
    const long maxDistance = 300;    // Maximum distance to measure (in centimeters)
    int trackedAngle = servoMinAngle;  // Angle where the object is detected
    bool reverseDirection = false;   // Flag to reverse servo direction

    public:
    void init() {
        servo2.write(servoMinAngle);
        delay(500);
        trackedAngle = servoMinAngle;
        reverseDirection = false;
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
        pingTime = pulseIn(PIN_SONIC_ECHO, HIGH, SONIC_TIMEOUT); // Wait HC-SR04 returning to the high level and measure out this waitting time
        if (pingTime != 0)
          distance = (float)pingTime * SOUND_VELOCITY / 2 / 10000; // calculate the distance according to the time
        else
          distance = MAX_DISTANCE;
        return distance; // return the distance value
    }

    int getAngle() {
      return trackedAngle;
    }
};
