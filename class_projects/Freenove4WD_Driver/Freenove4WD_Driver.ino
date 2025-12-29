#include "Freenove_4WD_Car_for_Arduino.h"
#include "Servo.h"   //include servo library
#include "Car.cpp"

// My dell: USB right side port 5, bluetooth port 7

// Commands supplied from Serial monitor to make the car do different things
#define MODE_TRACK_LINE  1
#define MODE_TRACK_WALL  2
#define MODE_FIND_LINE_OR_WALL 3
#define MODE_DO_NOTHING 4
#define MODE_FOLLOW_OBJECT 5
#define MODE_ULTRASONIC_KF 6
#define MODE_MPU6050 7
#define MODE_HISTOGRAM_FILTER 8
#define MODE_ULTRASONIC_RADAR 9

int tk_VoltageCompensationToSpeed;  //define Voltage Speed Compensation

u8 currMode = MODE_DO_NOTHING;
u8 prevMode = currMode;
bool modeChanged = false;

Servo servo2;             //create servo object
byte servoOffset = 0;
Car car; // create car object
char incomingByte = 0;

void setup() {
  Serial.begin(9600);
  servo2.attach(PIN_SERVO);        //initialize servo 
  servo2.write(180 + servoOffset);  // change servoOffset to Calibrate servo
  pinsSetup(); //set up pins
}

void loop() {
  // Only when you receive data:
  if(Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
    
    if(incomingByte >= 48 && incomingByte <= 57) {
      currMode = incomingByte - '0';
      //Serial.print("currMode: ");
      //Serial.println(currMode);
      if(currMode != prevMode) {
        modeChanged = true;
        prevMode = currMode;
      }
    }
  }
  
  switch(currMode) {
    case MODE_TRACK_LINE:
      trackLine();
      break;
    case MODE_TRACK_WALL:
      trackWall();
      break;
    case MODE_FIND_LINE_OR_WALL:
      findLineOrWall();
      break;
    case MODE_FOLLOW_OBJECT: // 5
      followObject(incomingByte);
      break;
    case MODE_ULTRASONIC_KF: // 6
      readUltraSonicKF(incomingByte);
      break;
    case MODE_MPU6050: // 7
      readAccelerometer();
      break;
    case MODE_HISTOGRAM_FILTER: // 8
      histogramFilter(incomingByte);
      break;
    case MODE_ULTRASONIC_RADAR: // 9
      ultrasonicRadar(incomingByte);
      break;
    case MODE_DO_NOTHING: // 4
      car.move(0);
      break;
  }
  modeChanged = false;
  incomingByte = 0;
}

void histogramFilter(char incomingByte) {
}

void readAccelerometer() { // mode 7
}

void readUltraSonicKF(char incomingByte) {
}

void followObject(char incomingByte) {
}

void trackLine() {
}

void trackWall() {
}

void findLineOrWall() {
}

void ultrasonicRadar(char incomingByte) {
  if(modeChanged) {
    // initialize
    car.getSonar().init();  // move servo to start position
  }
  
  // Move the servo to the tracked angle
  car.getSonar().move();

  // measure distance
  float distance = car.getSonar().getDistance();

  // Print the angle and distance to the serial monitor
  Serial.print("Angle: ");
  Serial.print(car.getSonar().getAngle());
  Serial.print(", Distance: ");
  Serial.print(distance);
  Serial.print(" cm");
  Serial.println();
}

void tk_CalculateVoltageCompensation() {
  getBatteryVoltage();
  float voltageOffset = 7 - batteryVoltage;
  tk_VoltageCompensationToSpeed = 30 * voltageOffset;
}

float speedController(float dist_error, char incomingByte) {
}
