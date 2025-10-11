#include "Freenove_4WD_Car_for_Arduino.h"
#include "Servo.h"   //include servo library
#include "Car.cpp"
#include <ArduinoJson.h>

// My dell: USB right side port 5, bluetooth port 7

// Commands supplied from Serial monitor to make the car do different things
#define MODE_TRACK_LINE  1
#define MODE_NAVIGATE 3
#define MODE_DO_NOTHING 4
#define MODE_FOLLOW_OBJECT 5
#define MODE_ULTRASONIC_KF 6
#define MODE_MPU6050 7
#define MODE_ULTRASONIC_RADAR 9

#define OUTPUT_WAIT_TIME 20 // msec

int tk_VoltageCompensationToSpeed;  //define Voltage Speed Compensation

u8 currMode = MODE_DO_NOTHING;
u8 prevMode = currMode;
bool modeChanged = false;

Servo servo2;             //create servo object
byte servoOffset = 90;
Car car; // create car object
Controller pid; // create PID controller
char incomingByte = '\0';
byte wpIdx = 0; // way points index

// *** Start of Json example code
// Receive with start- and end-markers

const int numChars = 256;
char receivedChars[numChars] = {'\0'};
StaticJsonDocument<256> doc; //(numChars);

boolean newData = false;
unsigned long prevTime1 = 0;
unsigned long prevTime2 = 0;

void setup() {
  Serial.begin(9600);
  servo2.attach(PIN_SERVO);        //initialize servo 
  //servo2.write(0 + servoOffset);  // change servoOffset to Calibrate servo
  car.getSonar().move(servoOffset);
  pinsSetup(); //set up pins
}

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static int ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (rc == startMarker) {
      ndx = 0;
      receivedChars[ndx] = '\0';
      recvInProgress = true;
        
    } else if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
        //receivedChars[ndx] = '\0';
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
        //return;
      }
    }
  }
}

void showNewData() {
  if (newData == true) {
    Serial.print("This just in ... ");
    Serial.println(receivedChars);
    newData = false;
  }
}
// *** End of example code

void loop() { 
  recvWithStartEndMarkers();

  if(newData == true) {
    // Deserialize the JSON document
    // Uncomment the following to debug using Arduino IDE and Monitor
    Serial.println(receivedChars);
    DeserializationError error = deserializeJson(doc, receivedChars);
    newData = false;
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.print(error.f_str());
      Serial.print(": ");
      Serial.println(receivedChars);
      currMode = 4; // Stop
      if(currMode != prevMode) {
        modeChanged = true;
        prevMode = currMode;
      }
      // TODO: Do something to handle the error
    } else {
      u8 mode = doc["mode"];
      if(mode > 0) {
        currMode = mode;
        if(currMode != prevMode) {
          modeChanged = true;
          prevMode = currMode;
        }
      }
    }
  }
  
  switch(currMode) {
    case MODE_TRACK_LINE:
      trackLine();
      break;
    case MODE_NAVIGATE: // 3
      navigate(doc, incomingByte);
      //navigate(incomingByte);
      break;
    case MODE_FOLLOW_OBJECT: // 5
      followObject(incomingByte);
      break;
    case MODE_ULTRASONIC_KF: // 6
      readUltraSonicKF(incomingByte);
      break;
    case MODE_MPU6050: // 7
      calibrateAccelerometer();
      break;
    case MODE_ULTRASONIC_RADAR: // 9
      ultrasonicRadar(doc, incomingByte);
      break;
    case MODE_DO_NOTHING: // 4
      car.move(0, 0);
      prevTime1 = 0;
      prevTime2 = 0;
      break;
  }
  Serial.flush();
  modeChanged = false;
  incomingByte = '\0';
  doc.clear();
  doc = NULL; // this line causes a compiler error with DynamicJsonDOcument, use doc.clear() to reset the Json Document.
}

// Arduino test input: <{"mode": 7}>
// Stop: <{"mode": 4}>
void calibrateAccelerometer() { // mode 7
  if(modeChanged) {
    car.init();
  }
  
  Gyroscope gyro = car.readGyroscope();
  
  Serial.print("acx:");
  Serial.print(gyro.accl_x, 2);
  Serial.print(",acy:");
  Serial.print(gyro.accl_y, 2);
  Serial.print(",acz:");
  Serial.print(gyro.accl_z, 2);
  Serial.print(",gyx:");
  Serial.print(gyro.gyro_x, 2);
  Serial.print(",gyy:");
  Serial.print(gyro.gyro_y, 2);
  Serial.print(",gyz:");
  Serial.print(gyro.gyro_z, 2);
  Serial.print(",yaw:");
  Serial.println(gyro.yaw, 2);
  
  delay(10);
}

void readUltraSonicKF(char incomingByte) { // mode 6
}

void followObject(char incomingByte) {
  if(modeChanged) {
    car.getSonar().move(90);  // move servo to face front
    pid.init();
  }

  switch(incomingByte) {
    case 'a':
      car.getSonar().move(90); // move servo to front
      pid.setSpeedMode();
      break;
    case 'b':
      car.getSonar().move(0); // move servo to right
      pid.setDistRMode();
      break;
    case 'c':
      car.getSonar().move(180); // move servo to left
      pid.setDistLMode();
      break;
  }
  
  float distance = car.getSonar().getDistance();
  
  float vel = 0;
  float adjust = 0;
  
  // if obstacle is within a certain distance
  if(distance > 60) {
    car.move(0);
  } else {
    if(pid.getSpeedMode()) {
      // keep within 25 cm of the obstacle...
      vel = pid.speedController(distance - 25, incomingByte);
      car.move(vel);
    }
    else if(pid.getDistLMode()) {
      // keep moving at a speed within 25 cm distance from left
      adjust = pid.genericController(distance - 25, incomingByte);
      car.move(80 - adjust, 80 + adjust);
    }
  }

  Serial.print(distance);
  Serial.print(", ");
  Serial.print(vel);
  Serial.println();

  delay(50); // set frequency of the loop to 20 hz (20 times per second)
}

void trackLine() {
}

// Arduino test input: <{"mode": 3, "pos": [0, 0, 1.51], "wp": [[0,10],[0,20],[0,30]], "pwr": [80, 80]}>
// Stop: <{"mode": 4}>
void navigate(StaticJsonDocument<256> command, byte incomingByte) { // MODE = 3
  if(modeChanged) {
    car.init();
    car.initPID(0.55, 0, 0);
    prevTime1 = 0;
    //car.getSonar().setServoStep(3);
    car.getSonar().setMaxTrackedDistance(70);
  }
 /**
   * JSON input
   * command = {
        "mode": 3,
        "pos": [x, y, heading], # x, y, heading
        "wp": way_points # [[x1, y1], [x2, y2], [x3, y3]]
    }
   */
  
  float* wayPoint;
  String mode = command["mode"];
  int pwrL = 0;
  int pwrR = 0;
  
  if(command != NULL) {
    car.setX(command["pos"][0]);
    car.setY(command["pos"][1]);
    car.setHeading(command["pos"][2]);
    
    JsonArray arr = command["wp"];
    if(arr != NULL) {
      car.initWayPoints(); // since a command with waypoints was received, 
                           // reinitialize way points array
      
      for(JsonArray obj: arr) {
        car.addWayPoint(obj[0], obj[1]);
      }
    }

    JsonArray pwrArr = command["pwr"];
    if(pwrArr != NULL) {
      pwrL = command["pwr"][0];
      pwrR = command["pwr"][1];
      car.setMovePowers(pwrL, pwrR);
      //Serial.println("Set power");
    }
  } 
  
  wayPoint = car.getNextWayPoint();
  //MotionModel mModel;
  
  // if no new wayPoint or out of wayPoints, then stop the car
  if(false) { //wayPoint == NULL) {
    car.move(0, 0); 
  } else {
    // Moved controller code out of Arduino and into python 
    // for quick experimentation/iterations. 
    
    // set error in x coordinate
    //float errX = car.getX() - wayPoint[0];
  
    // calculate target heading, and set the heading error
    //float targetHeading = atan2(wayPoint[1] - car.getY(), wayPoint[0] - car.getX());
    //float errHeading = targetHeading - car.getHeading();
  
    // call the car's PID controller with weighted errors
    //mModel = car.controller(0.7 * errX - 0.3 * errHeading, incomingByte);

    car.move(); // Left and right powers are set in the car before. 
                
  }

  Gyroscope gyro = car.readGyroscope();
  //Power pwr = car.getPower();
  
  long currTime = millis();
  
  if(currTime - prevTime1 >= OUTPUT_WAIT_TIME) {
    Serial.print("<");
    Serial.print("wpx:");
    Serial.print(wayPoint[0]);
    Serial.print(",wpy:");
    Serial.print(wayPoint[1]);
    Serial.print(",acx:");
    Serial.print(gyro.accl_x, 2);
    Serial.print(",acy:");
    Serial.print(gyro.accl_y, 2);
    Serial.print(",acz:");
    Serial.print(gyro.accl_z, 2);
    Serial.print(",gyx:");
    Serial.print(gyro.gyro_x, 2);
    Serial.print(",gyy:");
    Serial.print(gyro.gyro_y, 2);
    Serial.print(",gyz:");
    Serial.print(gyro.gyro_z, 2);
    Serial.print(",yaw:");
    Serial.print(gyro.yaw, 2);
    Serial.print(",pwL:");
    Serial.print(pwrL);
    Serial.print(",pwR:");
    Serial.print(pwrR);
    
    long int dt = 0;
    if(prevTime1 != 0) {
      dt = currTime - prevTime1;
    }
    Serial.print(",dt:");
    Serial.print(dt);
    Serial.print(",an:");
    Serial.print(car.getSonar().getAngle());
    Serial.print(",di:");
    Serial.print(car.getSonar().getDistance());
    Serial.println(">");
    prevTime1 = currTime;
  }
}

// test: <{"mode": 9, "step": 3, "an": 90}>
Position ultrasonicRadar(StaticJsonDocument<256> command, char incomingByte) {
  int stepSize = command["step"];
  int angle = command["an"];
    
  if(command != NULL) {
    stepSize = command["step"];
    angle = command["an"];
  }
    
  if(modeChanged) {
    // initialize
    car.getSonar().init(angle);  // move servo to start position
    prevTime2 = 0;
  }

  if(command != NULL) {
    if( stepSize != 0) {
      car.getSonar().setServoStep(stepSize);
    } else if(angle != 0) {
      car.getSonar().setServoStep(0);
      car.getSonar().move(angle);
    } 
  }

  // Move the servo to the tracked angle
  car.getSonar().move();
  
  // measure distance
  float distance = car.getSonar().getDistance();

  Position pos;
  pos.angle = car.getSonar().getAngle();
  pos.distance = distance;

  if(millis() - prevTime2 >= OUTPUT_WAIT_TIME) {
    prevTime2 = millis();
    // Print the angle and distance to the serial monitor
    Serial.print("<");
    Serial.print("an:");
    Serial.print(car.getSonar().getAngle());
    Serial.print(",di:");
    Serial.print(distance);
    Serial.println(">");
  }

  return pos;
  
}

void tk_CalculateVoltageCompensation() {
  getBatteryVoltage();
  float voltageOffset = 7 - batteryVoltage;
  tk_VoltageCompensationToSpeed = 30 * voltageOffset;
}
