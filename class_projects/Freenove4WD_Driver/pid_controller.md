This file walks you through the steps to download and implement a PID controller to follow an object with your Arduino robotic car kit.

### Click the image below to see a demo of the end result.

[![Watch the video](https://img.youtube.com/vi/Q6CoairHs-M/default.jpg)](https://youtu.be/Q6CoairHs-M)


1. Download the starter code for this project (this folder). This project consists solely of Arduino code.
2. There is a new class PidController.cpp. If you already downloaded this folder before for another project, 
then you can download only this file. Otherwise download the entire folder.

3. Make sure the Arduino sketch (.ino file) exists in the directory with the same name, along with all the 
supporting files in the directory.

4. You can skip to the next step if you already did this step in the previous project. Copy the files 
Freenove_4WD_Car_for_Arduino.h and Freenove_4WD_Car_for_Arduino.cpp from the Freenove 4WD car repo. It is 
available under multiple sketches, e.g., under Sketches\03.2_Automatic_Tracking_Line. (If you are using a 
different Arduino car kit, you may have to declare your own initializers, e.g., the pins that your kit uses, 
especially for servo and ultrasonic range sensor.)


5. Go to PidController.cpp in Arduino IDE (or your preferred C++ IDE) and implement the TODOs. There are some initializations:
```
class Controller {
  private:
    float kp = 0;
    float kd = 0;
    float ki = 0;
    
    float sum_pid_error = 0;
    float prev_error = 0; // previous distance error
    
  public:
    void init() {
      // TODO: Initialize the following...
      kp = 0;
      kd = 0;
      ki = 0;
      
      float sum_pid_error = 0;
      float prev_error = 0; // previous distance error
    }
    
    .
    .
    .
};
```

and a method you need to complete:

```
float speedController(float dist_error, char incomingByte) {
      
      // Adjust gain values based on serial input.
      // You can use different values to adjust by
      if(incomingByte == 'p') {
        kp -= 0.001;
      } else if(incomingByte == 'P') {
        kp += 0.001;
      } else if(incomingByte == 'd') {
        kd -= 0.0001;
      } else if(incomingByte == 'D') {
        kd += 0.0001;
      } else if(incomingByte == 'i') {
        ki -= 0.00001; 
      } else if(incomingByte == 'I') {
        ki += 0.00001;
      }
    
      // TODO: Implement PID logic here, including needed 
      // calculations for integral and differential terms
      
      float alpha = 0; // pid equation
      
      return alpha;
    }
```

6. In the Arduino sketch (.ino file), you will see the following code block. 
It declares an object of class Controller, then implements a function followObject:

```
.
.
.
Controller pid; // create PID controller
.
.
.
void followObject(char incomingByte) {
  if(modeChanged) {
    servo2.write(90 + servoOffset);  // move servo to straight
    delay(500); // Make sure servo moves to target before doing anything else
    pid.init(); // initialize PID controller
  }
  
  float distance = car.getSonar().getDistance();
  
  float vel = 0;

  // TODO: Implement your code here
  //   Check for any distance thresholds you may need to.
  //   Then call pid controller to keep the car a certain distance from 
  //   the object, e.g. 25cm.

  Serial.print(distance);
  Serial.print(" ");
  Serial.print(vel);
  Serial.println();

  delay(50); // set frequency of the loop to 20 hz (20 times per second = 50ms delay)
}
```
You need to implement the TODO part in the function followObject(). The Controller class
is defined in PidController cpp file which is included in the class Car.cpp. Since Car.cpp
is included in the sketch, you should not need to include PidController separately.

7. Once you implement the above, compile the sketch and resolve any errors that you see.

8. Connect your car kit to your computer via a USB port.

9. In Arduino IDE, choose the correct COM port from Tools > Port. This is the port where 
your car kit's Arduino board is plugged in to your computer via USB.

![image](../../images/arduino_IDE.png)

10. Make sure to remove the Bluetooth module from your kit. Now upload the sketch onto your board.

11. If everything works, your car kit should be ready to run the sketch when it receives an input of 5. 
Disconnect the USB connection from your computer and plug in the bluetooth module on your car's board. 
Make sure it is paired with your computer. Now select the port for your bluetooth devide in the Arduino IDE.

12. Place your car on the floor. Start the Serial Monitor or Serial Plotter from the same menu in 
Arduino IDE and type in 5 to start the program. 

Your car should behave as in the video shown above when you bring an object in front of it. You can enter 
p, P, d, D, i or I to decrease or increase the corresponding gains to adjust the behavior. You may need 
to do some back of the envelope calculations to come up with some reasonable initial gain values and the 
change values, then run some experimentation to tune them.