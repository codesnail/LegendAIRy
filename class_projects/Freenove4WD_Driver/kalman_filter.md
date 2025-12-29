This folder contains the Arduino sketches and project files for Freenove4WD Arduino kit.

### 1-D Kalman Filter:
The following video walks you through the implementation of a 1-D Kalman filter to start off. This will help you understand the project setup and structure of the files, and some basics of working with Arduino while you implement a simple Kalman Filter. 

Click the image below to watch the complete video.

[![Watch the video](https://img.youtube.com/vi/o7fJ4DfH0rE/default.jpg)](https://www.youtube.com/watch?v=o7fJ4DfH0rE)

### 2-D Kalman Filter:
This section will walk you through getting and running the starter code for a 2-D Kalman Filter on the Freenove4WD Arduino car kit. It consists of both an Arduino sketch for the kit, and a corresponding python program that runs on your computer. You will learn basics of how to integrate between python and Arduino, while implementing a realistic (albeit slow) object tracker. At the end of the project, you can expect something like this working:

[![Demo of 2-D Kalman Filter](https://img.youtube.com/vi/BLeZFGtPXm8/default.jpg)](https://youtu.be/BLeZFGtPXm8)

This code can also work for other Arduino car kits, although you will need to customize it, especially the pins used as they can be different for different kits. For Raspberry Pi kits or others, you will need to replace the Arduino portion of this project with the appropriate code.

1. Download the starter code for this project (this entire folder: Freenove4WD_Driver)
2. Open the Arduino sketch file Freenove4WD_Driver.ino in the Arduino IDE. Note that this file has to exist in the directory with the same name. If you downloaded this folder along with its contents in the above step, you should be fine.
3. Copy the files Freenove_4WD_Car_for_Arduino.h and Freenove_4WD_Car_for_Arduino.cpp from the Freenove 4WD car repo. It is available under multiple sketches, e.g., under Sketches\03.2_Automatic_Tracking_Line. (If you are using a different Arduino car kit, you may have to declare your own initializers, e.g., the pins that your kit uses, especially for servo and ultrasonic range sensor.)
   ![image](../../images/freenove_librariese.png)
4. Open the sketch Freenove4WD_Driver.ino in the Arduino IDE. Compile the sketch and resolve any errors that you see.

5. Connect your car kit to your computer via a USB port.

6. In Arduino IDE, choose the correct COM port from Tools > Port. This is the port where your Arduino board is plugged in to your computer.
   ![image](../../images/arduino_IDE.png)

7. Make sure to remove the Bluetooth module from your kit. Now upload the sketch onto your board.

8. If everything works, your car kit should be ready to run the sketch when it receives an input of 9. Disconnect the USB connection from your computer and plug in the bluetooth module on your car's board. Make sure it is paired with your computer.

9. Place your car on the floor. Place an object anywhere in the scan angle range of the ultrasonic sensor, and within about 70 cm. Turn on the power of the car kit (it will set its servo to the starting position but otherwise shouldn't do anything).

10. Now open ultrasonic_radar.py in a python editor. Go to line 64:
```
s = serial.Serial(port = "COM7", baudrate=9600, 
               bytesize=8, timeout=5, stopbits=serial.STOPBITS_ONE)
```
Change the value of the first parameter 'port' to the bluetooth port of your computer. Save the file.

11. Now open a command prompt and navigate to the folder RAIT_Freenove4WD_Ard_Driver.

12. Run the python file ultrasonic_radar.py:
```
python ultrasonic_radar.py
```
 You should see something like the following:
 
![image](../../images/sonar_radar_demo.JPG)

This code creates a serial connection to your Arduino and sends the input command (the number 9) to start the ultrasonic radar function of the Arduino sketch you uploaded. It receives back the measurements, parses them, then updates the plot with the measurements.

13. Now go to line number 101:
```
if(r <= tracking_threshold): 
    # TODO: Add your code here for tracking the object
    # and using KalmanFilter
    
    # End Code
```
Here, you will add your code to track the object and use a Kalman Filter to predict its position and velocity. The tracking_threshold is set to 70 (cm) and is a way to limit tracking to a single object (by placing only one object within 70 cm of the car).

14. Open KalmanFilter.py and implement the provided methods of the KalmanFilter class. You should be able to reuse most of your code from the Kalman Filter project. 

15. The file ultrasonic_radar.py sets the global variable 'kf' to an instance of this KalmanFilter class. Use this to update the KalmanFilter and predict the position and velocity of the object. These are printed on the console.

16. Go to line 118:
```
 # TODO: Update the points below to the position predicted by
 # the Kalman Filter. Note that these are polar coordinates.
 #  
 kf_point.set_offsets([0, 0]) 
 # end code
```
Update this line of code to set the position predicted by the Kalman Filter.

17. Run the program again:
```
python ultrasonic_radar.py
``` 
Now you will see something like the above video along with the red dot showing the Kalman Filter predicted position of the object. 

If the object stays at one place, the red dot may move a little bit with each scan but should stabilize to one position after a few scans, and velocity printed on the console should go to zero. 

18. Now move the object (I move the object by hand just a little bit at a time when the sensor faces away from the object, making sure to avoid the sensor). The red dot should update its position and it should show the velocity in the console.

