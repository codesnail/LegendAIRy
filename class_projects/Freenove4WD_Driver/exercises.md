Some suggestions for further exploration:

Task 2.1: Add motor movement to 1D Kalman Filter to measure velocity of car (against a wall or another stationary object).

Task 2.2: Find and track objects by using their mid-points. Use the mid-point in a KF to track a single object. Slow and not practical, but pedagogical value to understand the end-to-end pipeline for a KF.

Task 2.3: Alternately (instead of implementing a 2D Kalman Filter), expand point array processing to identify multiple landmarks. This can be used later to build a map of the env.

#### Key takeaways from HW perspective: 

Divide tasks between Arduino and python: Once you start developing/modifying your code frequently, the compile/build/upload cycle of Arduino becomes pretty cumbersome. Not to mention that Arduino (and other microcontrollers) will often be limited in their resources, so higher order algorithms will need to run on more powerful computers. Transferring some higher order tasks to your computer and building a communication layer between python and Arduino will help a lot going forward. The code in task 2 above provides a rudimentary comms layer. In the future we will see and use a more advanced one.

Creating threads in Arduino: Arduino is intrinsically single threaded, but we used a threading implementation (you might not realize it, but we did). In Task 1 we experienced using the servo motor and ultrasonic sensor. In particular, we saw that the call to move the servo motor is non-blocking, so we had to use an explicit wait (delay() call) in our program to wait for the servo to finish its motion to the desired angle before activating the ultrasonic sensor. In this task, we used the servo to scan the whole 180 degree range. But we did this in a non-blocking way (well semi non-blocking). If we were to do this in a single threaded way, we would put the servo motion in a loop, moving it by 1 degree followed by a wait, until it scans 180 degrees. We wouldn't be able to do any other processing during this time. Instead, we kept track of the active thread by using the MODE variable, and used other state variables to keep track of the servo. This way we are able to perform multiple operations in a multi-threaded way with Arduino.

Point Array Processing: The ultrasonic distance sensor gives us an array of points as it scans the environment in a 180 degree sweep. It is slower, but it is a similar kind of data that a LIDAR sensor will give us in the form of Point Clouds. Consequently, we can practice with and learn various algorithms for point cloud processing with simpler 2-D point arrays. Task 2.3 above asks you to do one such processing to distinguish landmarks.

This will do it for now. Happy roboticizing and keep humans in the loop!
