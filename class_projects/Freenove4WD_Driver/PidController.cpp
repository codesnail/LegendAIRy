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
};