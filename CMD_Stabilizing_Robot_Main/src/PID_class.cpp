/* 
COLLINEAR MECANUM DRIVE ROBOT
By Carl Johan Malmerot and Peter Stensson
MF133X Bachelors Project in Mechatronics

All code is also available at https://github.com/CarlMalmerot/CMD_robot.git

This class is used for all PID-loops.
*/

#include <CMD_robot.h>

// Constructor sets the coefficients
PID::PID(float pid_P_set, float pid_I_set, float pid_D_set, int min_val, int max_val, float d_alpha_val){
    pid_P = pid_P_set;
    pid_I = pid_I_set;
    pid_D = pid_D_set;
    max = max_val;
    min = min_val;
    d_alpha = d_alpha_val;
}

/*
Inputs a target value for a sensor and the current sensor values.
Returns a float with the calculated PID-value.
Needs to be constrained outside of the function.
*/
float PID::calc_pid(float target, float value, int print_type = 0){
    
    error = target - value;
    
    dt = double(micros() - lastmicros) / 1000000; // dt in seconds
    
    // Proportional part
    p_part = pid_P * error;
    
    // Integral part
    i_part = pid_I * error * dt;
    i_part = constrain(i_part, -max/20, max/20);   // Constrain this iteration, ramp-up restriction
    i_sum += i_part;
    i_sum = constrain(i_sum, -max/2, max/2);  // Constrain the I-part overall
    
    // Derivative part
    d_part = pid_D * (value - last_value) / dt; // Only using process variable to ignore large setpoint changes

    // Band-Limited Differentiation of the D-part (Low pass filter)
    d_part = (1 - d_alpha) * last_d_part + d_alpha * (d_part);
   
    // Prints raw values at an interval for troubleshooting
    if(print_type == 1){
        static int i;
        if(i > 50){
            Serial.print("Target: ");
            Serial.print(target);
            Serial.print("  Error: ");
            Serial.print(error);
            Serial.print("  Value: ");
            Serial.print(value);
            Serial.print("  P-part: ");
            Serial.print(p_part);
            Serial.print("  I-part: ");
            Serial.print(i_sum);
            Serial.print("  D-part: ");
            Serial.print(d_part);
            Serial.print("  Output: ");
            Serial.println(constrain(p_part + i_sum + d_part, min, max));
            i = 0;
        }
        i++;
   }
   
    last_value = value;
    last_d_part = d_part;
    lastmicros = micros();
    
    return constrain(p_part + i_sum + d_part, min, max);
}

// Resets the integral to zero
void PID::reset_integral(){
    i_sum = 0;
}