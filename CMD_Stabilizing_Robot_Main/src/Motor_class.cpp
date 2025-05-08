/* 
COLLINEAR MECANUM DRIVE ROBOT
By Carl Johan Malmerot and Peter Stensson
MF133X Bachelors Project in Mechatronics

All code is also available at https://github.com/CarlMalmerot/CMD_robot.git

This class drives the motors.
*/

#include <CMD_robot.h>

Motor::Motor(int L_pwm_set, int R_pwm_set, bool direction){
    // R_en = R_en_set;
    // L_en = L_en_set;
    
    if(direction){   // Flip the pins if direction is false
        R_pwm = R_pwm_set;
        L_pwm = L_pwm_set;
    }
    else{
        R_pwm = L_pwm_set;
        L_pwm = R_pwm_set;
    }
    
}


// Write PWM duty cycle to the motor
void Motor::write_duty(int duty_cycle){
    duty_cycle = constrain(duty_cycle, -255, 255);  // Safety for invalid duty cycle inputs
    if(duty_cycle >= 0){
        // digitalWrite(L_en, 1);
        // digitalWrite(R_en, 1);
        analogWrite(L_pwm, 0);
        analogWrite(R_pwm, duty_cycle);
    }
    else{
        // digitalWrite(R_en, 1);
        // digitalWrite(L_en, 1);
        analogWrite(R_pwm, 0);
        analogWrite(L_pwm, abs(duty_cycle));
    }
}

void Motor::disable(){
    // digitalWrite(L_en, 0);
    // digitalWrite(R_en, 0);
    analogWrite(L_pwm, 0);
    analogWrite(R_pwm, 0);

}