/* 
COLLINEAR MECANUM DRIVE ROBOT
By Carl Johan Malmerot and Peter Stensson
MF133X Bachelors Project in Mechatronics

All code is also available at https://github.com/CarlMalmerot/CMD_robot.git

This is the main header file, included in all other files. 
Contains class declarations, relevant addresses, structs and enums.
*/


#include <Wire.h>
#include <stdint.h>
#include <Arduino.h>

// Definition of data structure for telemetry broadcast
typedef struct telemetry_msg {
    int speed;
    int battery_voltage;
    int kalman_angle;
    int raw_angle;
    int raw_gyro;
    int angle_pid;
    int angle_command;
    int motor_1_pid;
    int motor_2_pid;
    int yaw_rate;
    int angular_rate;
    int m1_rpm;
    int m2_rpm;
    int yaw;
    int side_speed_rpm_command;
  } telemetry_msg;

  typedef struct control_msg {
    int control_speed;
    int control_yaw_rate;
    int control_side_input;
  } control_msg;


/*  START OF MPU-RELATED */
constexpr uint8_t MPU_ADDRESS_STD = 0x68;
constexpr uint8_t DLPF_ADDRESS = 0x1A;      // Address for DLPF address setting register
constexpr uint8_t GYRO_MAX_ADDRESS = 0x1B;  // Address for gyro range setting register
constexpr uint8_t ACCEL_MAX_ADDRESS = 0x1C; // Address for gyro range setting register
constexpr uint8_t PWR_MGMT_1 = 0x6B;        // Address for power management registry
constexpr uint8_t ACCEL_OUT_1ST = 0x3B;     // Address for 1st accel data byte
constexpr uint8_t GYRO_OUT_1ST = 0x43;      // Address for 1st gyro data byte
constexpr uint8_t GYRO_OUT_Y = 0x45;        // Address for gyro Y byte

// Using constexpr uint8_t for declaring single byte values
// Just using #define defaults for ints

// Strongly typed enums, declaring def. of 8 bit integers to prevent conversions to 16-bit integers
enum DLPFSetting : uint8_t{
    LPF_BW_256HZ = 0x00, //.98 ms delay - Filter disabled. 8kHz sample-frekvens
    LPF_BW_188HZ = 0x01, //1.9 ms delay  1kHz sample-freq for all others
    LPF_BW_98HZ = 0x02,  //2.8 ms delay  1kHz
    LPF_BW_42HZ = 0x03,  //4.8 ms delay  1kHz
    LPF_BW_20HZ = 0x04,  //8.3 ms delay  1kHz
    LPF_BW_10HZ = 0x05,  //13.4 ms delay  1kHz
    LPF_BW_5HZ = 0x06,   //18.6 ms delay  1kHz
};
enum GyroRange : uint8_t{
    GYRO_MAX_250DEG = 0x00,  //250 deg/s osv.
    GYRO_MAX_500DEG = 0x08, //500 deg/s
    GYRO_MAX_1000DEG = 0x10, //1000 deg/s
    GYRO_MAX_2000DEG = 0x18, //2000 deg/s
};
enum AccelRange : uint8_t{
    g_2 = 0x00,  // +/- 2g
    g_4 = 0x08,  // +/- 4g
    g_8 = 0x10,  // +/- 8g
    g_16 = 0x18, // +/- 16g
};

enum GyroLSBSens{   // Doubles not allowed in enum. Not used now, values are coded into method 
    FSR_250_SENS,   // 131 LSB's per degree per second
    FSR_500_SENS,   // 65.5 LSB's per degree per second
    FSR_1000_SENS,  // 32.8 LSB's per degree per second
    FSR_2000_SENS,  // 16.4 LSB's per degree per second
}; 


/*      MPU-Class declaration       */

class MPU6050 {
    private:
        uint16_t address = MPU_ADDRESS_STD; // Standard adress
        float accX, accY, accZ; // Acc data variables
        float gyroX, gyroY, gyroZ;  // Gyro data variables
        float accel_LSBs, gyro_LSBs;    // LSB's per g and per deg/s
        float accX_cal, accY_cal, accZ_cal; // Calibration values
        float gyroX_cal, gyroY_cal, gyroZ_cal;
        int error;

        DLPFSetting dlpf;
        GyroRange gyro_range;
        AccelRange accel_range;

        int16_t raw_buffer; // 2-bytes for handling raw data. Signed since MPU gives values of +/- 32768

    public:
        
        MPU6050(DLPFSetting dlpf_set = LPF_BW_188HZ, GyroRange gyro_range_set = GYRO_MAX_250DEG, AccelRange accel_range_set = g_2);

        // Method declarations
        int reset();
        void setGyroRange(GyroRange range);
        void setAccelRange(AccelRange range);
        void setDLPF(DLPFSetting new_setting);
        int getDLPF();
        void updateAcc();   
        void updateGyro();
        void updateGyro_Y();
        void runCalibration();
        void set_cal_values(float gyroX_cal_set, float gyroY_cal_set, float gyroZ_cal_set, float accelX_cal_set, float accelY_cal_set, float accelZ_cal_set);

        // Small getter-methods are implemented here in header
        // Easier to find than in the cpp file

        // Get X-acceleration (float)
        float getAccX(){ return accX;}  
        // Get Y-acceleration (float)
        float getAccY(){ return accY;}  
        // Get Z-acceleration (float)
        float getAccZ(){ return accZ;}  

        // Get X-acceleration (float)
        float getGyroX(){ return gyroX;}  
        // Get Y-acceleration (float)
        float getGyroY(){ return gyroY;}  
        // Get Z-acceleration (float)
        float getGyroZ(){ return gyroZ;}  

        float getAccPitch();

};
/*  END OF MPU-RELATED */


/*  START OF KALMAN */
class Kalman_Angle{
    private:
        float dt, prev_state, curr_state, prev_gyro, uncertainty, prev_uncertainty, gain, Q, R;
        unsigned long last_time_us;

    public:
        // Arguments: Settings of Q and R
        Kalman_Angle(float Q_set, float R_set){
            Q = Q_set;
            R = R_set;
        }

        float calc_angle(float acc_angle, float gyro_reading){

            dt = (micros() - last_time_us) * (1.0f / 1000000.0f); // dt in seconds

            // Predict current state
            curr_state = prev_state + dt * prev_gyro;

            // Calc the uncertainty of prediction
            //uncertainty = prev_uncertainty + Q;
            uncertainty = prev_uncertainty + Q * pow(dt, 2);    // In python we did dt**2 * 5 * 5

            // Calculate gain
            gain = uncertainty / (uncertainty + R);

            // Update state with measurement and gain
            curr_state = curr_state + gain * (acc_angle - curr_state);

            // Update uncertainty
            uncertainty = uncertainty * (1 - gain);
            

            // Get ready for the next iteration
            last_time_us = micros();
            prev_gyro = gyro_reading;
            prev_state = curr_state;
            prev_uncertainty = uncertainty;
            
            // Serial.print("dt: ");
            // Serial.print(dt);
            // Serial.print(" Kalman state: ");
            // Serial.println(curr_state);
            // Return the angle state
            return curr_state;  
        }

};
/*  END OF KALMAN-RELATED */




/*  START OF AS5600-RELATED */

// Class for controlling I2C multiplexer
// Sets the address to communicate with
// Input argument 0 to 3 for each channel 
// Adresses on TCA board:
// Motor 0: 4
// Motor 1: 5
// Motor 2: 6
// Motor 3: 7
class MUX{
    private:
    // None needed
    public:
    void set_address(int address){
        Wire.beginTransmission(0x70);   // TCA adress 0x70
        Wire.write(1 << address + 4);     // Writes a byte with the 1 shifted to the correct position
        Wire.endTransmission();  
    }
};



// ADDRESSES
constexpr uint8_t AS5600_ADDRESS = 0x36;
constexpr uint8_t ANGLE_ADDRESS = 0x0E;
constexpr uint8_t STATUS_ADDRESS = 0x0B;

//  STATUS REGISTERS
const uint8_t AS5600_AGC       = 0x1A;
const uint8_t AS5600_MAGNITUDE = 0x1B;   //  + 0x1C
const uint8_t AS5600_BURN      = 0xFF;

//  STATUS BITS
const uint8_t AS5600_MAGNET_HIGH   = 0x08;
const uint8_t AS5600_MAGNET_LOW    = 0x10;
const uint8_t AS5600_MAGNET_DETECT = 0x20;

// CONF REGISTER
const uint8_t AS5600_CONF = 0x07;
// Datasheet page 9 - If supplied by 3.3V it needs a 1uF cap on the input. 5V and 3.3V pins should be tied together.
// I think the board handles it? Datasheet is only related to the chip itself.

class AS5600{
    private:
        bool direction; // True if increasing angle is clockwise
        float last_rpms[4], rpm, current_rpm;  // The last 4 rpm readings for averaging rotational speed, index 0 to 3
        int lastangle, angle, angle_diff, error, bytes_received;
        unsigned long last_time;
        float cumulative_angle;  // Stores the accumulated angle moved in degrees
        int16_t raw_buffer; // 2-byte buffer
        int8_t raw_buffer_8bit; // 1-byte buffer

    public:
        // Angle increases clockwise if direction_set is true
        AS5600(bool direction_set);

        int get_angle();
        int get_current_rpm();
        long get_cumulative_angle();
        void update_rot_speed();

        // Status-methods
        int is_connected();
        int get_status();
        int get_AGC();
        int get_Magnitude();
        int get_MagnetStatus();
        void set_filters();
        void zero_cum_angle();

};
/*  END OF AS5600-RELATED */



/* START OF MOTOR-RELATED */
// Class for the motors.
// Create object with direction set as false if direction is wrong.
// This will flip the pins in code resulting in the opposite direction.
class Motor{
    private:
        int R_pwm, L_pwm;
        int pwm_duty;

    public:
        Motor(int L_pwm_set, int R_pwm_set, bool direction = true);

        void write_duty(int duty_cycle);
        void disable();
};

/* END OF MOTOR-RELATED */

/* PID-CLASS */
class PID{
    private:
        float pid_P, pid_I, pid_D;  // Coefficients
        float error, last_value, i_sum, last_values[4], d_alpha;    // Array for last errors used in D-part
        float p_part, i_part, d_part;
        int max, min;
        float dt, last_d_part;
        unsigned long lastmicros;
    public:
        PID(float pid_P_set, float pid_I_set, float pid_D_set, int max_val, int min_val, float d_alpha_val);

        float calc_pid(float target, float value, int print_type);
        void reset_integral();
};
