/* 
COLLINEAR MECANUM DRIVE ROBOT
By Carl Johan Malmerot and Peter Stensson
MF133X Bachelors Project in Mechatronics

All code is also available at https://github.com/CarlMalmerot/CMD_robot.git

This class is used for reading the MPU and calculating pitch.
*/

#include <CMD_robot.h>

//Constructor. Default values are assigned only in header, not here.
// Not setting any initial values since global objects are created before I2C is started. 
// Need to enable wire before any changes are done, can't enable it until setup
MPU6050::MPU6050(DLPFSetting dlpf_set, GyroRange gyro_range_set, AccelRange accel_range_set){  

    // Sets variables to given parameters 
    dlpf = dlpf_set;
    gyro_range = gyro_range_set;
    accel_range = accel_range_set;
    
    // This is called in setup instead via setter-methods
    /* setDLPF(dlpf);  // Set initial settings
    setGyroRange(gyro_range);
    setAccelRange(accel_range); */
}

// Resets and cycles the MPU, delays 100ms after
int MPU6050::reset(){
    Wire.beginTransmission(MPU_ADDRESS_STD);
    Wire.write(PWR_MGMT_1);     // Write to power management
    Wire.write(0x00);   // Write zeros to reset
    Wire.endTransmission();
    byte error = Wire.endTransmission();
  
    if (error == 0) {   // Shows any error in resetting
        Serial.println("MPU6050 reset successfully!");
        return 0;
    } else {
        Serial.print("MPU6050 reset failed! I2C error: ");
        Serial.println(error);
        return 1;
    }
    delay(100);     // Wait for reset
}

// Sets the digital low pass filter setting
void MPU6050::setDLPF(DLPFSetting new_setting){    // Sets DLPF value  
    Wire.beginTransmission(MPU_ADDRESS_STD);    // Initialise a transmission
    Wire.write(DLPF_ADDRESS);   // Queue bytes to transmit. First byte is registry address to write to. 
    Wire.write(static_cast<uint8_t>(new_setting));   // What is written to the registry
    Wire.endTransmission();     // Transmit and then end transmission.

    dlpf = new_setting;
}

// Returns the digital low pass filter setting, only the variable and not from the MPU
int MPU6050::getDLPF(){
    return static_cast<int>(dlpf);
}

// Sets the gyro range and corresponding LSB's per degree per second
void MPU6050::setGyroRange(GyroRange range){  // Sets DLPF value
    Wire.beginTransmission(MPU_ADDRESS_STD);    // Initialise a transmission
    Wire.write(GYRO_MAX_ADDRESS);   // Queue bytes to transmit. First byte is registry address to write to. 
    Wire.write(static_cast<uint8_t>(range));   // What is written to the registry
    Wire.endTransmission();     // Transmit and then end transmission.

    if(range == GYRO_MAX_250DEG){ gyro_LSBs = 131.0f;}    // Set the corresponding LSB values
    else if(range == GYRO_MAX_500DEG){ gyro_LSBs = 65.5f;}
    else if(range == GYRO_MAX_1000DEG){ gyro_LSBs = 32.8f;}
    else if(range == GYRO_MAX_2000DEG){ gyro_LSBs = 16.4f;}

    Serial.print("Gyro LSBs:");
    Serial.println(gyro_LSBs);
    delay(500);

    gyro_range = range;


    Wire.beginTransmission(MPU_ADDRESS_STD);
    Wire.write(GYRO_MAX_ADDRESS);
    Wire.endTransmission();
    Wire.requestFrom(MPU_ADDRESS_STD, (uint8_t) 1);

    Serial.print("Read gyro max: ");
    Serial.print(Wire.read());    // Read gyro byte
}

// Sets the accelerometer range and corresponding LSB's per g
void MPU6050::setAccelRange(AccelRange range){  // Sets Accelerometer range
    Wire.beginTransmission(MPU_ADDRESS_STD);    // Initialise a transmission
    Wire.write(ACCEL_MAX_ADDRESS);   // Queue bytes to transmit. First byte is registry address to write to. 
    Wire.write(static_cast<uint8_t>(range));   // What is written to the registry
    Wire.endTransmission();     // Transmit and then end transmission.

    if(range == g_2){ accel_LSBs = 16384.0f / 9.82f;}    // Set the corresponding LSB values to get m/s^2
    else if(range == g_4){ accel_LSBs = 8192.0f / 9.82f;}
    else if(range == g_8){ accel_LSBs = 4096.0f / 9.82f;}
    else if(range == g_16){ accel_LSBs = 2048.0f / 9.82f;}

    accel_range = range;

}

// Midpoint of 0g at 0 as raw value
// Min/Max g-values are at +/- 32768
// To get actual values, divide by number of LSB's per m/s^2
// Same principle for gyro, deg/s

// Update acceleration values
void MPU6050::updateAcc(){
    Wire.beginTransmission(MPU_ADDRESS_STD) ;
    Wire.write(ACCEL_OUT_1ST);
    Wire.endTransmission();

    if(error != 0){
        Serial.print("Wire error ");
        Serial.print(error);
        Serial.println(" when reading MPU");
    }
    Wire.requestFrom(MPU_ADDRESS_STD, (uint8_t) 6);   // rq 6 bytes
    
    // Decode the bytes, two at a time for each axis
    raw_buffer = Wire.read()<<8|Wire.read();    // read first 2 bytes
    // <<8 shifts 8 bytes left, leaves the second byte as zeroes. Then | does a bitwise OR, "overwrites" the zeroes with data.
    accX = float(raw_buffer) / accel_LSBs  - accX_cal;  // Save first accel, x-direction
    
    raw_buffer = Wire.read()<<8|Wire.read();    // Y-direction
    accY = float(raw_buffer) / accel_LSBs - accY_cal;

    raw_buffer = Wire.read()<<8|Wire.read();    // Z-direction
    accZ = float(raw_buffer) / accel_LSBs - accZ_cal;
}


// Updates gyro values
void MPU6050::updateGyro(){
    Wire.beginTransmission(MPU_ADDRESS_STD);
    Wire.write(GYRO_OUT_1ST);
    Wire.endTransmission();
    Wire.requestFrom(MPU_ADDRESS_STD, (uint8_t) 6);   // rq 6 bytes

    raw_buffer = Wire.read()<<8|Wire.read();    // Read X-dir
    gyroX = float(raw_buffer) / gyro_LSBs  - gyroX_cal;

    raw_buffer = Wire.read()<<8|Wire.read();    // Read Y-dir
    gyroY = float(raw_buffer) / gyro_LSBs  - gyroY_cal;

    raw_buffer = Wire.read()<<8|Wire.read();    // Read Z-dir
    gyroZ = float(raw_buffer) / gyro_LSBs - gyroZ_cal;
}

// Updates only gyro Y-reading
void MPU6050::updateGyro_Y(){
    Wire.beginTransmission(MPU_ADDRESS_STD);
    Wire.write(GYRO_OUT_Y);
    Wire.endTransmission();
    Wire.requestFrom(MPU_ADDRESS_STD, (uint8_t) 2);   // rq 6 bytes

    raw_buffer = Wire.read()<<8|Wire.read();    // Read Y-dir
    gyroY = float(raw_buffer) / gyro_LSBs  - gyroY_cal;
}

// Gets 500 values and averages to calibrate Acc and Gyro to zero. Assumes X is down.
// Takes about two seconds to run. 
// TODO: Could integrate a warning if value is outside of normal range.
void MPU6050::runCalibration(){
    float accX_sum, accY_sum, accZ_sum; // Only runs once so allocation is fine
    float gyroX_sum, gyroY_sum, gyroZ_sum;

    accX_cal = 0;
    accX_cal = 0;
    accY_cal = 0;    
    accZ_cal = 0;
    gyroX_cal = 0;
    gyroY_cal = 0;
    gyroZ_cal = 0;

    for(int i=0; i<500; i++){   // Loop 500 times
        updateAcc();    // Update acceleration
        accX_sum += accX;
        accY_sum += accY;   // Sum the accelerations
        accZ_sum += accZ;

        updateGyro();
        gyroX_sum += gyroX;
        gyroY_sum += gyroY; // Sum the gyro values
        gyroZ_sum += gyroZ;
        delay(4);   // Wait 4ms for 2s total
    }

    // Divide all the values to get average error. 
    // X is calibrated to 9.82 m/s^2 acceleration downwards (g)
    accX_cal = accX_sum / 500.0f;
    accX_cal -= 9.82f;
    accY_cal = accY_sum / 500.0f;    
    accZ_cal = accZ_sum / 500.0f;
    gyroX_cal = gyroX_sum / 500.0f;
    gyroY_cal = gyroY_sum / 500.0f;
    gyroZ_cal = gyroZ_sum / 500.0f;
    
    Serial.print("aX cal: ");
    Serial.print(accX_cal);
    Serial.print(" aY cal: ");
    Serial.print(accY_cal);
    Serial.print(" aZ cal: ");
    Serial.print(accZ_cal);
    
    Serial.print("   gX cal: ");
    Serial.print(gyroX_cal);
    Serial.print(" gY cal: ");
    Serial.print(gyroX_cal);
    Serial.print(" gZ cal: ");
    Serial.println(gyroX_cal);
}

void MPU6050::set_cal_values(float gyroX_cal_set, float gyroY_cal_set, float gyroZ_cal_set, float accelX_cal_set, float accelY_cal_set, float accelZ_cal_set){
    accX_cal = accelX_cal_set - 9.82f;
    accY_cal = accelY_cal_set;    
    accZ_cal = accelZ_cal_set;
    gyroX_cal = gyroX_cal_set;
    gyroY_cal = gyroY_cal_set;
    gyroZ_cal = gyroZ_cal_set;
    Serial.println("Fixed MPU calibration values set!");
}

// Calculates the pitch around Y+-axis from accelerometer
float MPU6050::getAccPitch(){
    return atan(accZ / accX) * 180 / 3.14159;
}