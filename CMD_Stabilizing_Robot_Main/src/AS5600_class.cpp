/* 
COLLINEAR MECANUM DRIVE ROBOT
By Carl Johan Malmerot and Peter Stensson
MF133X Bachelors Project in Mechatronics

All code is also available at https://github.com/CarlMalmerot/CMD_robot.git

This class is used for reading the AS5600 angle sensors.
Also calculates speeds based on the readings.
*/

#include <CMD_robot.h>

// Constructor
AS5600::AS5600(bool direction_set){
    direction = direction_set;
}

// Returns the current angle reading in raw LSB's
int AS5600::get_angle(){
    Wire.beginTransmission(AS5600_ADDRESS);
    Wire.write(ANGLE_ADDRESS);
    error = Wire.endTransmission();
    if(error != 0){
        Serial.print("Wire error ");
        Serial.print(error);
        Serial.println(" when reading AS5600 angle");
    }
    delayMicroseconds(10);
    bytes_received = Wire.requestFrom(AS5600_ADDRESS, (uint8_t) 2);   // rq 2 bytes
    if(bytes_received != 2){
        Serial.println("Wire received wrong byte amount");
    }
    raw_buffer = Wire.read()<<8|Wire.read();    // Read angle bytes
    return static_cast<int>(raw_buffer);
}


// Updates the current rotation speed in rpm
// Also adds angle difference to the cumulative angle change
// Needs to be called on a millisecond basis to not lose track of rotation speed
// At 3000rpm we have 50 revs per second. 4 readings per rev gives 200Hz reading, 5ms interval maximum
// Use get_current_rpm to get the rpm
void AS5600::update_rot_speed(){
    angle = get_angle();
    angle_diff = angle - lastangle; // LSB's
    
    if(angle_diff > 2048){   // If rotated over midpoint in negative direction
        angle_diff -= 4096;
    }
    else if(angle_diff < -2048){     // If rotated over midpoint in positive direction
        angle_diff += 4096;
    }

    // Add the angle change to cumulative count
    if(direction){cumulative_angle -= (float(angle_diff) / 11.37777f);}
    else{cumulative_angle += (float(angle_diff) / 11.37777f);}

    rpm = float(angle_diff) / (float(micros() - last_time) / 1000000.0f); // Current speed, degrees per second
    rpm = rpm * 60 / (360*11.37777f); // Conversion to rpm. Conv from LSB's moved here!

    current_rpm = rpm;
    lastangle = angle;
    last_time = micros();
}

// Gets the current rpm as an int
int AS5600::get_current_rpm(){
    if(direction) {return current_rpm;}
    else {return -current_rpm;}
}

// Zeros the cumulative angle for resetting "home" position
void AS5600::zero_cum_angle(){
    cumulative_angle = 0.0f;
}

// Gets the cumulative angle change
// Returns a long value
// An int would overflow after 182 revolutions, 182*360 = 2^16 (ish)
long AS5600::get_cumulative_angle(){
    return cumulative_angle;
}

// Sets the status bytes. See datasheet for what each setting means
void AS5600::set_filters(){
    Wire.beginTransmission(AS5600_ADDRESS);    // Initialise a transmission
    Wire.write(AS5600_CONF);
    Wire.write(0x809);   // Chosen filter setting
    error = Wire.endTransmission();     // Transmit and then end transmission.
    if(error != 0){
        Serial.print("Wire error ");
        Serial.print(error);
        Serial.println(" when setting AS5600 filters!!");
        delay(3000);
    }
}



/*      STATUS METHODS       */
// These methods are mainly used for troubleshooting
// They check all the onboard status registers and translates to errors


// Checks that Wire returns 0 when polling the AS5600
// Prints any success or errors over serial
// Delays for reading the message
int AS5600::is_connected(){
    Wire.beginTransmission(AS5600_ADDRESS);
    error = Wire.endTransmission();
    
    if(error != 0){
        Serial.print("Wire error ");
        Serial.print(error);
        Serial.println(" when connecting to AS5600!!");
        delay(3000);
        return 1;
    }
    else{
        return 0;
    }
}

// Returns the status byte converted to int
// Used in get_MagnetStatus which returns the state, this returns the raw value
int AS5600::get_status(){
    Wire.beginTransmission(AS5600_ADDRESS);
    Wire.write(STATUS_ADDRESS);
    Wire.endTransmission();
    Wire.requestFrom(AS5600_ADDRESS, (uint8_t) 1);

    raw_buffer_8bit = Wire.read();    // Read status byte

    return static_cast<int>(raw_buffer_8bit);
}


// Returns the setting of the AGC, Automatic Gain Control
// 0 to 255 in 5V mode, 0 to 128 in 3.3V
// The gain value should be in the center of this range
// The airgap of the physical system can be adjusted to achieve this value
// Also use get_MagnetStatus for telling if the field is within spec for the AGC via the status bits
int AS5600::get_AGC(){
    Wire.beginTransmission(AS5600_ADDRESS);
    Wire.write(AS5600_AGC);
    Wire.endTransmission();
    Wire.requestFrom(AS5600_ADDRESS, (uint8_t) 1);   // rq

    raw_buffer_8bit = Wire.read();    // Read status byte

    return static_cast<int>(raw_buffer_8bit);
}

// From datasheet: The MAGNITUDE register indicates the magnitude value of the internal CORDIC.
int AS5600::get_Magnitude(){
    Wire.beginTransmission(AS5600_ADDRESS);
    Wire.write(AS5600_AGC);
    Wire.endTransmission();
    Wire.requestFrom(AS5600_ADDRESS, (uint8_t) 2);   // rq

    raw_buffer = Wire.read()<<8|Wire.read();    // Read 2 bytes

    return static_cast<int>(raw_buffer);
}

// Returns the status of magnet reading. Values: 
// 0: Not detected
// 1: Detected, within spec 
// 2: Low field strength
// 3: Too high field strength
// 4: Error reading status
int AS5600::get_MagnetStatus(){
    static int status = get_status();
    if(status == 0) return 0;
    else if(status == AS5600_MAGNET_DETECT) return 1;
    else if(status == AS5600_MAGNET_LOW) return 2;
    else if(status == AS5600_MAGNET_HIGH) return 3;
    else{
        Serial.println("Error reading magnet status.");
        return 4;
    }
}