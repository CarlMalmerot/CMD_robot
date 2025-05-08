/* 
COLLINEAR MECANUM DRIVE ROBOT
By Carl Johan Malmerot and Peter Stensson
MF133X Bachelors Project in Mechatronics

All code is also available at https://github.com/CarlMalmerot/CMD_robot.git

This is the main arduino file for running the robot.
Class declarations can be found in CMD_robot.h, and class definitions in their own files.
Exception is the Kalman filter and multiplexer, defined directly in the header file.
*/

#include <CMD_robot.h>
#include <WiFi.h>
#include <esp_now.h>

/* PIN DECLARATIONS */
#define SDA_1_PIN 21
#define SCL_1_PIN 22
#define BUZZER_PIN 15

// Motor PWM-pins
#define M0_PWM_L 16 
#define M0_PWM_R 17
#define M1_PWM_L 32
#define M1_PWM_R 25
#define M2_PWM_L 14
#define M2_PWM_R 12
#define M3_PWM_L 2
#define M3_PWM_R 13

// Motor enable pins
#define M0M1_ENABLE 4
#define M2M3_ENABLE 5

 // Voltage divider ADC pin
#define VOLTAGE_SENS 35  

#define MAX_ANGLE 15  // Max allowable tilt

// Function to call for setting pinmodes
void set_pinModes(){
  pinMode(SDA_1_PIN, OUTPUT);
  pinMode(SCL_1_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(M0_PWM_L, OUTPUT);
  pinMode(M0_PWM_R, OUTPUT);
  pinMode(M1_PWM_L, OUTPUT);
  pinMode(M1_PWM_R, OUTPUT);
  pinMode(M2_PWM_L, OUTPUT);
  pinMode(M2_PWM_R, OUTPUT);
  pinMode(M3_PWM_L, OUTPUT);
  pinMode(M3_PWM_R, OUTPUT);
  pinMode(M0M1_ENABLE, OUTPUT);
  pinMode(M2M3_ENABLE, OUTPUT);
  pinMode(VOLTAGE_SENS, INPUT);
}

/* COEFFICIENT VARIABLE DEFINITIONS */

// Kalman filter
const float kalman_Q = 25.0f;  // (dt**2) * 5 * 5 in py. dt is added in method. Base: 25.0f.
const float kalman_R = 0.5625f; // 0.75 ** 2 in py. Base: 0.5625f

// Speed PID
const float speed_Kp = 6.0f;  // 5 -> 1m/s error is 5 degrees output.
const float speed_Ki = 0.4f;  // 0.2
const float speed_Kd = 0.0f;

// Angle PID
const float angle_Kp = 400.0f;
const float angle_Ki = 16.0f;
const float angle_Kd = 0.0f;

// Motor PID
const float motor_Kp = 0.2f;
const float motor_Ki = 0.1f;
const float motor_Kd = 0.0f;

// Yaw PID
const float yaw_Kp = 8.0f;
const float yaw_Ki = 0.5f;
const float yaw_Kd = 0.0f;

// Control input modifiers
const float control_max_speed = 0.3f; // Max control input value, target in m/s
const float control_division = 100.0f / control_max_speed; // 100/0.5 -> Divide by 200.
float mapped_control_speed;

const float side_control_modifier = 4.0f;
float yaw_target;



/* OBJECT CREATIONS */

MPU6050 mpu(LPF_BW_98HZ, GYRO_MAX_500DEG, g_4);
MUX mux;
Kalman_Angle kalman_filter(kalman_Q, kalman_R);

PID speed_PID(speed_Kp, speed_Ki, speed_Kd, -3.0, 3.0, 0.2f);
PID angle_PID(angle_Kp, angle_Ki, angle_Kd, -1700, 1700, 0.3f);
PID motor_0_PID(motor_Kp, motor_Ki, motor_Kd, -255, 255, 0.3f);
PID motor_1_PID(motor_Kp, motor_Ki, motor_Kd, -255, 255, 0.3f);
PID motor_2_PID(motor_Kp, motor_Ki, motor_Kd, -255, 255, 0.3f);
PID motor_3_PID(motor_Kp, motor_Ki, motor_Kd, -255, 255, 0.3f);
PID yaw_PID(yaw_Kp, yaw_Ki, yaw_Kd, -700, 700, 0.3f);

AS5600 angle_sensors[4] {{false}, 
                         {false}, 
                         {true}, 
                         {true}};

Motor motors[4] {{M0_PWM_L, M0_PWM_R, false},
                 {M1_PWM_L, M1_PWM_R, false}, 
                 {M2_PWM_L, M2_PWM_R, true}, 
                 {M3_PWM_L, M3_PWM_R, true}};


/* Catastrophic error catcher.
Sends the program into an infinite loop.
Beeps at 1kHz every 2 seconds to indicate error. */
void catastrophic_error(int error_code) {
  while(true){
    Serial.print("Catastrophic error caugth!! ");
    Serial.print("Error code: ");
    Serial.println(error_code);
    tone(BUZZER_PIN,1000, 500);
    delay(2000);
  }
}


/* ESP-NOW RELATED FUNCTIONS */

control_msg control_input;
telemetry_msg telemetry_output;
const uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // Broadcast to all devices


// Called when data is received
void receive_cb(const uint8_t *macAddr, const uint8_t *data, int len) {   
  memcpy(&control_input, data, sizeof(control_input));

  // Calculate moving setpoint for yaw target
  yaw_target += 1.4f * (float(control_input.control_yaw_rate) / 100);

}

// Called when data is sent
void sent_cb(const uint8_t *macAddr, esp_now_send_status_t status) {
  // Nothing needed here
}

// Broadcasts a message to all devices
void broadcast(telemetry_msg message) {
  esp_now_peer_info_t peerInfo = {};
  memcpy(&peerInfo.peer_addr, broadcastAddress, 6);
  if (!esp_now_is_peer_exist(broadcastAddress))  
  {
    esp_now_add_peer(&peerInfo);
  }
  // Send message
  esp_err_t result = esp_now_send(broadcastAddress, (const uint8_t *) &message, sizeof(message));
  // Print results to serial monitor
  if (result == ESP_OK){
    //Serial.println("Broadcast message success");
    return;
  }
  else{
    Serial.print("WARNING: ESP-NOW error ");
    Serial.print(result);
    Serial.println(" when broadcasting telemetry");
  }
}


/* SETUP */

void setup() {
  Serial.begin(115200);
  set_pinModes();
  Serial.println(" Running... ");
  ledcSetup(0,1000,8);  // Initialise LED-controller (ledc) to avoid annoying message in serial (tone uses ledc, still works without but gives an error)
  ledcAttachPin(BUZZER_PIN, 0);  // Attach pin 4 to ledc
  tone(BUZZER_PIN, 1000, 300); // Startup tone

  analogWriteFrequency(20000);  // Set PWM-frequency to 20kHz

  Wire.begin(21, 22);  // Start I2C
  Wire.setClock(400000);  // Set to fast mode, 400kHz
  
  delay(50);  // Delay before using i2c

  for(int i = 0; i < 4; i++) {
    motors[i].disable();  // Zero any writes to the motors
    mux.set_address(i);  // Set Mux address
    static int error = angle_sensors[i].is_connected(); // Check active connection
    if(!error){   // If an error is found it's printed in the class
      Serial.print("AS5600 no.");
      Serial.print(i);
      Serial.println(" connected!");
      angle_sensors[i].set_filters();
    }
    else{
      Serial.print("Error when connecting to AS5600 no.");
      Serial.println(i);
      catastrophic_error(1); // Else go to the error-catching loop.
    }
    // Update the speed and zero starting angle
    angle_sensors[i].update_rot_speed();
    angle_sensors[i].zero_cum_angle();  
  }
  
  if(mpu.reset() != 0) {   // Reset the MPU
    catastrophic_error(2);
  }

  mpu.setDLPF(LPF_BW_98HZ);  // Set initial MPU settings 
  mpu.setGyroRange(GYRO_MAX_500DEG);
  mpu.setAccelRange(g_4);

  //mpu.runCalibration(); // MPU Calibration
 
  mpu.set_cal_values(-0.96f, -0.96f, -0.96f, 10.22f, -0.14f, 2.45f);  // Setting fixed calibration values for speed

  // ESP-NOW Initialisation
  WiFi.mode(WIFI_STA);
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESP-NOW Init Success");
    Serial.print("MAC Address: ");
    Serial.println(WiFi.macAddress());  
    esp_now_register_recv_cb(receive_cb);  // Register callback functions
    esp_now_register_send_cb(sent_cb); 
  }
  else {
    Serial.println("ESP-NOW Init Failed");
    delay(3000);
    ESP.restart();
    catastrophic_error(3);
  }

  delay(2000);  // Allow time for setting the robot down
  digitalWrite(M0M1_ENABLE, HIGH);  // Enable H-bridges
  digitalWrite(M2M3_ENABLE, HIGH);  
  tone(BUZZER_PIN, 3000, 200);
}


/* FUNCTION DECLARATIONS */
float calc_speed(int shaft_rpm);
float calc_avg_speed(float wheel_speeds[4]);
double calc_voltage(int adc_value);
void print_rpms(float rpms[4]);
void print_pwms(int pwms[4]);
float low_pass(float new_value, float last_value, float coefficient);


/* LOOP VARIABLES */
const float gear_ratio = 20.0f/98.0f;
float mpu_pitch, mpu_gyro, kalman_angle, angle_command, angle_rpm_command, fwd_speed, wheel_speed[4], last_fwd_speed;
unsigned long telemetry_time = millis();
unsigned long loop_time = micros();   // For loop-timing
unsigned long print_time = millis();  // For test-prints 

bool cut_out = false; // To keep the PID's from growing when angle is exceeded

int motor_pwms[4], last_pwms[4], yaw_pid_output, side_output, shaft_rpm[4];
float last_kalman_angle,last_angle_command, last_angle_pwm, motor_rpm[4], last_rpms[4], current_yaw;
float last_safe_angle, last_angle;

unsigned long loop_1_timer, loop_2_timer, loop_3_timer,pos_pid_timer,kalman_timer, gyro_yaw_timer;


void loop() {
  loop_time = micros();
  // Speed PID and wheel angle control
  if(micros() - pos_pid_timer > 10000){
    // +/-100 div by 200 means target of 0.5m/s
    if(!cut_out){
      angle_command = speed_PID.calc_pid(float(-control_input.control_speed)/control_division, -fwd_speed, 0);
      angle_command = low_pass(angle_command, last_angle_command, 0.3f);
      last_angle_command = angle_command;

      // Integrate gyro X to keep track of yaw angle
      current_yaw -= mpu.getGyroX() * (micros() - gyro_yaw_timer) / 1000000;
      gyro_yaw_timer = micros();

      // Calc RPM-outputs to reach desired yaw angle
      yaw_pid_output = yaw_PID.calc_pid(yaw_target, current_yaw, 0);

      //Side output directly converted to RPM targets
      side_output = -control_input.control_side_input * side_control_modifier;

    }
    pos_pid_timer = micros();
  }

  // Loop 1: ~500 uS
  // Update the kalman filter more often to keep it up to date
  if(micros() - kalman_timer > 1500){
    // Update MPU readings
    mpu.updateAcc();
    //mpu.updateGyro_Y();
    mpu.updateGyro();
    // Get MPU-readings
    mpu_pitch = mpu.getAccPitch();
    mpu_gyro = mpu.getGyroY();

    if(mpu_gyro > 480){
      tone(BUZZER_PIN, 1000, 200);
      mpu_gyro = 480.0f;
    }
    
    // Calc Kalman angle approximation
    kalman_angle = kalman_filter.calc_angle(mpu_pitch, mpu_gyro);
    
    // Ensure no unwanted spikes get through
    if(abs(kalman_angle - last_angle) > 10){
      kalman_angle = last_safe_angle;
    }
    else{
      last_safe_angle = kalman_angle;
    }
    last_angle = kalman_angle;
    
    // Filter the kalman output a bit
    // kalman_angle = low_pass(kalman_angle, last_kalman_angle, 0.5);
    // last_kalman_angle = kalman_angle;

    kalman_timer = micros();
  }

  
  
  // Loop 2: Angle pid and motor output. Updates rotation speeds as well to the speed-PID.
  // Time: 550uS
  if(micros() - loop_1_timer > 5000){
    // Calc the common motor speed for balancing
    // Input: A target angle. Output: Motor PWM
    if(!cut_out){
      angle_rpm_command = angle_PID.calc_pid(angle_command, kalman_angle, 0);
      angle_rpm_command = low_pass(angle_rpm_command, last_angle_pwm, 0.5);
      last_angle_pwm = angle_rpm_command;
    }
    
    // Calc output of PWM duty cycle to the motor PID from the rotation speed
    motor_pwms[0] = motor_0_PID.calc_pid(angle_rpm_command + yaw_pid_output + side_output, motor_rpm[0], 0);
    motor_pwms[1] = motor_1_PID.calc_pid(angle_rpm_command + yaw_pid_output - side_output, motor_rpm[1], 0);
    motor_pwms[2] = motor_2_PID.calc_pid(angle_rpm_command - yaw_pid_output + side_output, motor_rpm[2], 0);
    motor_pwms[3] = motor_3_PID.calc_pid(angle_rpm_command - yaw_pid_output - side_output, motor_rpm[3], 0);
    
    // Write duty cycle to the motors
    if(abs(kalman_angle) <= MAX_ANGLE){
      for(int i=0;i<4;i++){
        motors[i].write_duty(motor_pwms[i]);
      }
      cut_out = false;
    }
    else{
      // Disable motors if it's fallen over. Reset all PID-integrals as well.
      cut_out = true;
      motors[0].disable();
      motors[1].disable();
      motors[2].disable();
      motors[3].disable();
      angle_PID.reset_integral();
      speed_PID.reset_integral();
      motor_0_PID.reset_integral();
      motor_1_PID.reset_integral();
      motor_2_PID.reset_integral();
      motor_3_PID.reset_integral();
      angle_sensors[1].zero_cum_angle();
      angle_sensors[2].zero_cum_angle();
      last_safe_angle = 0;
      delay(1000);  // Wait to give time for standing it up again
    }

    loop_1_timer = micros();
  }

  
  // Wheel speed loop
  // ~650 to 750 uS w/o motor pid, 1ms with
  if(micros() - loop_3_timer > 2000){
    // Calculate the current speed at each wheel
    for(int i = 0; i < 4; i++) {
      mux.set_address(i);  // Set Mux address
      angle_sensors[i].update_rot_speed();
      motor_rpm[i] = low_pass(float(angle_sensors[i].get_current_rpm()), last_rpms[i], 0.15);
      last_rpms[i] = motor_rpm[i];
      wheel_speed[i] = calc_speed(angle_sensors[i].get_current_rpm());
    }
    
    // Calc fwd speed for the speed-PID. Using the average of all 4 wheels
    fwd_speed = calc_avg_speed(wheel_speed);
    fwd_speed = low_pass(fwd_speed, last_fwd_speed, 0.15);
    last_fwd_speed = fwd_speed;

    loop_3_timer = micros();
  }

  // Send telemetry every 30ms
  // Mult by 100 to send only small ints. Divided at receiver for precision.
  if(millis() - telemetry_time > 30){
    telemetry_output.speed = int(fwd_speed*100);
    telemetry_output.kalman_angle = int(kalman_angle*100);
    telemetry_output.raw_angle = int(mpu_pitch*100);
    telemetry_output.raw_gyro = int(mpu_gyro*100);
    telemetry_output.angle_pid = int(angle_rpm_command*100);
    telemetry_output.angle_command = int(angle_command*100);
    telemetry_output.motor_1_pid = motor_pwms[1]*100;
    telemetry_output.motor_2_pid = motor_pwms[2]*100;
    telemetry_output.speed = int(fwd_speed*100);
    telemetry_output.battery_voltage = int(calc_voltage(analogRead(VOLTAGE_SENS))*100);
    telemetry_output.m1_rpm = motor_rpm[1] * 100;
    telemetry_output.m2_rpm = motor_rpm[2] * 100;
    telemetry_output.yaw = int(current_yaw*100);
    telemetry_output.yaw_rate = int(mpu.getGyroX()*100);
    telemetry_output.side_speed_rpm_command = side_output*100;
    broadcast(telemetry_output);
    telemetry_time = millis();
  }
  
  // Timed prints
  if(millis() - print_time > 300){
    Serial.print("Yaw: ");
    Serial.print(current_yaw);
    Serial.print("  Side output: ");
    Serial.print(side_output);
    Serial.print("  Yaw target: ");
    Serial.print(yaw_target);
    Serial.println();
    print_pwms(motor_pwms);

    print_time = millis();
  }

  loop_time = micros() - loop_time;   // Full loop time <2500 uS, varying depending on what runs
}


/* FUNCTION DEFINITIONS */

// Returns the speed in m/s at the wheel
// v = wr = 2*pi*n*r / 60
float calc_speed(int shaft_rpm) {
  return (2.0f*PI*float(shaft_rpm)*0.097f*gear_ratio/2.0f) / 60.0f;
}

/* Returns the average forward speed of all four wheels */
float calc_avg_speed(float wheel_speeds[4]){
  return float((wheel_speeds[0] + wheel_speeds[1] + wheel_speeds[2] + wheel_speeds[3]) / 4);
}

void print_rpms(float rpms[4]) {
  Serial.print(" RPM 1: ");
  Serial.print(rpms[0]);
  Serial.print(" RPM 2: ");
  Serial.print(rpms[1]);
  Serial.print(" RPM 3: ");
  Serial.print(rpms[2]);
  Serial.print(" RPM 4: ");
  Serial.println(rpms[3]);
}

void print_pwms(int pwms[4]) {
  Serial.print("PWM 1: ");
  Serial.print(pwms[0]);
  Serial.print("PWM 2: ");
  Serial.print(pwms[1]);
  Serial.print("PWM 3: ");
  Serial.print(pwms[2]);
  Serial.print("PWM 4: ");
  Serial.println(pwms[3]);
}



// First order low pass filter
float low_pass(float new_value, float last_value, float coefficient){
  return ((1 - coefficient) * last_value + coefficient * (new_value));
}

// Adding 1.7 to compensate error from lower resistance over the voltage divider
double calc_voltage(int adc_value){
  return 1.7 + adc_value * (3.3 / 4096) * 110000 / (110000 - 100000);
}
/* 
R1 = 100k, R2 = 10k
I = U / Rtot
U2 = R2 * I
U2 = U - U1 = Rtot * I - R1 * I = (Rtot - R1) * I = (Rtot - R1) * U / Rtot  -> U = U2 * Rtot / (Rtot - R1)  Checks out!

Measuring ADC: 0-3.3V divided over 4096 steps. 
U_measured = ADC_value * (3.3 / 4096)
*/