/* 
Carl Johan Malmerot, Peter Stensson

Credit to xxx for their article on ESP-NOW, found at ...
 */

#include <CMD_robot.h>
#include <WiFi.h>
#include <esp_now.h>

/* PIN DECLARATIONS */
#define SDA_1_PIN 21
#define SCL_1_PIN 22
#define SDA_2_PIN 18 // FREE AFTER MUX CHANGE
#define SCL_2_PIN 19 // FREE AFTER MUX CHANGE

#define BUZZER_PIN 15
#define RGB_R_PIN 4 // Taken for motor en 3
#define RGB_B_PIN 16  // Swapped L IN 1
#define RGB_G_PIN 17  // Swapped L IN 2

#define MUX_S0_PIN 27 // FREE AFTER MUX CHANGE
#define MUX_S1_PIN 26 // FREE AFTER MUX CHANGE
#define MUX_S2_PIN 25 // FREE AFTER MUX CHANGE

#define M1_EN_L 16  // M1 En Left - Switch: M0 PWM L
#define M1_EN_R 17  // M1 En Right - Switch: M0 PWM R
#define M1_PWM_R 25  // M1 PWM R  OBS: Fel ordning, omvänt i pins jmf med M2. Bytt R och L här.
#define M1_PWM_L 32  // M1 PWM L


// Motor 1: PWM R: 25, PWM L 32, EN L 16, EN R 17. Felkopplade PWM R till 25 ist för 33 men bytt nu. 

#define M2_EN_L 13 // M3 EN L - Switch: M3 PWM L
#define M2_EN_R 2  // M3 EN R - Switch: M3 PWM R
#define M2_PWM_L 14 // M4 PWM L
#define M2_PWM_R 12 // M4 PWM R

#define MOTOR_ENABLE_2 5 // Används ej efter nya H-bryggor
#define MOTOR_ENABLE_3 4 // Används ej efter nya H-bryggor

#define VOLTAGE_SENS 35

// Free pins: 4, 5, 33, 26, 27, 18, 19

// TODO: Lägg till pins för M0 och M3
// Kan koppla EN direkt till 3v3 för att spara pins



void set_pinModes(){
  pinMode(SDA_1_PIN, OUTPUT);
  pinMode(SCL_1_PIN, OUTPUT);
  pinMode(SDA_2_PIN, OUTPUT);
  pinMode(SCL_2_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RGB_R_PIN, OUTPUT);
  pinMode(RGB_B_PIN, OUTPUT);
  pinMode(RGB_G_PIN, OUTPUT);
  pinMode(MUX_S0_PIN, OUTPUT);
  pinMode(MUX_S1_PIN, OUTPUT);
  pinMode(MUX_S2_PIN, OUTPUT);
  pinMode(M1_EN_L, OUTPUT);
  pinMode(M1_EN_R, OUTPUT);
  pinMode(M1_PWM_R, OUTPUT);
  pinMode(M1_PWM_L, OUTPUT);
  pinMode(M2_EN_L, OUTPUT);
  pinMode(M2_EN_R, OUTPUT);
  pinMode(M2_PWM_L, OUTPUT);
  pinMode(M2_PWM_R, OUTPUT);
  pinMode(MOTOR_ENABLE_2, OUTPUT);
  pinMode(MOTOR_ENABLE_3, OUTPUT);
  pinMode(VOLTAGE_SENS, INPUT);
}



/* 

Idé: Skippa att ha en motor-pid och låt angle-pid direkt ge output till motorer. 
Feedback från vinkelsensorer används då bara direkt av speed och position-PID.
Minskar mängden variabler och noise från vinkelsensorer påverkar då inte angle-PID på samma sätt. 

*/

/* COEFFICIENT VARIABLE DEFINITIONS */
// Larger Q - faster change
// Larger R - Faster response, more noise?
const float kalman_Q = 25.0f;  // (dt**2) * 5 * 5 in py. dt is added in method. Base: 25.0f.
const float kalman_R = 0.5625f; // 0.75 ** 2 in py. Base: 0.5625f

// Q 10, R 0.8 verkade bra men ger statiskt fel

const float speed_Kp = 32.0f;  // 5 -> 1m/s error is 5 degrees output.
const float speed_Ki = 3.0f;
const float speed_Kd = 3.5f;

// Bäst: 4,0.5,0.1, med Angle 80,10,1

const float position_Kp = 0.0f;  // 1m fel -> 2 grader command vid 2. 20 typ ok men d-part viktigare.
const float position_Ki = 0.0f;  // Constraining causes error for small min/max?
const float position_Kd = 0.0f; // 9 lite för högt. 6 bäst hittills

const float angle_Kp = 90.0f; // 200 ok med låg d-part
const float angle_Ki = 5.0f;  // 20 ok
const float angle_Kd = 2.0f;  // 4?

const float motor_Kp = 0.15f;
const float motor_Ki = 0.0f;
const float motor_Kd = 0.0f;

// Kaskadreglering: Inre loop bör vara 3 till 10 ggr snabbare än yttre. 


const float control_max = 0.3;
float mapped_control_speed;






/* OBJECT CREATIONS */

MPU6050 mpu(LPF_BW_188HZ, GYRO_MAX_250DEG, g_2);
MUX mux(MUX_S0_PIN, MUX_S1_PIN);
Kalman_Angle kalman_filter(kalman_Q, kalman_R);

PID position_PID(position_Kp, position_Ki, position_Kd, -10, 10, 0.2f);
PID speed_PID(speed_Kp, speed_Ki, speed_Kd, -50, 50, 0.2f); // degrees
PID angle_PID(angle_Kp, angle_Ki, angle_Kd, -1700, 1700, 0.3f); // Motor RPM
PID motor_1_PID(motor_Kp, motor_Ki, motor_Kd, -255, 255, 0.3f); // Motor PWM
PID motor_2_PID(motor_Kp, motor_Ki, motor_Kd, -255, 255, 0.3f);

AS5600 angle_sensors[4] {{false}, 
                         {false}, 
                         {true}, 
                         {true}};

// All four motors are initialised but only two are given signals for the two wheel test
Motor motors[4] {{M1_PWM_L, M1_PWM_R, false}, // Change
                 {M1_PWM_L, M1_PWM_R, false}, 
                 {M2_PWM_L, M2_PWM_R, true}, 
                 {M1_PWM_L, M1_PWM_R, true}};  // Change

/* 
Catastrophic error catcher.
Sends the program into an infinite loop.
Beeps at 1kHz every 2 seconds to indicate error. 
*/
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
const uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


// Called when data is received
void receive_cb(const uint8_t *macAddr, const uint8_t *data, int len) {   
  memcpy(&control_input, data, sizeof(control_input));
  Serial.print("Data received: ");
  Serial.print(len);
  Serial.print("  Speed input: ");
  Serial.print(control_input.control_speed);
  Serial.print("  Yaw rate input: ");
  Serial.print(control_input.control_yaw_rate);

  mapped_control_speed = map(float(control_input.control_speed), -100.0f, 100.0f, control_max, -control_max); // Set zeroing timer for safety?
  Serial.print("Map speed: ");
  Serial.println(mapped_control_speed);
}

// Called when data is sent
void sent_cb(const uint8_t *macAddr, esp_now_send_status_t status) {
  // Serial.print("Last Telemetry Packet Send Status: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");  // Always a success when broadcasting so unnecessary
}

// Broadcasts a message to every device in range
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
  tone(BUZZER_PIN, 1000, 500); // Startup tone

  analogWriteFrequency(20000);  // Set PWM-frequency to 10kHz

  Wire.begin(21, 22);  // Start I2C
  Wire.setClock(400000);  // Set to fast mode, 400kHz
  
  delay(200);  // Delay before using i2c

  for(int i = 1; i < 3; i++) {    // Changed for two wheel test
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

  tone(BUZZER_PIN, 2000, 300);
  mpu.setDLPF(LPF_BW_98HZ);  // Set initial MPU settings. 98Hz good. 
  mpu.setGyroRange(GYRO_MAX_500DEG);
  mpu.setAccelRange(g_2);
  
  /* mpu.runCalibration(); // MPU Calibration


  for(int i = 0; i < 10; i++){
    mpu.runCalibration();
    tone(BUZZER_PIN, 3000, 200);
    delay(500);
  }
 */
  mpu.set_cal_values(-1.83f, -1.83f, -1.83f, 10.43f, -0.14f, 2.48f);

  /* tone(BUZZER_PIN, 1000, 200);
  delay(50);
  tone(BUZZER_PIN, 3000, 300); */

  // ESP-NOW Initialisation
  WiFi.mode(WIFI_STA);
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESP-NOW Init Success");
    Serial.print("MAC Address: ");
    Serial.println(WiFi.macAddress());  
    esp_now_register_recv_cb(receive_cb);  // Callback functions
    esp_now_register_send_cb(sent_cb); 
  }
  else {
    Serial.println("ESP-NOW Init Failed");
    delay(3000);
    ESP.restart();
    catastrophic_error(3);
  }

  // Write high to EN pins, chg to direct 3v3 later
  digitalWrite(M1_EN_L, 1);
  digitalWrite(M1_EN_R, 1);
  digitalWrite(M2_EN_L, 1);
  digitalWrite(M2_EN_R, 1);
}


/* FUNCTION DECLARATIONS */
float calc_speed(int shaft_rpm);
float calc_avg_speed(float wheel_speeds[4]);
double calc_voltage(int adc_value);
float calc_distance(long cum_angle);

void print_rpms(int rpms[4]);
void print_pwms(int pwms[4]);
float low_pass(float new_value, float last_value, float coefficient);

int shaft_rpm[4];
const float gear_ratio = 20.0f/98.0f;
float mpu_pitch, mpu_gyro, kalman_angle, angle_command, angle_rpm_command, fwd_speed, wheel_speed[4], last_speeds[5];
unsigned long telemetry_time = millis();
unsigned long loop_time = micros();   // For loop-timing
unsigned long print_time = millis();  // For test-prints
unsigned long pwm_time = micros();  

bool cut_out = false; // To keep the PID's from growing when angle is exceeded

int motor_pwms[4], motor_rpm[4], last_rpms[4];
int yaw_outputs[4];
long cum_angles[4], cum_avg;
float last_kalman_angle, last_gyro, last_angle_command, last_angle_pwm;
bool mode;  // False = Position hold, True = Speed hold

unsigned long test_timer_1, test_timer_2;
unsigned long loop_1_timer, loop_2_timer, pos_pid_timer,kalman_timer; // Loop 1 every 10 ms, Loop 2 every 2 ms

void loop() {
  // Calculate the angle command


  //angle_command = speed_PID.calc_pid(control_input.control_speed, float(fwd_speed), 0);

  // Implement a statement here to figure out if to use position pid or speed pid

  if(micros() - pos_pid_timer > 15000){
    // +/-100 div by 100 means target of 1m/s
    if(!cut_out){angle_command = speed_PID.calc_pid(mapped_control_speed, -fwd_speed, 0);}
    angle_command = low_pass(angle_command, last_angle_command, 0.3f);
    last_angle_command = angle_command;
    pos_pid_timer = micros();
  }

  // Position pid updates every 50ms
  // Implement zeroing the distance later
/*   if(micros() - pos_pid_timer > 20000){
    for(int i = 1; i < 3; i++){
      cum_angles[i] = angle_sensors[i].get_cumulative_angle();
    }
    //cum_avg = (cum_angles[0] + cum_angles[1] + cum_angles[2] + cum_angles[3]) / 4;  // Average needed?

    cum_avg = (cum_angles[1] + cum_angles[2]) / 2;

    if(!cut_out){angle_command = position_PID.calc_pid(0, float(calc_distance(cum_avg)), 0);}
    pos_pid_timer = micros();
  } */


  // Loop 1: 700 uS, sometimes 1500 uS? Jumps between and stays for a while at each but mostly 700 uS

  test_timer_1 = micros();

  // Update the kalman filter more often to keep it up to date
  if(micros() - kalman_timer > 1500){
    // Update MPU readings
    mpu.updateAcc();
    mpu.updateGyro_Y();
    // Get MPU-readings
    mpu_pitch = mpu.getAccPitch();
    mpu_gyro = mpu.getGyroY();

    mpu_pitch = low_pass(mpu_pitch, last_kalman_angle, 0.5f);
    mpu_gyro = low_pass(mpu_gyro, last_gyro, 0.5f);

    last_kalman_angle = mpu_pitch;
    last_gyro = mpu_gyro;

    // Calc Kalman angle approximation
    kalman_angle = kalman_filter.calc_angle(mpu_pitch, mpu_gyro);
    kalman_timer = micros();
  }
  
  // Loop 1, Angle pid is slowed down
  if(micros() - loop_1_timer > 6000){
    // Calc the common motor speed for balancing
    // Input: A target angle. Output: A rotation speed to the motors
    if(!cut_out){angle_rpm_command = angle_PID.calc_pid(angle_command, kalman_angle, 0);}
    angle_rpm_command = low_pass(angle_rpm_command, last_angle_pwm, 0.3);
    last_angle_pwm = angle_rpm_command;
    loop_1_timer = micros();
  }

  test_timer_1 = micros() - test_timer_1;
  // TODO: Calc motor speeds from the yaw input and add on to the motor speed commands here!


  // Loop 2: 800 uS

  test_timer_2 = micros();

  // Loop 2 at a faster rate, for motor driving
  if(micros() - loop_2_timer > 1500){
    // Calculate the current speed at each wheel
    for(int i = 1; i < 3; i++) {    // TWO WHEEL TEST 
      mux.set_address(i);  // Set Mux address
      angle_sensors[i].update_rot_speed();
      motor_rpm[i] = low_pass(angle_sensors[i].get_current_rpm(), last_rpms[i], 0.15);
      last_rpms[i] = motor_rpm[i];
      wheel_speed[i] = calc_speed(angle_sensors[i].get_current_rpm());
    }
    
    fwd_speed = calc_avg_speed(wheel_speed);
    fwd_speed = low_pass(fwd_speed, last_speeds[0], 0.1);
    last_speeds[0] = fwd_speed;
    
/*     // Averaging for input to the speed-pid
    last_speeds[4] = last_speeds[3];
    last_speeds[3] = last_speeds[2];
    last_speeds[2] = last_speeds[1];
    last_speeds[1] = last_speeds[0];
    last_speeds[0] = fwd_speed;
    fwd_speed = (last_speeds[0] + last_speeds[1] + last_speeds[2] + last_speeds[3] + last_speeds[4]) / 5; */

    // Calc output of PWM duty cycle to the motor PID from the rotation speed
    motor_pwms[0] = 0;
    motor_pwms[1] = motor_1_PID.calc_pid(angle_rpm_command, motor_rpm[1], 0);
    motor_pwms[2] = motor_2_PID.calc_pid(angle_rpm_command, motor_rpm[2], 0);
    motor_pwms[3] = 0;
  
    loop_2_timer = micros();
  }
  
  if(abs(kalman_angle) <= 23){
    motors[1].write_duty(motor_pwms[1]);
    motors[2].write_duty(motor_pwms[2]);
    cut_out = false;
  }
  else{
    cut_out = true;
    motors[1].disable();
    motors[2].disable();
    angle_PID.reset_integral();
    position_PID.reset_integral();
    speed_PID.reset_integral();
  }

  test_timer_2 = micros() - test_timer_2;


  // PWM Left ger framåt på vänster sida, bakåt på höger

/*   digitalWrite(LEFT_IN_3, 0);
  digitalWrite(LEFT_IN_4, 1);
  analogWrite(MOTOR_ENABLE_2, 200);

  digitalWrite(RIGHT_IN_1, 1);
  digitalWrite(RIGHT_IN_2, 0);
  analogWrite(MOTOR_ENABLE_3, 200); */

  // Send telemetry every 20ms
  // Mult by 100 to send ints. Divided at receiver.
  if(millis() - telemetry_time > 50){
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
    broadcast(telemetry_output);
    telemetry_time = millis();

    /* 
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
    */


  }
  


  // Timed prints
  if(millis() - print_time > 300){
/*     Serial.print("  Motor 0: ");
    Serial.print(angle_sensors[0].get_angle());
    Serial.print("  Motor 1: ");
    Serial.print(angle_sensors[1].get_angle());
    Serial.print("  Motor 2: ");
    Serial.print(angle_sensors[2].get_angle());
    Serial.print("  Motor 3: ");
    Serial.println(angle_sensors[3].get_angle()); */

    //print_rpms(motor_rpm);
    //Serial.println(telemetry_output.battery_voltage);
    //Serial.println(analogRead(VOLTAGE_SENS));
    
/*     Serial.print("Timer 1: ");
    Serial.print(test_timer_1);
    Serial.print("  Timer 2: ");
    Serial.println(test_timer_2); */
    
    // mux.set_address(1);
    // Serial.print(angle_sensors[1].get_MagnetStatus());
    // Serial.print("  ");
    // mux.set_address(2);
    // Serial.print(angle_sensors[2].get_MagnetStatus());  // 1 target
    // Serial.print("  ");
    // mux.set_address(3);
    // Serial.print(angle_sensors[3].get_MagnetStatus());
    // Serial.print("  ");
    // mux.set_address(1);
    // Serial.print(angle_sensors[1].get_AGC()); // 64 target
    // Serial.print("  ");
    // mux.set_address(2);
    // Serial.print(angle_sensors[2].get_AGC());
    // Serial.print("  ");
    // mux.set_address(3);
    // Serial.print(angle_sensors[3].get_AGC());

    // 16.85 ute, 15.0 inne -> flytta in 1.8

/*     Serial.print(angle_sensors[1].get_cumulative_angle());
    Serial.print("  ");
    Serial.print(angle_sensors[2].get_cumulative_angle());
    Serial.print("  ");
    Serial.print(calc_distance(angle_sensors[1].get_cumulative_angle()));
    Serial.print("  ");
    Serial.print(calc_distance(angle_sensors[2].get_cumulative_angle())); */

    Serial.print(angle_command);

    Serial.println(); 
    print_time = millis();
  }
  // Serial.print("Avg wheel speed: ");
  // Serial.println(calc_avg_speed(wheel_speed));

  
/*   while(micros() - loop_time < 3000){
    yield();  // Wait for 3ms if loop is faster
  } */

/*   if(micros() - loop_time > 10000){    // Slow loop warning
    Serial.print("Slow loop, time: ");
    Serial.println(micros() - loop_time);
  } */
  loop_time = micros();
}

/*
Speed control commands a pitch angle.
Angle control commands motor RPMs.
Motor control commands PWM to motors.
*/


/* FUNCTION DEFINITIONS */

// Returns the speed in mm/s at the wheel
// v = wr = 2*pi*n*r / 60
float calc_speed(int shaft_rpm) {
  return (2.0f*PI*float(shaft_rpm)*0.097f*gear_ratio/2.0f) / 60.0f;
}

/* Returns the average forward speed of all four wheels */
float calc_avg_speed(float wheel_speeds[4]){
  return float((wheel_speeds[0] + wheel_speeds[1] + wheel_speeds[2] + wheel_speeds[3]) / 2); // CHANGE FOR 4 WHEELS
}

// Returns distance in meters based on cumulative motor angle
// cum_angle / gear_ratio = wheel rotation angle
// wheel rotations = wheel rot. angle / 360
// wheel rotations * 2*pi*r / 360 = distance
float calc_distance(long cum_angle){
  return float(cum_angle * PI * 0.097 * gear_ratio / 360);
}

void print_rpms(int rpms[4]) {
  Serial.print("RPM 1: ");
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

// Adding 1.7 to compensate error from lower resistance over the voltage divider
double calc_voltage(int adc_value){
  return 1.7 + adc_value * (3.3 / 4096) * 110000 / (110000 - 100000);
}


float low_pass(float new_value, float last_value, float coefficient){
  return ((1 - coefficient) * last_value + coefficient * (new_value));
}

/* 
R1 = 100k, R2 = 10k

I = U / Rtot
U2 = R2 * I
U2 = U - U1 = Rtot * I - R1 * I = (Rtot - R1) * I = (Rtot - R1) * U / Rtot  -> U = U2 * Rtot / (Rtot - R1)  Checks out!
Drar 6mW som max

Mätning ADC: 0-3.3V delat på 4096 steg. 
Umät = ADC * (3.3 / 4096)
*/

// 2.127 vid 25.2, 1.76 vid 21
// 19.3 VID 21v, 23,5 vid 25,2V -> 1.7V offset, plussa på

/* void calc_yaw_output(int &yaw_array){
  static const float r_1 = 0.2;
  static const float r_2 = 0.26;

  static int inner_speed = r_1 * control_input.control_yaw_rate * 3.14159 / 180;
  static int outer_speed = r_2 * control_input.control_yaw_rate * 3.14159 / 180;

  static int output[2] = {{}, {}};
} */
