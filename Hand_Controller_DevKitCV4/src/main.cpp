/* 
COLLINEAR MECANUM DRIVE ROBOT
By Carl Johan Malmerot and Peter Stensson
MF133X Bachelors Project in Mechatronics

All code is also available at https://github.com/CarlMalmerot/CMD_robot.git

This is the main file for the hand controller.
Includes controller.h which contains a few bitmaps and all the library includes needed.
Uses Adafruit GFX to drive the SH110X screen.
*/

#include <controller.h>

// Pin definitions

#define LS_x 36
#define LS_y 35
#define LS_sw 26

#define RS_x 39
#define RS_y 33
#define RS_sw 25
#define BUZZER 18

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 128 // OLED display height, in pixels
#define OLED_RESET -1     // can set an oled reset pin if desired

#define MIDPOINT_MEASURED 1875
#define MIDPOINT_ACTUAL 2048
#define DEADZONE 100

// Use SH110X_WHITE for clarity with this library
Adafruit_SH1107 display = Adafruit_SH1107(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 1000000, 100000);


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
} telemetry_msg;

typedef struct control_msg {
  int control_speed;
  int control_yaw_rate;
  int control_side_input;
} control_msg;

control_msg control_input;
int display_mode = 0;
int ls_x, ls_y, rs_x, rs_y, deadzone;
bool ls_sw, rs_sw, ls_pressed, rs_pressed;


bool is_connected;  // True if established connection to robot. Changes state if success sending data.
unsigned long cross_timer;
bool draw_cross;

telemetry_msg received_telem;


// MAC Address of responder - edit as required
uint8_t broadcastAddress[] = {0x64, 0xb7, 0x08, 0xcc, 0xcd, 0xc8};
// 64:b7:08:cc:cd:c8

// Peer info
esp_now_peer_info_t peerInfo;

// Callback function called when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status:");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if(status == ESP_NOW_SEND_SUCCESS){
    Serial.println("Delivery Success");
    is_connected = true;
  }
  else{
    Serial.println("Delivery Fail");
    is_connected = false;
  }
}


// Function declarations
int stick_map(int value);
void draw_startup();
void draw_sticks();
void draw_raw_telem();
void draw_control();
void set_display_mode();





void receive_cb(const uint8_t *macAddr, const uint8_t *data, int len){
  memcpy(&received_telem, data, sizeof(received_telem));
  Serial.println("Telemetry received");
}

void setup() {
  // Set up Serial Monitor
  Serial.begin(115200);
  // Short delay to allow serial monitor to connect if needed
  delay(500);
  Serial.println("Serial Initialized");

  // Initialize OLED
  // No need for Wire.begin() separately, display.begin() handles it.
  if(!display.begin(0x3D, true)) { // Address 0x3D, perform reset
    Serial.println(F("SH110X allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  Serial.println("Display Initialized");
  display.clearDisplay();
  display.display(); // Show initial message
  delay(1000); // Allow screen to start up fully and show message

/*   Serial.println("Drawing startup screen...");
  draw_startup();
  Serial.println("Startup screen done."); */

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.print("WiFi Mode Set. MAC Address: ");
  Serial.println(WiFi.macAddress());

  // Initialize GPIO pins
  pinMode(BUZZER, OUTPUT);
  pinMode(LS_x, INPUT);
  pinMode(LS_y, INPUT);
  pinMode(RS_x, INPUT);
  pinMode(RS_y, INPUT);
  pinMode(LS_sw, INPUT_PULLUP);
  pinMode(RS_sw, INPUT_PULLUP);

  Serial.println("GPIO Initialized");

  // Test buzzer
  //tone(BUZZER, 1000, 300);
  
  Serial.println("Initializing ESP-NOW...");
  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    // Optionally add display feedback here too
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("ESP-NOW Init Failed!");
    display.display();
    for(;;); // Don't proceed, loop forever
  }
  Serial.println("ESP-NOW Initialized");

  // Register callbacks
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(receive_cb);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    // Optionally add display feedback
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("ESP-NOW Add Peer Failed!");
    display.display();
    for(;;); // Don't proceed, loop forever
  }
  Serial.println("ESP-NOW Peer Added");
  Serial.println("Setup Complete.");

  // Print MAC Address to Serial monitor
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  display_mode = 0;
  set_display_mode();
}


unsigned long loop_timer = micros();

void loop() {

  // Read analog inputs with mapping center correction
  ls_x = stick_map(analogRead(LS_x));
  ls_y = stick_map(analogRead(LS_y));
  rs_x = stick_map(analogRead(RS_x));
  rs_y = stick_map(analogRead(RS_y));

  // Read digital inputs, LOW when pressed
  ls_sw = digitalRead(LS_sw);
  rs_sw = digitalRead(RS_sw);

/*   // Print values to Serial Monitor for debugging
  Serial.print("LS x:"); Serial.print(ls_x);
  Serial.print("  LS y:"); Serial.print(ls_y);
  Serial.print("  RS x:"); Serial.print(rs_x);
  Serial.print("  RS y:"); Serial.print(rs_y);
  Serial.print("  LS sw:"); Serial.print(ls_sw); // 1 = not pressed, 0 = pressed
  Serial.print("  RS sw:"); Serial.println(rs_sw); // 1 = not pressed, 0 = pressed
 */
  // --- Prepare control message ---
  control_input.control_speed = map(ls_y, 0, 4095, -100, 100); // Inverted Y
  control_input.control_yaw_rate = map(ls_x, 0, 4095, 100, -100);
  control_input.control_side_input = map(rs_x, 0, 4095, -100, 100);

  // --- Send message via ESP-NOW ---
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &control_input, sizeof(control_input));

  if (result == ESP_OK) {
    Serial.println("ESP-NOW Send OK");
  } else {
    Serial.print("ESP-NOW Send Error: ");
    Serial.println(result); // Print the specific error code
  }

  // Switch screen with RS button
  if(!rs_sw & !rs_pressed){
    rs_pressed = true;
    display_mode++;
    if(display_mode > 2){
      display_mode = 0;
    }
  }
  else if(rs_sw){
    rs_pressed = false;
  }

  set_display_mode();

  // 50Hz timer
  while(micros() - loop_timer < 20000){
    yield();
  }
}



// FUNCTION DEFINITIONS

// Maps the actual range to the 0-4096 it should be
int stick_map(int value) {
  // Check if value is within the deadzone
  if (abs(value - MIDPOINT_MEASURED) <= DEADZONE) {
    return MIDPOINT_ACTUAL; // Return the middle of the output range
  }

  // Map the value from the ranges
  if (value < MIDPOINT_MEASURED) {
    // Map from 0 to midpoint - deadzone/2
    return map(value, 0, MIDPOINT_MEASURED - DEADZONE, 0, MIDPOINT_ACTUAL - DEADZONE);
  } else {
    // Map from midpoint + deadzone/2 to 4096
    return map(value, MIDPOINT_MEASURED + DEADZONE, 4096, MIDPOINT_ACTUAL + DEADZONE, 4096);
  }
}

void set_display_mode(){
  switch (display_mode) {
    case 0:
      draw_control();
      break;
    case 1:
    draw_sticks();
      break;
    case 2:
      draw_raw_telem();
      break;
  }
}


void draw_startup(){
  display.clearDisplay(); 

  // --- Draw the robot bitmap ---
  int16_t robot_w = 56;
  int16_t robot_h = 110;
  int16_t robot_x = (display.width()  - robot_w) / 2;
  int16_t robot_y = (display.height() - robot_h) / 2;

  for(int i = -60; i < robot_x; i+=2){
    display.clearDisplay();
    display.drawBitmap(i, robot_y, robot, robot_w, robot_h, SH110X_WHITE);
    display.display(); // Show the bitmap(s)
  }

  delay(1000);

  for(int i = robot_x; i < 190; i+=2){
    display.clearDisplay();
    display.drawBitmap(i, robot_y, robot, robot_w, robot_h, SH110X_WHITE);
    display.display(); // Show the bitmap(s)
  }
}

// --- Placeholder functions for drawing stick positions and telemetry ---

void draw_sticks(){
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);

    // Draw crosshairs for stick centers
    static int centerX = display.width() / 2;
    static int centerY = display.height() / 2;
    display.drawFastVLine(centerX, 0, display.height(), SH110X_WHITE);
    display.drawFastHLine(0, centerY, display.width(), SH110X_WHITE);

    // Map analog values (0-4095) to screen coordinates (0-127)
    // Adjust mapping if your sticks don't use the full analog range
    int ls_screen_x = map(ls_x, 4095, 0, 0, display.width() -1);
    int ls_screen_y = map(ls_y, 4095, 0, 0, display.height() -1); // Might need inversion: display.height()-1, 0
    int rs_screen_x = map(rs_x, 0, 4095, 0, display.width() -1);
    int rs_screen_y = map(rs_y, 0, 4095, 0, display.height() -1); // Might need inversion


    // Draw circles representing stick positions
    display.drawCircle(ls_screen_x, ls_screen_y, 5, SH110X_WHITE); // Left stick
    display.drawCircle(rs_screen_x, rs_screen_y, 5, SH110X_WHITE); // Right stick

    // Optionally display raw or mapped values
    display.setCursor(0, 0);
    display.print("LX:"); display.print(ls_x);
    display.setCursor(display.width()/2, 0);
    display.print("RX:"); display.print(rs_x);
    display.setCursor(0, 10);
    display.print("LY:"); display.print(ls_y);
    display.setCursor(display.width()/2, 10);
    display.print("RY:"); display.print(rs_y);
    display.setCursor(0, 20);
    display.print("Spd out:"); display.println(control_input.control_speed);
    display.print("Yaw out:"); display.println(control_input.control_yaw_rate);

    // Display switch states (LOW = pressed)
    display.setCursor(0, display.height() - 10);
    if (ls_sw == LOW) display.print("LS_SW");
    display.setCursor(display.width()/2, display.height() - 10);
     if (rs_sw == LOW) display.print("RS_SW");


    display.display();
}

void draw_raw_telem(){
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.print("Speed: "); display.println(float(received_telem.speed)/100);
    display.print("Batt V: "); display.println(float(received_telem.battery_voltage)/100);
    display.print("K-Angle: "); display.println(float(received_telem.kalman_angle)/100);
    display.print("Raw Angle: "); display.println(float(received_telem.raw_angle)/100);
    display.print("Raw Gyro: "); display.println(float(received_telem.raw_gyro)/100);
    display.print("Angle PID: "); display.println(float(received_telem.angle_pid)/100);
    display.print("Angle Command: "); display.println(float(received_telem.angle_command)/100);
    display.print("M1 PID: "); display.println(float(received_telem.motor_1_pid)/100);
    display.print("M2 PID: "); display.println(float(received_telem.motor_2_pid)/100);
    display.print("Yaw Rate: "); display.println(float(received_telem.yaw_rate)/100);
    display.print("Ang Rate: "); display.println(float(received_telem.angular_rate)/100);
    display.print("M1 RPM: "); display.println(float(received_telem.m1_rpm)/100);
    display.print("M2 RPM: "); display.println(float(received_telem.m2_rpm)/100);
    display.print("Speed out:"); display.println(control_input.control_speed);
    display.print("Yaw out:"); display.println(control_input.control_yaw_rate);

    display.display();
}

/*   int speed;
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
  int m2_rpm; */

void draw_control(){
  display.clearDisplay();
  
  // Draw two rectangles in the bottom with a center dot that moves with the sticks
  // Local coordinates from the middle of each rectangle
  static int square_side = 30;

  int ls_screen_x = map(ls_x, 4095, 0, -square_side/2, square_side/2);
  int ls_screen_y = map(ls_y, 4095, 0, -square_side/2, square_side/2); // Might need inversion: display.height()-1, 0
  int rs_screen_x = map(rs_x, 0, 4095, -square_side/2, square_side/2);
  int rs_screen_y = map(rs_y, 0, 4095, -square_side/2, square_side/2); // Might need inversion

  // Draw two small squares with circles indicating stick positions
  display.drawRect(10, SCREEN_HEIGHT - square_side - 10, square_side, square_side, 1);  // Left square
  display.drawCircle(10 + square_side/2 + ls_screen_x, SCREEN_HEIGHT - (square_side / 2) + ls_screen_y - 10, 2, 1); // LS circle

  display.drawRect(SCREEN_WIDTH - 10 - square_side, SCREEN_HEIGHT - square_side - 10, square_side, square_side, 1);  // Right square
  display.drawCircle(SCREEN_WIDTH - 10 - square_side/2 + rs_screen_x, SCREEN_HEIGHT - (square_side / 2) + rs_screen_y - 10, 2, 1); // RS circle


  // Draw a connection symbol, has a blinking X over it if no connection
  display.drawBitmap(10, 8, connection_symbol, 24, 18, 1);
  if(!is_connected){   // If not connected and 500ms passed
    if(draw_cross){   // If not drawn
      display.drawBitmap(10, 5, cross, 24, 24, 1);
      draw_cross = true;
      Serial.println("Cross");
    }

    if(millis() - cross_timer > 800){   // Change draw state after a time
      draw_cross = !draw_cross;
      cross_timer = millis();
    }
  }

  // Write telemetry in bigger font size
  // Speed and angle most important
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(50, 15);
  display.print(float(received_telem.battery_voltage / 100));
  display.print("V");

  display.setTextSize(2);   // 2 is 12x16
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 40);
  display.print("SPD:"); 
  display.println(float(received_telem.speed) / 100);

  display.setCursor(0, 65);
  display.print("ANG:"); 
  display.println(float(received_telem.kalman_angle) / 100);

  display.display();
}