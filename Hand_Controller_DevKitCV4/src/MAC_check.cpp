

/* #include <Arduino.h>
ESP32 MAC Address printout
esp32-mac-address.ino
Prints MAC Address to Serial Monitor

DroneBot Workshop 2022
https://dronebotworkshop.com
*/
/* 
// Include WiFi Library
#include "WiFi.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"


void setup() {

// Setup Serial Monitor
Serial.begin(115200);
Serial.println("innan");
// Put ESP32 into Station mode
//WiFi.mode(WIFI_MODE_STA);

// Drar för mycket ström när wifi startas, behöver stänga av brownout detector
uint32_t brown_reg_temp = READ_PERI_REG(RTC_CNTL_BROWN_OUT_REG); //save WatchDog register
WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
WiFi.mode(WIFI_MODE_STA); // turn on WiFi
delay(1000); 
WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, brown_reg_temp); //enable brownout detector

Serial.println("efter");
delay(1000);
// Print MAC Address to Serial monitor
Serial.print("MAC Address: ");
Serial.println(WiFi.macAddress());
}

void loop() {

}
 */