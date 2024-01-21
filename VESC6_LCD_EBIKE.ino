/*
This code is used to gather data from a VESC6 and display on an LCD screen.
Written for Arduino Nano and LCD I2C 1602.

Original VESCUART.h code written by SolidGeek.
 */
#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>
#include <VescUart.h>
#include <SimpleKalmanFilter.h>

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0); 

/** Initiate VescUart class */
VescUart UART;

int rpm;
float voltage;
float current;
int power;
float amphour;
float tach;
float distance;
float velocity;
float watthour;
float batpercentage;

SimpleKalmanFilter Filter1(2, 2, 0.01); // used to calculate power

// used for speed calcs (change depending on spec of motor)
int poles = 14; // num of poles in motor
float wheel_diameter = 0.72; // diameter of wheel in meters
float motor_wheel_pulley_ratio = (16/185); // ratio of the motor pulley teeth to the wheel pulley teeth
int batt_cells = 12; // amount of battery cells in series 

////////// Custom Characters //////////
byte ph[] = {
  B11100,
  B10100,
  B11100,
  B10000,
  B10101,
  B00111,
  B00101,
  B00101
};

byte mi[] = {
  B10001,
  B11011,
  B10101,
  B10001,
  B00100,
  B00000,
  B00100,
  B00110
};

byte km[] = {
  B01000,
  B01010,
  B01100,
  B01010,
  B10001,
  B11011,
  B10101,
  B10001
};


byte ah[] = {
  B00100,
  B01010,
  B01110,
  B01010,
  B00000,
  B01010,
  B01110,
  B01010
};
byte percent[] = {
  B11001,
  B11001,
  B00010,
  B00100,
  B01000,
  B10011,
  B10011,
  B00000
};

byte m[] = {
  B10001,
  B11011,
  B10101,
  B10001,
  B10001,
  B00000,
  B00000,
  B00000
};

byte k[] = {
  B01000,
  B01010,
  B01100,
  B01010,
  B01010,
  B00000,
  B00000,
  B00000
};

byte bat[] = {
  B01110,
  B11011,
  B10001,
  B10001,
  B10001,
  B11111,
  B11111,
  B11111
};

byte sprk[] = {
  B00000,
  B01000,
  B00100,
  B00010,
  B00100,
  B01000,
  B00100,
  B00010
};



void setup() {

  /** Setup Serial port to display data */
  Serial.begin(115200);

  while (!Serial) {;}

  /** Define which ports to use as UART */
  UART.setSerialPort(&Serial);
    
  // CAN ALSO WRITE A START SCREEN IF YOU WANT
  u8g2.begin(); // initialize oled
  u8g2.clearBuffer();         // clear the internal memory
  u8g2.setFont(u8g2_font_logisoso28_tr);  // choose a suitable font at https://github.com/olikraus/u8g2/wiki/fntlistall
  u8g2.drawStr(0,24,"Startup Scrn"); // first 2 params are x, y coordinates respectively
  delay(5000);
}
void loop() {
  
////////// Read values //////////  
 if ( UART.getVescValues() ) {

  rpm = (UART.data.rpm)/(poles/2);          // The '7' is the number of pole pairs in the motor. Most motors have 14 poles, therefore 7 pole pairs
  voltage = (UART.data.inpVoltage);
  current = (UART.data.avgInputCurrent);
  power = voltage*current;
  amphour = (UART.data.ampHours);
  watthour = amphour*voltage;
  tach = (UART.data.tachometerAbs)/(poles*3);          // The '42' is the number of motor poles multiplied by 3
  distance = tach*3.142*(1/1000)*wheel_diameter*(motor_wheel_pulley_ratio);          // Motor RPM x Pi x (1 / meters in a mile or km) x Wheel diameter x (motor pulley / wheelpulley)
  velocity = rpm*3.142*(60/1000)*wheel_diameter*(motor_wheel_pulley_ratio);          // Motor RPM x Pi x (seconds in a minute / meters in a mile) x Wheel diameter x (motor pulley / wheelpulley)
  batpercentage = ((voltage-38.4)/batt_cells)*100;          // ((Battery voltage - minimum voltage) / number of cells) x 100

////////// Filter //////////
  // calculate the estimated value with Kalman Filter
  float powerfiltered = Filter1.updateEstimate(power);
  

////////// OLED ////////// 
// First line  
  char velocityStr[6] = {};
  dtostrf(velocity, 6, 2, velocityStr); // convert velocity to a string representation
  u8g2.clearBuffer(); 
  u8g2.drawStr(31,24, velocityStr);
  u8g2.sendBuffer();
  delay(800); 
}
else
  {    
    u8g2.clearBuffer(); 
    u8g2.drawStr(31,24,"failed to get data");
    u8g2.sendBuffer();
    delay(800);
  }
  delay(50);
  
}