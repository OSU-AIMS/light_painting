#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>

int red_light_pin= 11;
int green_light_pin = 10;
int blue_light_pin = 9;

int red = 0;
int blue = 0;
int green = 0;

int value = 0;

void subscriberCallback(const std_msgs::UInt16& rgb_msg)
{
  value = rgb_msg;
//  red = rgb_msg.data(0)
//  ggreen = rgb_msg.data(1)
//  blue = rgb_msg.data(2)
  RGB_color(red,green,blue);
}

void setup() {
  pinMode(red_light_pin, OUTPUT);
  pinMode(green_light_pin, OUTPUT);
  pinMode(blue_light_pin, OUTPUT);
}
void loop() {
  while(Serial.available())
  {
    red = Serial.read();
    green = Serial.read();
    blue = Serial.read();
    RGB_color(red,green,blue);
  }
  

}
//  RGB_color(255, 0, 0); // Red
//  delay(1000);
//  RGB_color(0, 255, 0); // Green
//  delay(1000);
//  RGB_color(0, 0, 255); // Blue
//  delay(1000);
//  RGB_color(255, 255, 125); // Raspberry
//  delay(1000);
//  RGB_color(0, 255, 255); // Cyan
//  delay(1000);
//  RGB_color(255, 0, 255); // Magenta
//  delay(1000);
//  RGB_color(255, 255, 0); // Yellow
//  delay(1000);
//  RGB_color(255, 255, 255); // White
//  delay(1000);

void RGB_color(int red_light_value, int green_light_value, int blue_light_value)
 {
  analogWrite(red_light_pin, red_light_value);
  analogWrite(green_light_pin, green_light_value);
  analogWrite(blue_light_pin, blue_light_value);
}
//#include <ros.h>
//
//int red_light_pin= 11;
//int green_light_pin = 10;
//int blue_light_pin = 9;
//void setup() {
//  pinMode(red_light_pin, OUTPUT);
//  pinMode(green_light_pin, OUTPUT);
//  pinMode(blue_light_pin, OUTPUT);
//}
//void loop() {
//  RGB_color(255, 0, 0); // Red
//  delay(1000);
//  RGB_color(0, 255, 0); // Green
//  delay(1000);
//  RGB_color(0, 0, 255); // Blue
//  delay(1000);
//  RGB_color(255, 255, 125); // Raspberry
//  delay(1000);
//  RGB_color(0, 255, 255); // Cyan
//  delay(1000);
//  RGB_color(255, 0, 255); // Magenta
//  delay(1000);
//  RGB_color(255, 255, 0); // Yellow
//  delay(1000);
//  RGB_color(255, 255, 255); // White
//  delay(1000);
//  RGB_color(0,0,0);
//  delay(1000);
//}
//void RGB_color(int red_light_value, int green_light_value, int blue_light_value)
// {
//  analogWrite(red_light_pin, red_light_value);
//  analogWrite(green_light_pin, green_light_value);
//  analogWrite(blue_light_pin, blue_light_value);
//}
