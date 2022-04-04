#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>

#define BUTTON 8
#define LED 13

#define LED_red 12

int value=0;
// ros::NodeHandle node_handle;

// std_msgs::String button_msg;
// std_msgs::UInt16 led_msg;

void subscriberCallback(const std_msgs::UInt16& led_msg) {
  if (led_msg.data  == 1) {
    digitalWrite(LED, HIGH); 
  } else {
    digitalWrite(LED, LOW);
  }
}


// ros::Publisher button_publisher("button_press", &button_msg);
// // publisher name: button_publisher
// // publishes button_msg to button_press topic
// ros::Subscriber<std_msgs::UInt16> led_subscriber("toggle_led", &subscriberCallback);


//// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(9600);
//  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_red,OUTPUT);
  digitalWrite(LED_red,HIGH); // for keeping RED LED ON
  digitalWrite(LED_BUILTIN,LOW);
  Serial.println("Connection Established...");
}
//
/// the loop function runs over and over again forever
void loop() {
  while(Serial.available())
    {
      value = Serial.read();
    }
  if (value == '1')
    digitalWrite(LED_BUILTIN,HIGH);
  else if (value=='0')
    digitalWrite(LED_BUILTIN,LOW);
}
//  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
//  delay(1000);                       // wait for a second
//  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
//  delay(1000);                       // wait for a second

//}
