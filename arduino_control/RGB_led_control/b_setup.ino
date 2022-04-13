// -------------------
// ROS Initialization
// -------------------


// ROS Node Setup (max one node allowed)
ros::NodeHandle nh;

// Initialize a message instance
light_painting::RGBState led_msg;

ros::Subscriber<light_painting::RGBState> sub_led_state("paintbrush_color", &paintColorCallback );



// -------------------
// Setup
// -------------------


void setup() {
  
  // Setup Output
  pinMode(led_r, OUTPUT);
  pinMode(led_g, OUTPUT);
  pinMode(led_b, OUTPUT);

  // Setup ROS
  nh.getHardware()->setBaud(500000);
  nh.initNode();
  nh.subscribe(sub_led_state);
}
