// -------------------
// ROS Callback
// -------------------


void paintColorCallback(const light_painting::RGBState& rgb_msg) {
  state_r   = rgb_msg.red;
  state_g   = rgb_msg.green;
  state_b   = rgb_msg.blue;
}

void updatePaintbrushOutput() {
  analogWrite(led_r, state_r);
  analogWrite(led_g, state_g);
  analogWrite(led_b, state_b);
}



// -------------------
// Main Loop
// -------------------


void loop() {

  // Turn Off LED if ROS-2-Arduino connection broken
  if (!nh.connected()) {
    state_r = 0;
    state_g = 0;
    state_b = 0;
  }

  // Spin ROS
  nh.spinOnce();

  // Update Indicator Light
  updatePaintbrushOutput();
  delay(10);
}
