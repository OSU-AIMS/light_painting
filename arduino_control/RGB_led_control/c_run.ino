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

  if (nh.connected()) {
    nh.spinOnce();
    updatePaintbrushOutput();
  }
  
  delay(100);
}
