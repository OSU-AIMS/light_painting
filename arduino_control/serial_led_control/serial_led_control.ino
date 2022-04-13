/*
 * Serial based, binary control of an single led on an Arduino
 * 
 * Copyright (c) 2022, The Ohio State University
 * The Artificially Intelligent Manufacturing Systems Lab (AIMS)
 * Author: Mohammad Khan
 */


/* Support */
#include <Arduino.h>

/* Define Output Pins */
#define BUTTON 8
#define LED 13
#define LED_red 12

/* Control Variables */
int value=0;



///////////
/* Setup */
///////////

void setup()
{  
  // Communication Interface
  Serial.begin(9600);
  
  // Set Output Capability
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_red,OUTPUT);

  // Set Default Output State
  digitalWrite(LED_red,HIGH); // for keeping RED LED ON
  digitalWrite(LED_BUILTIN,LOW);

  // Report
  Serial.println("Connection Established...");
}



//////////
/* Loop */
//////////

void loop()
{
  while(Serial.available())
    {
      value = Serial.read();
    }
  if (value == '1')
    digitalWrite(LED_BUILTIN,HIGH);
  else if (value=='0')
    digitalWrite(LED_BUILTIN,LOW);
}
