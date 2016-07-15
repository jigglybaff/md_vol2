/*
 * IRremote: IRsendDemo - demonstrates sending IR codes with IRsend
 * An IR LED must be connected to Arduino PWM pin 3.
 * Version 0.1 July, 2009
 * Copyright 2009 Ken Shirriff
 * http://arcfn.com
 */

#include <IRremote.h>

IRsend irsend;

void setup()
{
  Serial.begin(9600);
}
//unsigned int irSignal[] = {8950, 4400, 550, 550, 600, 500, 600, 1650, 550, 550, 600, 500, 600, 550, 600, 500, 600, 550, 550, 1650, 600, 1600, 600, 550, 550, 1650, 600, 1600, 600, 1600, 600, 1650, 600, 1600, 600, 500, 600, 550, 600, 500, 600, 1600, 600, 550, 600, 500, 600, 550, 600, 500, 600, 1600, 600, 1650, 600, 1600, 600, 500, 600, 1650, 550, 1650, 600, 1600, 600, 1600, 600}; // 0x20DF10EF

void loop() {
  if (Serial.read() != -1) {
   irsend.sendNEC(0x20DF08F7,32);//SEND LG TV POWER
                delay(100);
    }
  
}

