/*
  TouchPadButton.h - Library for Touch Pad as a Button signal.
  Created by Edwin Kestler, May 29, 2018.
  Released into the public domain.
*/

#include "Arduino.h"
#include "TouchPadButton.h"

TouchPadButton::TouchPadButton(int pin){
  pinMode(pin, INPUT);
  _pin = pin;
}

bool TouchPadButton::check(){
  int Buttonpin = digitalRead(_pin);
  if (Buttonpin == HIGH){
    return true;
  }
  else {
    return false;
  }  
}
