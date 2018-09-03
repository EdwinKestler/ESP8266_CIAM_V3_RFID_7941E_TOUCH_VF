/*
  TouchPadButton.h - Library for Touch Pad as a Button signal.
  Created by Edwin Kestler, May 29 , 2018.
  Released into the public domain.
*/

#ifndef TouchPadButton_h
#define TouchPadButton_h

#include "Arduino.h"


class TouchPadButton{
  public:
    TouchPadButton(int pin);
    bool check();
  private:
    int _pin;
};

#endif

