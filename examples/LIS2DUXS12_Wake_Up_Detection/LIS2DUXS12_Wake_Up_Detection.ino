/*
   @file    LIS2DUXS12_Wake_Up_Detection.ino
   @author  STMicroelectronics
   @brief   Example to use the LIS2DUXS12 Wake Up Detection
 *******************************************************************************
   Copyright (c) 2022, STMicroelectronics
   All rights reserved.
   This software component is licensed by ST under BSD 3-Clause license,
   the "License"; You may not use this file except in compliance with the
   License. You may obtain a copy of the License at:
                          opensource.org/licenses/BSD-3-Clause
 *******************************************************************************
*/


#include <LIS2DUXS12Sensor.h>

#define INT1_pin A3

LIS2DUXS12Sensor LIS2DUXS12(&Wire);

//Interrupts.
volatile int mems_event = 0;

void INT1Event_cb();

void setup() {

  // Initlialize serial.
  Serial.begin(115200);
  delay(1000);

  // Initlialize Led.
  pinMode(LED_BUILTIN, OUTPUT);

  // Initlialize i2c.
  Wire.begin();

  // Enable INT1 pin.
  attachInterrupt(INT1_pin, INT1Event_cb, RISING);
    
  // Initlialize components.
  LIS2DUXS12.begin();
  LIS2DUXS12.Enable_X();

  // Enable Wake Up Detection.
  LIS2DUXS12.Enable_Wake_Up_Detection(LIS2DUXS12_INT1_PIN);
}

void loop() {
  if (mems_event)
  {
    mems_event = 0;
    LIS2DUXS12_Event_Status_t status;
    LIS2DUXS12.Get_X_Event_Status(&status);
    if (status.WakeUpStatus)
    {
      // Led blinking.
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      
      Serial.println("Wake up Detected!");
    }
  }
}

void INT1Event_cb()
{
  mems_event = 1;
}
