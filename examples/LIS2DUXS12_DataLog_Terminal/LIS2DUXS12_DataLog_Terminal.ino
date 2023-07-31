/*
   @file    LIS2DUXS12_DataLog_Terminal.ino
   @author  STMicroelectronics
   @brief   Example to use the LIS2DUXS12 inertial measurement sensor
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

LIS2DUXS12Sensor LIS2DUXS12 (&Wire);
int32_t accel[3];

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Wire.begin();
  LIS2DUXS12.begin();
  LIS2DUXS12.Enable_X();
}

void loop() {
  LIS2DUXS12.Get_X_Axes(accel);

  Serial.print("Accel-X[mg]:");
  Serial.print(accel[0]);
  Serial.print(",Accel-Y[mg]:");
  Serial.print(accel[1]);
  Serial.print(",Accel-Z[mg]:");
  Serial.println(accel[2]);

  blink(LED_BUILTIN);
}

inline void blink(int pin) {
  digitalWrite(pin, HIGH);
  delay(25);
  digitalWrite(pin, LOW);
  delay(975);
}
