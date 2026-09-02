/*
   @file    LIS2DUXS12_DataLog_Terminal_I3C.ino
   @author  STMicroelectronics
   @brief   Example to use the LIS2DUXS12 sensor with I3C and SETDASA command
 *******************************************************************************
   Copyright (c) 2026, STMicroelectronics
   All rights reserved.

   This software component is licensed by ST under BSD 3-Clause license,
   the "License"; You may not use this file except in compliance with the
   License. You may obtain a copy of the License at:
                          opensource.org/licenses/BSD-3-Clause

 *******************************************************************************
*/
#include "LIS2DUXS12Sensor.h"

#define LIS2DUXS12_DYNAMIC_ADDRESS 0x30

LIS2DUXS12Sensor sensor(&I3C, LIS2DUXS12_I3C_ADD_H);

void setup()
{
  Serial.begin(115200);
  while (!Serial) {}
  delay(1000);

  Serial.println("=== LIS2DUXS12 SETDASA ===");

  if (!I3C.begin(I3C_SDA, I3C_SCL, 1000000U)) {
    Serial.println("begin() failed");
    while (1) {}
  }

  if (!I3C.resetDynamicAddresses()) {
    Serial.println("resetDynamicAddresses() failed");
    while (1) {}
  }
  if (!I3C.assignDynamicAddress(sensor.getStaticAddress(), LIS2DUXS12_DYNAMIC_ADDRESS)) {
    Serial.println("assignDynamicAddress() failed");
    while (1) {}
  }

  if (sensor.begin(LIS2DUXS12_DYNAMIC_ADDRESS) != LIS2DUXS12_STATUS_OK) {
    Serial.println("sensor.begin() failed");
    while (1) {}
  }

  if (!I3C.setClock(12500000)) {
    Serial.println("setClock() failed");
    while (1) {}
  }

  if (sensor.Enable_X() != LIS2DUXS12_STATUS_OK) {
    Serial.println("sensor.Enable_X() failed");
    while (1) {}
  }

  Serial.println("LIS2DUXS12 ready");
}

void loop()
{
  int32_t accel[3] = {0};

  if (sensor.Get_X_Axes(accel) == LIS2DUXS12_STATUS_OK) {
    Serial.print("Accel-X[mg]:");
    Serial.print(accel[0]);
    Serial.print(",Accel-Y[mg]:");
    Serial.print(accel[1]);
    Serial.print(",Accel-Z[mg]:");
    Serial.println(accel[2]);
  } else {
    Serial.println("Read failed");
  }

  delay(500);
}
