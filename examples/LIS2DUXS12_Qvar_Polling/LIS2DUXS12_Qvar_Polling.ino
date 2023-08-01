/*
   @file    LIS2DUXS12_Qvar.ino
   @author  STMicroelectronics
   @brief   Example to use LIS2DUXS12 Qvar features
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
LIS2DUXS12Sensor LIS2DUXS12(&Wire);

void setup() {
  
  Serial.begin(115200);
  Wire.begin();
  
  // Initialize LIS2DUXS12.
  LIS2DUXS12.begin();
  
  // Enable accelerometer and gyroscope.
  LIS2DUXS12.Enable_X();

  // Enable QVAR
  if(LIS2DUXS12.Enable_QVAR()!= 0)
  {
    Serial.println("Error during initialization of QVAR");
    while(1);
  }
  Serial.println("LIS2DUXS12 QVAR Demo");
}

void loop() {
  uint8_t qvar_status;
  float qvar_data;
  // Check if QVAR data is ready
  LIS2DUXS12.Get_QVAR_Status(&qvar_status);
  if(qvar_status)
  {
    // Get QVAR data 
    LIS2DUXS12.Get_QVAR_Data(&qvar_data);
    Serial.println(qvar_data);
  }
}