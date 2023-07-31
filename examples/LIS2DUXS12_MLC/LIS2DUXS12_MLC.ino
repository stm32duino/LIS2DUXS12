/*
   @file    LIS2DUXS12_MLC.ino
   @author  STMicroelectronics
   @brief   Example to use the LIS2DUXS12 Machine Learning Core
 *******************************************************************************
   Copyright (c) 2023, STMicroelectronics
   All rights reserved.
   This software component is licensed by ST under BSD 3-Clause license,
   the "License"; You may not use this file except in compliance with the
   License. You may obtain a copy of the License at:
                          opensource.org/licenses/BSD-3-Clause
 *******************************************************************************
*/
// Includes
#include "LIS2DUXS12Sensor.h"
#include "lis2duxs12_activity_recognition_for_mobile.h"

#define INT_1 A3

//Interrupts.
volatile int mems_event = 0;

LIS2DUXS12Sensor sensor(&Wire);

// MLC
ucf_line_ext_t *ProgramPointer;
int32_t LineCounter;
int32_t TotalNumberOfLine;
uint8_t mlc_out[4];

void INT1Event_cb();
void printMLCStatus(uint8_t status);

void setup()
{
  // Led.
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize serial for output.
  Serial.begin(115200);

  // Initlialize i2c.
  Wire.begin();

  // Initlialize components.
  sensor.begin();
  sensor.Enable_X();

  /* Feed the program to Machine Learning Core */
  /* Activity Recognition Default program */
  ProgramPointer = (ucf_line_ext_t *)lis2duxs12_activity_recognition_for_mobile;
  TotalNumberOfLine = sizeof(lis2duxs12_activity_recognition_for_mobile) / sizeof(ucf_line_ext_t);
  Serial.println("Activity Recognition for LIS2DUXS12 MLC");
  Serial.print("UCF Number Line=");
  Serial.println(TotalNumberOfLine);

  for (LineCounter = 0; LineCounter < TotalNumberOfLine; LineCounter++) {
    if(ProgramPointer[LineCounter].op == MEMS_UCF_OP_WRITE){
		if (sensor.Write_Reg(ProgramPointer[LineCounter].address, ProgramPointer[LineCounter].data)) {
		  Serial.print("Error loading the Program to LIS2DUXS12Sensor at line: ");
		  Serial.println(LineCounter);
		  while (1) {
			// Led blinking.
			digitalWrite(LED_BUILTIN, HIGH);
			delay(250);
			digitalWrite(LED_BUILTIN, LOW);
			delay(250);
		  }
		}
	}
   else if(ProgramPointer[LineCounter].op == MEMS_UCF_OP_DELAY){
    delay(ProgramPointer[LineCounter].data);
  }
  
  }

  Serial.println("Program loaded inside the LIS2DUXS12 MLC");

  //Interrupts.
  pinMode(INT_1, INPUT);
  attachInterrupt(INT_1, INT1Event_cb, RISING);

  /* We need to wait for a time window before having the first MLC status */
  delay(3000);

  sensor.Get_MLC_Output(mlc_out);
  printMLCStatus(mlc_out[0]);
}

void loop()
{
  if (mems_event) {
    mems_event = 0;
    lis2duxs12_mlc_status_mainpage_t  status;
    sensor.Get_MLC_Status(&status);
    if (status.is_mlc1) {
      sensor.Get_MLC_Output(mlc_out);
      printMLCStatus(mlc_out[0]);
    }
  }
}

void INT1Event_cb()
{
  mems_event = 1;
}

void printMLCStatus(uint8_t status)
{
  switch (status) {
    case 0:
      Serial.println("Activity: Stationary");
      break;
    case 1:
      Serial.println("Activity: Walking");
      break;
    case 4:
      Serial.println("Activity: Jogging");
      break;
    case 8:
      Serial.println("Activity: Biking");
      break;
    case 12:
      Serial.println("Activity: Driving");
      break;
    default:
      Serial.println("Activity: Unknown");
      break;
  }
}
