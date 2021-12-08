// Lab17_Control.c
// Runs on MSP432
// Implementation of the control system.
// Daniel and Jonathan Valvano
// September 12, 2017

/* This example accompanies the books
   "Embedded Systems: Introduction to the MSP432 Microcontroller",
       ISBN: 978-1512185676, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Interfacing to the MSP432 Microcontroller",
       ISBN: 978-1514676585, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Operating Systems for ARM Cortex-M Microcontrollers",
       ISBN: 978-1466468863, , Jonathan Valvano, copyright (c) 2017
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2017, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

#include <stdint.h>
#include <stdio.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/PWM.h"
#include "../inc/LaunchPad.h"
#include "../inc/UART0.h"
#include "../inc/Motor.h"
#include "../inc/Bump.h"
#include "../inc/ADC14.h"
#include "../inc/TimerA1.h"
#include "../inc/IRDistance.h"
#include "../inc/Nokia5110.h"
#include "../inc/LPF.h"
#include "../inc/FlashProgram.h"
#include "../inc/SysTickInts.h"
#include "../inc/Tachometer.h"
#include "../inc/Reflectance.h"

/**************Initial values used for all programs*******************/
#define PWMNOMINAL 6000
#define DESIRED_SPEED 50
#define TACHBUFF 10                      // number of elements in tachometer array

volatile uint32_t ControllerFlag; // set every 10ms on controller execution

int32_t PauseBump = 0;
int32_t Mode = 0;
int32_t XstarL, XstarR;     // Desired speed
int32_t XprimeL, XprimeR;   // Actual speed
int32_t ErrorL, ErrorR;     // X* - X'
int32_t UL, UR;             // Controller output PWM duty 2 to 14,998
uint16_t LeftTach[TACHBUFF];             // tachometer period of left wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection LeftDir;              // direction of left rotation (FORWARD, STOPPED, REVERSE)
int32_t LeftSteps;                       // number of tachometer steps of left wheel (units of 220/360 = 0.61 mm traveled)
uint16_t RightTach[TACHBUFF];            // tachometer period of right wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection RightDir;             // direction of right rotation (FORWARD, STOPPED, REVERSE)
int32_t RightSteps;                      // number of tachometer steps of right wheel (units of 220/360 = 0.61 mm traveled)
int i = 0;
uint32_t Time; // in 0.01 sec
int32_t Kp=3;  // proportional controller gain
#define SWING 1500
#define PWMIN (PWMNOMINAL-SWING)
#define PWMAX (PWMNOMINAL+SWING)


/**************Functions used by all programs*******************/

void Pause3(void){
    int j;
  while(Bump_Read()){ // wait for release
    Clock_Delay1ms(200); LaunchPad_Output(0); // off
    Clock_Delay1ms(200); LaunchPad_Output(1); // red
  }
  while(Bump_Read()==0){// wait for touch
    Clock_Delay1ms(100); LaunchPad_Output(0); // off
    Clock_Delay1ms(100); LaunchPad_Output(3); // red/green
  }
  while(Bump_Read()){ // wait for release
    Clock_Delay1ms(100); LaunchPad_Output(0); // off
    Clock_Delay1ms(100); LaunchPad_Output(4); // blue
  }
  for(j=1000;j>100;j=j-200){
    Clock_Delay1ms(j); LaunchPad_Output(0); // off
    Clock_Delay1ms(j); LaunchPad_Output(2); // green
  }
  Mode = 1;
  ControllerFlag = 0;
}

// ------------avg------------
// Simple math function that returns the average
// value of an array.
// Input: array is an array of 16-bit unsigned numbers
//        length is the number of elements in 'array'
// Output: the average value of the array
// Note: overflow is not considered
uint16_t avg(uint16_t *array, int length){
  int i;
  uint32_t sum = 0;
  for(i=0; i<length; i=i+1){
    sum = sum + array[i];
  }
  return (sum/length);
}

/**************Program17_1*******************/
// go straight using Tachometers

void LCDClear1(void){
    Nokia5110_Init();
    Nokia5110_Clear();
    Nokia5110_OutString("Desired(RPM)L     R     Actual (RPM)L     R     Error(RPM)  L     R     ");
}
void LCDOut1(void){
    Nokia5110_SetCursor(1, 1);         // one leading space, second row
    Nokia5110_OutSDec1(DESIRED_SPEED);
    Nokia5110_SetCursor(7, 1);         // seven leading spaces, second row
    Nokia5110_OutSDec1(DESIRED_SPEED);
    Nokia5110_SetCursor(1, 3);       // one leading space, fourth row
    Nokia5110_OutUDec(XprimeL);
    Nokia5110_SetCursor(7, 3);       // seven leading spaces, fourth row
    Nokia5110_OutUDec(XprimeR);
    Nokia5110_SetCursor(1, 5);       // zero leading spaces, sixth row
    Nokia5110_OutSDec1(ErrorL);
    Nokia5110_SetCursor(7, 5);       // six leading spaces, sixth row
    Nokia5110_OutSDec1(ErrorR);
}

//******************************************************
// Incremental speed control
void Controller1(void){
    // write this as part of Lab 17
    if(Mode){
        Tachometer_Get(&LeftTach[i], &LeftDir, &LeftSteps, &RightTach[i], &RightDir, &RightSteps);
        i = i + 1;
        if(i >= TACHBUFF){
            i = 0;
            XstarL = DESIRED_SPEED; // desired rpm
            XstarR = DESIRED_SPEED; // desired rpm
            XprimeL = 2000000/avg(LeftTach, TACHBUFF); // actual rpm
            XprimeR = 2000000/avg(RightTach, TACHBUFF); // actual rpm
            ErrorL = XstarL - XprimeL;
            ErrorR = XstarR - XprimeR;
            UR = UR+Kp*ErrorR;
            UL = UL+Kp*ErrorL;
            if (UL < 2) UL = 2;
            if (UR < 2) UR = 2;
            if (UL > 14998) UL = 14998;
            if (UR > 14998) UR = 14998;
            Motor_Forward(UL, UR);
			ControllerFlag = 1;
        }
    }
}

void Program17_1(void){
  DisableInterrupts();
  // write this as part of Lab 17
// initialization
  Clock_Init48MHz();
  LaunchPad_Init();
  Bump_Init();
  Tachometer_Init();
  Motor_Init();
  TimerA1_Init(&Controller1,5000); // 100 Hz
  Motor_Stop();
  UR = UL = PWMNOMINAL;
  EnableInterrupts();
  LCDClear1();
  ControllerFlag = 0;
  Pause3();

	while(1){
	    if(Bump_Read()){
	        Mode = 0;
	        Motor_Stop();
	        Pause3();
	    }

		if(ControllerFlag){
			LCDOut1();
			ControllerFlag = 0;
		}
  }
}

/**************Program17_2*******************/
// proportional control, wall distance

volatile uint32_t nr, nc, nl; // raw distance values
int32_t Left, Center, Right; // IR distances in mm
volatile uint32_t ADCflag; // Set every 500us on ADC sample


void IRsampling(void){
    uint32_t raw17, raw12, raw16;
    ADC_In17_12_16(&raw17, &raw12, &raw16);
    nr = LPF_Calc(raw17);
    nc = LPF_Calc2(raw12);
    nl = LPF_Calc3(raw16);
    Left = LeftConvert(nl);
    Center = CenterConvert(nc);
    Right = RightConvert(nr);
    ADCflag = 1;
}

// distances in mm

#define DESIRED_DIST 172

int32_t SetPoint = 172;
uint32_t PosError;
int32_t WallError;

void LCDClear2(void){
  Nokia5110_Init();
  Nokia5110_Clear(); // erase entire display
  Nokia5110_OutString("17: control");
  Nokia5110_SetCursor(0,1); Nokia5110_OutString("IR distance");
  Nokia5110_SetCursor(0,2); Nokia5110_OutString("L= "); Nokia5110_OutSDec(0); Nokia5110_OutString(" mm");
  Nokia5110_SetCursor(0,3); Nokia5110_OutString("C= "); Nokia5110_OutSDec(0); Nokia5110_OutString(" mm");
  Nokia5110_SetCursor(0,4); Nokia5110_OutString("R= "); Nokia5110_OutSDec(0); Nokia5110_OutString(" mm");
  Nokia5110_SetCursor(0,5); Nokia5110_OutString("E= "); Nokia5110_OutSDec(0); Nokia5110_OutString(" mm");
}

void LCDOut2(void){
  Nokia5110_SetCursor(3,2); Nokia5110_OutSDec(Left);
  Nokia5110_SetCursor(3,3); Nokia5110_OutSDec(Center);
  Nokia5110_SetCursor(3,4); Nokia5110_OutSDec(Right);
  Nokia5110_SetCursor(3,5); Nokia5110_OutSDec(WallError);
  // left
  if(Time%5 == 0){
      UART0_OutUDec5(Left);UART0_OutString(" mm,");
      UART0_OutUDec5(Center);UART0_OutString(" mm,");
      UART0_OutUDec5(Right);UART0_OutString(" mm,");
      UART0_OutUDec5(UR);UART0_OutString(" %,");
      UART0_OutUDec5(UL);UART0_OutString(" %,");
      if(WallError < 0){
          PosError = WallError*(-1);
          UART0_OutString("-");UART0_OutUDec5(PosError);UART0_OutString("\n");
      }
      else{
          UART0_OutUDec5(WallError);UART0_OutString("\n");
      }

  }
}

void SysTick_Handler(void){
    if(Mode){
        //
        if((Left>DESIRED_DIST)&&(Right>DESIRED_DIST))
        {
            SetPoint = (Left+Right)/2;
        }else{
            SetPoint = DESIRED_DIST;
        }
        //
        if(Left < Right){
            WallError = Left-SetPoint;
        }else{
            WallError = SetPoint-Right;
        }
        UR = PWMNOMINAL+Kp*WallError;
        UL = PWMNOMINAL-Kp*WallError;
        if(UR < (PWMNOMINAL-SWING)) UR = PWMIN;
        if(UR > (PWMNOMINAL+SWING)) UR = PWMAX;
        if(UL < (PWMNOMINAL-SWING)) UL = PWMIN;
        if(UL > (PWMNOMINAL+SWING)) UL = PWMAX;
        Motor_Forward(UL, UR);
        ControllerFlag = 1;
    }
}

void Program17_2(void){
    uint32_t raw17,raw12,raw16;
    DisableInterrupts();
    Clock_Init48MHz();
    LaunchPad_Init();
    Bump_Init();

    Motor_Init();
    TimerA1_Init(&IRsampling,250);  // 2000 Hz sampling
    Motor_Stop();
    LCDClear2();
    Mode = 0;
    PauseBump = 0;
    UR = UL = PWMNOMINAL;
    ADCflag = ControllerFlag = 0;   // semaphores

    ADC0_InitSWTriggerCh17_12_16();   // initialize channels 17,12,16
    ADC_In17_12_16(&raw17,&raw12,&raw16);  // sample
    LPF_Init(raw17,64);     // P9.0/channel 17
    LPF_Init2(raw12,64);    // P4.1/channel 12
    LPF_Init3(raw16,64);    // P9.1/channel 16

    SysTick_Init(480000,2); // 100 Hz
    Pause3();

    EnableInterrupts();
    while(1){
        if(Bump_Read()){
            Mode = 0;
            Motor_Stop();
            Pause3();
        }
        if(ControllerFlag){ // 100 Hz, not real time
            LCDOut2();
            ControllerFlag = 0;
        }
    }
}

/**************Program17_3*******************/
// proportional control, line following

#define PWMNOMINAL3 6000
#define SWING3 3000
#define PWMIN3 (PWMNOMINAL3-SWING3)
#define PWMAX3 (PWMNOMINAL3+SWING3)
int32_t Kp3=20;  // proportional controller gain

uint8_t LineData;       // direct measure from line sensor
int32_t Position;      // position in 0.1mm relative to center of line

void LCDClear3(void){
  Nokia5110_Init();
  Nokia5110_Clear(); // erase entire display
  Nokia5110_OutString("17: control");
  Nokia5110_SetCursor(0,1); Nokia5110_OutString("Line Follow");
  Nokia5110_SetCursor(0,2); Nokia5110_OutString("D =  "); Nokia5110_OutUDec(0);
  Nokia5110_SetCursor(0,3); Nokia5110_OutString("P = "); Nokia5110_OutSDec(0);
  Nokia5110_SetCursor(0,4); Nokia5110_OutString("UR=  "); Nokia5110_OutUDec(0);
  Nokia5110_SetCursor(0,5); Nokia5110_OutString("UL=  "); Nokia5110_OutUDec(0);
}
void LCDOut3(void){
  Nokia5110_SetCursor(5,2); Nokia5110_OutUDec(LineData);
  Nokia5110_SetCursor(4,3); Nokia5110_OutSDec(Position);
  Nokia5110_SetCursor(5,4); Nokia5110_OutUDec(UR);
  Nokia5110_SetCursor(5,5); Nokia5110_OutUDec(UL);
}

void Controller3(void){
    Time = Time + 1;
    if(Time%10==1){
        Reflectance_Start();
    }
    if(Time%10==2){
        LineData = Reflectance_End();
        Position = Reflectance_Position(LineData);
        if(Mode){
            UR = PWMNOMINAL3 - Kp3*Position;
            UL = PWMNOMINAL3 + Kp3*Position;
            if(UR < (PWMIN3)) UR = PWMIN3;
            if(UR > (PWMAX3)) UR = PWMAX3;
            if(UL < (PWMIN3)) UL = PWMIN3;
            if(UL > (PWMAX3)) UL = PWMAX3;
            Motor_Forward(UL, UR);
        }
    }
    ControllerFlag = 1;
}

void Program17_3(void){
    DisableInterrupts();
    Clock_Init48MHz();
    LaunchPad_Init();
    Bump_Init();
    Reflectance_Init();
    Motor_Init();
    TimerA1_Init(&Controller3, 500);    // 1000 Hz interrupt, 100 Hz sampling, 100 Hz controller
    Motor_Stop();
    LCDClear3();
    Mode = 0;
    Time = 0;
    UR = UL = PWMNOMINAL3;
    EnableInterrupts();
    Pause3();
    while(1){
        if(Bump_Read()){
            Mode = 0;
            Motor_Stop();
            Pause3();
        }
      if(ControllerFlag){ // 100 Hz , not real time
        LCDOut3();
        ControllerFlag = 0;
      }
    }
}

int main(void){
//    Program17_1();
//    Program17_2();
    Program17_3();
}
