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
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/PWM.h"
#include "../inc/LaunchPad.h"
#include "../inc/UART0.h"
#include "../inc/Motor.h"
#include "../inc/ADC14.h"
#include "../inc/TimerA1.h"
#include "../inc/IRDistance.h"
#include "../inc/Nokia5110.h"
#include "../inc/LPF.h"
#include "../inc/FlashProgram.h"
#include "../inc/SysTickInts.h"
#include "../inc/Tachometer.h"
#include "../inc/Reflectance.h"
#include "../inc/Classifier.h"
#include "../inc/BumpInt.h"

/**************Initial values used for multiple programs*******************/
#define PWMNOMINAL 6000
#define TACHBUFF 10                      // number of elements in tachometer array

uint16_t LeftTach[TACHBUFF];             // tachometer period of left wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection LeftDir;              // direction of left rotation (FORWARD, STOPPED, REVERSE)
int32_t LeftSteps;                       // number of tachometer steps of left wheel (units of 220/360 = 0.61 mm traveled)
uint16_t RightTach[TACHBUFF];            // tachometer period of right wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection RightDir;             // direction of right rotation (FORWARD, STOPPED, REVERSE)
int32_t RightSteps;                      // number of tachometer steps of right wheel (units of 220/360 = 0.61 mm traveled)

int i = 0;

/**************Project_4*******************/
// distances in mm
#define SQUARE_DIST 360
#define DESIRED_DIST 172

int32_t TurnDegree;
uint32_t PrevLSteps=0;
uint32_t PrevRSteps=0;
int32_t SetPoint = 172;
int32_t WallError = 0;
int32_t UL, UR;             // Controller output PWM duty 2 to 14,998

// Semaphores
int32_t Mode = 0;
int32_t PauseBump = 0;
int32_t Turn = 0;
int32_t NextState = 0;
int32_t NextSquare;
volatile uint32_t ControllerFlag; // set every 10ms on controller execution

// Proportional controller
#define SWING 1500
#define PWMIN (PWMNOMINAL-SWING)
#define PWMAX (PWMNOMINAL+SWING)
int32_t Kp2=12;  // gain when walls on both sides
int32_t Kp4=4;  // gain when wall on one side

// IR globals
volatile uint32_t nr, nc, nl; // raw distance values
int32_t Left, Center, Right; // IR distances in mm
volatile uint32_t ADCflag; // Set every 500us on ADC sample

enum scenario wall_state = Error;
enum scenario next_state = Error;

void LCDClear4(void){
  Nokia5110_Init();
  Nokia5110_Clear(); // erase entire display
  Nokia5110_OutString("17: control");
  Nokia5110_SetCursor(0,1); Nokia5110_OutString("IR distance");
  Nokia5110_SetCursor(0,2); Nokia5110_OutString("L= "); Nokia5110_OutSDec(0); Nokia5110_OutString(" mm");
  Nokia5110_SetCursor(0,3); Nokia5110_OutString("C= "); Nokia5110_OutSDec(0); Nokia5110_OutString(" mm");
  Nokia5110_SetCursor(0,4); Nokia5110_OutString("R= "); Nokia5110_OutSDec(0); Nokia5110_OutString(" mm");
  Nokia5110_SetCursor(0,5);
}

void LCDOut4(void){
  Nokia5110_SetCursor(3,2); Nokia5110_OutSDec(Left);
  Nokia5110_SetCursor(3,3); Nokia5110_OutSDec(Center);
  Nokia5110_SetCursor(3,4); Nokia5110_OutSDec(Right);
  Nokia5110_SetCursor(0,5);
  if (wall_state == Error){
      Nokia5110_OutString("Error");
  }
  else if(wall_state == LeftTooClose){
      Nokia5110_OutString("LeftTooClose");
  }
  else if(wall_state == RightTooClose){
      Nokia5110_OutString("RightTooClose");
  }
  else if(wall_state == CenterTooClose)
  {
      Nokia5110_OutString("CenterTooClose");
  }
  else if(wall_state == Straight){
      Nokia5110_OutString("Straight");
  }
  else if(wall_state == LeftTurn){
      Nokia5110_OutString("LeftTurn");
  }
  else if(wall_state == RightTurn){
      Nokia5110_OutString("RightTurn");
  }
  else if(wall_state == RightTurn){
      Nokia5110_OutString("RightTurn");
  }
  else if(wall_state == TeeJoint){
      Nokia5110_OutString("TeeJoint");
  }
  else if(wall_state == LeftJoint){
      Nokia5110_OutString("LeftJoint");
  }
  else if(wall_state == RightJoint){
      Nokia5110_OutString("RightJoint");
  }
  else if(wall_state == CrossRoad)
  {
      Nokia5110_OutString("CrossRoad");
  }
  else if(wall_state == Blocked){
      Nokia5110_OutString("Blocked");
  }
}

void Pause4(void){
    int j;
    LCDOut4();
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

  UR = UL = PWMNOMINAL;    // reset parameters
  PauseBump = 0;
  ControllerFlag = 0;
  Turn = 0;
  NextSquare = 1;
  NextState = 0;
  next_state = Straight;
  PrevLSteps = LeftSteps;
  PrevRSteps = RightSteps;
  Mode = 1;
}

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

void Collision4(uint8_t bumpSensor){
    if(PauseBump == 1){
        Mode = 0;
        PauseBump = 0;
        Motor_Stop();
        Pause4();
    }
    else{
        PauseBump = 1;
    }
}

void SysTick_Handler(void){
    uint32_t degreesL;
    uint32_t degreesR;

    Tachometer_Get(&LeftTach[i], &LeftDir, &LeftSteps, &RightTach[i], &RightDir, &RightSteps);
    i = (i + 1)%10;
    wall_state = Classify(Left, Center, Right);

	if(Mode){
		// error state, pause
		if(wall_state < Straight && !Turn){
            ControllerFlag = 1;
            Mode = 0;
            Motor_Stop();
        }

		// semaphores: NextSquare - move to next square when true
		//             Turn - begin turn when true
		//             NextState - next state has been recorded when true

		// code to move to next square, obtain next state when halfway to
		// next square, and then set turn variables when in next square
		// if turn is about to occur
		if(NextSquare){
		    // walls on both sides, so use proportional controller to stay in middle
		    if((wall_state == Straight && next_state == Straight) || (wall_state == Blocked && next_state == Blocked)){
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
                UR = PWMNOMINAL+Kp2*WallError;
                UL = PWMNOMINAL-Kp2*WallError;
                if(UR < (PWMNOMINAL-SWING)) UR = PWMIN;
                if(UR > (PWMNOMINAL+SWING)) UR = PWMAX;
                if(UL < (PWMNOMINAL-SWING)) UL = PWMIN;
                if(UL > (PWMNOMINAL+SWING)) UL = PWMAX;
                Motor_Forward(UL, UR);
            }
		    // gap, so either stick to wall or
		    else{
		        if((wall_state == LeftJoint) ||  (wall_state == LeftTurn))
		        {
		            WallError = DESIRED_DIST - Right;

		        }
		        else if((wall_state == RightJoint) || (wall_state == RightTurn)){
		            WallError = Left - DESIRED_DIST;
		        }
		        else{
		            WallError = 0;
		        }
                UR = PWMNOMINAL + Kp4*WallError;
                UL = PWMNOMINAL - Kp4*WallError;
                Motor_Forward(UL, UR);
		    }

		    // when halfway to next square, gather next state information
		    if((((LeftSteps-PrevLSteps)*220)/360 >= SQUARE_DIST/2) && !NextState){
		        next_state = wall_state;
                NextState=1;
		    }

		    // when in next square, if turn, set turn variables
		    // reset tachometer step counter for turn or next square math
		    // distance = (change in steps*wheel circ)/(360 pulses per rotation)
		    else if((((LeftSteps-PrevLSteps)*220)/360 >= SQUARE_DIST) && NextState){
		        PrevLSteps = LeftSteps;
                PrevRSteps = RightSteps;
                NextState = 0;

                // if a turn is available, determine if turn is necessary
                // currently not smart: turns in direction available with left
                // as first choice
		        if (next_state > Straight){
	                // not going to next square
		            NextSquare = 0;

		            // begin turn
		            Turn = 1;

		            // determine turn degree
		            if(next_state == Blocked){
                        TurnDegree = 190;
                    }
                    else if(next_state == LeftTurn){
                        TurnDegree = 90;
                    }
                    else if(next_state == RightTurn){
                        TurnDegree = 270;
                    }
                    else if(next_state == TeeJoint){
                        TurnDegree = 270;
                    }
                    else if(next_state == CrossRoad){
                        TurnDegree = 90;
                    }
                    else if(next_state == LeftJoint){
                        TurnDegree = 0;
                    }
                    else if(next_state == RightJoint){
                        TurnDegree = 0;
                    }
                    else{
                        TurnDegree = 0;
                    }
		        }
		        // if no turn available move to next square
		        else{
		            NextSquare = 1;
		        }
		    }
		}

		// begin turn
		else if(Turn){
		    // calculate number of degrees turned so far
		    // (change in steps * circ of wheel)/circ of robot
		    degreesL = ((LeftSteps-PrevLSteps)*70)/120;
            degreesR = ((RightSteps-PrevRSteps)*70)/120;

            // if one wheel has turned enough, stop turning
            if(abs(degreesL) >= TurnDegree){
                UL = 0;
            }
            if(abs(degreesR) >= TurnDegree){
                UR = 0;
            }
            Motor_Left(UL, UR);

            // Turn complete, reset variables and move to next square
            if(abs(degreesR) >= TurnDegree && abs(degreesL) >= TurnDegree){
                Turn = 0;
                Motor_Stop();
                NextSquare = 1;
                PrevLSteps = LeftSteps;
                PrevRSteps = RightSteps;
                UR = UL = PWMNOMINAL;
            }

		}
		ControllerFlag = 1;
	}
}

// maze exploration
void Project_4(void){
    uint32_t raw17,raw12,raw16;
    UR = UL = PWMNOMINAL;

    // Initialize components
    DisableInterrupts();
    Clock_Init48MHz();
    LaunchPad_Init();
    Tachometer_Init();
    Motor_Init();
    LCDClear4();

    // Semaphores
    Mode = 0;
    Turn = 0;
    NextState = 0;
    NextSquare = 1;
    ControllerFlag = 0;

    // initialize tachometer readings
    Tachometer_Get(&LeftTach[i], &LeftDir, &LeftSteps, &RightTach[i], &RightDir, &RightSteps);
    PrevLSteps = LeftSteps;
    PrevRSteps = RightSteps;

    // initialize channels 17,12,16
    ADC0_InitSWTriggerCh17_12_16();
    ADC_In17_12_16(&raw17,&raw12,&raw16);  // sample
    LPF_Init(raw17,64);     // P9.0/channel 17
    LPF_Init2(raw12,64);    // P4.1/channel 12
    LPF_Init3(raw16,64);    // P9.1/channel 16

    // Interrupts
    BumpInt_Init(&Collision4);
    TimerA1_Init(&IRsampling,250);  // 2000 Hz sampling
    SysTick_Init(480000,2); // 100 Hz

    Motor_Stop();
    Pause4();

    EnableInterrupts();
    while(1){
        if(ControllerFlag){ // 100 Hz, not real time
            LCDOut4();
            ControllerFlag = 0;
        }
    }
}

int main(void){
    Project_4();
}
