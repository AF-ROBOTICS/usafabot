//// BeyerBot v1.0
//// Runs on MSP432
//// Integration of control systems and UART
//// Capt Steven Beyer
//// 26 November 2019
//
///*
// * This program reads commands from the UART and executes with a controller.
// * Feedback is provided via the tachometer and control systems are used to
// * turn the robot and make the robot go straight. Will stop and wait for
// * command on bump. LCD is updated at 100 Hz to display current command
// * and speed of wheels (rpm) when going straight.
// *
// * UART settings:
// *  Baud Rate: 115,200
// *  Parity: None
// *  Stop bits: 1 bit
// *  Data: 8 bits
// *  Endianness: LSB first
// */
//
//#include "msp.h"
//#include <stdint.h>
//#include <string.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include "..\inc\Clock.h"
//#include "..\inc\CortexM.h"
//#include "../inc/PWM.h"
//#include "../inc/LaunchPad.h"
//#include "../inc/UART0.h"
//#include "../inc/Motor.h"
//#include "../inc/Bump.h"
//#include "../inc/ADC14.h"
//#include "../inc/TimerA1.h"
//#include "../inc/IRDistance.h"
//#include "../inc/Nokia5110.h"
//#include "../inc/LPF.h"
//#include "../inc/FlashProgram.h"
//#include "../inc/SysTickInts.h"
//#include "../inc/Tachometer.h"
//#include "../inc/Reflectance.h"
//#include "../inc/Classifier.h"
//#include "../inc/BumpInt.h"
//#include "..\inc\EUSCIA0.h"
//#include "../inc/FIFO0.h"
//
//
//
///**************Initial values used for multiple programs*******************/
//#define PWMNOMINAL 500
//#define DESIRED_SPEED 100
//#define TACHBUFF 10                      // number of elements in tachometer array
//#define TURN 100                         // used with angular data z for turn rate
//
//uint16_t LeftTach[TACHBUFF];             // tachometer period of left wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
//enum TachDirection LeftDir;              // direction of left rotation (FORWARD, STOPPED, REVERSE)
//int32_t LeftSteps;                       // number of tachometer steps of left wheel (units of 220/360 = 0.61 mm traveled)
//uint16_t RightTach[TACHBUFF];            // tachometer period of right wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
//enum TachDirection RightDir;             // direction of right rotation (FORWARD, STOPPED, REVERSE)
//int32_t RightSteps;                      // number of tachometer steps of right wheel (units of 220/360 = 0.61 mm traveled)
//
//// counters
//int t = 0;
//int Time = 0;
//
//// distances in mm
//#define SQUARE_DIST 300
//
//// turn degrees
//#define DEGREES 90
//
//// Command
//char xBuf[40];
//char zBuf[40];
//
//int32_t xLin = 0;
//int32_t zAng = 0;
//
//int32_t ActualL, ActualR;   // Actual speed
//int32_t ErrorL, ErrorR;     // X* - X'
//uint32_t PrevLSteps=0;
//uint32_t PrevRSteps=0;
//int32_t UL, UR;             // Controller output PWM duty 2 to 14,998
//
//// Semaphores
//int Running = 0; // 0 means stopped
//int32_t Init = 0;
//volatile uint32_t ControllerFlag; // set every 10ms on controller execution
//
//
//// Proportional controller
//#define SWING 1000
//#define PWMIN (PWMNOMINAL-SWING)
//#define PWMAX (PWMNOMINAL+SWING)
//int32_t Kp=8;
//
//// ------------avg------------
//// Simple math function that returns the average
//// value of an array.
//// Input: array is an array of 16-bit unsigned numbers
////        length is the number of elements in 'array'
//// Output: the average value of the array
//// Note: overflow is not considered
//uint16_t avg(uint16_t *array, int length){
//  int i;
//  uint32_t sum = 0;
//  for(i=0; i<length; i=i+1){
//    sum = sum + array[i];
//  }
//  return (sum/length);
//}
//
//// if collision, stop the robot
//void Collision(uint8_t bumpSensor){
//    Motor_Stop();
//    ActualL = ActualR = 0;
//    Running = 0;
//    Time = 0;
//}
//
//// character tests
//int isWhite(char c){
//  if( c == ' ' )  return 1; // space
//  if( c == '\t' ) return 1; // tab
//  return 0; // not space or tab
//}
//
//int isPrinting(char c){
//  if( c < 0x21 ) return 0; // non printing or space
//  if( c > 0x7E ) return 0; // non printing
//  return 1; // between 21 and 7E
//}
//
//void LCDClear(void){
//  Nokia5110_Init();
//  Nokia5110_Clear(); // erase entire display
//  Nokia5110_OutString("USAFABOT");
//  Nokia5110_SetCursor(0,1); Nokia5110_OutString("xLin:");
//  Nokia5110_SetCursor(0,2); Nokia5110_OutString("zAng:");
//}
//
//void LCDOut(void){
//    Nokia5110_SetCursor(5, 1); Nokia5110_OutSDec(xLin);   // one leading space, second row
//    Nokia5110_SetCursor(5, 2); Nokia5110_OutSDec(zAng);   // seven leading spaces, second row
//
//}
//
//// Controller that calls appropriate command from UART
//void Controller(void){
////    Tachometer_Get(&LeftTach[t], &LeftDir, &LeftSteps, &RightTach[t], &RightDir, &RightSteps);
//    if(Running){
//        Time++;
////        // initialize values once per command
////        if(Init){
////            PrevLSteps = LeftSteps;
////            PrevRSteps = RightSteps;
////            Init = t = 0;
////            UL = UR = PWMNOMINAL;
////        }
//        if (xLin > 0) {
//            UL = xLin*PWMNOMINAL - zAng*TURN;
//            UR = xLin*PWMNOMINAL + zAng*TURN;
//            Motor_Forward(UL,UR);
//        }
//        else if (xLin < 0) {
//            UL = -xLin*PWMNOMINAL - zAng*TURN;
//            UR = -xLin*PWMNOMINAL + zAng*TURN;
//            Motor_Backward(UL,UR);
//        }
//        else {
//            if (zAng > 0) {
//                UL = UR = zAng*PWMNOMINAL;
//                Motor_Left(UL,UR);
//            }
//            else if (zAng < 0) {
//                UL = UR = -zAng*PWMNOMINAL;
//                Motor_Right(UL,UR);
//            }
//            else
//            {
//                Motor_Stop();
//                Running = 0;
//                Time = 0;
//            }
//        }
//        if (Time%10 == 0) LCDOut();
//    }
//    ControllerFlag = 1;
//}
//
//void main(void){
//    // initialize variables
//    UR = UL = PWMNOMINAL;
//    char buf[40];
//    int i, j;
//
//    // initialize components
//    DisableInterrupts();
//    Clock_Init48MHz();
//    LaunchPad_Init();
//    Tachometer_Init();
//    Motor_Init();
//    LCDClear();
//    EUSCIA0_Init();
//
//
//    // Semaphores
//    ControllerFlag = 1;
//    Running = 0;
//
//    // initialize tachometer readings
////    Tachometer_Get(&LeftTach[t], &LeftDir, &LeftSteps, &RightTach[t], &RightDir, &RightSteps);
////    PrevLSteps = LeftSteps;
////    PrevRSteps = RightSteps;
//
//    // Interrupts
//    BumpInt_Init(&Collision);
//    TimerA1_Init(&Controller,250);  // 2000 Hz sampling
//
//    EnableInterrupts();
//
//    while(1){
//        // Read from UART at 100Hz based on controller flag
//        if(ControllerFlag){
//            // read from UART
//            EUSCIA0_InString(buf,39);
//            i = 0;
//            while(buf[i]){
//                while(isWhite(buf[i])) i++;
//                if(isPrinting(buf[i])){
//                    j = 0;
//                    while((isPrinting(buf[i])) && (buf[i] != ',')){
//                        xBuf[j] = buf[i];                   //*
//                        j++; i++;                           //*
//                    }
//                    xBuf[j] = '\0'; // Null terminated      //*
////                    sscanf(xBuf, "%f", &xLin);              //*
//                    xLin = atoi(xBuf);
//
//                    j = 0;                                  //*
//                    i++; // skip comma                      //*
//
//                    while(isPrinting(buf[i])) {             //*
//                        zBuf[j] = buf[i];                   //*
//                        j++; i++;                           //*
//                    }                                       //*
//                    zBuf[j] = '\0'; // Null terminated      //*
////                    sscanf(zBuf, "%f", &zAng);              //*
//                    zAng = atoi(zBuf);
//
//                }
//                else i++;
//            }
//            if ((xLin == 0) && (zAng == 0)) {
//                Running = 0;
//                Motor_Stop();
//            }
//            else {
//                Running = 1;
//            }
//
//            ControllerFlag = 0;
//        }
//    }
//
//}
//
//
//
