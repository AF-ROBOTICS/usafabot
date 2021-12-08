//// BeyerBot v2.0
//// Runs on MSP432
//// Integration of control systems and UART
//// Capt Steven Beyer
//// 4 December 2019
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
// *
// * Current commands implemented:
// *  (cX, cY, cYaw, nX, nY)
// *  [current X, current Y, current Yaw, next X, next Y]
// *
// *  Command must be in the format: 'X.zz,X.zz,Y.z,X.zz,X.zz\r\n'
// *  where X can be between 0 and 99, Y can be between -999 and 999, and z can be between 0 and 9.
// *
//*/
//
//#include <stdint.h>
//#include <string.h>
//#include <stdio.h>
//#include <math.h>
//#include "msp.h"
//#include "../inc/Clock.h"
//#include "../inc/CortexM.h"
//#include "../inc/LaunchPad.h"
//#include "../inc/UART0.h"
//#include "../inc/ADC14.h"
//#include "../inc/TimerA1.h"
//#include "../inc/Nokia5110.h"
//#include "../inc/EUSCIA0.h"
//#include "../inc/FIFO0.h"
//#include "../inc/Motor.h"
//
//#define PI 3.141592654
//#define PWMNOMINAL 2000
//#define DSTTHRESH 0.5
//#define HDGTHRESH 30
//#define Kh 8.0 // exaggerated for visual confirmation
//
///**************Initial values used for multiple routines*******************/
//
//// Command
//float cX, cY, cYaw, nX, nY;
//float yawErr = 0;
//float gYaw360 = 0;
//
//// Provides
//float findYawErr(float cYaw, float cX, float cY, float nX, float nY){
//    // set gYaw to -180/180 hdg in degs
//    float gYaw = atan2f(nY - cY, nX - cX);
//    gYaw = gYaw * 180.0/PI;
//
//    // conv both gYaw/cYaw to 360 hdg
//    float cYaw360, gYaw360 = 0;
//    if (cYaw < 0) cYaw360 = 360 + cYaw;
//    else cYaw360 = cYaw;
//    if (gYaw < 0) gYaw360 = 360.0 + gYaw;
//    else gYaw360 = gYaw;
//
//    // find 360 frame diff in hdg
//    float retVal = gYaw360 - cYaw360;
//    if (retVal > 180) retVal = retVal - 360;
//    if (retVal < -180) retVal = retVal + 360;
//    return retVal;
//}
//
//void LCDClear(void){
//  Nokia5110_Init();
//  Nokia5110_Clear(); // erase entire display
//  Nokia5110_OutString("BeyerBot");
//  Nokia5110_SetCursor(0,1); Nokia5110_OutString("Current                 Desired                    ");
//}
//
//void LCDOut(void){
//    Nokia5110_SetCursor(0, 2);         // one leading space, second row
//    (cX >= 0.0) ? Nokia5110_OutFloatingPoint(cX) : Nokia5110_OutFloatingPoint2(cX);
//    Nokia5110_SetCursor(7, 2);         // seven leading spaces, second row
//    (cY >= 0.0) ? Nokia5110_OutFloatingPoint(cY) : Nokia5110_OutFloatingPoint2(cY);
//    Nokia5110_SetCursor(0, 4);       // one leading space, fourth row
//    (nX >= 0.0) ? Nokia5110_OutFloatingPoint(nX) : Nokia5110_OutFloatingPoint2(nX);
//    Nokia5110_SetCursor(7, 4);       // seven leading spaces, fourth row
//    (nY >= 0.0) ? Nokia5110_OutFloatingPoint(nY) : Nokia5110_OutFloatingPoint2(nY);
//    Nokia5110_SetCursor(0, 5);
//    (cYaw >= 0.0) ? Nokia5110_OutFloatingPoint(cYaw) : Nokia5110_OutFloatingPoint2(cYaw);
//    Nokia5110_SetCursor(7, 5);
//    (yawErr >= 0.0) ? Nokia5110_OutFloatingPoint(yawErr) : Nokia5110_OutFloatingPoint2(yawErr);
//}
//
//// character tests
//int isWhitespace(char c){
//  if( c == ' ' )  return 1; // space
//  if( c == '\t' ) return 1; // tab
//  return 0; // not space or tab
//}
//
//int isPrintable(char c){
//  if( c < 0x21 ) return 0; // non printing or space
//  if( c > 0x7E ) return 0; // non printing
//  return 1; // between 21 and 7E
//}
//
//// Controller that calls appropriate command from UART
//void Controller(void){
//    LCDOut();
//    if (sqrt(powf(nY - cY, 2) + powf(nX - cX, 2)) > DSTTHRESH){
//        yawErr = findYawErr(cYaw, cX, cY, nX, nY);
//        if (abs(yawErr) > HDGTHRESH){
//            if (yawErr > 0) {
//                Motor_Forward(0, PWMNOMINAL);
//            }
//            else {
//                Motor_Forward(PWMNOMINAL, 0);
//            }
//        }
//        else {
//            int adjL, adjR = 0;
//            if (yawErr > 0) {
//                adjR = (int)(Kh * yawErr);
//                adjL = 0;
//            }
//            else {
//                adjR = 0;
//                adjL = (int)(-Kh * yawErr);
//            }
//            Motor_Forward(PWMNOMINAL + adjL, PWMNOMINAL + adjR);
//        }
//    } else {Motor_Stop();}
//}
//
//void main(void){
//
//    // initialize variables
//    char buf[40];
//
//	// initialize components
//    DisableInterrupts();
//    Clock_Init48MHz();
//    LaunchPad_Init();
//    LCDClear();
//    EUSCIA0_Init();
//    Motor_Init();
//
//    // Interrupts
//    TimerA1_Init(&Controller,500);  // 1000 Hz sampling
//
//    EnableInterrupts();
//
//    while(1){
//        // read from UART
//        EUSCIA0_InString(buf,39);
//        sscanf(buf, "%f,%f,%f,%f,%f", &cX, &cY, &cYaw, &nX, &nY);
//        buf[0] = '\0';
//    }
//}
