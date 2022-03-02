// Motor.c
// Beyer
// Runs on MSP432
// Provide mid-level functions that initialize ports and
// set motor speeds to move the robot. Lab 13 starter
// Daniel Valvano
// July 8, 2017

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

// Sever VCCMD=VREG jumper on Motor Driver and Power Distribution Board and connect VCCMD to 3.3V.
//   This makes P3.7 and P3.6 low power disables for motor drivers.  0 to sleep/stop.
// Sever nSLPL=nSLPR jumper.
//   This separates P3.7 and P3.6 allowing for independent control
// Left motor direction connected to P1.7 (J2.14)
// Left motor PWM connected to P2.7/TA0CCP4 (J4.40)
// Left motor enable connected to P3.7 (J4.31)
// Right motor direction connected to P1.6 (J2.15)
// Right motor PWM connected to P2.6/TA0CCP3 (J4.39)
// Right motor enable connected to P3.6 (J2.11)

#include <stdint.h>
#include <stdlib.h>
#include "msp.h"
#include "../inc/CortexM.h"
#include "../inc/PWM.h"

// *******Lab 13 solution*******

// ------------Motor_Init------------
// Initialize GPIO pins for output, which will be
// used to control the direction of the motors and
// to enable or disable the drivers.
// The motors are initially stopped, the drivers
// are initially powered down, and the PWM speed
// control is uninitialized.
// Input: none
// Output: none
void Motor_Init(void){
  // write this as part of Lab 13
    // Initialize P1.7 and P1.6 - PH DIRL/DIRR
//    P1->SEL0 &= ~0xC0;
//    P1->SEL1 &= ~0xC0;  // configure as GPIO
//    P1->DIR |= 0xC0;    // make pins out
//    P1->OUT  &= ~0xC0;  // stop each motor

    // Switch to P5.4 and P5.5 for TI RSLK MAX PH DIRL/DIRR
    P5->SEL0 &= ~0x30;
    P5->SEL1 &= ~0x30;  // configure as GPIO
    P5->DIR |= 0x30;    // make pins out
    P5->OUT  &= ~0x30;  // stop each motor

    // Initialize P3.7 and P3.6 - nSLEEP nSLPL
    P3->SEL0 &= ~0xC0;
    P3->SEL1 &= ~0xC0;  // configure as GPIO
    P3->DIR |= 0xC0;    // make pins out
    P3->OUT  &= ~0xC0;  // low current sleep mode

    // initialize PWM, 100kHz, 0% duty
    PWM_Init34(15000, 0, 0);
}

// ------------Motor_Stop------------
// Stop the motors, power down the drivers, and
// set the PWM speed control to 0% duty cycle.
// Input: none
// Output: none
void Motor_Stop(void){
  // write this as part of Lab 13
    Motor_Init();

//    P1->OUT &= ~0xC0;

//    PWM_Init34(15000, 0, 0);
    PWM_Duty3(0);
    PWM_Duty4(0);

//    P3->OUT &= ~0xC0;
}

// ------------Motor_Forward------------
// Drive the robot forward by running left and
// right wheels forward with the given duty
// cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Forward(uint16_t leftDuty, uint16_t rightDuty){ 
  // write this as part of Lab 13
    long sr;
    sr = StartCritical();
//    P1->OUT &= ~0xC0;   // dirL/R = 0 both forward
    P5->OUT &= ~0x30;   // dirL/R = 0 both forward - TI RSLK MAX
//    PWM_Init34(15000, rightDuty, leftDuty);
    PWM_Duty3(rightDuty);
    PWM_Duty4(leftDuty);
    P3->OUT |= 0xC0;    // sleepL/R = 1 activate
    EndCritical(sr);
}

// ------------Motor_Right------------
// Turn the robot to the right by running the
// left wheel forward and the right wheel
// backward with the given duty cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Right(uint16_t leftDuty, uint16_t rightDuty){ 
  // write this as part of Lab 13
    long sr;
    sr = StartCritical();
//    P1->OUT &= ~0x80;    // dirL = 0 forward
//    P1->OUT |= 0x40;   // dirR = 1 backward

    // Pins used for TI RSLK MAX
    P5->OUT &= ~0x10;    // dirL = 0 forward
    P5->OUT |= 0x20;   // dirR = 1 backward
    //PWM_Init34(15000, rightDuty, leftDuty);
    PWM_Duty3(rightDuty);
    PWM_Duty4(leftDuty);
    P3->OUT |= 0xC0;    // sleepL/R = 1 activate
    EndCritical(sr);
}

// ------------Motor_Left------------
// Turn the robot to the left by running the
// left wheel backward and the right wheel
// forward with the given duty cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Left(uint16_t leftDuty, uint16_t rightDuty){ 
  // write this as part of Lab 13
    long sr;
    sr = StartCritical();
//    P1->OUT |= 0x80;   // dirL = 1 backward
//    P1->OUT &= ~0x40;  // dirR = 0 forward

    // Pins used for TI RSLK MAX
    P5->OUT |= 0x10;   // dirL = 1 backward
    P5->OUT &= ~0x20;  // dirR = 0 forward
    PWM_Init34(15000, rightDuty, leftDuty);
//    PWM_Duty3(rightDuty);
//    PWM_Duty4(leftDuty);
    P3->OUT |= 0xC0;    // sleepL/R = 1 activate
    EndCritical(sr);
}

// ------------Motor_Backward------------
// Drive the robot backward by running left and
// right wheels backward with the given duty
// cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Backward(uint16_t leftDuty, uint16_t rightDuty){ 
  // write this as part of Lab 13
    long sr;
    sr = StartCritical();
//    P1->OUT |= 0xC0;    // dirL/R = 1 backward
    P5->OUT |= 0x30;    // dirL/R = 1 backward - TI RSLK MAX
    //PWM_Init34(15000, rightDuty, leftDuty);
    PWM_Duty3(rightDuty);
    PWM_Duty4(leftDuty);
    P3->OUT |= 0xC0;    // sleepL/R = 1 activate
    EndCritical(sr);
}

void Motor_Drive(int16_t leftDuty, int16_t rightDuty){
    long sr;
    sr = StartCritical();
    // set direction of left wheel
    if(leftDuty > 0){
        P5->OUT &= ~0x10;    // dirL = 0 forward
    }
    else{
        P5->OUT |= 0x10;   // dirL = 1 backward
    }
    // set direction of right wheel
    if(rightDuty > 0){
        P5->OUT &= ~0x20;  // dirR = 0 forward
    }
    else{
        P5->OUT |= 0x20;   // dirR = 1 backward
    }
    PWM_Duty3((uint16_t)(abs(rightDuty)));
    PWM_Duty4((uint16_t)(abs(leftDuty)));
    P3->OUT |= 0xC0;    // sleepL/R = 1 activate
    EndCritical(sr);
}
