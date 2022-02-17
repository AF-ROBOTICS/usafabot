// BeyerBot v1.0
// Runs on MSP432
// Integration of control systems and UART
// Mr. Steven Beyer
// 26 November 2019

/*
 * This program reads commands from the UART and executes with a controller.
 * Feedback is provided via the tachometer and control systems are used to
 * turn the robot and make the robot go straight. Will stop and wait for
 * command on bump. LCD is updated at 100 Hz to display current command
 * and speed of wheels (rpm) when going straight.
 *
 * UART settings:
 *  Baud Rate: 115,200
 *  Parity: None
 *  Stop bits: 1 bit
 *  Data: 8 bits
 *  Endianness: LSB first
 */

#include "msp.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "..\inc\Clock.h"
#include "..\inc\CortexM.h"
#include "..\inc\PWM.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\UART0.h"
#include "..\inc\Motor.h"
#include "..\inc\Bump.h"
#include "..\inc\ADC14.h"
#include "..\inc\TimerA1.h"
#include "..\inc\IRDistance.h"
#include "..\inc\Nokia5110.h"
#include "..\inc\LPF.h"
#include "..\inc\FlashProgram.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\Tachometer.h"
#include "..\inc\Reflectance.h"
#include "..\inc\Classifier.h"
#include "..\inc\BumpInt.h"
#include "..\inc\EUSCIA0.h"
#include "..\inc\FIFO0.h"


#define PWMNOMINAL 1000
#define MIN_RPM 7
#define PWM_MIN 800
#define PWM_MAX 14998
#define TACHBUFF 10         // number of elements in tachometer array
#define LENGTH_BASE 149     // in units of 1 mm
#define WHEEL_RAD 35        // in units of 1 mm

uint16_t LeftTach[TACHBUFF];             // tachometer period of left wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection LeftDir;              // direction of left rotation (FORWARD, STOPPED, REVERSE)
int32_t LeftSteps;                       // number of tachometer steps of left wheel (units of 220/360 = 0.61 mm traveled)
uint16_t RightTach[TACHBUFF];            // tachometer period of right wheel (number of 0.0833 usec cycles to rotate 1/360 of a wheel rotation)
enum TachDirection RightDir;             // direction of right rotation (FORWARD, STOPPED, REVERSE)
int32_t RightSteps;                      // number of tachometer steps of right wheel (units of 220/360 = 0.61 mm traveled)

// counters
uint32_t Time = 0;

int32_t UL, UR;             // Controller output PWM duty 2 to 14,998

// Semaphores
volatile int Running = 0; // 0 means stopped

int32_t Kp = 5;
int32_t Ki = 2;

int32_t Ka = 0;
int32_t Kb = 0;

volatile float desiredL, desiredR;     // Desired speed
float actualL, actualR;   // Actual speed
float ErrorL, ErrorR;     // Actual - Desired
float PrevErrorL, PrevErrorR;     // previous errors
int i = 0;
float v_r = 0, v_l = 0;
float w_r = 0, w_l = 0;
volatile float actualV_r = 0, actualV_l = 0;
float xLin = 0; // m/s
float zAng = 0; // rad/s

// ------------avg------------
// Simple math function that returns the average
// value of an array.
// Input: array is an array of 16-bit unsigned numbers
//        length is the number of elements in 'array'
// Output: the average value of the array
// Note: overflow is not considered
uint16_t avg(uint16_t *array, int length){
  int j;
  uint32_t sum = 0;
  for(j=0; j<length; j=j+1){
    sum = sum + array[j];
  }
  return (sum/length);
}

// if collision, stop the robot
void Collision(uint8_t bumpSensor){
    Motor_Stop();
    Running = 0;
    Time = 0;
    i = 0;
}

void LCDClear(void){
  Nokia5110_Init();
  Nokia5110_Clear(); // erase entire display
  Nokia5110_OutString("USAFABOT");
  Nokia5110_SetCursor(0,1); Nokia5110_OutString("rSpd:");
  Nokia5110_SetCursor(0,2); Nokia5110_OutString("lSpd:");
  Nokia5110_SetCursor(0,3); Nokia5110_OutString("Actual");
  Nokia5110_SetCursor(0,4); Nokia5110_OutString("rSpd:");
  Nokia5110_SetCursor(0,5); Nokia5110_OutString("lSpd:");
}

void LCDOut(void){
    Nokia5110_SetCursor(5, 1); Nokia5110_OutSDec((desiredR*220)/60);   // one leading space, second row
    Nokia5110_SetCursor(5, 2); Nokia5110_OutSDec((desiredL*220)/60);   // seven leading spaces, second row
    Nokia5110_SetCursor(5, 4); Nokia5110_OutSDec((actualR*220)/60);
    Nokia5110_SetCursor(5, 5); Nokia5110_OutSDec((actualL*220)/60);
}

// Controller that calls appropriate command from UART
void Controller(void){
    // proportional controller to drive the robot at desired speeds
    if(Running){
//        i=(i+1)%TACHBUFF;
        Tachometer_Get(&LeftTach[i], &LeftDir, &LeftSteps, &RightTach[i], &RightDir, &RightSteps);
        i++;
         if(i >= TACHBUFF){
            i = 0;
            Time++;
            actualL = 250010/avg(LeftTach, TACHBUFF); // actual rpm
            actualR = 250010/avg(RightTach, TACHBUFF); // actual rpm

            if(desiredL == 0 && desiredR == 0)  {
                Motor_Stop();
                actualV_r = 0;
                actualV_l = 0;
            }
            else{
                // PID Controller
                ErrorL = abs(desiredL) - abs(actualL);
                ErrorR = abs(desiredR) - abs(actualR);
                UR = UR+Ka*ErrorR+Kb*PrevErrorR;
                UL = UL+Ka*ErrorL+Kb*PrevErrorL;
//                UR = UR +3*ErrorR;
//                UL = UL +3*ErrorL;
                PrevErrorR = ErrorR;
                PrevErrorL = ErrorL;

                // Bounds checking. Minimum PWM to move the motors is approximately 700% duty cycle
                if (UL < PWM_MIN) UL = PWM_MIN;
                if (UR < PWM_MIN) UR = PWM_MIN;
                if (UL > PWM_MAX) UL = PWM_MAX;
                if (UR > PWM_MAX) UR = PWM_MAX;

                if(LeftDir == REVERSE) actualL = -actualL;
                if(RightDir == REVERSE) actualR = -actualR;
                int32_t UL_new = 0;
                int32_t UR_new = 0;
                UL_new = (desiredL < 0) ? -UL : UL;
                UR_new = (desiredR < 0) ? -UR : UR;

                w_l = (actualL*2*3142)/(1000*60); // counterclockwise actual angular speed rad/sec
                w_r = (actualR*2*3142)/(1000*60); // clockwise actual angular speed rad/sec

                actualV_r = (actualR * 220)/(1000*60); // linear speed meters/second
                actualV_l = (actualL * 220)/(1000*60); // linear speed meters/second

                Motor_Drive(UL_new, UR_new);
            }
            if (Time%100 == 0) LCDOut();
        }
    }
}
int counter = 0;
void SysTick_Handler(void){
//    char velocity[15];
    char velocity[75];
    if(Running){
        sprintf(velocity, "%.2f,%.2f", actualV_r, actualV_l);
//        sprintf(velocity, "%.2f,%.2f", ErrorR, ErrorL);
//        sprintf(velocity, "%.2f,%.2f,%.2f,%.2f", v_r, w_r, v_l, w_l); // angular
//        sprintf(velocity, "%.2f,%.2f,%.2f,%.2f",desiredR, actualR, desiredL, actualL); // rpm
//        sprintf(velocity, "%.2f,%.2f,%.2f,%.2f",xLin, actualV_r, xLin, actualV_l); // linear
        strcat(velocity, "\n");
        EUSCIA0_OutString(velocity);
        counter++;
    }
}

void update_speeds(float xLin, float zAng){
    v_r = ((2*xLin*1000) + (zAng*LENGTH_BASE))/(2*WHEEL_RAD); // clockwise angular speed rad/sec
    v_l = ((2*xLin*1000) - (zAng*LENGTH_BASE))/(2*WHEEL_RAD); // counterclockwise angular rad/sec
    desiredL = (v_l*60*1000)/(2*3142); // desired speed in rpm
    desiredR = (v_r*60*1000)/(2*3142); // desired speed in rpm

    // Bounds checking to ensure desired speed does not go below capable speed
    // The tachometer can read down to about 4 rpm, but the wheels can only
    // operate reliably at about 7 rpm, so set min speed at 7 rpm.
    if(0 < desiredL && desiredL < MIN_RPM) desiredL = MIN_RPM;
    if(0 < desiredR && desiredR < MIN_RPM) desiredR = MIN_RPM;
    if((-MIN_RPM) < desiredL && desiredL < 0) desiredL = (-MIN_RPM);
    if((-MIN_RPM) < desiredR && desiredR < 0) desiredR = (-MIN_RPM);
}

void main(void){
    // initialize variables
    UR = UL = PWMNOMINAL;
    char buf[40];   // commands from UART

    int result = 0; // number of variables written by sscanf

    // initialize components
    DisableInterrupts();
    Clock_Init48MHz();
    LaunchPad_Init();
    Tachometer_Init();
    Motor_Init();
    LCDClear();
    EUSCIA0_Init();

    // initialize tachometer readings
    Tachometer_Get(&LeftTach[i], &LeftDir, &LeftSteps, &RightTach[i], &RightDir, &RightSteps);
    PrevErrorL = PrevErrorR = 0;

    Ka = Kp + Ki;
    Kb = -Kp + Ki;

    // Interrupts
    // BumpInt_Init(&Collision);
    TimerA1_Init(&Controller,150);  // 2000 Hz sampling   drives the robot
    SysTick_Init(4800000,2); // 10 Hz   used to send wheel velocities to Pi
    EnableInterrupts();
    while(1){
        // read from UART
        if(!Running){
            EUSCIA0_InString2(buf,39);

            if(buf[0] == 's' || buf[1] == 's'){
                Running = 1;
                EUSCIA0_OutString("start\n");
            }
        }
        else{
            EUSCIA0_InString2(buf,39);
            // Try to store the input into a linear and angular component
            result = sscanf(buf, "%f,%f", &xLin, &zAng);

            // if the UART input is 2 floats, then process input
            if (result == 2){
                update_speeds(xLin, zAng);

            }
            // else check if it is a command string
            else{
                if(buf[0] == 'q' || buf[1] == 'q'){
                    Running = 0;
                }
            }
        }
    }
}



