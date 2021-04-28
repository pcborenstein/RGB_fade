//*****************************************************************************
//
// blinky.c - Simple example to blink the on-board LED.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"

#include "inc/hw_types.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Blinky (blinky)</h1>
//!
//! A very simple example that blinks the on-board LED using direct register
//! access.
//
//*****************************************************************************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1);
}
#endif

//*****************************************************************************
//
// Blink the on-board LED.
//
//*****************************************************************************

void setColor(uint8_t r, uint8_t g, uint8_t b){

    uint8_t outOn = 0;
    uint8_t outOff = 0;
    if(r != 0){
        outOn |= PWM_OUT_5_BIT;
    }else{
        outOff |= PWM_OUT_5_BIT;
    }
    if(b != 0){
        outOn |= PWM_OUT_6_BIT;
    }else{
        outOff |= PWM_OUT_6_BIT;
    }
    if(g != 0){
        outOn |= PWM_OUT_7_BIT;
    }else{
        outOff |= PWM_OUT_7_BIT;
    }
    PWMOutputState(PWM1_BASE, outOff, false);
    PWMOutputState(PWM1_BASE, outOn, true);

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, r);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, g);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, b);
}
int
main(void)
{

    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //set sys clock to 80 MHz
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);


    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;

    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 |  GPIO_PIN_3);

    GPIOPinConfigure(GPIO_PF1_M1PWM5);
    GPIOPinConfigure(GPIO_PF2_M1PWM6);
    GPIOPinConfigure(GPIO_PF3_M1PWM7);


    PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);

    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, 256);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 256);

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 0);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 0);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 0);

    PWMGenEnable(PWM1_BASE, PWM_GEN_2);
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);

    PWMOutputState(PWM1_BASE, (PWM_OUT_5_BIT | PWM_OUT_7_BIT), true);

    //
    // Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    //
    //GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

    //
    // Loop forever.
    //
    uint16_t r = 0;
    uint16_t g = 0;
    uint16_t b = 0;
#define COLOR_DELAY     100000
#define COLOR_DELAY2    10000000
#define COLOR_MAX       200
    SysCtlDelay(COLOR_DELAY);
    while(1){
        while(g != COLOR_MAX){
            g++;
            setColor(r,g,b);
            SysCtlDelay(COLOR_DELAY);
        }
        while(r != 0){
            r--;
            setColor(r,g,b);
            SysCtlDelay(COLOR_DELAY);
        }
        SysCtlDelay(COLOR_DELAY2);
        while(b != COLOR_MAX){
            b++;
            setColor(r,g,b);
            SysCtlDelay(COLOR_DELAY);
        }
        while(g != 0){
            g--;
            setColor(r,g,b);
            SysCtlDelay(COLOR_DELAY);
        }
        SysCtlDelay(COLOR_DELAY2);
        while(r != COLOR_MAX){
            r++;
            setColor(r,g,b);
            SysCtlDelay(COLOR_DELAY);
        }
        while(b != 0){
            b--;
            setColor(r,g,b);
            SysCtlDelay(COLOR_DELAY);
        }
        SysCtlDelay(COLOR_DELAY2);
    }
}
