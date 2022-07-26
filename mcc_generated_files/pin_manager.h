/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This header file provides APIs for driver for .
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.8
        Device            :  PIC16F1619
        Driver Version    :  2.11
    The generated drivers are tested against the following:
        Compiler          :  XC8 2.36 and above
        MPLAB 	          :  MPLAB X 6.00	
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

/**
  Section: Included Files
*/

#include <xc.h>

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set RA5 procedures
#define RA5_SetHigh()            do { LATAbits.LATA5 = 1; } while(0)
#define RA5_SetLow()             do { LATAbits.LATA5 = 0; } while(0)
#define RA5_Toggle()             do { LATAbits.LATA5 = ~LATAbits.LATA5; } while(0)
#define RA5_GetValue()              PORTAbits.RA5
#define RA5_SetDigitalInput()    do { TRISAbits.TRISA5 = 1; } while(0)
#define RA5_SetDigitalOutput()   do { TRISAbits.TRISA5 = 0; } while(0)
#define RA5_SetPullup()             do { WPUAbits.WPUA5 = 1; } while(0)
#define RA5_ResetPullup()           do { WPUAbits.WPUA5 = 0; } while(0)

// get/set RB5 procedures
#define RB5_SetHigh()            do { LATBbits.LATB5 = 1; } while(0)
#define RB5_SetLow()             do { LATBbits.LATB5 = 0; } while(0)
#define RB5_Toggle()             do { LATBbits.LATB5 = ~LATBbits.LATB5; } while(0)
#define RB5_GetValue()              PORTBbits.RB5
#define RB5_SetDigitalInput()    do { TRISBbits.TRISB5 = 1; } while(0)
#define RB5_SetDigitalOutput()   do { TRISBbits.TRISB5 = 0; } while(0)
#define RB5_SetPullup()             do { WPUBbits.WPUB5 = 1; } while(0)
#define RB5_ResetPullup()           do { WPUBbits.WPUB5 = 0; } while(0)
#define RB5_SetAnalogMode()         do { ANSELBbits.ANSB5 = 1; } while(0)
#define RB5_SetDigitalMode()        do { ANSELBbits.ANSB5 = 0; } while(0)

// get/set RB7 procedures
#define RB7_SetHigh()            do { LATBbits.LATB7 = 1; } while(0)
#define RB7_SetLow()             do { LATBbits.LATB7 = 0; } while(0)
#define RB7_Toggle()             do { LATBbits.LATB7 = ~LATBbits.LATB7; } while(0)
#define RB7_GetValue()              PORTBbits.RB7
#define RB7_SetDigitalInput()    do { TRISBbits.TRISB7 = 1; } while(0)
#define RB7_SetDigitalOutput()   do { TRISBbits.TRISB7 = 0; } while(0)
#define RB7_SetPullup()             do { WPUBbits.WPUB7 = 1; } while(0)
#define RB7_ResetPullup()           do { WPUBbits.WPUB7 = 0; } while(0)
#define RB7_SetAnalogMode()         do { ANSELBbits.ANSB7 = 1; } while(0)
#define RB7_SetDigitalMode()        do { ANSELBbits.ANSB7 = 0; } while(0)

// get/set Pot aliases
#define Pot_TRIS                 TRISCbits.TRISC0
#define Pot_LAT                  LATCbits.LATC0
#define Pot_PORT                 PORTCbits.RC0
#define Pot_WPU                  WPUCbits.WPUC0
#define Pot_OD                   ODCONCbits.ODC0
#define Pot_ANS                  ANSELCbits.ANSC0
#define Pot_SetHigh()            do { LATCbits.LATC0 = 1; } while(0)
#define Pot_SetLow()             do { LATCbits.LATC0 = 0; } while(0)
#define Pot_Toggle()             do { LATCbits.LATC0 = ~LATCbits.LATC0; } while(0)
#define Pot_GetValue()           PORTCbits.RC0
#define Pot_SetDigitalInput()    do { TRISCbits.TRISC0 = 1; } while(0)
#define Pot_SetDigitalOutput()   do { TRISCbits.TRISC0 = 0; } while(0)
#define Pot_SetPullup()          do { WPUCbits.WPUC0 = 1; } while(0)
#define Pot_ResetPullup()        do { WPUCbits.WPUC0 = 0; } while(0)
#define Pot_SetPushPull()        do { ODCONCbits.ODC0 = 0; } while(0)
#define Pot_SetOpenDrain()       do { ODCONCbits.ODC0 = 1; } while(0)
#define Pot_SetAnalogMode()      do { ANSELCbits.ANSC0 = 1; } while(0)
#define Pot_SetDigitalMode()     do { ANSELCbits.ANSC0 = 0; } while(0)

// get/set S1 aliases
#define S1_TRIS                 TRISCbits.TRISC4
#define S1_LAT                  LATCbits.LATC4
#define S1_PORT                 PORTCbits.RC4
#define S1_WPU                  WPUCbits.WPUC4
#define S1_OD                   ODCONCbits.ODC4
#define S1_SetHigh()            do { LATCbits.LATC4 = 1; } while(0)
#define S1_SetLow()             do { LATCbits.LATC4 = 0; } while(0)
#define S1_Toggle()             do { LATCbits.LATC4 = ~LATCbits.LATC4; } while(0)
#define S1_GetValue()           PORTCbits.RC4
#define S1_SetDigitalInput()    do { TRISCbits.TRISC4 = 1; } while(0)
#define S1_SetDigitalOutput()   do { TRISCbits.TRISC4 = 0; } while(0)
#define S1_SetPullup()          do { WPUCbits.WPUC4 = 1; } while(0)
#define S1_ResetPullup()        do { WPUCbits.WPUC4 = 0; } while(0)
#define S1_SetPushPull()        do { ODCONCbits.ODC4 = 0; } while(0)
#define S1_SetOpenDrain()       do { ODCONCbits.ODC4 = 1; } while(0)

// get/set D7 aliases
#define D7_TRIS                 TRISCbits.TRISC5
#define D7_LAT                  LATCbits.LATC5
#define D7_PORT                 PORTCbits.RC5
#define D7_WPU                  WPUCbits.WPUC5
#define D7_OD                   ODCONCbits.ODC5
#define D7_SetHigh()            do { LATCbits.LATC5 = 1; } while(0)
#define D7_SetLow()             do { LATCbits.LATC5 = 0; } while(0)
#define D7_Toggle()             do { LATCbits.LATC5 = ~LATCbits.LATC5; } while(0)
#define D7_GetValue()           PORTCbits.RC5
#define D7_SetDigitalInput()    do { TRISCbits.TRISC5 = 1; } while(0)
#define D7_SetDigitalOutput()   do { TRISCbits.TRISC5 = 0; } while(0)
#define D7_SetPullup()          do { WPUCbits.WPUC5 = 1; } while(0)
#define D7_ResetPullup()        do { WPUCbits.WPUC5 = 0; } while(0)
#define D7_SetPushPull()        do { ODCONCbits.ODC5 = 0; } while(0)
#define D7_SetOpenDrain()       do { ODCONCbits.ODC5 = 1; } while(0)

/**
   @Param
    none
   @Returns
    none
   @Description
    GPIO and peripheral I/O initialization
   @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize (void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);



#endif // PIN_MANAGER_H
/**
 End of File
*/