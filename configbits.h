/* 
 * File:   configbits.h
 * Author: Mike
 *
 * Created on October 9, 2012, 1:50 PM
 */

#ifndef CONFIGBITS_H
#define	CONFIGBITS_H

/* 20 MHz crystal run at 80 mhz

 peripher clock = at 10 MHz (80 MHz/8)

 */

#pragma config FNOSC = PRIPLL // Oscillator selection
#pragma config POSCMOD = HS // Primary oscillator mode
#pragma config FPLLIDIV = DIV_5 // PLL input divider (20 -> 4)
#pragma config FPLLMUL = MUL_20 // PLL multiplier  ( 4x20 = 80)
#pragma config FPLLODIV = DIV_1 // PLL output divider
#pragma config FPBDIV = DIV_8 // Peripheral bus clock divider 10 mhz
#pragma config FSOSCEN = OFF // Secondary oscillator enable
/* Clock control settings
*/
#pragma config IESO = OFF // Internal/external clock switchover
#pragma config FCKSM = CSDCMD // Clock switching (CSx)/Clock monitor (CMx)
#pragma config OSCIOFNC = OFF // Clock output on OSCO pin enable
/* USB Settings
*/
#pragma config UPLLEN = ON // USB PLL enable
#pragma config UPLLIDIV = DIV_2 // USB PLL input divider
#pragma config FVBUSONIO = OFF // VBUS pin control
#pragma config FUSBIDIO = OFF // USBID pin control
/* Other Peripheral Device settings
*/
#pragma config FWDTEN = OFF // Watchdog timer enable
#pragma config WDTPS = PS1024 // Watchdog timer post-scaler
#pragma config FSRSSEL = PRIORITY_7 // SRS interrupt priority

#pragma config	ICESEL	= ICS_PGx1	// ICE pin selection
#endif	/* CONFIGBITS_H */

