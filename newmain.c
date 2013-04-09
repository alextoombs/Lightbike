/*
 * File:   newmain.c
 * Author: Mike Mellitt
 *         Ben Coffey
 *         Jake Thordahl
 *         Pat Bowlds
 *         Alex Toombs
 *
 * Main file for program to read voltage from current sensor and adjust current
 *      sourced to battery accordingly, using I2C to communicate with MCP
 *      4706DAC.
 *
 * Created on November 26, 2012, 8:26 PM
 *
 * Last Modified:  April 3, 2013
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/attribs.h>
#include "configbits.h"
#include <xc.h>
#include <plib.h>
#include <math.h>
#include "UART.h"
#include "Other.h"
#define A 0x41
#define B 0x42
#define C 0x43
#define D 0x44
#define E 0x45
#define F 0x46
#define G 0x47
#define H 0x48
#define I 0x49
#define J 0x4A
#define K 0x4B
#define L 0x4C
#define M 0x4D
#define N 0x4E
#define O 0x4F
#define P 0x50
#define Q 0x51
#define R 0x52
#define S 0x53
#define T 0x54
#define U 0x55
#define V 0x56
#define W 0x57
#define X 0x58
#define Y 0x59
#define Z 0x5A
#define LED 0x50
#define LEDREG 0xFE
#define DAC 0xC0
#define LEDCLR 0x51
#define LEDRIGHT 0x4A
#define ZERO 0x30
#define ONE 0x31
#define TWO 0x32
#define THREE 0x33
#define FOUR 0x34
#define FIVE 0x35
#define SIX 0x36
#define SEVEN 0x37
#define EIGHT 0x38
#define NINE 0x39
#define COLON 0x3A
#define EQUAL 0x3D
#define DECIMAL 0x2E

// voltage reading from current sensor; global variable
double Cvolt=0;
double Bvolt=0;
unsigned int offset;

// Loops continuously to adjust current source output
int main(int argc, char** argv) {
    // Uart Config 57600
    UartConfig();
    // Timer Interrupts
    ConfigTime();
    // Analog Config pin B7
    ConfigAnalog();
    // I2C Config
    ConfigI2C();

    // Configure output ports
    TRISE = 0;
    LATE = 0xFF;
      
    // get analog voltage reading, store, write to DAC Vout.  Always runs
    while(1)
    {
        // logic for increasing/decreasing current and voltage to battery
        CurrentControl();
    }
    return (EXIT_SUCCESS);
}

// Configure timer and print present reading to PuTTY
void __ISR(8, IPL3AUTO) Timer2Hand(void)
{
        INTClearFlag(INT_T2);
        // Output to UART in text, showing voltage formatted as double
        printf("\nVoltage Reading is: %5.2f \n", Cvolt);
        printf("Voltage 2 Reading is: %5.2f \n", Bvolt);

        // Display current voltage reading from analog pin B7 on LCD
        SendI2C3(LED,LEDREG,LEDCLR);
        SendI2C2(LED,V);
        SendI2C2(LED,O);
        SendI2C2(LED,L);
        SendI2C2(LED,T);
        SendI2C2(LED,A);
        SendI2C2(LED,G);
        SendI2C2(LED,E);
        SendI2C2(LED,COLON);
        SendI2C3(LED,LEDREG,LEDRIGHT);
        SendI2C2(LED,ParseFirst());
        SendI2C2(LED,DECIMAL);
        SendI2C2(LED,ParseSecond());
        SendI2C2(LED,ParseThird());
        SendI2C3(LED, LEDREG, LEDRIGHT);
        SendI2C2(LED,V);
}

// Configure bits for Timer operation
void ConfigTime()
{
    // Stops Timer and Clears register
    T2CON = 0x0;
    TMR2 = 0x0;
    // Set PR to 65535 originally 16000 with 3E80
    PR2 = 0xFFFF;     
    
    // Set prescaler at 1:256
    T2CONSET = 0x0070;
    // Start Timer
    T2CONSET = 0x8000;
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
    // Enables Global Interrupts
    INTEnableInterrupts();
    // Enables Timer2 Interrupts
    INTEnable(INT_T2, INT_ENABLED);
    // Clears timer2 flag
    INTClearFlag(INT_T2);
    // Timer2 has priority 3
    INTSetVectorPriority(INT_T2,3); 
}

// Configure analog registers to read value from sensor
void ConfigAnalog()
{
    // ensure the ADC is off before setting the configuration
    CloseADC10();
    // Turn module on  |ouput in integer| trigger mode auto | enable autosample
    #define PARAM1  ADC_MODULE_ON | ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON
    // ADC ref external   | disable offset test    | disable scan mode
    //      | perform 8 samples | use dual buffers | use alternate mode
    #define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_2 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_ON
    // use ADC PB clock| set sample time | auto
    #define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_15
    // AN7 as analog inputs
    #define PARAM4	ENABLE_AN7_ANA | ENABLE_AN8_ANA
    // do not assign channels to scan
    #define PARAM5	SKIP_SCAN_ALL

    // configure to sample AN7 B7 and AN8
    SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN7|  ADC_CH0_NEG_SAMPLEB_NVREF | ADC_CH0_POS_SAMPLEB_AN8);
    // configure ADC using the parameters defined above
    OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 );
    // Note the 65 NS minimum TAD from datasheet, don't use FRM
    //AD1CON3bits.ADCS=0x01;
    EnableADC10();
}

// Read in analog voltage from ADC
void getAnalog()
{
    while ( ! mAD1GetIntFlag() )
    {
        // wait for the first conversion to complete so there
        // will be vaild data in ADC result registers
    }
        // Reads the buffer that is not being populated
        // (we don't want to read the active buffer)
	offset = 8 * ((~ReadActiveBufferADC10() & 0x01));  // determine which buffer is idle and create an offset

		Cvolt = ReadADC10(offset)*.003185;  		// read the result of channel 4 conversion from the idle buffer
		Bvolt = ReadADC10(offset + 1)*.003185;  	// read the result of channel 5 conversion from the idle buffer
    mAD1ClearIntFlag();
    // Clear ADC interrupt flag
}

// Configure I2C registers
void ConfigI2C()
{
    //1 USES RD10 AS SCL1 AND RD 9 AS SDA1
    /// I2CxCON I2CxSTAT I2CxADD I2CxMSK I2CxTRN I2CxRCV
    I2C1BRG=0x030;                  //390 for 80MHz to 100KHz
    I2C1CONbits.A10M=0;             //Use 7-bit addresses
    I2C1CONbits.DISSLW=1;           //disable slew control for standard
    I2C1CONbits.ACKDT=0;            //Use and ACK not NACK
    I2C1ADD=22;                     //Sets slave address for PIC32
    TRISD=1;                        //Sets Port D to input
    I2C1CONbits.ON=1;               //turn on I2C
}

// Start I2C
void I2C_start(void)
{
    I2C1CONbits.SEN=1;          //send start
    while(I2C1CONbits.SEN){}  //waits till start bit detected
}

// Restart I2C
void I2C_restart(void)
{
    I2C1CONbits.RSEN=1;         //send restart
    while(I2C1CONbits.RSEN){}  //waits till start bit detected
}

// Stop I2C
void I2C_stop(void)
{
    I2C1CONbits.PEN=1;          //send stop
    while(I2C1CONbits.PEN){}    //waits till stop bit detected
}

// Write char of data to I2C line
char I2C_write(char data)
{
    I2C1TRN=data;                   //sends data to transmit register
    while(I2C1STATbits.TRSTAT==1){} //waits to finsh transmission
    return(I2C1STATbits.ACKSTAT);   //returns 0 for ack received
}

// Write int of data to I2C line
int I2C_writeDAC(int data)
{
    I2C1TRN=data;                   //sends data to transmit register
    while(I2C1STATbits.TRSTAT==1){} //waits to finsh transmission
    return(I2C1STATbits.ACKSTAT);   //returns 0 for ack received
}

// Check for acknowledgement
void mAckI2C1(void)
{
    I2C1CONbits.ACKDT=0;
    I2C1CONbits.ACKEN=1;
    while(I2C1CONbits.ACKEN){}
}

// Check for lack of acknowledgement
void mNAckI2C1(void)
{
I2C1CONbits.ACKDT=1;
I2C1CONbits.ACKEN=1;
while(I2C1CONbits.ACKEN){}
}

// Read data back from I2C line
char I2C_read(char ack)
{
    I2C1CONbits.RCEN=1;
    while(I2C1CONbits.RCEN){}
    //Reception is started, send ack/nack after read
    if(ack==0)
    {mNAckI2C1();}
    else
    {mAckI2C1();}
    //Reception should be complete - pull out data
    return(I2C1RCV);
}

// Make I2C line wait for registers to clear
void I2C_idle()
{
    while((I2C1CON&0x001F)!=0){}
    // Wait for Acken, Rcen, Pen, Rsen and Sen to clear
}

// Send data to I2C line at given address
void SendI2C3(char addrs,char regis, char data)
{
    char ack;
    I2C_start();
    ack=I2C_write(addrs); //Address for LED is 0x50
    ack=I2C_write(regis); //0xFE for LED
    ack=I2C_write(data);  //0x20to0x7F standard
    I2C_stop();
}

// General I2C call for DAC (just checks for ACK)
void SendI2CGen(char regis) {
    char ack;
    I2C_start();
    ack=I2C_write(regis);
    I2C_stop();  
}

// Writes to standard registers
void SendI2C2(char addrs, char data)
{
    char ack;
    I2C_start();
    ack=I2C_write(addrs); //Address for LED is 0x50
    ack=I2C_write(data);  //0x20to0x7F standard
    I2C_stop();
}

//void SendI2CDAC(char addrs, int dacVal)
//{
//    char ack;
//    int readVal;
//    I2C_start();
//    ack=I2C_write(addrs); //Address for DAC is 0x60
//    readVal=I2C_writeDAC(dacVal);
//    I2C_stop();
//}

// Parse first digit (ones place) of Cvolt for printing to LCD
char ParseFirst()
{
    if(Cvolt<1)
        return ZERO;
    if(Cvolt<2)
        return ONE;
    if(Cvolt<3)
        return TWO;
    if(Cvolt<4)
        return THREE;
}

// Parse second digit (tenth place) of Cvolt for printing to LCD
char ParseSecond()
{
   if(fmod(Cvolt*10,10.0)<1)
        return ZERO;
    if(fmod(Cvolt*10,10.0)<2)
        return ONE;
    if(fmod(Cvolt*10,10.0)<3)
        return TWO;
    if(fmod(Cvolt*10,10.0)<4)
        return THREE;
    if(fmod(Cvolt*10,10.0)<5)
        return FOUR;
    if(fmod(Cvolt*10,10.0)<6)
        return FIVE;
    if(fmod(Cvolt*10,10.0)<7)
        return SIX;
    if(fmod(Cvolt*10,10.0)<8)
        return SEVEN;
    if(fmod(Cvolt*10,10.0)<9)
        return EIGHT;
    if(fmod(Cvolt*10,10.0)<10)
        return NINE;
}

// Parse third digit (hundredths place) of Cvolt for LCD outputting
char ParseThird()
{
   if(fmod(Cvolt*100,10.0)<1)
        return ZERO;
    if(fmod(Cvolt*100,10.0)<2)
        return ONE;
    if(fmod(Cvolt*100,10.0)<3)
        return TWO;
    if(fmod(Cvolt*100,10.0)<4)
        return THREE;
    if(fmod(Cvolt*100,10.0)<5)
        return FOUR;
    if(fmod(Cvolt*100,10.0)<6)
        return FIVE;
    if(fmod(Cvolt*100,10.0)<7)
        return SIX;
    if(fmod(Cvolt*100,10.0)<8)
        return SEVEN;
    if(fmod(Cvolt*100,10.0)<9)
        return EIGHT;
    if(fmod(Cvolt*100,10.0)<10)
        return NINE;
}

// Convert decimal number to binary for DAC output purposes
// Returns dec as binary char
char decToDAC(int dec) {
    // confirm that dec is 8 bits or less
    if(dec <= 255|dec>=0) {
        char bits[] = "0b00000000";
        int j;

        // counts down from most significant to least
        // j = 7 for 8 bits (0 to 7 inclusive)
        for(j = 7; j >= 0; j--) {
            int k = dec >> j;

            if(k & 1)
                bits[9-j] = '1';
            else if(k & 0)
                bits[9-j] = '0';

            // UART debug statements
//            printf("decToDAC Bit #%2d: ", 8-j);
//            printf("%8s\n\n", bits);
        }
        return bits;
    }
    else if(dec<=255) {
        char high[] = "0b11111111";
//        char *highP = &high;
        return high;
    }
    else {
        char low[] = "0b00000000";
//        char *lowP = &low;
        return low;
    }
}

// Control voltage sent to DAC as a function of Cvolt read from current sensor
void CurrentControl()
{
    double current;
    //char dacVal[] = "0b00000000";

    // shift is the decimal value we use to control DAC
    int shift = 0;
    // Gets values for Cvolt and Bvolt
    getAnalog();
    // ARBITRARY CONVERSION, NEED TO CHANGE
    current = Cvolt*.0035;

    // Current should be between 8A and 9A at all times for safety
    if(current <= 8)
    {
        // if current is less than 8A, increase DAC value
        shift++;

        // safety control; keep shift at 255 (max) if it tries to go higher
        if(shift > 255)
            shift = 255;
        char dacVal=decToDAC(shift);
        printf("\nshift is: %3d\n", shift);
        printf("dacVal: %8s", dacVal);
        // write value to DAC Vout register
//        char *dacValP = &dacVal;
        SendI2C3(DAC,0b00000000,dacVal);
    }
    else if(current >= 9)
    {
        // if current is more than 9A, decrease DAC value
        shift--;

        // safety control; keep shift at 0 if it tries to go lower
        if( shift < 0)
            shift = 0;
        char dacVal=decToDAC(shift);
        printf("shift is: %3d\n", shift);
        printf("dacVal: %8s", dacVal);
        // write value to DAC Vout register
//        char *dacValP = &dacVal;
        SendI2C3(DAC,0b00000000,dacVal);
    }
}