/*
 * File:   newmain.c
 * Author: Mike Mellitt
 *
 * Main file for program to read voltage from current sensor and adjust current
 *      sourced to battery accordingly, using I2C to communicate with MCP
 *      4706DAC.
 *
 * Created on November 26, 2012, 8:26 PM
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
#define DAC 0x60
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
int check=0;

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
/*
    SendI2C3(LED,LEDREG,LEDCLR);
    SendI2C2(LED,A);
    SendI2C2(LED,B);
    SendI2C2(LED,C);
    SendI2C2(LED,D);
    SendI2C2(LED,E);
    SendI2C2(LED,F);
    SendI2C2(LED,G);
    SendI2C2(LED,H);
    SendI2C2(LED,I);
    SendI2C2(LED,J);
    SendI2C2(LED,K);
    SendI2C2(LED,L);
    SendI2C2(LED,M);
    SendI2C2(LED,N);
    SendI2C2(LED,O);
    SendI2C2(LED,P);
    SendI2C2(LED,Q);
    SendI2C2(LED,R);
    SendI2C2(LED,S);
    SendI2C2(LED,T);
    SendI2C2(LED,U);
    SendI2C2(LED,V);
    SendI2C2(LED,W);
    SendI2C2(LED,X);
    SendI2C2(LED,Y);
    SendI2C2(LED,Z);
    SendI2C2(LED,ZERO);
    SendI2C2(LED,ONE);
    SendI2C2(LED,TWO);
    SendI2C2(LED,THREE);
    SendI2C2(LED,FOUR);
    SendI2C2(LED,FIVE);
    SendI2C2(LED,SIX);
    SendI2C2(LED,SEVEN);
    SendI2C2(LED,EIGHT);
    SendI2C2(LED,NINE);
    SendI2C2(LED,COLON);
    SendI2C2(LED,EQUAL);
*/



    
      
    // get analog voltage reading, store, write to DAC output.  Always runs
    while(1)
    {
        
        SendI2C3(0xC0,0b01100000,0b10010110);
        SendI2C3(0xCC,0b01100000,0b10010110);
        SendI2C3(0xCE,0b01100000,0b10010110);
        SendI2C3(0xCA,0b01100000,0b10010110);
        SendI2C3(0xC8,0b01100000,0b10010110);
        SendI2C3(0xC6,0b01100000,0b10010110);
        SendI2C3(0xC4,0b01100000,0b10010110);
        SendI2C3(0xC2,0b01100000,0b10010110);
        
        // I2C magic
        Cvolt = getAnalog();

    }
    return (EXIT_SUCCESS);
}

// Configure timer and print present reading to PuTTY
void __ISR(8, IPL3AUTO) Timer2Hand(void)
{


        
        
        INTClearFlag(INT_T2);
        // output to UART in text, showing voltage formatted as double
        printf("Voltage Reading is: %5.2f, %d \n", Cvolt,check);
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
    #define PARAM1  ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON
    // ADC ref external   | disable offset test    | disable scan mode
    //      | perform 8 samples | use dual buffers | use alternate mode
    #define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_8 | ADC_ALT_BUF_ON        | ADC_ALT_INPUT_OFF
    // use ADC PB clock| set sample time | auto
    #define PARAM3  ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_2 | ADC_CLK_AUTO
    // AN7 as analog inputs
    #define PARAM4	ENABLE_AN7_ANA
    // do not assign channels to scan
    #define PARAM5	SKIP_SCAN_ALL

    // configure to sample AN7 B7
    SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN7);
    // configure ADC using the parameters defined above
    OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 );
    // Note the 65 NS minimum TAD from datasheet, don't use FRM
    AD1CON3bits.ADCS=0x01;
    EnableADC10();
}

//
double getAnalog()
{
    int Voltage;
    while ( ! mAD1GetIntFlag() )
    {
        // wait for the first conversion to complete so there
        // will be vaild data in ADC result registers
    }
        // Reads the buffer that is not being populated
        // (we don't want to read the active buffer)
    if(AD1CON2bits.BUFS==1)
    {
	Voltage=ReadADC10(0); 
        // Obviously I can read 8 buffers here,
        // but one will do for a sanity check
    }
    else
    {
	Voltage=ReadADC10(8); 
        // Obviously I can read 8 buffers here,
        // but one will do for a sanity check
    }
    mAD1ClearIntFlag();
    // Clear ADC interrupt flag
    check=Voltage;
    return((Voltage*.003185));
}

// Configure I2C registers
void ConfigI2C()
{
    //1 USES RD10 AS SCL1 AND RD 9 AS SDA1
    /// I2CxCON I2CxSTAT I2CxADD I2CxMSK I2CxTRN I2CxRCV
    I2C1BRG=0x186;                  //390 for 80MHz to 100KHz
    I2C1CONbits.A10M=0;             //Use 7-bit addresses
    I2C1CONbits.DISSLW=1;           //disable slew control for standard
    I2C1CONbits.ACKDT=0;            //Use and ACK not NACK
    I2C1ADD=22;                     //Sets slave address for PIC32
    TRISD=0;                        //Sets Port D to output
    I2C1CONbits.ON=1;               //turn on I2C
}

// Start
void I2C_start(void)
{
    I2C1CONbits.SEN=1;          //send start
    while(I2C1CONbits.SEN){}  //waits till start bit detected
}
void I2C_restart(void)
{
    I2C1CONbits.RSEN=1;         //send restart
    while(I2C1CONbits.RSEN){}  //waits till start bit detected
}
void I2C_stop(void)
{
    I2C1CONbits.PEN=1;          //send stop
    while(I2C1CONbits.PEN){}    //waits till stop bit detected
}
char I2C_write(char data)
{
    I2C1TRN=data;                   //sends data to transmit register
    while(I2C1STATbits.TRSTAT==1){} //waits to finsh transmission
    return(I2C1STATbits.ACKSTAT);   //returns 0 for ack received
}

int I2C_writeDAC(int data)
{
    I2C1TRN=data;                   //sends data to transmit register
    while(I2C1STATbits.TRSTAT==1){} //waits to finsh transmission
    return(I2C1STATbits.ACKSTAT);   //returns 0 for ack received
}


void mAckI2C1(void)
{
I2C1CONbits.ACKDT=0;
I2C1CONbits.ACKEN=1;
while(I2C1CONbits.ACKEN){}
}

void mNAckI2C1(void)
{
I2C1CONbits.ACKDT=1;
I2C1CONbits.ACKEN=1;
while(I2C1CONbits.ACKEN){}
}

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
void I2C_idle()
{
while((I2C1CON&0x001F)!=0){}
//Wait for Acken, Rcen, Pen, Rsen and Sen to clear
}
void SendI2C3(char addrs,char regis, char data)
{
    char ack;
    I2C_start();
    ack=I2C_write(addrs); //Address for LED is 0x50
    ack=I2C_write(regis); //0xFE for LED
    ack=I2C_write(data);  //0x20to0x7F standard
    I2C_stop();
}


void SendI2C2(char addrs, char data)
{
    char ack;
    I2C_start();
    ack=I2C_write(addrs); //Address for LED is 0x50
    ack=I2C_write(data);  //0x20to0x7F standard
    I2C_stop();
}

void SendI2CDAC(char addrs, int dacVal)
{
    char ack;
    int readVal;
    I2C_start();
    ack=I2C_write(addrs); //Address for DAC is 0x60
    readVal=I2C_writeDAC(dacVal);
    I2C_stop();
}

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