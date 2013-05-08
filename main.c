/*
 * File:   main.c
 * Author: Mike Mellitt
 *         Ben Coffey
 *         Jake Thordahl
 *         Pat Bowlds
 *         Alex Toombs
 *
 * Main file for program to read voltage from current sensor and adjust current
 *      sourced to battery accordingly, using I2C to communicate with MCP
 *      4706DAC.
 * Notre Dame Senior Design 2013
 * Lightbike Group
 *
 * LED Status Codes:
 * (excludes LED on RE0, status to peripheral board)
 * (1 is high)      RE3   RE1   RE2    RE0
 * State            Top LED      Bottom LED
 *  10 Amp charge   0     0     0     1
 *  High Temp       0     0     1     1
 *  Trickle charge  0     1     0     1
 *  Charge done     1     0     0     1
 *
 * Analog pinout on board:
 *      A12:  Current Sensor
 *      A7:  Battery Stack Voltage (through divider)
 *      A6:  Power MOSFET Voltage
 *      A2,A3,A4,A5,A8,A9:  Temperature Sensors
 *
 * Created on November 26, 2012, 8:26 PM
 *
 * Last Modified:  May 3, 2013
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
#define DACOUT 0x00
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

#define true 1
#define false 0

// boolean to set debug (UART out or not)
int debug = false;
// boolean to set temp control
int tempControlOn = false;

// voltage reading from current sensor; global variable
double Csensor_volt=0;
double Battery_volt=0;

// current through stack
double current = 0;

// real stack voltage converted up
double realStack = 0;
// MOSFET voltage
double mosVolt = 0;
// real-world mosfet voltage
double realMos = 0;

// should hopefully be real battery voltage
double realBatt = 0;

// max current allowed into battery, Amps.  Critical limit to stay under
double maxCurrent = 8.0;

// max voltage across stack, Volts, if 15 V per battery limit
double maxVoltage = 90;

// offset to adjust the DAC
//      "shift" is the value written to the DAC, which is off the gate of the
//      array of power MOSFETs.  Lowering "shift" decreases the current allowed
//      through the MOSFET and thus the battery stack that the FET is in
//      series with.
int shift = 0;

// read-in voltages from temperature sensors, in Volts
double vt1 = 0;
double vt2 = 0;
double vt3 = 0;
double vt4 = 0;
double vt5 = 0;
double vt6 = 0;

// converted temp sensor readings, in degrees C
double temp1;
double temp2;
double temp3;
double temp4;
double temp5;
double temp6;

// battery temp limit, in degrees C
static double tempLimit = 50.0;

// int timer to control 1 hour @ 2 Amp charging routine
int trickleTimer = 0;
// timer to control delay at beginning of charge before entering full charge state
int delayTimer = 0;

// boolean to determine if full charge (to current < 1.0 A) is done
int isHighCurrentDone = false;
// boolean to determine if charge is totally done
int isTrickleDone = false;
// boolean to determine if batteries are too hot
int isTooHot = false;

// Loops continuously to adjust current source output
int main(int argc, char** argv) {
    // Uart Config 57600 (serial baud rate)
    UartConfig();
    // Timer Interrupts
    ConfigTime();
    // Analog Config pin B7
    ConfigAnalog();
    // I2C Config 
    ConfigI2C();
    
    TRISEbits.TRISE3=0;
    LATEbits.LATE3=0;

    TRISEbits.TRISE2=0;
    LATEbits.LATE2=0;

    TRISEbits.TRISE1=0;
    LATEbits.LATE1=0;

    TRISEbits.TRISE0=0;
    LATEbits.LATE0=1;

    // run until charge routine is done
    while(1)
    {
    }
    return (EXIT_SUCCESS);
}

// Interrupt fires every half second
void __ISR(8, IPL3AUTO) Timer2Hand(void)
{
    INTClearFlag(INT_T2);
    // Display shift value on LCD
    SendI2C3(LED,LEDREG,LEDCLR);
    SendI2C2(LED,S);
    SendI2C2(LED,H);
    SendI2C2(LED,I);
    SendI2C2(LED,F);
    SendI2C2(LED,T);
    SendI2C2(LED,COLON);
    SendI2C3(LED,LEDREG,LEDRIGHT);
    SendI2C2(LED,ParseFirstShift());
    SendI2C2(LED,ParseSecondShift());
    SendI2C2(LED,ParseThirdShift());
    SendI2C3(LED,LEDREG,LEDRIGHT);
    SendI2C3(LED,LEDREG,LEDRIGHT);
    SendI2C3(LED,LEDREG,LEDRIGHT);
    SendI2C3(LED,LEDREG,LEDRIGHT);
    SendI2C3(LED,LEDREG,LEDRIGHT);
    SendI2C3(LED,LEDREG,LEDRIGHT);
    SendI2C3(LED,LEDREG,LEDRIGHT);
    SendI2C3(LED,LEDREG,LEDRIGHT);
    SendI2C3(LED,LEDREG,LEDRIGHT);
    SendI2C3(LED,LEDREG,LEDRIGHT);

    // Display current sensor reading on line two
    SendI2C2(LED,C);
    SendI2C2(LED,U);
    SendI2C2(LED,R);
    SendI2C2(LED,R);
    SendI2C2(LED,COLON);
    SendI2C3(LED,LEDREG,LEDRIGHT);
    SendI2C2(LED,ParseTens(current));
    SendI2C2(LED,ParseFirst(current));
    SendI2C2(LED,DECIMAL);
    SendI2C2(LED,ParseSecond(current));
    SendI2C2(LED,ParseThird(current));
    SendI2C3(LED,LEDREG,LEDRIGHT);
    SendI2C2(LED,A);
    SendI2C3(LED,LEDREG,LEDRIGHT);
    SendI2C3(LED,LEDREG,LEDRIGHT);
    SendI2C3(LED,LEDREG,LEDRIGHT);
    SendI2C3(LED,LEDREG,LEDRIGHT);
    SendI2C3(LED,LEDREG,LEDRIGHT);
    SendI2C3(LED,LEDREG,LEDRIGHT);
    SendI2C3(LED,LEDREG,LEDRIGHT);

    // Display current sensor reading on line two
    SendI2C2(LED,B);
    SendI2C2(LED,R);
    SendI2C2(LED,E);
    SendI2C2(LED,A);
    SendI2C2(LED,L);
    SendI2C2(LED,COLON);
    SendI2C3(LED,LEDREG,LEDRIGHT);
    SendI2C2(LED,ParseTens(realBatt));
    SendI2C2(LED,ParseFirst(realBatt));
    SendI2C2(LED,DECIMAL);
    SendI2C2(LED,ParseSecond(realBatt));
    SendI2C2(LED,ParseThird(realBatt));
    SendI2C3(LED,LEDREG,LEDRIGHT);
    SendI2C2(LED,V);
    SendI2C3(LED,LEDREG,LEDRIGHT);
    SendI2C3(LED,LEDREG,LEDRIGHT);
    SendI2C3(LED,LEDREG,LEDRIGHT);
    SendI2C3(LED,LEDREG,LEDRIGHT);
    SendI2C3(LED,LEDREG,LEDRIGHT);
    SendI2C3(LED,LEDREG,LEDRIGHT);

    // Display battery stack reading on line three
    SendI2C2(LED,R);
    SendI2C2(LED,M);
    SendI2C2(LED,O);
    SendI2C2(LED,S);
    SendI2C2(LED,COLON);
    SendI2C3(LED,LEDREG,LEDRIGHT);
    SendI2C2(LED,ParseFirst(realMos));
    SendI2C2(LED,DECIMAL);
    SendI2C2(LED,ParseSecond(realMos));
    SendI2C2(LED,ParseThird(realMos));
    SendI2C3(LED,LEDREG,LEDRIGHT);
    SendI2C2(LED,V);

    // populate sensor readings, update values
    getAnalog();
    
    // update temperature sensor values from battery
    updateTemps();
    updateValues();

    // delay timer to control delay period
    delayTimer = delayTimer + 1;

    if(debug==true) {
        // show values (estimated)
        printf("\nDivided Stack Voltage Reading: %5.2f V\n", Battery_volt);
        printf("Corresponds to real voltage: %5.2f V\n", realStack);
        printf("Estimated current is: %5.2f A\n", current);
        printf("MOSFET voltage is: %5.2f V\n", mosVolt);
    }

    // All charging logic follows below

    // Shuts down charger entirely if temperatures get too hot.
    // Switches between isTooHot false/true states
    if(tempControlOn == true) {
        printf("\nTemperature control is on...\n");
        if(temp1 >= tempLimit || temp2 >= tempLimit || temp3 >= tempLimit ||
                temp4 >= tempLimit || temp5 >= tempLimit || temp6 >= tempLimit) {
            shift = 0;

            isTooHot = true;
            writeToDAC();

            // RE3 on as warning light for high temp
            LATEbits.LATE3=1;
            LATEbits.LATE2=0;
            LATEbits.LATE1=0;
            LATEbits.LATE0=1;
        }
        // goes back into charging state if temp limit goes low enough
        else if(temp1 >= tempLimit - 10 || temp2 >= tempLimit - 10 || temp3 >= tempLimit - 10||
                temp4 >= tempLimit - 10 || temp5 >= tempLimit - 10 || temp6 >= tempLimit - 10) {
            // resets delay timer to avoid early transition to trickle state
            delayTimer = 0;
            isTooHot = false;
        }
    }
    else {
        if(debug == true)
            printf("\nWARNING:  Temperature control is OFF!\n");
    }

    // when stack voltage is low but current isn't low, keep doing full charge
    if(!isHighCurrentDone && !isTooHot && !isTrickleDone) {
        fullCharge();

        if(debug == true)
            printf("\nDoing full charge routine");
        LATEbits.LATE3=0;
        LATEbits.LATE2=0;
        LATEbits.LATE1=0;
        LATEbits.LATE0=1;
    }
    // when stack is high AND current is low, it's time to trickle charge
    else if(isHighCurrentDone && delayTimer >= 62 && !isTooHot && !isTrickleDone) {
        trickleTimer = trickleTimer + 1;

        trickleCharge();
        if(debug == true)
            printf("\nTrickle charging");
        LATEbits.LATE3=0;
        LATEbits.LATE2=0;
        LATEbits.LATE1=1;
        LATEbits.LATE0=1;
    }

    // if trickleCharge runs for 1 hour (currently ~7200 ISRs), charge is done.
    // adjust trickle charge state done
    if(trickleTimer >= 3605 && isTrickleDone) {
        shift = 0;
        writeToDAC();

        isTrickleDone = true;
        LATEbits.LATE3=1;
        LATEbits.LATE2=0;
        LATEbits.LATE1=0;
        LATEbits.LATE0=1;

        if(debug == true) {
            printf("\nCharging routine is done!\n");
        }
    }
    // when current drops below 1.0 amp, switch to conditioning trickle state
    if(current < 1.0 && delayTimer >= 62) {
        isHighCurrentDone = true;
    }
}

// Configure bits for Timer operation.  Timer fires every half-second.
void ConfigTime()
{
    // Stops Timer and Clears register
    T2CON = 0x0;
    TMR2 = 0xFD8E;

    // Set PR to 65535 originally 16000 with 3E80
    PR2 = 0x0FA0;
    
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
    // enable analog pins 8, 9, 12
    DDPCONbits.JTAGEN = 0;
    
    IFS1CLR = 2; //clear ADC conversion interrupt
    IEC1SET = 2; //enable ADC interrupt
    AD1PCFG = 0x0000; //Configure relevant pins to analog
    AD1CON1 = 0b00000000000000001000000011100110; //Configure Sample clock source
    AD1CON2 = 0b0000010000100000; //Configure ADC voltage reference

    AD1CON3 = 0x0000; //Configure ADC conversion clock
    AD1CON3bits.SAMC = 0b00001;    //auto sample at 2TAD
    AD1CON3bits.ADCS = 0b00000001; //TAD = 4TPB
    AD1CHS = 0x00000000; //Configure input channels- CH0+ input,

    AD1CON2bits.CSCNA=1;
    AD1CSSL = 0b0001001111111100;
    AD1CON1SET = 0x8000; //Turn on the ADC module
}

// Get all analog values
void getAnalog() {
    while( ! IFS1bits.AD1IF); //wait until buffers contain new samples
        AD1CON1bits.ASAM = 0;     //stop automatic sampling (shut down ADC basically)

        vt1 = ADC1BUF0*.003185;
        vt2 = ADC1BUF1*.003185;
        vt3 = ADC1BUF2*.003185;
        vt4 = ADC1BUF3*.003185;
        mosVolt = ADC1BUF4*.003185;
        Battery_volt = ADC1BUF5*.003185;
        vt5 = ADC1BUF6*.003185;
        vt6 = ADC1BUF7*.003185;
        Csensor_volt = ADC1BUF8*.003185;
        
        IFS1bits.AD1IF = 0;
        AD1CON1bits.ASAM = 1;  //restart ADC and sampling
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

// Charge full stack of batteries from zero until they can be used.
//    Constant voltage phase
//    Operating conditions:  82.8V to 90V whole stack, current < 10A (fuse)
//    For full charge, voltage must be controlled at ~88V
void fullCharge()
{
    if(current >= maxCurrent - 1.5) {
        // constrict MOSFET output by 1 if current gets too high
        shift = shift - 1;
        writeToDAC();
    }
    if(realStack < (maxVoltage)) {
        // increment DAC out value by 1 if stack voltage is too low
        shift = shift + 1;

        // Output to UART in text
        if(debug == true) {
            printf("\nShifting up...");
        }
        writeToDAC();
    }
    else if(realStack >= (maxVoltage)) {
        // decrease DAC out value by 1 if stack voltage is too high
        shift = shift - 1;

        // Output to UART in text
        if(debug == true) {
            printf("\nShifting down...");
        }
        writeToDAC();
    }
}

// Last hour of charge should control current @ 2A
//    Constant current phase
void trickleCharge() {
    double trickleCurrent = maxCurrent / 2.0;

    // increase DAC out value by 1 to push it closer to required current
    if(current < trickleCurrent) {
        shift = shift + 1;
        writeToDAC();
    }
    // decrease DAC out value by 1 if current gets too high
    else if(current > trickleCurrent) {
        shift = shift - 1;
        writeToDAC();
    }
}

// Constrict value of shift to 8 bits, 0 through 255
void shiftSafety() {
    if(shift > 255)
        shift = 255;
    else if(shift < 0)
        shift = 0;
}

// update variables that store current/voltage
void updateValues() {
    // linear fit conversion from voltage to current
    //   experientially derived
    double fakeCVolt = Csensor_volt * 2.50 / 1.30 - 2.50;
    current = 15.132 * fakeCVolt - 0.0537;

    // conversion of mosVolt to read
    realMos = mosVolt / .18;
    // gives real stack voltage
    realBatt = Battery_volt / 0.03101 - realMos;
}

// updates temperatures of battery stack
void updateTemps() {
    temp1 = vt1 * 100.0;
    temp2 = vt2 * 100.0;
    temp3 = vt3 * 100.0;
    temp4 = vt4 * 100.0;
    temp5 = vt5 * 100.0;
    temp6 = vt6 * 100.0;

    // print temps to UART
    if(debug == true) {
        printf("\nTemp1: %3.2f C\n", temp1);
        printf("Temp2: %3.2f C\n", temp2);
        printf("Temp3: %3.2f C\n", temp3);
        printf("Temp4: %3.2f C\n", temp4);
        printf("Temp5: %3.2f C\n", temp5);
        printf("Temp6: %3.2f C\n", temp6);
    }
}

// Write value of shift to DAC output register, controlling shift value as 8 bits
void writeToDAC() {
    shiftSafety();
    if(debug == true) {
        printf("Writing shift value of: %3d\n", shift);
        printf("Calc. Current is: %3.2f A\n", current);
        printf("Estimated DAC Output voltage is: %5.3f V\n", 5.00/255.0 * shift);
        printf("Current Sensor Reading is: %5.2f V\n", Csensor_volt);
        printf("Stack Voltage Reading is: %5.2f V\n", Battery_volt);
    }
    // write value to DAC Vout register
    SendI2C3(DAC,DACOUT,shift);
}

// parse tens place of input double for printing to LCD
char ParseTens(double in) {
   if(fmod(in/10,10.0)<1)
        return ZERO;
   else if(fmod(in/10,10.0)<2)
        return ONE;
   else if(fmod(in/10,10.0)<3)
        return TWO;
   else if(fmod(in/10,10.0)<4)
        return THREE;
   else if(fmod(in/10,10.0)<5)
        return FOUR;
   else if(fmod(in/10,10.0)<6)
        return FIVE;
   else if(fmod(in/10,10.0)<7)
        return SIX;
   else if(fmod(in/10,10.0)<8)
        return SEVEN;
   else if(fmod(in/10,10.0)<9)
        return EIGHT;
   else if(fmod(in/10,10.0)<10)
        return NINE;
}

// Parse first digit (ones place) of input double  for printing to LCD
char ParseFirst(double in)
{
   if(fmod(in,10.0)<1)
        return ZERO;
   else if(fmod(in,10.0)<2)
        return ONE;
   else if(fmod(in,10.0)<3)
        return TWO;
   else if(fmod(in,10.0)<4)
        return THREE;
   else if(fmod(in,10.0)<5)
        return FOUR;
   else if(fmod(in,10.0)<6)
        return FIVE;
   else if(fmod(in,10.0)<7)
        return SIX;
   else if(fmod(in,10.0)<8)
        return SEVEN;
   else if(fmod(in,10.0)<9)
        return EIGHT;
   else if(fmod(in,10.0)<10)
        return NINE;
}

// Parse second digit (tenth place) of input double for printing to LCD
char ParseSecond(double in)
{
   if(fmod(in*10,10.0)<1)
        return ZERO;
   else if(fmod(in*10,10.0)<2)
        return ONE;
   else if(fmod(in*10,10.0)<3)
        return TWO;
   else if(fmod(in*10,10.0)<4)
        return THREE;
   else if(fmod(in*10,10.0)<5)
        return FOUR;
   else if(fmod(in*10,10.0)<6)
        return FIVE;
   else if(fmod(in*10,10.0)<7)
        return SIX;
   else if(fmod(in*10,10.0)<8)
        return SEVEN;
   else if(fmod(in*10,10.0)<9)
        return EIGHT;
   else if(fmod(in*10,10.0)<10)
        return NINE;
}

// Parse third digit (hundredths place) of input double for LCD outputting
char ParseThird(double in)
{
   if(fmod(in*100,10.0)<1)
        return ZERO;
   else if(fmod(in*100,10.0)<2)
        return ONE;
   else if(fmod(in*100,10.0)<3)
        return TWO;
   else if(fmod(in*100,10.0)<4)
        return THREE;
   else if(fmod(in*100,10.0)<5)
        return FOUR;
   else if(fmod(in*100,10.0)<6)
        return FIVE;
   else if(fmod(in*100,10.0)<7)
        return SIX;
   else if(fmod(in*100,10.0)<8)
        return SEVEN;
   else if(fmod(in*100,10.0)<9)
        return EIGHT;
   else if(fmod(in*100,10.0)<10)
        return NINE;
}

// Parse the hundreds place of the integer shift for display
char ParseFirstShift() {
    if(fmod(shift/100,10)<1)
        return ZERO;
    else if(fmod(shift/100,10)<2)
        return ONE;
    else if(fmod(shift/100,10)<3)
        return TWO;
    else if(fmod(shift/100,10)<4)
        return THREE;
    else if(fmod(shift/100,10)<5)
        return FOUR;
    else if(fmod(shift/100,10)<6)
        return FIVE;
    else if(fmod(shift/100,10)<7)
        return SIX;
    else if(fmod(shift/100,10)<8)
        return SEVEN;
    else if(fmod(shift/100,10)<9)
        return EIGHT;
    else
        return NINE;
}

// Parse the tens place of the integer shift for display
char ParseSecondShift() {
    if(fmod(shift/10,10)<1)
        return ZERO;
    else if(fmod(shift/10,10)<2)
        return ONE;
    else if(fmod(shift/10,10)<3)
        return TWO;
    else if(fmod(shift/10,10)<4)
        return THREE;
    else if(fmod(shift/10,10)<5)
        return FOUR;
    else if(fmod(shift/10,10)<6)
        return FIVE;
    else if(fmod(shift/10,10)<7)
        return SIX;
    else if(fmod(shift/10,10)<8)
        return SEVEN;
    else if(fmod(shift/10,10)<9)
        return EIGHT;
    else
        return NINE;
}

// Parse the ones place of the integer shift for display
char ParseThirdShift() {
    if(fmod(shift,10)<1)
        return ZERO;
    else if(fmod(shift,10)<2)
        return ONE;
    else if(fmod(shift,10)<3)
        return TWO;
    else if(fmod(shift,10)<4)
        return THREE;
    else if(fmod(shift,10)<5)
        return FOUR;
    else if(fmod(shift,10)<6)
        return FIVE;
    else if(fmod(shift,10)<7)
        return SIX;
    else if(fmod(shift,10)<8)
        return SEVEN;
    else if(fmod(shift,10)<9)
        return EIGHT;
    else
        return NINE;
}