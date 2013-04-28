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
 *
 * Analog pinout on board:
 *      A12:  Current Sensor
 *      A7:  Battery Stack Voltage (through divider)
 *      A6:  Power MOSFET Voltage
 *      A2,A3,A4,A5,A8,A9:  Temperature Sensors
 *
 * Created on November 26, 2012, 8:26 PM
 *
 * Last Modified:  April 27, 2013
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

 #define true 1
 #define false 0

// boolean to set debug (UART out or not)
int debug = true;
// boolean to set temp control
int tempControlOn = false;

// voltage reading from current sensor; global variable
double Csensor_volt=0;
double Battery_volt=0;

// current through stack
double current = 0;

// max rating (in volts) through A7 if stack is at 88 V
double maxStackRead = 2.73;
// real stack voltage converted up
double realStack = 0;
// MOSFET voltage
double mosVolt = 0;

// max current allowed into battery, Amps.  Critical limit to stay under
double maxCurrent = 9.0;
// max voltage across stack, Volts.  Generous window
double maxVoltage = 88.0;

// offset to adjust the DAC
//      "shift" is the value written to the DAC, which is off the gate of the
//      array of power MOSFETs.  Lowering "shift" decreases the current allowed
//      through the MOSFET and thus the battery stack that the FET is in
//      series with.
int shift = 20;

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

// int timer to control 1 hour @ 2 Amp charging routine; 120 => been 1 hour
int trickleTimer = 0;

// boolean to determine if full charge (to current < 1.0 A) is done
int isFullDone = false;
// boolean to determine if charge is totally done
int isChargeDone = false;

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

    // disable JTAG so pin 12 can be used
    DDPCONbits.JTAGEN = 0;

    // Configure output ports
    TRISE = 0;
    LATE = 0xFF;

    // run until charge routine is done
    while(!isChargeDone)
    {

    }
    return (EXIT_SUCCESS);
}

// Interrupt fires every half second
void __ISR(8, IPL3AUTO) Timer2Hand(void)
{
    INTClearFlag(INT_T2);
    if(debug==true) {
        // Display current voltage reading from analog pin B12 on LCD
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

    // update temperature sensor values from battery
    updateTemps();

    // populate sensor readings, update values
    getAnalog();
    updateValues();

    if(debug==true) {
        // show values (estimated)
        printf("\nDivided Stack Voltage Reading: %5.2f V\n", Battery_volt);
        printf("Corresponds to real voltage: %5.2f V\n", realStack);
        printf("Estimated current is: %5.2f A\n", current);
        printf("MOSFET voltage is: %5.2f V\n", mosVolt);
    }

    // Shuts down charger entirely if temperatures get too hot.
    if(tempControlOn == true) {
        printf("\nTemperature control is on...\n");
        if(temp1 >= tempLimit | temp2 >= tempLimit | temp3 >= tempLimit |
                temp4 >= tempLimit | temp5 >= tempLimit | temp6 >= tempLimit) {
            shift = 0;

            writeToDAC();
        }
    }
    else {
        if(debug == true)
            printf("\nWARNING:  Temperature control is OFF!\n");
    }

    // when stack voltage is low but current isn't low, keep doing full charge
    if(realStack < maxVoltage || current > 1.0) {
        fullCharge();
        if(debug == true)
            printf("\nDoing full charge routine");
    }
    // when stack is high AND current is low, it's time to trickle charge!
    else if(realStack >= maxVoltage - 2.0 && current < 1.0) {
        isFullDone = true;

        trickleTimer = trickleTimer + 1;

        trickleCharge();
        if(debug == true)
            printf("\nTrickle charging");
    }

    // if trickleCharge runs for 1 hour (currently 120 ISRs), charge is done.
    if(trickleTimer >= 120) {
        shift = 0;
        writeToDAC();

        isChargeDone = true;
        if(debug == true) {
            printf("\nCharging routine is done!\n");
        }

        // TO-DO:  logic to output charge is done to LCD display
    }
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
// TO-DO: ADD SIX ANALOG READ PINS FOR TEMP SENSORS
void ConfigAnalog()
{
    // ensure the ADC is off before setting the configuration
    CloseADC10();
    // Turn module on  |ouput in integer| trigger mode auto | enable autosample
    #define PARAM1  ADC_MODULE_ON | ADC_FORMAT_INTG16 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON
    // ADC ref external   | disable offset test    | disable scan mode
    //      | perform 8 samples | use dual buffers | use alternate mode
    #define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_12 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF
    // use ADC PB clock| set sample time | auto
    #define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_13 | ADC_CONV_CLK_16Tcy
    // analog inputs for current sensor, battery voltage,
    //      mosfet voltage, temp sensors.
    #define PARAM4  ENABLE_ALL_ANA
    // do not assign channels to scan
    #define PARAM5  0

    // configure all analog pins to read in (using sample A)
    SetChanADC10(ADC_CH0_NEG_SAMPLEA_NVREF);
    // configure ADC using the parameters defined above
    OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 );
    // Note the 65 NS minimum TAD from datasheet, don't use FRM
    //AD1CON3bits.ADCS=0x01;
    EnableADC10();
}

// Read analog pin values for voltages and temperatures
void getAnalog()
{
    while ( ! mAD1GetIntFlag() )
    {
        // wait for the first conversion to complete so there
        // will be vaild data in ADC result registers
    }
    Csensor_volt = ReadADC10(12)*.003185; // pin 12 (APPEARS BROKEN)
    Battery_volt = ReadADC10(7)*.003185;  // pin 7

    // get mosfet voltage
    mosVolt = ReadADC10(6) * .003185; // pin 6

    // get temp sensor voltages
    vt1 = ReadADC10(2) * .003185; // pin 2
    vt2 = ReadADC10(3) * .003185; // pin 3
    vt3 = ReadADC10(4) * .003185; // pin 4
    vt4 = ReadADC10(5) * .003185; // pin 5
    vt5 = ReadADC10(8) * .003185; // pin 8
    vt6 = ReadADC10(9) * .003185; // pin 9

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

// Parse first digit (ones place) of Csensor_volt for printing to LCD
char ParseFirst()
{
    if(Csensor_volt<1)
        return ZERO;
    if(Csensor_volt<2)
        return ONE;
    if(Csensor_volt<3)
        return TWO;
    if(Csensor_volt<4)
        return THREE;
}

// Parse second digit (tenth place) of Csensor_volt for printing to LCD
char ParseSecond()
{
   if(fmod(Csensor_volt*10,10.0)<1)
        return ZERO;
    if(fmod(Csensor_volt*10,10.0)<2)
        return ONE;
    if(fmod(Csensor_volt*10,10.0)<3)
        return TWO;
    if(fmod(Csensor_volt*10,10.0)<4)
        return THREE;
    if(fmod(Csensor_volt*10,10.0)<5)
        return FOUR;
    if(fmod(Csensor_volt*10,10.0)<6)
        return FIVE;
    if(fmod(Csensor_volt*10,10.0)<7)
        return SIX;
    if(fmod(Csensor_volt*10,10.0)<8)
        return SEVEN;
    if(fmod(Csensor_volt*10,10.0)<9)
        return EIGHT;
    if(fmod(Csensor_volt*10,10.0)<10)
        return NINE;
}

// Parse third digit (hundredths place) of Csensor_volt for LCD outputting
char ParseThird()
{
   if(fmod(Csensor_volt*100,10.0)<1)
        return ZERO;
    if(fmod(Csensor_volt*100,10.0)<2)
        return ONE;
    if(fmod(Csensor_volt*100,10.0)<3)
        return TWO;
    if(fmod(Csensor_volt*100,10.0)<4)
        return THREE;
    if(fmod(Csensor_volt*100,10.0)<5)
        return FOUR;
    if(fmod(Csensor_volt*100,10.0)<6)
        return FIVE;
    if(fmod(Csensor_volt*100,10.0)<7)
        return SIX;
    if(fmod(Csensor_volt*100,10.0)<8)
        return SEVEN;
    if(fmod(Csensor_volt*100,10.0)<9)
        return EIGHT;
    if(fmod(Csensor_volt*100,10.0)<10)
        return NINE;
}

// Charge full stack of batteries from zero until they can be used.
//    Operating conditions:  82.8V to 90V whole stack, current < 10A (fuse)
//    For full charge, voltage must be controlled at ~88V
void fullCharge()
{
    if(current >= maxCurrent) {
        // constrict MOSFET output by 5 if current gets too high
        shift = shift - 5;

        if(debug == true) {
            printf("\nWARNING:  CURRENT IS AT/ABOVE LIMIT!");
            printf("Shifting down by 5!\n");
        }
        writeToDAC();
    }
    else if(realStack < maxVoltage && current < maxCurrent) {
        // increment DAC out value by 1 if stack voltage is too low
        shift = shift + 1;

        // Output to UART in text
        if(debug == true) {
            printf("\nShifting up...");
        }
        writeToDAC();
    }
    else if(realStack > maxVoltage && current < maxCurrent) {
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
void trickleCharge() {
    double trickleCurrent = 2.0;

    if(current < trickleCurrent) {
        // increase DAC out value by 1 to push it closer to required current
        shift = shift + 1;
        writeToDAC();
    }
    else if(current > trickleCurrent) {
        // decrease DAC out value by 1 if current gets too high
        shift = shift - 1;
        writeToDAC();
    }
    else {
        // hold current DAC value at 2 Amps; shift stays constant
    }
}

// Constrict value of shift to 8 bits, 0 through 255
void shiftSafety() {
    if(shift >= 255)
        shift = 255;
    else if(shift <= 0)
        shift = 0;
}

// update variables that store current/voltage
void updateValues() {
    // linear fit conversion from voltage to current
    //      Needs more work, probably
    current = abs(10*(1.482*Csensor_volt - 3.7085));
    // estimated voltage divider conversion from analog pin read in
    realStack = Battery_volt * 88.0 / maxStackRead;
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
    if(debug == true) {
        printf("Writing shift value of: %3d\n", shift);
        printf("Calc. Current is: %3.2f A\n", current);
        printf("Estimated DAC Output voltage is: %5.3f V\n", 5.00/255.0 * shift);
        printf("Current Sensor Reading is: %5.2f V\n", Csensor_volt);
        printf("Stack Voltage Reading is: %5.2f V\n", Battery_volt);
    }

    shiftSafety();
    // write value to DAC Vout register
    SendI2C3(DAC,0b00000000,shift);
}