/* 
 * File:   Other.h
 * Author: mmellitt
 *
 * Configures I2C on board and calls other configure functions
 *
 * Created on February 18, 2013, 3:04 PM
 */

#ifndef OTHER_H
#define	OTHER_H

#ifdef	__cplusplus
extern "C" {
#endif

void ConfigTime();
void ConfigAnalog();
void getAnalog();

void ConfigI2C();
void I2C_start(void);
void I2C_restart(void);
void I2C_stop(void);
char I2C_write(char data);
int I2C_writeDAC(int data);

void mAckI2C1(void);
void mNAckI2C1(void);

char ParseFirst();
char ParseSecond();
char ParseThird();

char I2C_read(char ack);
void I2C_idle();
void SendI2C3(char addrs, char regis, char data);
void SendI2C2(char addrs, char data);
void SendI2CGen(char regis);
char decToDAC(int dec);
void CurrentControl();

#ifdef	__cplusplus
}
#endif

#endif	/* OTHER_H */

