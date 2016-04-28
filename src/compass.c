/*
 * WMXZ Teensy device library
 * Copyright (c) 2016 Walter Zimmer.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
 //compass.c
#include <stdint.h>
#include "core_pins.h"
//
#include "iic.h"
#include "jobs.h"
#include "pdb.h"
#include "compass.h"

int m_compass_cnt=0;
int m_compass_done=0;
I2C_FUNC m_compass_funct=0;

int usb_serial_putchar(uint8_t c);
int usb_serial_write(const void *buffer, uint32_t size);
int usb_serial_write_buffer_free(void);
void usb_serial_flush_output(void);

void logg(uint8_t c) {	usb_serial_putchar(c); usb_serial_flush_output();}

typedef struct
{ 	uint8_t address;
	uint8_t reg;
	uint8_t *data;
	uint8_t ndat;
} COMPASS_STR;

COMPASS_STR *m_compass;

#define NSENS (4)
COMPASS_STR compass_seq[]=
{	{ gyroAddress, L3G_OUT_X_L | (1<<7), 0, 6},
	{ accAddress,  ACC_BASE    | (1<<7), 0, 6},
	{ magAddress,  MAG_BASE    | (1<<7), 0, 6},
	{ magAddress,  TMP_BASE    | (1<<7), 0, 2}
};

void compass_initRead(Ptr cd,  Ptr dd);
void compass_readData(Ptr cd, Ptr dd);
void compass_TX_CB(Ptr d, int n);
void compass_RX_CB(Ptr d, int n);

//(Fxn_t funct, Ptr state, Ptr data, int nice, uint32_t uDelay)
void compass_TX_CB(Ptr d, int n)
{	DELAY_add(compass_readData,0,0,-1,25);
}

void compass_RX_CB(Ptr d, int n)
{	//
	m_compass=&compass_seq[m_compass_cnt++];
	//
	if(m_compass_cnt<=NSENS)
		DELAY_add(compass_initRead,0,0,-1,1000); // delay between compass readings in usec
	else
		m_compass_funct(0,0);
}

void compass_initRead(Ptr cd,  Ptr dd)
{	m_compass->data[0]=m_compass->reg; // add continuous read-out mode
	i2c_send(m_compass->address,m_compass->data,1,0, compass_TX_CB);
}

void compass_readData(Ptr cd, Ptr dd)
{	i2c_receive(m_compass->address,m_compass->data,m_compass->ndat,1,compass_RX_CB);
}

void compass_getData(uint8_t *data, I2C_FUNC funct)
{	int ii;
	m_compass_funct=funct;
	// adjust data addresses
	compass_seq[0].data=data;
	for(ii=1;ii<NSENS;ii++)
	{	compass_seq[ii].data=compass_seq[ii-1].data+compass_seq[ii-1].ndat;
	}
	// start compass processing sequence
	m_compass_cnt=0;
	m_compass=&compass_seq[m_compass_cnt++];
	//
	compass_initRead(0,0);
}

//--------------------------------------------------------------------------------------------------
/*
uint8_t *readData(uint8_t address, uint8_t reg, uint8_t *data, uint8_t ndat)
{
  data[0]=reg; // add continuous read-out mode
	i2c_send(address,data,1,0,0); while(!i2c_done());
	delayMicroseconds(25);
	i2c_receive(address,data,ndat,1,0); while(!i2c_done());
  return data;
}
*/
uint8_t *readData(uint8_t address, uint8_t reg, uint8_t *data, uint8_t ndat)
{
  data[0]=reg; // add continuous read-out mode
  IIC_write(data, 1, address,0);
  IIC_read(data, ndat, address,1);
  return data;
}

//=====================acc=================
uint8_t accInit(void)
{ uint8_t retVal=0;
//
    retVal=readRegister(accAddress,WHO_AM_I);
    
    // Accelerometer
    // ODR = 0110 (100 Hz ODR); LPen = 0 (normal mode); Zen = Yen = Xen = 1 (all axes enabled)
    writeRegister(accAddress,CTRL1, 0x67); //100 Hz all accel
	writeRegister(accAddress,CTRL2, 0xC0); //50 Hz LP +-2g

  return retVal;
}

inline uint8_t *accRead( uint8_t *data)
{ return readData(accAddress, ACC_BASE | (1<<7), data,6); //read continuous 6 values
}

//=====================mag=================
uint8_t magInit(void)
{ uint8_t retVal=0;
    retVal=readRegister(magAddress, WHO_AM_I);
  
    // Magnetometer
	writeRegister(accAddress,CTRL5, 0x94); //100 Hz update
    writeRegister(accAddress,CTRL6, 0x00); //D-
    writeRegister(accAddress,CTRL7, 0x00); //D- continuous mode
  
return retVal;
}

inline uint8_t *magRead( uint8_t *data)
{ return readData(magAddress,MAG_BASE | (1<<7),data,6); // add continuous read-out mode
}

inline uint8_t *tempRead(uint8_t *data)
{ return readData(magAddress,TMP_BASE | (1<<7),data,2); // add continuous read-out mode
}

//=====================gyro=================

uint8_t gyroInit(void)
{ uint8_t retVal;

  retVal=readRegister(gyroAddress,L3G_WHO_AM_I);
  writeRegister(gyroAddress,L3G_CTRL_REG1,0x3F); // normal power mode, all axes enabled, 100 Hz fs,  25 Hz BW
  writeRegister(gyroAddress,L3G_CTRL_REG4,0x10); // 500 dps full scale
  return retVal;
}

inline uint8_t *gyroRead(uint8_t *data)
{  return readData(gyroAddress,L3G_OUT_X_L | (1<<7),data,6); // add continuous read-out mode
}

//======================== compass ===================
uint8_t compassInit(void)
{ int ret1,ret2,ret3;
  ret1=gyroInit(); 
  ret2=accInit();
  ret3=magInit();
  //
  return ret3;
}

#ifdef LSM303D
	void getGyro(uint16_t *data){ gyroRead((uint8_t *)data);}
	void getAccel(uint16_t *data){ accRead((uint8_t *)data);}
	void getMag(uint16_t *data){ magRead((uint8_t *)data);}
	void getTemp(uint16_t *data){ tempRead((uint8_t *)data);}
#else
	void getGytro(int16_t *data)
	{	int ii;
		int16_t buf[4]; uint8_t *tt=(uint8_t *) &buf[0];
		gyroRead(tt); for(ii=0;ii<3;ii++) data[ii]=buf[ii];
	}
	void getAccel(int16_t *data)
	{	int ii;
		int16_t buf[4]; uint8_t *tt=(uint8_t *) &buf[0];
		accRead(tt); for(ii=0;ii<3;ii++) data[3+ii]=buf[ii]>>4;
	}
	void getMag(int16_t *data)
	{	int ii;
		int16_t buf[4]; uint8_t *tt=(uint8_t *) &buf[0];
		magRead(tt); for(ii=0;ii<3;ii++) data[6+ii]=ByteSwap(buf[ii]);//(int16_t)(tt[jj++]<<8 | tt[jj++]);
	}
	void getTemp(int16_t *data)
	{	int ii;
		int16_t buf[4]; uint8_t *tt=(uint8_t *) &buf[0];
		tempRead(tt); data[9]=ByteSwap(buf[0])>>4;// ((int16_t)(tt[0]<<8 | tt[1]))>>4;
	}
#endif

uint8_t compassRead(int16_t *data)
{ int ii;
  int16_t buf[4]; uint8_t *tt=(uint8_t *) &buf[0];

  // data are LSB first no need for byte swap
  gyroRead(tt); for(ii=0;ii<3;ii++) data[ii]=buf[ii];
  
  #ifdef LSM303D
	  accRead(tt); for(ii=0;ii<3;ii++) data[3+ii]=buf[ii];
	  magRead(tt); for(ii=0;ii<3;ii++) data[6+ii]=buf[ii];
	  tempRead(tt); data[9]=buf[0];
  #else
	  accRead(tt); for(ii=0;ii<3;ii++) data[3+ii]=buf[ii]>>4;
	  magRead(tt); for(ii=0;ii<3;ii++) data[6+ii]=ByteSwap(buf[ii]);//(int16_t)(tt[jj++]<<8 | tt[jj++]);
	  tempRead(tt); data[9]=ByteSwap(buf[0])>>4;// ((int16_t)(tt[0]<<8 | tt[1]))>>4;
  #endif
  //
  return 1;
}



