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
// LTC1564.c
// will be programmed via TCA9554
#include <stdint.h>
#include "kinetis.h"
#include "core_pins.h"
#include "iic.h"

int16_t setAudioGainFilter(int chan, uint8_t gain, uint8_t factor) 
{	uint8_t address[4]={32,33,34,35};
	uint8_t data[2], err=0;
	
	// configure all pins as output
	data[0]=3; data[1]=0;
	err=IIC_write(data, 2,address[chan],1);
	if(err>0) return -1; 
	//
	// set pin values
	// P0-P3 connect to Q9-Q12 i.e. G0-G3 gain
	// P4-P7 connect to Q8-Q5  i.e. F0-F3 filter //to be checked
	// if necessary use bit-reversal
//	freq=0; for(ii=0;ii<8;ii++)  freq |= ((factor>>ii) & 1)<<(7-ii); // frequency must be bit-reversed
	//
	data[0]=1; data[1]=(gain &0xf) | (factor&0xf)<<4;
	err=IIC_write(data, 2,address[chan],1);
	if(err>0) return -2; 
	else return 0;
}

