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
// CS5361.c
#include "kinetis.h"
#include "core_pins.h"

#include "CS5361.h"

// private definitions
#define nOFL2	(6)
#define nOFL1	(5)
#define M1		(4)
#define M0		(3)
#define nRST	(2)

// public varaiables
unsigned int CS5361_overflow1=0;
unsigned int CS5361_overflow2=0;

void pin5Isr(void)
{ 	CS5361_overflow1++;
}
void pin6Isr(void)
{ 	CS5361_overflow2++;
}

void CS5361_setup(int speed)
{
	pinMode(nOFL1,INPUT_PULLUP);					//nOVFL1
	pinMode(nOFL2,INPUT_PULLUP);					//nOVFL2
	attachInterrupt(nOFL1, pin5Isr, FALLING);
	attachInterrupt(nOFL2, pin6Isr, FALLING);

	pinMode(M1,OUTPUT); 
	pinMode(M0,OUTPUT);
	if(speed==1)
	{	digitalWrite(M1,LOW);	//M1
		digitalWrite(M0,LOW);	//M0
	}
	else if(speed==2)
	{	digitalWrite(M1,LOW);	//M1
		digitalWrite(M0,HIGH);	//M0
	}
	else if(speed==4)
	{	digitalWrite(M1,HIGH);	//M1
		digitalWrite(M0,LOW);	//M0
	}
	pinMode(nRST,OUTPUT); digitalWrite(nRST,LOW); 	//nRST
}

void CS5361_start(void)
{
	digitalWrite(nRST,HIGH);
}
