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
