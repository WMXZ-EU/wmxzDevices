//Copyright 2017 by Walter Zimmer
// Version 18-05-17
//
// general teensy includes
#include <kinetis.h>
#include <core_pins.h>
#include "usb_serial.h"
//
// general (WMXZ) core library
#include "dma.h"
#include "I2S.h"
//
// local class definition
#include "ICS43432.h"

/**
 *  connections
 *  defined in I2S.c			  wmxz
 *  pin 11, PTC6, I2S0_RX_BCLK    blue
 *  pin 12, PTC7, I2S0_RX_FS      green
 *  pin 13, PTC5, I2S0_RXD0       yellow
 *  pin 30, PTC11,I2S0_RXD1 T3.2
 *  pin 38, PTC11,I2S0_RXD1 T3.6
 *  GND                           white
 *  3V3                           red
 *  L/R                           purple
 */


extern "C" int iscl[];

uint16_t c_ICS43432::init(int32_t fsamp, int32_t *buffer, uint32_t nbuf)
{
  i2s_init();
  
  float fs = i2s_speedConfig(ICS43432_DEV,N_BITS, fsamp);
  Serial.printf("Fsamp requested: %.3f kHz  got %.3f kHz\n\rCoefficients: %d %d %d\n\r" ,
        fsamp/1000.0f, fs/1000.0f, iscl[0], iscl[1], iscl[2]);
  if(fs<1.0f) return 0;
  
  i2s_config(1, N_BITS, I2S_RX_2CH, 0);
  i2s_configurePorts(2);

  DMA_init();
  i2s_setupInput(buffer,nbuf,2,5); //port, prio (8=normal)
  return 1;
}

void c_ICS43432::start(void)
{
  i2s_enableInputDMA();
  DMA_startAll();
  i2s_startInput();
}

void c_ICS43432::stop(void)
{
  i2s_stop();
}

