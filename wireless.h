
#ifndef __wireless
#define __wireless
#endif

#include <stdio.h>

#include "cc2500.h"
#include "stm32f4xx.h"
#include "cmsis_os.h"


#define WIRELESS_PKT_DATALEN      10       /* Two 32bit floats = 8 bytes */

/* Packet struct */
typedef struct
{
  float f1;
  float f2;
  uint8_t b1;
  uint8_t b2;
}packet_t;


/*
	Packet control types, used to define the type of a packet for processing
*/
#define PACKET_CTRL1_PR         	 0x1
#define PACKET_CTRL1_BEGIN      	 0x2
#define PACKET_CTRL1_PITCH      	 0x3
#define PACKET_CTRL1_ROLL        	 0x4
#define PACKET_CTRL1_TIME        	 0x5
#define PACKET_CTRL1_END         	 0x6
#define PACKET_CTRL1_RECORD_BEGIN  0x7
#define PACKET_CTRL1_RECORD_PKT    0x8
#define PACKET_CTRL1_RECORD_END    0x9


/*
	Wireless application driver functions
*/
uint8_t init_wireless(void);
void transmit_pitchroll(float pitch, float roll, uint8_t ctrl2);
uint16_t receive_pitchroll(float* pitch, float* roll, uint8_t* ctrl2);

void transmit_keypad_begin();
void transmit_keypad_pitch(float pitch);
void transmit_keypad_roll(float roll);
void transmit_keypad_time(float time);
void transmit_keypad_end();
void receive_keypad(uint8_t *ctrl, float *value);

void transmit_record_sequence(int size, float *pitchBuffer, float *rollBuffer, float time_interval);
void receive_record_sequence(float *pitchBuffer, float *rollBuffer, float *time_interval);

int get_signal_strength(void);

void wait_for_idle(void);
void print_status(void);
