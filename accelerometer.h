
#ifndef __accelerometer
#define __accelerometer
#endif

#include <stdio.h>
#include "math.h"
//#include "stm32f4xx.h"
//#include "lis3dsh.h"
#include "LIS3DSH.h"
#include "atan_LUT.h"

// pg. 20, Application Note
// Output(axis) = measurement(axis) - 32 *  off_axis
#define OFFSET_X -10 // 
#define OFFSET_Y -12 //-18 
#define OFFSET_Z 10 //

//#define LIS302DL_SPI                       SPI1
//#define LIS302DL_SPI_CLK                   RCC_APB2Periph_SPI1

#define MEMSI_PIN               		GPIO_Pin_0                  /* PA.05 */
#define MEMSI_GPIO_PORT         		GPIOE                       /* GPIOA */
#define MEMSI_GPIO_CLK          		RCC_AHB1Periph_GPIOE

#define MEMSI_EXTI_PORT_SOURCE			EXTI_PortSourceGPIOE
#define MEMSI_EXTI_PIN_SOURCE       EXTI_PinSource0
#define MEMSI_EXTI_LINE							EXTI_Line0

#define MEMSI_NVIC_IRQ_CHANNEL			EXTI0_IRQn
#define MEMSI_PRIORITY 							0x01

void init_accelerometer(void);
void init_mems_interrupt(void);

void get_pitch_roll(float *pitch_pnt, float *roll_pnt);
