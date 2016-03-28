/**
   @file fputc_debug.c
   @brief Trying to redirect printf() to debug port
   @date 2012/06/25
*/
 
#include <stdio.h>
#include <rt_misc.h>
#include <stm32f4xx.h>
 
#pragma import(__use_no_semihosting_swi)
 
struct __FILE { int handle; };
FILE __stdout;
FILE __stdin;
 
int fputc(int c, FILE *stream)
{
   return(ITM_SendChar(c));
}

void _ttywrch(int c) {
  ITM_SendChar(c);
}

void _sys_exit(int return_code) {
label:  goto label;  // endless loop 
}


