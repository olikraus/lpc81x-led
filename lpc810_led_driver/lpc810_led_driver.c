/*
    
  lpc810_led_driver.c
  LED Step Up Converter with LPC810M021JN8 (Marking 4C)

  Copyright (C) 2014  olikraus@gmail.com

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  
  
  
  This project requires the files from directory "lpc_chip_8xx_lib" of the zipfile: lpcopen_2_01_lpcxpresso_nxp_lpcxpresso_812.zip
  Download location of lpcopen_2_01_lpcxpresso_nxp_lpcxpresso_812.zip: http://www.lpcware.com/content/nxpfile/lpcopen-software-development-platform-lpc8xx-packages
  
  
  ACMP reqires LPC810M021JN8 Marking 4C or later!

  Pin1: RESET/PIO0_5
  Pin2: PIO0_4/WAKEUP/TRST/U0_TXD			--> Mode Button
  Pin3: SWCLK/PIO0_3/TCK					--> Red LED
  Pin4: SWDIO/PIO0_2/TMS					--> Green LED
  
  Pin5: PIO0_1/ACMP_I2/TDO/ISP Entry Pin		--> Step up converter: output to MOSFET gate
  Pin6: VDD 3.3V supply voltage
  Pin7: VSS Ground
  Pin8: PIO0_0/ACMP_I1/CLKIN/TDI/U0_RXD		--> Step up converter: feedback input
  
  
  Inductance Calculation
  
  Uout: Expected output voltage
  Uin: Input voltage 
  f: Freq.
  Idelta: Accepted ocillation of the input current 
  L=(Uout-Uin)*Uin/(Uout*Idelta*f)

*/



#include <string.h>
#include "chip.h"	/* lpc_chip_8xx_lib/inc from lpcopen (lpcopen_2_01_lpcxpresso_nxp_lpcxpresso_812.zip) */

#define SYS_CORE_CLOCK 12000000UL
#define SYS_TICK_PERIOD_IN_MS 50

/* initial startup delay */
volatile uint8_t startup_delay;

/* inputs */

volatile uint8_t signal_fg_tick_timer = 0;			/* will count to 0. Foreground signal is active as long as != 0 */
volatile uint8_t signal_fg_permanent_color = 1;		/* defines, which color should be turned on permanently: bit 0: green, bit 1: red */
volatile uint8_t signal_fg_blink_color = 2;			/* defines, which color should blink: bit 0: green, bit 1: red */

volatile uint8_t signal_blink_ticks = 2;	/* 0: always on, else: blink */
volatile uint8_t signal_color = 2;		/* bit 0: green, bit 1: red */

/* internal */
volatile uint8_t signal_state = 0;
volatile uint8_t signal_cnt = 0;

volatile uint8_t signal_fg_blink_status = 0;

/*
  set a value
  value		action
  0			blink red
  1			permanently red
  2			permanently red, blink green
  3			permanently red, permanently green
  4			blink red, permanently green
  >=5		permanently green
  
  ticks: time to display the value, 1 tick = 50 ms
*/
void signal_set_fg_value(uint8_t value, uint8_t ticks)
{
  switch(value)
  {
    case 0:
      signal_fg_permanent_color = 0;
      signal_fg_blink_color = 2;
      break;
    case 1:
      signal_fg_permanent_color = 2;
      signal_fg_blink_color = 0;
      break;
    case 2:
      signal_fg_permanent_color = 2;
      signal_fg_blink_color = 1;
      break;
    case 3:
      signal_fg_permanent_color = 3;
      signal_fg_blink_color = 0;
      break;
    case 4:
      signal_fg_permanent_color = 1;
      signal_fg_blink_color = 2;
      break;
    default:
      signal_fg_permanent_color = 1;
      signal_fg_blink_color = 0;
      break;
  }
  signal_fg_tick_timer = ticks;
}

/* 
  set the signal color, based on the argument "color"
  bit 0: green, bit 1: red 
*/
void signal_set_leds(uint8_t color)
{
  if ( color & 1 )
    Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0, 2);    
  else
    Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0, 2);    
  
  if ( color & 2 )
    Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0, 3); 	
  else
    Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, 0, 3);    
}

void signal_init(void)
{
  Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, 2);	/* port 0, pin 2*/
  Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, 3);    
}

void signal_fg_task(void)
{
  if ( signal_fg_tick_timer != 0 )
  {
    signal_fg_tick_timer--;
    if ( signal_fg_tick_timer == 0 )
    {
      signal_fg_blink_status = 0;
      signal_set_leds(0);			/* turn off LEDs */
    }
    else
    {
      if ( signal_fg_blink_status == 0 )
	signal_fg_blink_status = 3;
      else
	signal_fg_blink_status = 0;
    }
    signal_set_leds((signal_fg_blink_color&signal_fg_blink_status)|signal_fg_permanent_color);
  }
}

void signal_bg_task(void)
{
  if ( signal_blink_ticks == 0 )
  {
    signal_set_leds(signal_color);
  }
  else
  {
    signal_cnt++;
    if ( signal_cnt >= signal_blink_ticks )
    {
      signal_cnt = 0;
      
      if ( signal_state == 0 )
	signal_state = 1;
      else
	signal_state = 0;

      if ( signal_state == 0 )
      {
	signal_set_leds(0);
      }
      else
      {
	signal_set_leds(signal_color);
      }
    }
  }
}

void signal_task(void)
{
  if ( signal_fg_tick_timer != 0 )
    signal_fg_task();
  else
    signal_bg_task();
}





/*
  the battery condition task will read and reset the upper counter.
*/
uint16_t battery_condition_raw_value = 0;
uint8_t battery_user_value = 0;

void battery_condition_task(void)
{
  /* read raw data, high value mean bad battery condition */
  battery_condition_raw_value = LPC_SCT->COUNT_H;

  /* clear high counter */
  LPC_SCT->CTRL_U |= SCT_CTRL_STOP_H;
  LPC_SCT->CTRL_U |= SCT_CTRL_HALT_H;	

  LPC_SCT->COUNT_H = 0;
  
  LPC_SCT->CTRL_U &= ~SCT_CTRL_HALT_H;	
  LPC_SCT->CTRL_U &= ~SCT_CTRL_STOP_H;
  
  

  /* derive user value, here 0 means very bad battery condition */
  battery_user_value = (((uint32_t)battery_condition_raw_value-0)*6UL) / 50000UL;
  if ( battery_user_value >= 5 )
    battery_user_value = 5;
  battery_user_value = 5- battery_user_value;
  
  signal_set_fg_value(battery_user_value, 40);
  
}


/* 
  lower counter:
    configure a simple PWM with 50% duty cycle
    count from 0 to TIMER_MAX 				(autolimit, match register 0)
    set output to HIGH on value TIMER_MAX/4		(event 0, match register 1)
    set output to LOW on value TIMER_MAX*3/4	(event 1, match register 2)
  upper counter: 
    free running
    started by falling edge of the PWM
    stopped by rising edge of the PWM
    Prescalar is 6
    Overflow of upper counter is 12MHz / ( 6 * 2 * 2^16 ) = 15 Hz  --> 65.54ms
    Expected max value in the 50ms SysTick ISR is 2^16 * 65.54 / 50 = (ca.) 50000 
    
*/

/*
  TIMER_MAX	 	Freq.
  120			100KHz
  60				200KHz
  30				400KHz
  12				1MHz

*/

#define TIMER_MAX (60)

/*
  disable PWM and put everything into a safe state.
*/
void pwm_stop(void)
{
  /* stop and halt both timers */
  LPC_SCT->CTRL_U |= SCT_CTRL_HALT_L|SCT_CTRL_HALT_H|SCT_CTRL_STOP_L|SCT_CTRL_STOP_H;
  
  
}

/*
  Instead of using 
    void Chip_ACMP_SetupVoltLadder(LPC_CMP_T *pACMP, uint32_t ladsel, bool ladrefVDDCMP)
  here is my own setup procedure. IMHO ladset is misleading as it requires that the arguments must be
  multiplied by 2.

  Instead, here is a special procedure for this project. 
  - select input voltage as source
  - set the ladder value (0..31)

*/
void analog_comperator_setup_ladder(uint32_t ladsel)
{
  LPC_CMP->LAD 
    = 1 << 0		/* enable ladder */
    | ladsel << 1 	/* apply ladder value */
    | 0 << 6		/* use power supply as reference voltage */
    ;
}




/*
  (re)start PWM
*/
void pwm_start(void)
{
  /* setup voltage ladder, use power supply pin (last arg = 0) 
    with 11 Ohm, bad battery condition (5V):
  
    1K + 10K
  
    5 = ???
    6 = 0.859V
    8 = 1.03V
    10 = 1.21V
    15 = 1.67V
    20 = 2.17V
    25 = 2.66V
      
  */
  analog_comperator_setup_ladder(6);

  /* start */
  LPC_SCT->CTRL_U &= ~SCT_CTRL_HALT_L;
  LPC_SCT->CTRL_U &= ~SCT_CTRL_STOP_L;
  
  LPC_SCT->CTRL_U &= ~SCT_CTRL_HALT_H;	
  LPC_SCT->CTRL_U &= ~SCT_CTRL_STOP_H;
  
  
}


void __attribute__ ((noinline)) pwm_init(void)
{
  /* init the SCT timer */
  Chip_SCT_Init(LPC_SCT);

  /* no reload, autolimit, bus clock */
  LPC_SCT->CONFIG 
    = 0<<0	/* disable UNIFY, operate as two independent */ 
    | (1<<7)	/* noreload for lower counter */
    | (1<<17)	/* enable autolimit lower counter */
    ;

  /* halt counter (to do the setup), count up, disable bidir */
  LPC_SCT->CTRL_U |= SCT_CTRL_HALT_L;
  LPC_SCT->CTRL_U |= SCT_CTRL_HALT_H;
  
  /* set prescalar to 1:1 (lower counter) and 1:6 (upper counter) */
  Chip_SCT_SetControl(LPC_SCT, SCT_CTRL_CLRCTR_L | SCT_CTRL_PRE_L(0) | SCT_CTRL_CLRCTR_H | SCT_CTRL_PRE_H(5));
  
  /* match register 0 is the autolimit value, this will devide the system clock by 12 */
  LPC_SCT->MATCH_L[0] = TIMER_MAX;
  
  /* match reload for register 0 */
  LPC_SCT->MATCHREL_L[0] = 0;

  /* program match values for a 50% duty cycle */
  LPC_SCT->MATCH_L[1] = TIMER_MAX/4;
  //LPC_SCT->MATCHREL_L[1] = 20;
  LPC_SCT->MATCH_L[2] = TIMER_MAX/4+TIMER_MAX/2;
  //LPC_SCT->MATCHREL_L[2] = 20+60;
  
  
  /* events 0 and 1 are assigned to state 0 (switching state) */
  /* match register 1, do not change state, only consider match register */
  LPC_SCT->EVENT[0].CTRL = 1 | (1 << 12) ;
  /* apply event 0 to state 0 */
  LPC_SCT->EVENT[0].STATE = 1;
  /* stop high counter with this event */
  LPC_SCT->STOP_H = 1;		/* event 0 is at bitpos 1 */

  /* match register 2, do not change state, only consider match register */
  /* event 1 will only occur if voltage is below threshold voltage */
  /* event 1 will also start the battery condition counter (high counter) */
  LPC_SCT->EVENT[1].CTRL
    = 2			/* match register */
    | (0 << 6)		/* i/o select */
    | (0 << 10) 		/* 11:10 IOCOND, 0=Low 1=Rise 2=Fall 3=High */
    | (3 << 12) 		/* 13:12 COMBMODE, 0=OR 1=Match 2=I/O 3=AND */
    | (0<<14) 		/* 14 STATELD, 0=Add 1=Load */
    | (0<<15);		/* 19:15 STATEV */
  /* apply event 1 to state 0 */
  LPC_SCT->EVENT[1].STATE = 1;
  /* start high counter with this event */
  LPC_SCT->START_H = 2;		/* event 1 is at bitpos 2 */

  /* clear output 0 register */
  LPC_SCT->OUT[0].CLR = 0;
  LPC_SCT->OUT[0].SET = 0;

  /* event 1 will set CTOUT_0 to high */
  LPC_SCT->OUT[0].SET |= 1<<1;
  
  /* event 0 will set CTOUT_0 to low */
  LPC_SCT->OUT[0].CLR |= 1<<0;

  /* assign SCT output CTOUT_0 to gpio port 0, pin 1 */
  Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, 1);	/* port 0, pin 1 */
  Chip_SWM_MovablePinAssign( SWM_CTOUT_0_O, 1 );

  /* connect negative input of the comparator to ladder output */
  Chip_ACMP_SetNegVoltRef(LPC_CMP, ACMP_NEGIN_VLO);
  
  /* connect positive input of the comparator to ACMP_I1 */
  Chip_ACMP_SetPosVoltRef(LPC_CMP, ACMP_POSIN_ACMP_I1);

  /* set hysteresis to 5mV */
  Chip_ACMP_SetHysteresis(LPC_CMP, ACMP_HYS_5MV);
  
  /* enable voltage ladder */
  //Chip_ACMP_EnableVoltLadder(LPC_CMP);

  /* enable ladder, set a default value to the ladder, will be overwritten in pwm_start() */
  analog_comperator_setup_ladder(4);

  /* Enable sync with bus for the comparator output because it will be connected to CTIN_0 */
  Chip_ACMP_EnableSyncCompOut(LPC_CMP);

  /* connect the comparator output to (none-existing) pin 17  */
  Chip_SWM_MovablePinAssign( SWM_ACMP_O_O, 17 );

  /* connect SCT input CTIN_0 with the comperator output on pin 17  */
  Chip_SWM_MovablePinAssign( SWM_CTIN_0_I, 17 );
  
  /* Enable input on ACMP_I1 */
  Chip_SWM_EnableFixedPin(SWM_FIXED_ACMP_I1);
  
}

void key_init(void)
{
  Chip_IOCON_PinSetMode(LPC_IOCON, IOCON_PIO4, PIN_MODE_PULLUP);
  Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 0, 4);
}

/* internal */

/* one tick equals 50ms */
#define KEY_MIN_PRESS_TICKS 2
#define KEY_LONG_PRESS_TICKS 20

#define KEY_STATE_WAIT 0
#define KEY_STATE_DEBOUNCE 1
#define KEY_STATE_SHORT_PRESS 2
#define KEY_STATE_LONG_PRESS 3
uint8_t key_state = KEY_STATE_WAIT;
uint8_t key_tick_cnt = 0;

void key_short_press_event(void)
{
  /*
  if ( signal_blink_ticks == 0 )
    signal_blink_ticks = 2;
  else
    signal_blink_ticks = 0;
  */
  signal_set_fg_value(battery_user_value, 40);
}

void key_long_press_event(void)
{
}

void key_task(void)
{
  uint8_t button_value;
  button_value = Chip_GPIO_GetPinState(LPC_GPIO_PORT, 0, 4);		/* active low */
  switch(key_state)
  {
    case KEY_STATE_WAIT:
      if ( button_value == 0 )
      {
	key_tick_cnt = 0;
	key_state = KEY_STATE_DEBOUNCE;
      }
      break;
    case KEY_STATE_DEBOUNCE:
      key_tick_cnt++;
      if ( button_value != 0 )
      {
	key_state = KEY_STATE_WAIT;
      }      
      else if ( key_tick_cnt >= KEY_MIN_PRESS_TICKS )
      {
	key_state = KEY_STATE_SHORT_PRESS;
      }
      break;
    case KEY_STATE_SHORT_PRESS:
      key_tick_cnt++;
      if ( button_value != 0 )
      {
	key_short_press_event();
	key_state = KEY_STATE_WAIT;
      }
      else if ( key_tick_cnt >= KEY_LONG_PRESS_TICKS )
      {
	key_state = KEY_STATE_LONG_PRESS;
      }
      break;
    case KEY_STATE_LONG_PRESS:
      if ( button_value != 0 )
      {
	key_long_press_event();
	key_state = KEY_STATE_WAIT;
      }
      break;
  }
  
}


volatile uint32_t sys_tick_irq_cnt=0;

void __attribute__ ((interrupt)) SysTick_Handler(void)
{
  sys_tick_irq_cnt++;

  if ( startup_delay > 0 )
  {
    startup_delay--;
    /* do nothing during the startup delay */
  }
  else
  {
    /* normal operation */
    
    if ( LPC_CMP->CTRL & ACMP_COMPSTAT_BIT )
      signal_color = 1;
    else
      signal_color = 2;
    
    
    battery_condition_task();
    key_task();
    signal_task();
  }
}


int __attribute__ ((noinline)) main(void)
{

  /* half second startup delay with 50ms sys tick */
  startup_delay = 10;		
  
  /* set systick and start systick interrupt */
  SysTick_Config(SYS_CORE_CLOCK/1000UL*(unsigned long)SYS_TICK_PERIOD_IN_MS);
  
  /* turn on GPIO */
  Chip_GPIO_Init(LPC_GPIO_PORT);
  
  /* setup direction of the two LEDs */
  signal_init();
  
  /* light both LEDs during the startup */
  signal_set_leds(3);
  
  /* turn on IOCON */
  Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_IOCON);
  
  /* turn on switch matrix */
  Chip_SWM_Init();
  
  /* activate analog comperator */
  Chip_ACMP_Init(LPC_CMP);

  /* disable SWCLK and SWDIO, after reset this has been activated */
  Chip_SWM_DisableFixedPin(2);
  Chip_SWM_DisableFixedPin(3);

  /* setup user key handling */
  key_init();

  /* wait until (nearly) end of start up delay */
  while( startup_delay > 2 )
    ;

  /* setup charge pump */
  pwm_init();

  pwm_start();


  /* after setting up the switch matrix, it can be turned off (will save 0.03mA) */
  //Chip_SWM_Deinit();
  
  
  
  
  
  /* enter sleep mode: Reduce from 1.4mA to 0.8mA with 12MHz */  
  while (1)
  {
    SCB->SCR |= (1UL << SCB_SCR_SLEEPONEXIT_Pos);		/* enter sleep mode after interrupt */ 
    Chip_PMU_SleepState(LPC_PMU);						/* enter sleep mode now */
  }
}

/*================================================================*/
/* 
  Reserve some space for the stack. This is used to check if global variables + stack exceed RAM size.
  If -Wl,--gc-sections is used, then also define -Wl,--undefined=arm_stack_area to keep the variable in RAM.
  The name of the variable (here: arm_stack_area) does not matter.

  Heap (=dynamic memory allocation) is not supported
*/
#ifndef __STACK_SIZE
#define __STACK_SIZE 0x100
#endif
unsigned char arm_stack_area[__STACK_SIZE] __attribute__ ((section(".stack"))) __attribute__ ((aligned(8)));


/*================================================================*/
/* isr system procedures */

/* make the top of the stack known to the c compiler, value will be calculated by the linker script */
void __StackTop(void);

void __attribute__ ((interrupt)) __attribute__ ((noreturn)) Reset_Handler(void)
{
  register unsigned long *ptr;
  register unsigned long *start;
  register unsigned long *end;

  /*     
  Loop to copy data from read only memory to RAM. The ranges
  of copy from/to are specified by following symbols evaluated in 
  linker script.
  __etext: End of code section, i.e., begin of data sections to copy from.
  __data_start__/__data_end__: RAM address range that data should be
  copied to. Both must be aligned to 4 bytes boundary.  
  */
  extern unsigned long __data_start__[];
  extern unsigned long __data_end__[];
  extern unsigned long __etext[];
  ptr = __etext;
  start = __data_start__;
  end = __data_end__;
  while( start < end )
  {
    *start = *ptr;
    start++;
    ptr++;
  }
  
  /*
  Loop to zero out BSS section, which uses following symbols
  in linker script:
  __bss_start__: start of BSS section. Must align to 4
  __bss_end__: end of BSS section. Must align to 4
  */
  extern unsigned long __bss_start__[];
  extern unsigned long __bss_end__[];
  ptr = __bss_start__;
  end = __bss_end__;
  while( ptr < end )
  {
    *ptr = 0;
    ptr++;
  }

  /* Call main procedure */  
  main();
  
  /* finished, do nothing. */
  for(;;)
    ;
}

/* "NMI_Handler" is used in the ld script to calculate the checksum */
void __attribute__ ((interrupt)) NMI_Handler(void)
{
}

/* "HardFault_Handler" is used in the ld script to calculate the checksum */
void __attribute__ ((interrupt)) HardFault_Handler(void)
{
}

/* make the checksum known to the c compiler, value will be calculated by the linker script */
void LPC_checksum(void);

/*================================================================*/
/* isr vector */

typedef void (*isr_handler_t)(void);
isr_handler_t __isr_vector[48] __attribute__ ((section(".isr_vector"))) __attribute__ ((aligned(4)))= 
{
  __StackTop,			/* Top of Stack, calculated by the linker script */
  Reset_Handler,		/* Reset Handler, DO NOT CHANGE THE ISR NAME (used for LPC_checksum calculation) */
  NMI_Handler,			/* NMI Handler, DO NOT CHANGE THE ISR NAME (used for LPC_checksum calculation) */
  HardFault_Handler,         /* Hard Fault Handler, DO NOT CHANGE THE ISR NAME (used for LPC_checksum calculation) */
  0,                                /* MemManage_Handler, must be 0 */
  0,                                /* BusFault_Handler, must be 0 */
  0,                                /* UsageFault_Handler, must be 0 */
  LPC_checksum,           /* Checksum, calculated by the linker script or the flash utility */
  0,                                /* Reserved */
  0,                                /* Reserved */
  0,                                /* Reserved */
  0,                                /* SVCall Handler */
  0,                                /* Reserved */
  0,                                /* Reserved */
  0,                                /* PendSV Handler */
  SysTick_Handler,         /* SysTick Handler */            
  
  0,  					/* SPI0 controller */
  0, 					/* SPI1 controller */
  0,					/* Reserved */
  0,					/* UART0 */
  0,					/* UART1 */
  0, 					/* UART2 */
  0,					/* Reserved */
  0,					/* Reserved */
  0,					/* I2C controller */
  0,					/* SCT */
  0,					/* Multi-Rate Timer */
  0,					/* Comparator */
  0,					/* WDT */
  0,					/* BOD Brown Out Detect */
  0,					/* Reserved */
  0,					/* WKT Self wake-up timer */
  0,					/* Reserved */
  0,					/* Reserved */
  0,					/* Reserved */
  0,					/* Reserved */
  0,					/* Reserved */
  0,					/* Reserved */
  0,					/* Reserved */
  0,					/* Reserved */
  0,					/* PIO INT0 */
  0,					/* PIO INT1 */
  0,					/* PIO INT2 */
  0,					/* PIO INT3 */
  0,					/* PIO INT4 */
  0,					/* PIO INT5 */
  0,					/* PIO INT6 */
  0					/* PIO INT7 */                      
};
