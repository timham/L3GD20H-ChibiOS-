/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012,2013 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.


    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "ch.h"
#include "hal.h"

#include "./l3gd20/stm32f3_discovery_l3gd20.h"

//////////////////////////////////////////////////////////////////////////

SPIDriver	gSpiDriver;
SPIConfig   	gSpiConfig;  

//////////////////////////////////////////////////////////////////////////

#undef   _TEST_LAMP
#undef   TEST_CHIPID

#define   gSpiDriver   SPID1
#define   SLEEP_TIME   80 
 
///////////////////////////////////////////////////////////////////////////
//
// External Function 
//

extern  void L3GD20_Init_Setting(L3GD20_InitTypeDef *L3GD20_InitStruct);

typedef enum  DIR {
	N =   GPIOE_LED3_RED,
	NE =  GPIOE_LED5_ORANGE,
	E =   GPIOE_LED7_GREEN,
	SE =  GPIOE_LED9_BLUE,
	S =   GPIOE_LED10_RED,
	SW =  GPIOE_LED8_ORANGE,
	W =   GPIOE_LED6_GREEN,
	NW = GPIOE_LED4_BLUE
}DIR_t;

bool   assert_check_led(DIR_t pos)
{
	if( (pos =(GPIOE_LED3_RED ||GPIOE_LED4_BLUE || GPIOE_LED5_ORANGE 
		||GPIOE_LED6_GREEN || GPIOE_LED7_GREEN || GPIOE_LED8_ORANGE
		|| GPIOE_LED9_BLUE || GPIOE_LED10_RED)
	     )
	   )
	return  TRUE;
	
	return FALSE;
}

void  searching_led_position(DIR_t  pos)
{
  static uint8_t   cur_pos = 0xff,  prev_pos = 0x00;

  if(assert_check_led(pos) == FALSE)
      return;
	
  cur_pos = pos;

  if(cur_pos != prev_pos) {
  	if(assert_check_led(prev_pos) == TRUE)
        {
  	   palClearPad(GPIOE, prev_pos);
  	   chThdSleepMilliseconds(SLEEP_TIME);
        }
	palSetPad(GPIOE, cur_pos);
	chThdSleepMilliseconds(SLEEP_TIME);
  	prev_pos = cur_pos;
  }
} 

uint8_t   rxBuf[0x38];

uint8_t   OUT_X_L, OUT_X_H, 
	  OUT_Y_L, OUT_Y_H,
	  OUT_Z_L, OUT_Z_H,
	  WHO_AM_I;

int16_t   gX, gY, gZ;
int8_t	  lhx, llx, lly,lhy, llz, lhz;


void  checking_coord(void)
{
#define  EX2
#define  DIV1  2048 //4096
		
#if defined(EX2)
   if( ( (gX < DIV1 * 1) && ( gX >= 0 ))
  	|| (gX >= (DIV1 * -1) && gX <0) )	
	searching_led_position(N);

   else if(gX >=  (DIV1 * (-3)) && gX < (DIV1 * (-1)))
	searching_led_position(NE);

   else if(gX >= ((-5)* DIV1) && gX < (-3)*DIV1)
        searching_led_position(E);

   else if(gX >= ((-7) * DIV1) && gX < (-5)* DIV1)
        searching_led_position(SE);

   else if(  (gX >= (-8 *  DIV1+1) 
	   && gX < (-7 * DIV1) )
	   ||  (gX >= (7 * DIV1) 
	   && gX < (8*DIV1-1) )  )
	searching_led_position(S);

   else if(gX >= 5*DIV1 && gX < 7*DIV1)
        searching_led_position(SW);

   else if(gX >= 3*DIV1 && gX < 5*DIV1)
        searching_led_position(W);

   else if(gX >= 1*DIV1 && gX < 3* DIV1)
        searching_led_position(NW);
#endif

}

///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////       
//      gRegStatus : 
//      if (addr_register == r/w)
//         gRegStatus  |= 0x00000001;
//      else(addr_register == r) {
//	   gRegStatus  &= ~(0x0001 << value);
//
//      - It needs to change register settings wholly
//
//
////////////////////////////////////////////////////////////////////////////////////////////////

const uint32_t	gRegStatus = 0x01FD403F;  // read / write depends from 0x20 address

/////////////////////////////////////////////////////////////////////////////////////////////////
// 
//

static uint8_t  gRegTable[0x18]={
	0x07, // 0x20 register
};


////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
//  
//  Function Name  : void  _l3gd_register_setting(uint8_t ctrl_reg, uint8_t regval)
//  
//  Object 	   : 
//	-  In order to set register address, it could be set, but in case of setting
//         read address it occurs serious result.
//	   Therefore, gRegTable could be written whatever types of results.
//
////////////////////////////////////////////////////////////////////////////////////////////////

void  _l3gd_register_setting(uint8_t ctrl_reg, uint8_t reg_val)
{
 uint8_t  regaddr;

   if(ctrl_reg < 0x20 || ctrl_reg > 0x38)  // In order to avoid wrong address register setting, 
   	    return;

   regaddr = ctrl_reg - 0x20; 	
	
   if( (gRegStatus >> regaddr) & 0x01) {
	gRegTable[regaddr] &= ~reg_val;
	gRegTable[regaddr] |= reg_val;
	L3GD20_Write(&gRegTable[regaddr], &ctrl_reg, 1);
   }
}

static WORKING_AREA(spi_thread_1_wa, 256);
static msg_t spi_thread_1(void *p) {

  (void)p;

  static uint8_t  addr;


  chRegSetThreadName("SPI thread 0");
  
  while (TRUE) {  	
	spiAcquireBus(&SPID1); 		/* Acquire ownership of the bus.	*/

#if defined(_TEST_LAMP)
	palSetPad(GPIOE, NE);
	chThdSleepMilliseconds(SLEEP_TIME);
#endif

	// L3GD20_Init(&SPID1, &gSpiConfig);

	spiStart(&SPID1, &gSpiConfig);     
    	spiSelect(&SPID1);		/* Slave Select assertion.  */	 

	_l3gd_register_setting(0x20, 0x08);  // Pd set.
	_l3gd_register_setting(0x23, 0x00);
	_l3gd_register_setting(0x2E, 0x40);  // stream mode change

	addr = L3GD20_WHO_AM_I_ADDR;  L3GD20_Read(&WHO_AM_I, &addr, 1);
	addr = L3GD20_OUT_X_L_ADDR;   L3GD20_Read(&OUT_X_L, &addr, 1);
	addr = L3GD20_OUT_X_H_ADDR;   L3GD20_Read(&OUT_X_H, &addr, 1);
	addr = L3GD20_OUT_Y_L_ADDR;   L3GD20_Read(&OUT_Y_L, &addr, 1);
	addr = L3GD20_OUT_Y_H_ADDR;   L3GD20_Read(&OUT_Y_H, &addr, 1);
	addr = L3GD20_OUT_Z_L_ADDR;   L3GD20_Read(&OUT_Z_L, &addr, 1);
	addr = L3GD20_OUT_Z_H_ADDR;   L3GD20_Read(&OUT_Z_H, &addr, 1);
	spiUnselect(&SPID1);     /* Slave Select de-assertion.       */

#if   defined(TEST_CHIPID) 
	if(WHO_AM_I == I_AM_L3GD20) {
		palSetPad(GPIOE, N);
	        chThdSleepMilliseconds(SLEEP_TIME);
        }
#endif

	llx = (OUT_X_L); lhx = (OUT_X_H);
	gX = (uint16_t)((lhx << 7) | (llx));
	
	lly = (OUT_Y_L); lhy = (OUT_Y_H);
	gY = (uint16_t)((lhy << 7)  | (lly));
	
	llz = (OUT_Z_L); lhz = (OUT_Z_H);
	gZ = (uint16_t)((lhz << 7) | (llz));

	checking_coord();
	
#if defined(_TEST_LAMP)
	palClearPad(GPIOE, NE);
	chThdSleepMilliseconds(SLEEP_TIME);
#endif	
	spiReleaseBus(&SPID1);    /* Ownership release.               */
  }

  return 0;
}

/*
 * SPI bus contender 1 0r 2.
 */
static WORKING_AREA(spi_thread_2_wa, 256);
static msg_t spi_thread_2(void *p) {

  (void)p;

  chRegSetThreadName("SPI thread 1");

  while (TRUE) 
  {
	spiAcquireBus(&SPID1); 	/* Acquire ownership of the bus.  */

#if defined(_TEST_LAMP)
	palSetPad(GPIOE,NW);
	chThdSleepMilliseconds(SLEEP_TIME);	
#endif

#if defined(_TEST_LAMP)
	palClearPad(GPIOE,NW);
	chThdSleepMilliseconds(SLEEP_TIME);				
#endif

	spiReleaseBus(&SPID1);
  }  
  return 0;
}


/*
 * Application entry point.
 */

int main(void) {
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  	halInit();
  	chSysInit();

	L3GD20_Init(&SPID1, &gSpiConfig);

	//palSetPad(GPIOE, E);
  	/*
   	* Starting the transmitter and receiver threads.
   	*/

  	chThdCreateStatic(spi_thread_1_wa, sizeof(spi_thread_1_wa),
                    NORMALPRIO + 1, spi_thread_1, NULL);


  	chThdCreateStatic(spi_thread_2_wa, sizeof(spi_thread_2_wa),
                    NORMALPRIO + 1, spi_thread_2, NULL);

  	return 0;
}
