/**
 *  Copyright 2008 Silicon Laboratories, Inc.
 *  http://www.silabs.com
 *
 *  @file main.c
 *  
 *  C File Description:
 *  @brief TODO
 *
 *  Project Name: EzRadio2 Laboratory Measurements Software
 * 
 *  @author Sz. Papp
 *
 *  @date   02/03/2012
 *
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 *  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 *  DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *  This software must be used in accordance with the End User License Agreement.
 */

/*****************************************************************************
 *  Includes
 *****************************************************************************/
/*! Compiler definitions */
#include "compiler_defs.h"

/*! MCU related */
#include "C8051F930_defs.h"

/*! Hardware related */
#include "hardware_defs.h"

/*! DCP software module includes */
#include "DCPModule/Source/DCPTarget.h"

/*! DCP software module includes */
#include "DCPModule/Source/DCPCore.h"

/*! AES Demo header */
#include "ReceiverDemo.h"

/*! Radio driver includes */
#include "Radio/Si4455/si4455_api_lib.h"
#include "Radio/Si4455/si4455_defs.h"
#include "Radio/radio_comm.h"
#include "Radio/radio_hal.h"


/*****************************************************************************
 *  Local Function Declarations
 *****************************************************************************/
void MCU_Init(void);

/*****************************************************************************
 *  Local Variable Declarations
 *****************************************************************************/
/* LED Blink control variable
 *
 * LEDBlink:
 * x x x x | x x x  b
 * | | | |   | | |  └── TRUE/FALSE: Blink LED1 continuously or not.
 * | | | |   | | |
 * └─┴─┴─┴───┴─┴─┴───── 7 bit number for Identification counter. Each Blink
 *                      cycle decreases its value until 0x01, then stops the
 *                      Identification.
 */
SEGMENT_VARIABLE(LEDBlink, U8, SEG_XDATA) = 1u;

/*****************************************************************************
 *  Function Definitions
 *****************************************************************************/

/**
 *  Main function.
 *
 *  @note
 *
 *  @author   Sz. Papp
 *
 *****************************************************************************/
void main()
{
  /* MCU init */
  MCU_Init();

  /* DCP module init */
  DCP_Init();

  /* Initialize AES encoder/decoder */
  ReceiverDemo_Init();

  for(;;)
  {
    /* DCP Module Poll-Handler */
    DCP_Pollhandler();

    /* AES Demo Poll-Handler */
    ReceiverDemo_Pollhandler();
  }
}

/**
 *  MCU initialization function. <br>
 *  \li Disable WatchDog
 *  \li Start internal precision 24.5Mhz oscillator
 *  \li Set Ports input/output
 *  \li Set Crossbar
 *  \li Turn off LEDs
 *  \li Enable UART
 *  \li Set SPI1 peripheral
 *
 *  @return   None.
 *
 *  @note
 *
 *  @author   Sz. Papp
 *
 *****************************************************************************/
void MCU_Init(void)
{
  U32 wDelay = 0x7FFFF;

  /* disable F930 watchdog */
  PCA0MD   &= ~0x40;

  /* Init Internal Precision Oscillator (24.5MHz) */
  FLSCL     = 0x40;
  OSCICN    = 0x8F;
  CLKSEL    = 0x00;

  /* Port IN/OUT init */
  P0MDOUT   = 0x10;
  P1MDOUT   = 0xED;
  P2MDOUT   = 0x1F;

  /* SMBus shift to P0.6-7 */
  P0SKIP    = 0x0F;

  /* Enable Crossbar */
  XBR2      = 0x40;

  /* latch all inputs to '1' */
  P0        = ~P0MDOUT;
  P1        = ~P1MDOUT;
  P2        = ~P2MDOUT;

  /* Avoid buzzer flick on startup */
  BUZZ      = TRUE;

  /* set all output to its default state */
  LED1      = EXTINGUISH;
  LED2      = EXTINGUISH;
  LED3      = EXTINGUISH;
  LED4      = ILLUMINATE;

  /* SPI1 Config */
  SPI1CFG   = 0x40;     /* Master mode enable */
  SPI1CN    = 0x00;
  SPI1CKR   = 0x0A;

  /* UART must be enabled, cannot be disabled */
  Comm_IF_EnableUART();

  /* SPI enabled by default, can be disabled  */
  Comm_IF_Spi1Enable();

  /* De-select radio & LCD */
  Comm_IF_SpiSetNsel(RF_NSELECT);

  for (; --wDelay; )  ;

  /* Enable ITs */
  EXIT_CRITICAL_SECTION;
}

/* Disable WATCHDOG if using SDCC to get rid of the never-ending WDT reset during startup */
#ifdef SDCC
U8 _sdcc_external_startup (void)
{
   PCA0MD &= ~0x40;                       // Disable Watchdog timer

   return 0u;
}
#endif
