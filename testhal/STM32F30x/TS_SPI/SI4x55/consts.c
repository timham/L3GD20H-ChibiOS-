/**
 *  Copyright 2008 Silicon Laboratories, Inc.
 *  http://www.silabs.com
 *
 *  @file pictures.c
 *
 *  C File Description:
 *  @brief Contains all the pictures used in the project.
 *
 *  Project Name: EzRadio2 Laboratory Measurements Software for MCU-LCDBB930
 *
 *  @date   03/20/2012
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

/*****************************************************************************
 *  Global Constant Definitions
 *****************************************************************************/
/*! DCP Hardware Identification String */
const SEGMENT_VARIABLE(Ascii_Info, tAsciiInfo, SEG_CODE) =
{
    "RFSTICK",  /* HW_NAME        */
    "01.1r",    /* HW_VERSION     */
    "01.0r",    /* APP_FW_VERSION */
    "1WD"       /* APP_FW_NAME    */
};
