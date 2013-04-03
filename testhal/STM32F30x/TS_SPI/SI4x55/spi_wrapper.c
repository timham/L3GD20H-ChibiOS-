/**
 *  Copyright 2008 Silicon Laboratories, Inc.
 *  http://www.silabs.com
 *
 *  @file spi_wrapper.c
 *  
 *  C File Description:
 *  @brief TODO
 *
 *  Project Name: EZRadio Si4x55
 * 
 * 
 *  @author Sz. Papp
 *
 *  @date 2012.08.09.
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
#include "compiler_defs.h"
#include "spi_wrapper.h"
#include "CommIF.h"


/*****************************************************************************
 *  Function definitions
 *****************************************************************************/

/**
 *  SPI Read/Write wrapper function.
 *
 *  @param[in]  data Byte to send.
 *
 *  @return     Byte received.
 *
 *  @note
 *
 */
U8 bSpi_ReadWriteSpi1 (U8 bData)
{
  return Comm_IF_Spi1ReadWrite(bData);
}

/**
 *  SPI Burst write wrapper function.
 *
 *  @param[in] length Byte count to send.
 *
 *  @param[in] pData  Pointer to the data to be sent.
 *
 *  @note
 *
 */
void vSpi_WriteDataSpi1 (U8 length, U8 *pData)
{
  while (length--)
  {
    Comm_IF_Spi1ReadWrite(*pData++);
  }
}

/**
 *  SPI Burst read wrapper function.
 *
 *  @param[in]  length  Byte count to read.
 *
 *  @param[out] pData   Pointer to the data buffer.
 *
 *  @note
 *
 */
void vSpi_ReadDataSpi1  (U8 length, U8 *pData)
{
  while (length--)
  {
    *pData++ = Comm_IF_Spi1ReadWrite(0xFF);
  }
}
