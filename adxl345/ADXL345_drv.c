/***************************************************************************//**
 *   @file   ADXL345.c
 *   @brief  Implementation of ADXL345 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: $WCREV$
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "ADXL345_drv.h"		// ADXL345 definitions.
//#include "Communication.h"	// Communication definitions.

/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/
s8 communicationType = 0;
s8 selectedRange     = 0;
s8 fullResolutionSet = 0;

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief Reads the value of a register.
 *
 * @param registerAddress - Address of the register.
 *
 * @return registerValue  - Value of the register.
*******************************************************************************/
u8 ADXL345_DRV_GetRegisterValue(u8 registerAddress)
{
    u8 dataBuffer[2] = {0, 0};
    u8 registerValue = 0;
    
    if(communicationType == ADXL345_DRV_SPI_COMM)
    {
        dataBuffer[0] = ADXL345_DRV_SPI_READ | registerAddress;
        dataBuffer[1] = 0;
        SPI_Read(ADXL345_DRV_SLAVE_ID, dataBuffer, 2);
        registerValue = dataBuffer[1];
    }
    else
    {
        I2C_Write(ADXL345_DRV_ADDRESS,  // Address of the slave device.
                  &registerAddress, // Transmission data.
                  1,                // Number of bytes to write.
                  0);               // Stop condition control.
        I2C_Read(ADXL345_DRV_ADDRESS,   // Address of the slave device.
                 &registerValue,    // Received data.
                 1,                 // Number of bytes to read.
                 1);                // Stop condition control.
    }

    return registerValue;
}

/***************************************************************************//**
 * @brief Writes data into a register.
 *
 * @param registerAddress - Address of the register.
 * @param registerValue   - Data value to write.
 *
 * @return None.
*******************************************************************************/
void ADXL345_DRV_SetRegisterValue(u8 registerAddress,
                              u8 registerValue)
{
    u8 dataBuffer[2] = {0, 0};

    if(communicationType == ADXL345_DRV_SPI_COMM)
    {
        dataBuffer[0] = ADXL345_DRV_SPI_WRITE | registerAddress;
        dataBuffer[1] = registerValue;
        SPI_Write(ADXL345_DRV_SLAVE_ID, dataBuffer, 2);
    }
    else
    {
	dataBuffer[0] = registerAddress;
	dataBuffer[1] = registerValue;
	I2C_Write(ADXL345_DRV_ADDRESS, dataBuffer, 2, 1);
    }
}

/***************************************************************************//**
 * @brief Initializes the communication peripheral and checks if the ADXL345
 *		  part is present.
 *
 * @param commProtocol - SPI or I2C protocol.
 *                       Example: ADXL345_DRV_SPI_COMM 
 *                                ADXL345_DRV_I2C_COMM 
 *
 * @return status      - Result of the initialization procedure.
 *                       Example: -1 - I2C/SPI peripheral was not initialized or
 *                                     ADXL345 part is not present.
 *                                 0 - I2C/SPI peripheral is initialized and 
 *                                     ADXL345 part is present.
*******************************************************************************/
s8 ADXL345_DRV_Init(s8 commProtocol)
{
    u8 status = 0;

    communicationType = commProtocol;
    if(commProtocol == ADXL345_DRV_SPI_COMM)
    {
        status = SPI_Init(0,        // Transfer format.
                          1000000,  // SPI clock frequency.
                          1,        // SPI clock polarity.
                          0);       // SPI clock edge.
    }
    else
    {
        status = I2C_Init(100000);  // I2C clock frequency.
    }
    if(ADXL345_DRV_GetRegisterValue(ADXL345_DRV_DEVID) != ADXL345_DRV_ID)
    {
        status = -1;
    }
    selectedRange = 2; // Measurement Range: +/- 2g (reset default).
    fullResolutionSet = 0;

    return status;
}

/***************************************************************************//**
 * @brief Places the device into standby/measure mode.
 *
 * @param pwrMode - Power mode.
 *			Example: 0x0 - standby mode.
 *				 0x1 - measure mode.
 *
 * @return None.
*******************************************************************************/
void ADXL345_DRV_SetPowerMode(u8 pwrMode)
{
    u8 oldPowerCtl = 0;
    u8 newPowerCtl = 0;
    
    oldPowerCtl = ADXL345_DRV_GetRegisterValue(ADXL345_DRV_POWER_CTL);
    newPowerCtl = oldPowerCtl & ~ADXL345_DRV_PCTL_MEASURE;
    newPowerCtl = newPowerCtl | (pwrMode * ADXL345_DRV_PCTL_MEASURE);
    ADXL345_DRV_SetRegisterValue(ADXL345_DRV_POWER_CTL, newPowerCtl);
}

/***************************************************************************//**
 * @brief Reads the raw output data of each axis.
 *
 * @param x - X-axis's output data.
 * @param y - Y-axis's output data.
 * @param z - Z-axis's output data.
 *
 * @return None.
*******************************************************************************/
void ADXL345_DRV_GetXyz(s16* x,
		    s16* y,
		    s16* z)
{
    u8 firstRegAddress = ADXL345_DRV_DATAX0;
    u8 readBuffer[7]   = {0, 0, 0, 0, 0, 0, 0};
    
    if(communicationType == ADXL345_DRV_SPI_COMM)
    {
        readBuffer[0] = ADXL345_DRV_SPI_READ |
                        ADXL345_DRV_SPI_MB |
                        firstRegAddress;
        SPI_Read(ADXL345_DRV_SLAVE_ID, readBuffer, 7);
        /* x = ((ADXL345_DRV_DATAX1) << 8) + ADXL345_DRV_DATAX0 */
        *x = ((s16)readBuffer[2] << 8) + readBuffer[1];
        /* y = ((ADXL345_DRV_DATAY1) << 8) + ADXL345_DRV_DATAY0 */
        *y = ((s16)readBuffer[4] << 8) + readBuffer[3];
        /* z = ((ADXL345_DRV_DATAZ1) << 8) + ADXL345_DRV_DATAZ0 */
        *z = ((s16)readBuffer[6] << 8) + readBuffer[5];
    }
    else
    {
        I2C_Write(ADXL345_DRV_ADDRESS,  // Adress of the slave device.
                  &firstRegAddress, // Transmission data.
                  1,                // Number of bytes to write.
                  0);               // Stop condition control.
        I2C_Read(ADXL345_DRV_ADDRESS,   // Adress of the slave device.
                 readBuffer,        // Received data.
                 6,                 // Number of bytes to read.
                 1);                // Stop condition control.
        /* x = ((ADXL345_DRV_DATAX1) << 8) + ADXL345_DRV_DATAX0 */
        *x = ((s16)readBuffer[1] << 8) + readBuffer[0];       
        /* y = ((ADXL345_DRV_DATAY1) << 8) + ADXL345_DRV_DATAY0 */
        *y = ((s16)readBuffer[3] << 8) + readBuffer[2];
        /* z = ((ADXL345_DRV_DATAZ1) << 8) + ADXL345_DRV_DATAZ0 */
        *z = ((s16)readBuffer[5] << 8) + readBuffer[4];
    }
}

/***************************************************************************//**
 * @brief Reads the raw output data of each axis and converts it to g.
 *
 * @param x - X-axis's output data.
 * @param y - Y-axis's output data.
 * @param z - Z-axis's output data.
 *
 * @return None.
*******************************************************************************/
void ADXL345_DRV_GetGxyz(float* x,
		     float* y,
		     float* z)
{
    s16 xData = 0;  // X-axis's output data.
    s16 yData = 0;  // Y-axis's output data.
    s16 zData = 0;  // Z-axis's output data.
 
    ADXL345_DRV_GetXyz(&xData, &yData, &zData);
    *x = (float)(fullResolutionSet ? (xData * ADXL345_DRV_SCALE_FACTOR) :
            (xData * ADXL345_DRV_SCALE_FACTOR * (selectedRange >> 1)));
    *y = (float)(fullResolutionSet ? (yData * ADXL345_DRV_SCALE_FACTOR) :
            (yData * ADXL345_DRV_SCALE_FACTOR * (selectedRange >> 1)));
    *z = (float)(fullResolutionSet ? (zData * ADXL345_DRV_SCALE_FACTOR) :
            (zData * ADXL345_DRV_SCALE_FACTOR * (selectedRange >> 1)));
}

/***************************************************************************//**
 * @brief Enables/disables the tap detection.
 *
 * @param tapType   - Tap type (none, single, double).
 *			Example: 0x0 - disables tap detection.	
 *				ADXL345_DRV_SINGLE_TAP - enables single tap 
 *                                                   detection.
 *				ADXL345_DRV_DOUBLE_TAP - enables double tap 
 *                                                   detection.
 * @param tapAxes   - Axes which participate in tap detection.
 *			Example: 0x0 - disables axes participation.
 *				ADXL345_DRV_TAP_X_EN - enables x-axis participation.
 *				ADXL345_DRV_TAP_Y_EN - enables y-axis participation.
 *				ADXL345_DRV_TAP_Z_EN - enables z-axis participation.
 * @param tapDur    - Tap duration. The scale factor is 625us is/LSB.
 * @param tapLatent - Tap latency. The scale factor is 1.25 ms/LSB.
 * @param tapWindow - Tap window. The scale factor is 1.25 ms/LSB.
 * @param tapThresh - Tap threshold. The scale factor is 62.5 mg/LSB.
 * @param tapInt    - Interrupts pin.
 *			Example: 0x0 - interrupts on INT1 pin.
 *				ADXL345_DRV_SINGLE_TAP - single tap interrupts on 
 *						     INT2 pin.
 *				ADXL345_DRV_DOUBLE_TAP - double tap interrupts on
 *						     INT2 pin.
 *
 * @return None.
*******************************************************************************/
void ADXL345_DRV_SetTapDetection(u8 tapType,
                             u8 tapAxes,
                             u8 tapDur,
                             u8 tapLatent,
                             u8 tapWindow,
                             u8 tapThresh,
                             u8 tapInt)
{
    u8 oldTapAxes   = 0;
    u8 newTapAxes   = 0;
    u8 oldIntMap    = 0;
    u8 newIntMap    = 0;
    u8 oldIntEnable = 0;
    u8 newIntEnable = 0;
    
    oldTapAxes = ADXL345_DRV_GetRegisterValue(ADXL345_DRV_TAP_AXES);
    newTapAxes = oldTapAxes & ~(ADXL345_DRV_TAP_X_EN |
                                ADXL345_DRV_TAP_Y_EN |
                                ADXL345_DRV_TAP_Z_EN);
    newTapAxes = newTapAxes | tapAxes;
    ADXL345_DRV_SetRegisterValue(ADXL345_DRV_TAP_AXES, newTapAxes);
    ADXL345_DRV_SetRegisterValue(ADXL345_DRV_DUR, tapDur);
    ADXL345_DRV_SetRegisterValue(ADXL345_DRV_LATENT, tapLatent);
    ADXL345_DRV_SetRegisterValue(ADXL345_DRV_WINDOW, tapWindow);
    ADXL345_DRV_SetRegisterValue(ADXL345_DRV_THRESH_TAP, tapThresh);
    oldIntMap = ADXL345_DRV_GetRegisterValue(ADXL345_DRV_INT_MAP);
    newIntMap = oldIntMap & ~(ADXL345_DRV_SINGLE_TAP | ADXL345_DRV_DOUBLE_TAP);
    newIntMap = newIntMap | tapInt;
    ADXL345_DRV_SetRegisterValue(ADXL345_DRV_INT_MAP, newIntMap);
    oldIntEnable = ADXL345_DRV_GetRegisterValue(ADXL345_DRV_INT_ENABLE);
    newIntEnable = oldIntEnable & ~(ADXL345_DRV_SINGLE_TAP | ADXL345_DRV_DOUBLE_TAP);
    newIntEnable = newIntEnable | tapType;
    ADXL345_DRV_SetRegisterValue(ADXL345_DRV_INT_ENABLE, newIntEnable);
}

/***************************************************************************//**
 * @brief Enables/disables the activity detection.
 *
 * @param actOnOff  - Enables/disables the activity detection.
 *				Example: 0x0 - disables the activity detection.
 *					 0x1 - enables the activity detection.
 * @param actAxes   - Axes which participate in detecting activity.
 *			Example: 0x0 - disables axes participation.
 *				ADXL345_DRV_ACT_X_EN - enables x-axis participation.
 *				ADXL345_DRV_ACT_Y_EN - enables y-axis participation.
 *				ADXL345_DRV_ACT_Z_EN - enables z-axis participation.
 * @param actAcDc   - Selects dc-coupled or ac-coupled operation.
 *			Example: 0x0 - dc-coupled operation.
 *				ADXL345_DRV_ACT_ACDC - ac-coupled operation.
 * @param actThresh - Threshold value for detecting activity. The scale factor 
                      is 62.5 mg/LSB.
 * @patam actInt    - Interrupts pin.
 *			Example: 0x0 - activity interrupts on INT1 pin.
 *				ADXL345_DRV_ACTIVITY - activity interrupts on INT2 
 *                                                 pin.
 * @return None.
*******************************************************************************/
void ADXL345_DRV_SetActivityDetection(u8 actOnOff,
                                  u8 actAxes,
                                  u8 actAcDc,
                                  u8 actThresh,
                                  u8 actInt)
{
    u8 oldActInactCtl = 0;
    u8 newActInactCtl = 0;
    u8 oldIntMap      = 0;
    u8 newIntMap      = 0;
    u8 oldIntEnable   = 0;
    u8 newIntEnable   = 0;
    
    oldActInactCtl = ADXL345_DRV_GetRegisterValue(ADXL345_DRV_INT_ENABLE);
    newActInactCtl = oldActInactCtl & ~(ADXL345_DRV_ACT_ACDC |
                                        ADXL345_DRV_ACT_X_EN |
                                        ADXL345_DRV_ACT_Y_EN |
                                        ADXL345_DRV_ACT_Z_EN);
    newActInactCtl = newActInactCtl | (actAcDc | actAxes);
    ADXL345_DRV_SetRegisterValue(ADXL345_DRV_ACT_INACT_CTL, newActInactCtl);
    ADXL345_DRV_SetRegisterValue(ADXL345_DRV_THRESH_ACT, actThresh);
    oldIntMap = ADXL345_DRV_GetRegisterValue(ADXL345_DRV_INT_MAP);
    newIntMap = oldIntMap & ~(ADXL345_DRV_ACTIVITY);
    newIntMap = newIntMap | actInt;
    ADXL345_DRV_SetRegisterValue(ADXL345_DRV_INT_MAP, newIntMap);
    oldIntEnable = ADXL345_DRV_GetRegisterValue(ADXL345_DRV_INT_ENABLE);
    newIntEnable = oldIntEnable & ~(ADXL345_DRV_ACTIVITY);
    newIntEnable = newIntEnable | (ADXL345_DRV_ACTIVITY * actOnOff);
    ADXL345_DRV_SetRegisterValue(ADXL345_DRV_INT_ENABLE, newIntEnable);
}

/***************************************************************************//**
 * @brief Enables/disables the inactivity detection.
 *
 * @param inactOnOff  - Enables/disables the inactivity detection.
 *			  Example: 0x0 - disables the inactivity detection.
 *				   0x1 - enables the inactivity detection.
 * @param inactAxes   - Axes which participate in detecting inactivity.
 *			  Example: 0x0 - disables axes participation.
 *				ADXL345_DRV_INACT_X_EN - enables x-axis.
 *				ADXL345_DRV_INACT_Y_EN - enables y-axis.
 *				ADXL345_DRV_INACT_Z_EN - enables z-axis.
 * @param inactAcDc   - Selects dc-coupled or ac-coupled operation.
 *			  Example: 0x0 - dc-coupled operation.
 *				ADXL345_DRV_INACT_ACDC - ac-coupled operation.
 * @param inactThresh - Threshold value for detecting inactivity. The scale 
                        factor is 62.5 mg/LSB.
 * @param inactTime   - Inactivity time. The scale factor is 1 sec/LSB.
 * @patam inactInt    - Interrupts pin.
 *		          Example: 0x0 - inactivity interrupts on INT1 pin.
 *				ADXL345_DRV_INACTIVITY - inactivity interrupts on
 *						     INT2 pin.
 *
 * @return None.
*******************************************************************************/
void ADXL345_DRV_SetInactivityDetection(u8 inactOnOff,
                                    u8 inactAxes,
                                    u8 inactAcDc,
                                    u8 inactThresh,
                                    u8 inactTime,
                                    u8 inactInt)
{
    u8 oldActInactCtl = 0;
    u8 newActInactCtl = 0;
    u8 oldIntMap      = 0;
    u8 newIntMap      = 0;
    u8 oldIntEnable   = 0;
    u8 newIntEnable   = 0;
    
    oldActInactCtl = ADXL345_DRV_GetRegisterValue(ADXL345_DRV_INT_ENABLE);
    newActInactCtl = oldActInactCtl & ~(ADXL345_DRV_INACT_ACDC |
                                        ADXL345_DRV_INACT_X_EN |
                                        ADXL345_DRV_INACT_Y_EN |
                                        ADXL345_DRV_INACT_Z_EN);
    newActInactCtl = newActInactCtl | (inactAcDc | inactAxes);
    ADXL345_DRV_SetRegisterValue(ADXL345_DRV_ACT_INACT_CTL, newActInactCtl);
    ADXL345_DRV_SetRegisterValue(ADXL345_DRV_THRESH_INACT, inactThresh);
    ADXL345_DRV_SetRegisterValue(ADXL345_DRV_TIME_INACT, inactTime);
    oldIntMap = ADXL345_DRV_GetRegisterValue(ADXL345_DRV_INT_MAP);
    newIntMap = oldIntMap & ~(ADXL345_DRV_INACTIVITY);
    newIntMap = newIntMap | inactInt;
    ADXL345_DRV_SetRegisterValue(ADXL345_DRV_INT_MAP, newIntMap);
    oldIntEnable = ADXL345_DRV_GetRegisterValue(ADXL345_DRV_INT_ENABLE);
    newIntEnable = oldIntEnable & ~(ADXL345_DRV_INACTIVITY);
    newIntEnable = newIntEnable | (ADXL345_DRV_INACTIVITY * inactOnOff);
    ADXL345_DRV_SetRegisterValue(ADXL345_DRV_INT_ENABLE, newIntEnable);
}

/***************************************************************************//**
 * @brief Enables/disables the free-fall detection.
 *
 * @param ffOnOff  - Enables/disables the free-fall detection.
 *			Example: 0x0 - disables the free-fall detection.
 *				 0x1 - enables the free-fall detection.
 * @param ffThresh - Threshold value for free-fall detection. The scale factor 
                     is 62.5 mg/LSB.
 * @param ffTime   - Time value for free-fall detection. The scale factor is 
                     5 ms/LSB.
 * @param ffInt    - Interrupts pin.
 *		        Example: 0x0 - free-fall interrupts on INT1 pin.
 *			         ADXL345_DRV_FREE_FALL - free-fall interrupts on 
 *                                                   INT2 pin.   
 * @return None.
*******************************************************************************/
void ADXL345_DRV_SetFreeFallDetection(u8 ffOnOff,
                                  u8 ffThresh,
                                  u8 ffTime,
                                  u8 ffInt)
{
    u8 oldIntMap    = 0;
    u8 newIntMap    = 0;
    u8 oldIntEnable = 0;
    u8 newIntEnable = 0;
    
    ADXL345_DRV_SetRegisterValue(ADXL345_DRV_THRESH_FF, ffThresh);
    ADXL345_DRV_SetRegisterValue(ADXL345_DRV_TIME_FF, ffTime);
    oldIntMap = ADXL345_DRV_GetRegisterValue(ADXL345_DRV_INT_MAP);
    newIntMap = oldIntMap & ~(ADXL345_DRV_FREE_FALL);
    newIntMap = newIntMap | ffInt;
    ADXL345_DRV_SetRegisterValue(ADXL345_DRV_INT_MAP, newIntMap);
    oldIntEnable = ADXL345_DRV_GetRegisterValue(ADXL345_DRV_INT_ENABLE);
    newIntEnable = oldIntEnable & ~ADXL345_DRV_FREE_FALL;
    newIntEnable = newIntEnable | (ADXL345_DRV_FREE_FALL * ffOnOff);
    ADXL345_DRV_SetRegisterValue(ADXL345_DRV_INT_ENABLE, newIntEnable);	
}

/***************************************************************************//**
 * @brief Sets an offset value for each axis (Offset Calibration).
 *
 * @param xOffset - X-axis's offset.
 * @param yOffset - Y-axis's offset.
 * @param zOffset - Z-axis's offset.
 *
 * @return None.
*******************************************************************************/
void ADXL345_DRV_SetOffset(u8 xOffset,
                       u8 yOffset,
                       u8 zOffset)
{
    ADXL345_DRV_SetRegisterValue(ADXL345_DRV_OFSX, xOffset);
    ADXL345_DRV_SetRegisterValue(ADXL345_DRV_OFSY, yOffset);
    ADXL345_DRV_SetRegisterValue(ADXL345_DRV_OFSZ, yOffset);
}

/***************************************************************************//**
 * @brief Selects the measurement range.
 *
 * @param gRange  - Range option.
 *                  Example: ADXL345_DRV_RANGE_PM_2G  - +-2 g
 *                           ADXL345_DRV_RANGE_PM_4G  - +-4 g
 *                           ADXL345_DRV_RANGE_PM_8G  - +-8 g
 *                           ADXL345_DRV_RANGE_PM_16G - +-16 g
 * @param fullRes - Full resolution option.
 *                   Example: 0x0 - Disables full resolution.
 *                            ADXL345_DRV_FULL_RES - Enables full resolution.
 *
 * @return None.
*******************************************************************************/
void ADXL345_DRV_SetRangeResolution(u8 gRange, u8 fullRes)
{
    u8 oldDataFormat = 0;
    u8 newDataFormat = 0;
    
    oldDataFormat = ADXL345_DRV_GetRegisterValue(ADXL345_DRV_DATA_FORMAT);
    newDataFormat = oldDataFormat & ~(ADXL345_DRV_RANGE(0x3) | ADXL345_DRV_FULL_RES);
    newDataFormat =  newDataFormat | ADXL345_DRV_RANGE(gRange) | fullRes;
    ADXL345_DRV_SetRegisterValue(ADXL345_DRV_DATA_FORMAT, newDataFormat);
    selectedRange = (1 << (gRange + 1));
    fullResolutionSet = fullRes ? 1 : 0;
}
