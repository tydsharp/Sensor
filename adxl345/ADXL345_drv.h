/***************************************************************************//**
 *   @file   ADXL345.h
 *   @brief  Header file of ADXL345 Driver.
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
#ifndef __ADXL345_DRV_H__
#define __ADXL345_DRV_H__

#include "ADXL345_types.h"

/******************************************************************************/
/******************************** ADXL345 *************************************/
/******************************************************************************/

/* Options for communicating with the device. */
#define ADXL345_DRV_SPI_COMM       0
#define ADXL345_DRV_I2C_COMM       1

/* SPI slave device ID */
#define ADXL345_DRV_SLAVE_ID       1

/* I2C address of the device */
#define ADXL345_DRV_ADDRESS			0x1D

/* SPI commands */
#define ADXL345_DRV_SPI_READ        (1 << 7)
#define ADXL345_DRV_SPI_WRITE       (0 << 7)
#define ADXL345_DRV_SPI_MB          (1 << 6)

/* ADXL345 Register Map */
#define	ADXL345_DRV_DEVID           0x00 // R   Device ID.
#define ADXL345_DRV_THRESH_TAP      0x1D // R/W Tap threshold.
#define ADXL345_DRV_OFSX            0x1E // R/W X-axis offset.
#define ADXL345_DRV_OFSY            0x1F // R/W Y-axis offset.
#define ADXL345_DRV_OFSZ            0x20 // R/W Z-axis offset.
#define ADXL345_DRV_DUR             0x21 // R/W Tap duration.
#define ADXL345_DRV_LATENT          0x22 // R/W Tap latency.
#define ADXL345_DRV_WINDOW          0x23 // R/W Tap window.
#define ADXL345_DRV_THRESH_ACT      0x24 // R/W Activity threshold.
#define ADXL345_DRV_THRESH_INACT    0x25 // R/W Inactivity threshold.
#define ADXL345_DRV_TIME_INACT      0x26 // R/W Inactivity time.
#define ADXL345_DRV_ACT_INACT_CTL   0x27 // R/W Axis enable control for activity
                                     // and inactivity detection.
#define ADXL345_DRV_THRESH_FF       0x28 // R/W Free-fall threshold.
#define ADXL345_DRV_TIME_FF         0x29 // R/W Free-fall time.
#define ADXL345_DRV_TAP_AXES        0x2A // R/W Axis control for tap/double tap.
#define ADXL345_DRV_ACT_TAP_STATUS  0x2B // R   Source of tap/double tap.
#define ADXL345_DRV_BW_RATE         0x2C // R/W Data rate and power mode control.
#define ADXL345_DRV_POWER_CTL       0x2D // R/W Power saving features control.
#define ADXL345_DRV_INT_ENABLE      0x2E // R/W Interrupt enable control.
#define ADXL345_DRV_INT_MAP         0x2F // R/W Interrupt mapping control.
#define ADXL345_DRV_INT_SOURCE      0x30 // R   Source of interrupts.
#define ADXL345_DRV_DATA_FORMAT     0x31 // R/W Data format control.
#define ADXL345_DRV_DATAX0          0x32 // R   X-Axis Data 0.
#define ADXL345_DRV_DATAX1          0x33 // R   X-Axis Data 1.
#define ADXL345_DRV_DATAY0          0x34 // R   Y-Axis Data 0.
#define ADXL345_DRV_DATAY1          0x35 // R   Y-Axis Data 1.
#define ADXL345_DRV_DATAZ0          0x36 // R   Z-Axis Data 0.
#define ADXL345_DRV_DATAZ1          0x37 // R   Z-Axis Data 1.
#define ADXL345_DRV_FIFO_CTL        0x38 // R/W FIFO control.
#define ADXL345_DRV_FIFO_STATUS     0x39 // R   FIFO status.

/* ADXL345_DRV_ACT_INACT_CTL definition */
#define ADXL345_DRV_ACT_ACDC        (1 << 7)
#define ADXL345_DRV_ACT_X_EN        (1 << 6)
#define ADXL345_DRV_ACT_Y_EN        (1 << 5)
#define ADXL345_DRV_ACT_Z_EN        (1 << 4)
#define ADXL345_DRV_INACT_ACDC      (1 << 3)
#define ADXL345_DRV_INACT_X_EN      (1 << 2)
#define ADXL345_DRV_INACT_Y_EN      (1 << 1)
#define ADXL345_DRV_INACT_Z_EN      (1 << 0)

/* ADXL345_DRV_TAP_AXES definition */
#define ADXL345_DRV_SUPPRESS        (1 << 3)
#define ADXL345_DRV_TAP_X_EN        (1 << 2)
#define ADXL345_DRV_TAP_Y_EN        (1 << 1)
#define ADXL345_DRV_TAP_Z_EN        (1 << 0)

/* ADXL345_DRV_ACT_TAP_STATUS definition */
#define ADXL345_DRV_ACT_X_SRC       (1 << 6)
#define ADXL345_DRV_ACT_Y_SRC       (1 << 5)
#define ADXL345_DRV_ACT_Z_SRC       (1 << 4)
#define ADXL345_DRV_ASLEEP          (1 << 3)
#define ADXL345_DRV_TAP_X_SRC       (1 << 2)
#define ADXL345_DRV_TAP_Y_SRC       (1 << 1)
#define ADXL345_DRV_TAP_Z_SRC       (1 << 0)

/* ADXL345_DRV_BW_RATE definition */
#define ADXL345_DRV_LOW_POWER       (1 << 4)
#define ADXL345_DRV_RATE(x)         ((x) & 0xF)

/* ADXL345_DRV_POWER_CTL definition */
#define ADXL345_DRV_PCTL_LINK       (1 << 5)
#define ADXL345_DRV_PCTL_AUTO_SLEEP (1 << 4)
#define ADXL345_DRV_PCTL_MEASURE    (1 << 3)
#define ADXL345_DRV_PCTL_SLEEP      (1 << 2)
#define ADXL345_DRV_PCTL_WAKEUP(x)  ((x) & 0x3)

/* ADXL345_DRV_INT_ENABLE / ADXL345_DRV_INT_MAP / ADXL345_DRV_INT_SOURCE definition */
#define ADXL345_DRV_DATA_READY      (1 << 7)
#define ADXL345_DRV_SINGLE_TAP      (1 << 6)
#define ADXL345_DRV_DOUBLE_TAP      (1 << 5)
#define ADXL345_DRV_ACTIVITY        (1 << 4)
#define ADXL345_DRV_INACTIVITY      (1 << 3)
#define ADXL345_DRV_FREE_FALL       (1 << 2)
#define ADXL345_DRV_WATERMARK       (1 << 1)
#define ADXL345_DRV_OVERRUN         (1 << 0)

/* ADXL345_DRV_DATA_FORMAT definition */
#define ADXL345_DRV_SELF_TEST       (1 << 7)
#define ADXL345_DRV_SPI             (1 << 6)
#define ADXL345_DRV_INT_INVERT      (1 << 5)
#define ADXL345_DRV_FULL_RES        (1 << 3)
#define ADXL345_DRV_JUSTIFY         (1 << 2)
#define ADXL345_DRV_RANGE(x)        ((x) & 0x3)

/* ADXL345_DRV_RANGE(x) options */
#define ADXL345_DRV_RANGE_PM_2G     0
#define ADXL345_DRV_RANGE_PM_4G     1
#define ADXL345_DRV_RANGE_PM_8G     2
#define ADXL345_DRV_RANGE_PM_16G    3

/* ADXL345_DRV_FIFO_CTL definition */
#define ADXL345_DRV_FIFO_MODE(x)    (((x) & 0x3) << 6)
#define ADXL345_DRV_TRIGGER         (1 << 5)
#define ADXL345_DRV_SAMPLES(x)      ((x) & 0x1F)

/* ADXL345_DRV_FIFO_MODE(x) options */
#define ADXL345_DRV_FIFO_BYPASS     0
#define ADXL345_DRV_FIFO_FIFO       1
#define ADXL345_DRV_FIFO_STREAM     2
#define ADXL345_DRV_FIFO_TRIGGER    3

/* ADXL345_DRV_FIFO_STATUS definition */
#define ADXL345_DRV_FIFO_TRIG       (1 << 7)
#define ADXL345_DRV_ENTRIES(x)      ((x) & 0x3F)

/* ADXL345 ID */
#define ADXL345_DRV_ID              0xE5

/* ADXL345 Full Resolution Scale Factor */
#define ADXL345_DRV_SCALE_FACTOR    0.0039 

typedef struct {
	s32 (*write)(u16 dev_addr,u8 reg_addr,u8* buf,u16 len);
	s32 (*read)(u16 dev_addr,u8 reg_addr,u8* buf,u16 len);
}adxl345_drv_t;

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/*! Reads the value of a register. */
u8 ADXL345_DRV_GetRegisterValue(u8 registerAddress);

/*! Writes data into a register. */
void ADXL345_DRV_SetRegisterValue(u8 registerAddress,
			      u8 registerValue);

/*! Init. the comm. peripheral and checks if the ADXL345 part is present. */
s8 ADXL345_DRV_Init(s8 commProtocol);

/*! Places the device into standby/measure mode. */
void ADXL345_DRV_SetPowerMode(u8 pwrMode);

/*! Reads the raw output data of each axis. */
void ADXL345_DRV_GetXyz(s16* x,
		    s16* y,
		    s16* z);

/*! Reads the raw output data of each axis and converts it to g. */
void ADXL345_DRV_GetGxyz(float* x,
		     float* y,
		     float* z);

/*! Enables/disables the tap detection. */
void ADXL345_DRV_SetTapDetection(u8 tapType,
                             u8 tapAxes,
                             u8 tapDur,
                             u8 tapLatent,
                             u8 tapWindow,
                             u8 tapThresh,
                             u8 tapInt);

/*! Enables/disables the activity detection. */
void ADXL345_DRV_SetActivityDetection(u8 actOnOff,
				  u8 actAxes,
				  u8 actAcDc,
				  u8 actThresh,
				  u8 actInt);

/*! Enables/disables the inactivity detection. */
void ADXL345_DRV_SetInactivityDetection(u8 inactOnOff,
				    u8 inactAxes,
				    u8 inactAcDc,
				    u8 inactThresh,
				    u8 inactTime,
				    u8 inactInt);

/*! Enables/disables the free-fall detection. */
void ADXL345_DRV_SetFreeFallDetection(u8 ffOnOff,
				  u8 ffThresh,
				  u8 ffTime,
				  u8 ffInt);

/*! Sets an offset value for each axis (Offset Calibration). */
void ADXL345_DRV_SetOffset(u8 xOffset,
		       u8 yOffset,
		       u8 zOffset);

/*! Selects the measurement range. */
void ADXL345_DRV_SetRangeResolution(u8 gRange, u8 fullRes);

#endif	/* __ADXL345_DRV_H__ */
