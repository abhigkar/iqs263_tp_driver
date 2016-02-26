/*
 *	iqs263.h - IQS263 Registers and Bit Definitions
 *
 *	Copyright (C) 2016 iWave Ltd
 *	Author: Hai Peng Yu <yuhp@iwave.cc>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *	iWave Ltd does not take responsibility for the use of this driver.
 *
 *	This header file contains the register and bit definitions for the
 *	IQS263 SAR Sensor, to be used in the driver. This makes the code
 *	cleaner to read.
 */
#ifndef IQS263_H
#define IQS263_H

/*	i2c slave device address	*/
#define IQS263_ADDR	0x44

/*	Definitions for the driver	*/
#define IQS_MAJOR				248
#define IQS_MINOR				0
#define IQS_DEV_NUMS			1
#define DEVICE_NAME             "iqs263"
#define IQS263_DRIVER_NAME      "iqs263_driver"

/*	Addresses of Registers on IQS263	*/
/*	Read	*/
#define DEVICE_INFO             0x00
#define SYS_FLAGS               0x01
#define COORDINATES             0x02
#define TOUCH_BYTES             0x03
#define COUNTS                  0x04
#define LTA                     0x05
#define DELTAS                  0x06
#define MULTIPLIERS             0x07
#define COMPENSATION            0x08
#define PROX_SETTINGS           0x09
#define THRESHOLDS              0x0A
#define TIMINGS_AND_TARGETS     0x0B
#define GESTURE_TIMERS          0x0C
#define ACTIVE_CHANNELS         0x0D
#define PROX_SETTINGS0			    0x09


/*	BIT DEFINITIONS FOR IQS263	*/

/*	Bit definitions	*/
/*	System Flags - STATUS	*/
/*	Indicates ATI is busy	*/
#define	ATI_BUSY			0x04
/*	Indicates reset has occurred	*/
#define	SHOW_RESET			0x80
/*	Bit definitions - Event Bytes	*/
#define	PROX_EVENT			    0x01
#define	TOUCH_EVENT			    0x02
#define	SLIDE_EVENT			    0x04
#define	MOVE_EVENT			    0x10
#define	TAP_EVENT			      0x20
#define	FLICKLEFT_EVENT			0x40
#define	FLICKRIGHT_EVENT		0x80


/*	Bit definitions - ProxSettings0	*/
/*	Reseed the IQS253	*/
#define	RESEED				0x08
/*	Redo ATI - switch on ATI	*/
#define	REDO_ATI			0x10
/*	0 - Partial ATI Off, 1 - Partial ATI On	*/
#define	ATI_PARTIAL			0x40
/*	0 - ATI Enabled, 1 - ATI Disabled	*/
#define	ATI_OFF				0x80


/*	Set IQS263 in Projected mode	*/
#define	PROJ_MODE			0x10

/*	Bit definitions - ProxSettings1	*/
/*	0-Streaming Mode, 1-Event Mode	*/
#define	EVENT_MODE			0x40


/*	Delays	*/
#define HANDSHAKE_DELAY_HOLD	11		/*	11ms	*/
#define HANDSHAKE_DELAY_SET		200		/*	200μs	*/

/*	Acknowledge that chip was reset	*/
#define ACK_RESET			0x80


#endif
