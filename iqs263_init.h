/*
 *	IQS263_init.h - IQS263 Registers and Bit Definitions
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
 *	This header file contains the setup for the IQS263 for this demo code
 *	of the IQS263 SAR Sensor. This file needs to be edited for each
 *	specific implementation of this driver. Use the IQS253 GUI from
 *	www.azoteq.com to change these settings and tune for a specific
 *	implementation.
 */

#ifndef IQS263_INIT_H
#define IQS263_INIT_H


/* Used to switch Projected mode & set Global Filter Halt (0x01 Byte1) */
#define SYSTEM_FLAGS_VAL					0x00
/* Enable / Disable system events (0x01 Byte2)*/
#define SYSTEM_EVENTS_VAL					0x00

/* Change the Multipliers & Base value (0x07 in this order) */
#define MULTIPLIERS_CH0						0x08//0x19 - default
#define MULTIPLIERS_CH1						0x08//0x08 - default
#define MULTIPLIERS_CH2						0x08//0x08 - default
#define MULTIPLIERS_CH3						0x08//0x08 - default
#define BASE_VAL						      0x08//0x44 - default

/* Change the Compensation for each channel (0x08 in this order) */
#define COMPENSATION_CH0					0x51
#define COMPENSATION_CH1					0x49
#define COMPENSATION_CH2					0x4A
#define COMPENSATION_CH3					0x49

/* Change the Prox Settings or setup of the IQS263 (0x09 in this order) */
//#define PROXSETTINGS0_VAL					0x00
//#define PROXSETTINGS1_VAL					0x1D
//#define PROXSETTINGS2_VAL					0x04
//#define PROXSETTINGS3_VAL					0x00
//#define EVENT_MASK_VAL						0x00
#define PROXSETTINGS0_VAL					0x00
#define PROXSETTINGS1_VAL					0x15 //2CH Slider 0B00010101
#define PROXSETTINGS2_VAL					0x40
#define PROXSETTINGS3_VAL					0x00
#define EVENT_MASK_VAL						0xff

/* Change the Thresholds for each channel (0x0A in this order) */
#define PROX_THRESHOLD						0x08
#define TOUCH_THRESHOLD_CH1			  0x20
#define TOUCH_THRESHOLD_CH2				0x20
#define TOUCH_THRESHOLD_CH3				0x20
#define MOVEMENT_THRESHOLD				0x03
#define RESEED_BLOCK						  0x00
#define HALT_TIME						      0x14
#define I2C_TIMEOUT						    0x04

/* Change the Timing settings (0x0B in this order) */
#define LOW_POWER						      0x00
#define ATI_TARGET_TOUCH					0x30
#define ATI_TARGET_PROX						0x40

/* Change the Gesture Timing settings (0x0C in this order) */
#define TAP_TIMER						      0x05
#define FLICK_TIMER						    0x51
#define FLICK_THRESHOLD						0x33

/* Set Active Channels (0x0D) */
#define ACTIVE_CHS						0x07 //only CH0 CH1 CH2


#endif	/* IQS253_INIT_H */
