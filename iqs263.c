/*
 * iqs263.c - Linux SAR Sensor Driver for IQS263
 *
 *	Copyright (C) 2016 iWave Ltd
 *	Author: Hai Peng Yu <yuhp@iwave.cc>
 *
 * Based on mcs5000_ts.c, azoteqiqs440_ts.c, iqs263.c
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *	Azoteq (Pty) Ltd does not take responsibility for the use of this driver
 *
 *	This driver is an example driver. It will need to be ported to
 *	the specific platform and for the specific case in which it is used.
 */
#include "iqs263.h"
#include "iqs263_init.h"

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

/*	Get time for the IQS263 Timeout	*/
#include <linux/time.h>


static bool debug = true;
#define dprintk(fmt, args...)	\
	do {							          \
		if (debug)					      \
			printk(KERN_DEBUG DEVICE_NAME ": " fmt, ## args);\
	} while (0)

/****************************
 *	Pins used in this setup	*
 ****************************/
/*	RDY Pin used on this specific setup	- needs to be changed
 *	for each case (hardware dependent)
 */
/*	GPIO C2 - Bank 2	*/
static int RDY_LINE_PIN = 66;


/* Boolean value used for the initial setup */
static bool doInitialSetup;
/*	Boolean to keep track of Chip Reset Events	*/
static bool showReset;

static bool reseed;

/*	Global variable to keep the currentState in	*/
u8 currentState;	/*	Always start at state 0	*/

/*	Each client has this additional data	*/
struct iqs263_sar_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	const struct iqs263_sar_platform_data *platform_data;
};

/*	Events data Struct	*/
struct events_data{
	/*	Boolean to keep track of Prox   Events	*/
	bool prox;
	/*	Boolean to keep track of Touch Events	*/
	bool touch;
	/*	Boolean to keep track of Tap Events	*/
	bool tap;
	/*	Boolean to keep track of FlickLeft Events	*/
	bool flickLeft;
	/*	Boolean to keep track of FlickRight Events	*/
	bool flickRight;
	/*	Boolean to keep track of Slide Events	*/
	bool slide;
};

static struct events_data events;

/********************************************************
 *		The IQS263 specific setup is done here			*
 *		This setup will change for each application		*
 ********************************************************/

/*	Command for reseeding the Azoteq IQS263	*/
static void __maybe_unused iqs263_reseed(struct iqs263_sar_data *data)
{
	i2c_smbus_write_byte_data(data->client, PROX_SETTINGS0, RESEED);
}

/*	Command for re-doing ATI on the Azoteq IQS263	*/
static void __maybe_unused iqs263_reati(struct iqs263_sar_data *data)
{
	/*	Redo ATI	*/
	i2c_smbus_write_byte_data(data->client, PROX_SETTINGS0, REDO_ATI);
}


/************************************************************************
 *							IQS263 Initial Setup						*
 ************************************************************************/
/* Setup the Active Channels for Azoteq IQS263 */
static void iqs263_init_setup(struct iqs263_sar_data *data)
{
	/* Data buffer for communication with the IQS263, set to an array size of
	*  8 elememnts, which can fit all the IQS263 settings. */
	u8 data_buffer[8];

	i2c_smbus_read_i2c_block_data(data->client, DEVICE_INFO, 2,
		data_buffer);

	// Switch the IQS263 into projection mode - if necessary
	i2c_smbus_write_byte_data(data->client, SYS_FLAGS, SYSTEM_FLAGS_VAL);

  // Set active channels
	i2c_smbus_write_byte_data(data->client, ACTIVE_CHANNELS, ACTIVE_CHS);

  // Setup touch and prox thresholds for each channel
	data_buffer[0] = PROX_THRESHOLD;
  data_buffer[1] = TOUCH_THRESHOLD_CH1;
  data_buffer[2] = TOUCH_THRESHOLD_CH2;
  data_buffer[3] = TOUCH_THRESHOLD_CH3;
  data_buffer[4] = MOVEMENT_THRESHOLD;
  data_buffer[5] = RESEED_BLOCK;
  data_buffer[6] = HALT_TIME;
  data_buffer[7] = I2C_TIMEOUT;
	i2c_smbus_write_i2c_block_data(data->client, THRESHOLDS, 8,
		data_buffer);

	// Set the ATI Targets (Target Counts)
	data_buffer[0] = LOW_POWER;
	data_buffer[1] = ATI_TARGET_TOUCH;
	data_buffer[2] = ATI_TARGET_PROX;
	i2c_smbus_write_i2c_block_data(data->client, TIMINGS_AND_TARGETS, 3,
		data_buffer);

	// Set the BASE value for each channel
  data_buffer[0] = MULTIPLIERS_CH0;
  data_buffer[1] = MULTIPLIERS_CH1;
  data_buffer[2] = MULTIPLIERS_CH2;
  data_buffer[3] = MULTIPLIERS_CH3;
	i2c_smbus_write_i2c_block_data(data->client, MULTIPLIERS, 4,
		data_buffer);

	// Setup prox settings
  data_buffer[0] = PROXSETTINGS0_VAL;
  data_buffer[1] = PROXSETTINGS1_VAL;
  data_buffer[2] = PROXSETTINGS2_VAL;
  data_buffer[3] = PROXSETTINGS3_VAL;
  data_buffer[4] = EVENT_MASK_VAL;
	i2c_smbus_write_i2c_block_data(data->client, PROX_SETTINGS, 5,
		data_buffer);

	// Setup Compensation (PCC)
  data_buffer[0] = COMPENSATION_CH0;
  data_buffer[1] = COMPENSATION_CH1;
  data_buffer[2] = COMPENSATION_CH2;
  data_buffer[3] = COMPENSATION_CH3;
	i2c_smbus_write_i2c_block_data(data->client, COMPENSATION, 4,
		data_buffer);

    // Set gesture timers on IQS263
  data_buffer[0] = TAP_TIMER;
  data_buffer[1] = FLICK_TIMER;
  data_buffer[2] = FLICK_THRESHOLD;
	i2c_smbus_write_i2c_block_data(data->client, GESTURE_TIMERS, 3,
		data_buffer);

  // Redo ati
  //data_buffer[0] = REDO_ATI;
	//i2c_smbus_write_i2c_block_data(data->client, PROX_SETTINGS, 1,
	//	data_buffer);
	i2c_smbus_write_byte_data(data->client, PROX_SETTINGS0, REDO_ATI);

	// Wait untill the ATI algorithm is done
	do
	{
			mdelay(10);
			i2c_smbus_read_i2c_block_data(data->client, SYS_FLAGS, 1,
				data_buffer);
	}while((data_buffer[0] & ATI_BUSY) == ATI_BUSY);

	// Setup prox settings
	data_buffer[0] = PROXSETTINGS0_VAL;
	data_buffer[1] = (PROXSETTINGS1_VAL|EVENT_MODE);   //go to event
	data_buffer[2] = (PROXSETTINGS2_VAL|0x02);
	data_buffer[3] = PROXSETTINGS3_VAL;
	data_buffer[4] = EVENT_MASK_VAL;
	i2c_smbus_write_i2c_block_data(data->client, PROX_SETTINGS, 5,
		data_buffer);

}

/************************************************************************
 *						State Machine Helper Functions					*
 ************************************************************************/

/**
 *	Check for Events on the IQS263. ATI, Prox and Touch event. Also check
 *	the Show Reset Flag
 *	The function returns no value - instead Global flags are set to
 *	indicate the events that ocurred
 */
static void readEvents(struct iqs263_sar_data *data, u8 *data_buffer)
{
	//u8 data_buffer[7];

	/*
	 *	Read the Bytes for Prox and Touch events -
	 *	save this state
	 */
	i2c_smbus_read_i2c_block_data(data->client, SYS_FLAGS, 2,
		&data_buffer[0]);

	udelay(20);
	i2c_smbus_read_i2c_block_data(data->client, TOUCH_BYTES, 2,
		&data_buffer[2]);

  udelay(20);
	i2c_smbus_read_i2c_block_data(data->client, COORDINATES, 3,
		&data_buffer[4]);

//	dprintk("SYS_FLAGS 0: 0x%x\n", (int)(data_buffer[0]));
//	dprintk("SYS_FLAGS 1: 0x%x\n", (int)(data_buffer[1]));
	//input_report_key(data->input_dev,KEY_ENTER,currentState);
	//input_report_abs(data->input_dev,ABS_X, 0);
	//input_report_abs(data->input_dev,ABS_Y, 0);
	//input_sync(data->input_dev);

	/*	We are interested in byte 0, byte 1 and byte 2	*/
	/*	These bytes will give us all of the info we need	*/
	if (data_buffer[0] & SHOW_RESET)
		showReset = true;
	else
		showReset = false;

	if (data_buffer[1] & PROX_EVENT){
		dprintk("PROX_EVENT: %d \n",currentState);
		events.prox = true;
	}else
		events.prox = false;

	if (data_buffer[1] & TOUCH_EVENT){
		dprintk("TOUCH_EVENT: %d \n",currentState);
		events.touch = true;
	}else
		events.touch = false;

	if (data_buffer[1] & SLIDE_EVENT){
		dprintk("SLIDE_EVENT: %d \n",currentState);
		events.slide = true;
	}else
		events.slide = false;

//	if (data_buffer[1] & MOVE_EVENT)
//		move = true;
//	else
//		move = false;

	if (data_buffer[1] & TAP_EVENT)
		events.tap = true;
	else
		events.tap = false;

	if (data_buffer[1] & FLICKLEFT_EVENT){
		dprintk("FLICKLEFT_EVENT: %d \n",currentState);
		events.flickLeft = true;
	}else
		events.flickLeft = false;

	if (data_buffer[1] & FLICKRIGHT_EVENT){
		dprintk("FLICKRIGHT_EVENT: %d \n",currentState);
		events.flickRight = true;
	}else
		events.flickRight = false;

}

/*************************************************************************/

/*	Platform data for the Azoteq IQS263 SAR driver	*/
struct iqs263_sar_platform_data {
	void (*cfg_pin)(void);
};

/**
 *	Because the IQS263 operates in event mode, implement a handshake
 *	function that will initiate comms with the IQS263 if we want to talk
 *	to it.
 */
void iqs263_event_mode_handshake(struct iqs263_sar_data *data)
{
	/********************************************************
	 *			Try and do an Event Mode Handshake			*
	 *******************************************************/
	/*
	 *	There might be another way to build in the delay.
	 *	Event mode handshake is done by manually pulling
	 *	the IQS263 RDY line low for ~10ms and then releasing
	 *	it. This will tell the IQS263 that the Master wants
	 *	to talk and it will then give a Communications window
	 *	which will call the interrupt request function.
	 *	From there the IQS263 can be read again
	 */
	/*	Pull RDY low	*/

	u8 data_buffer[2];
	dprintk("HANDSHAKE");
	gpio_direction_output(RDY_LINE_PIN, 0);
	/*	Hold the RDY low for ~10ms	*/
	mdelay(HANDSHAKE_DELAY_HOLD);
	/*	Release RDY line	*/
	gpio_direction_input(RDY_LINE_PIN);
	/*	Delay before talking to IQS263	*/
	udelay(HANDSHAKE_DELAY_SET);

	data_buffer[0] = PROXSETTINGS0_VAL;
	data_buffer[1] = (PROXSETTINGS1_VAL|EVENT_MODE);
	i2c_smbus_write_i2c_block_data(data->client, PROX_SETTINGS, 2,
		data_buffer);

}

/**
 *	Interrupt event fires on the Falling edge of
 *	the RDY signal from the Azoteq IQS263
 */
static irqreturn_t iqs263_sar_interrupt(int irq, void *dev_id)
{
	struct iqs263_sar_data *data = dev_id;
	u8 data_buffer[7];
	//u8 sliderCoords;
	//u8 touch0;

	/*
	 *	Check for a reset - if reset was seen,
	 *	Then setup has to been done all over again
	 *
	 *	If we need to setup the IQS263, traverse through the
	 *	setup
	 */
	if (doInitialSetup) {
		iqs263_init_setup(data);
		doInitialSetup = false;
		goto out;
	}

	/*	Setup done	*/

	/**********************	State Machine	***************************/

	readEvents(data,data_buffer);	/*	Check Events	*/


	if (showReset) {
		doInitialSetup = true;
	}

	/*	Wait for Prox event	*/
	/*	No Activation State	*/
	if (currentState == 0) {

		/*	Move to next state if Touch was seen	*/
		if (events.touch) {
			//touch0 = data_buffer[2];
			dprintk("Touch0 : %x\n",(int)(data_buffer[2]));
			if(events.slide | events.flickLeft | events.flickRight)
			    currentState = 1;
		} else
			currentState = 0;
	}

	/*
	 *	Activation State
	 *	Check for touch and prox - if a touch is seen,
	 *	move to the touch activation state and clear timer.
	 *	If only a prox is seen, stay in
	 *	this state and keep timer active
	 */
	if (currentState == 1) {
		/*	check the Slide Corrds */
		if (events.slide) {
			//sliderCoords = data_buffer[3];
			dprintk("SliderCoords : %x\n",(int)((data_buffer[6]<<8)|data_buffer[5]));
			input_report_key(data->input_dev,BTN_TOUCH,1);
			input_sync(data->input_dev);
			goto out;
		}

		/*	Touch is still active - check other event	*/
		if (events.touch) {
			dprintk("Touch0 : %x\n",(int)(data_buffer[2]));
			if(events.flickLeft){ //touch finished
				  input_report_key(data->input_dev,KEY_LEFT,1);
					input_report_key(data->input_dev,KEY_LEFT,0);
			}else if(events.flickRight){
				  input_report_key(data->input_dev,KEY_RIGHT,1);
				  input_report_key(data->input_dev,KEY_RIGHT,0);
			}
			currentState = 0;
			input_report_key(data->input_dev,BTN_TOUCH,0);
			input_sync(data->input_dev);
		}
	} /*	end State 1	*/

	/********************	State Machine End	*******************/

	/* Jump to end after each Communications window	*/
out:
  dprintk("========= currentState: %d =========\n", currentState);
	return IRQ_HANDLED;
}

/**
 *	The probe function is called when Linux is looking
 *	for the IQS263 on the I2C bus (I2C-0)
 */
static int iqs263_sar_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct iqs263_sar_data *data;
	struct input_dev *input_dev;
	//unsigned long flags;
	int ret;
	/*	Allocate memory	*/
	data = kzalloc(sizeof(struct iqs263_sar_data), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!data || !input_dev) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto err_free_mem;
	}


	/*	Save the stuctures to be used in the driver	*/
	data->client = client;
	data->input_dev = input_dev;
	data->platform_data = client->dev.platform_data;

	input_dev->name = "Azoteq IQS263 SAR Sensor";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);
	set_bit(KEY_LEFT, input_dev->keybit);
	set_bit(KEY_RIGHT, input_dev->keybit);


	/*	Request the RDY line for the IQS263	*/
	gpio_request(RDY_LINE_PIN, "interrupt");
	iqs263_event_mode_handshake(data);
	doInitialSetup = true;
	//iqs263_init_setup(data);

	/*	Request an interrup on the RDY line	*/
	client->irq = gpio_to_irq(RDY_LINE_PIN);
	//client->irq = data->gpiochip->to_irq(data->gpiochip, RDY_LINE_GPIO_PIN);
	dprintk("to_irq %d\n", client->irq);

	/* Set driver data */
	input_set_drvdata(input_dev, data);

	/*
	 *	Request the interrupt on a falling trigger
	 *	and only one trigger per falling edge
	 */
	ret = request_threaded_irq(client->irq, NULL, iqs263_sar_interrupt,
	              IRQF_TRIGGER_FALLING | IRQF_ONESHOT, IQS263_DRIVER_NAME, data);

	switch (ret) {
	case -EBUSY:
			printk(KERN_ERR IQS263_DRIVER_NAME
				": IRQ %d is busy\n",client->irq);
			goto exit_gpio_free_rdy_pin;
	case -EINVAL:
			printk(KERN_ERR IQS263_DRIVER_NAME
				": Bad irq number or handler\n");
			goto exit_gpio_free_rdy_pin;
	default:
	    dprintk("Interrupt %d obtained\n",
				client->irq);
				break;
	};

	/*	Register the device */
	ret = input_register_device(data->input_dev);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto exit_gpio_free_rdy_pin;
	}


	/*	Now, assign default values to global variables	*/
	/*	Periodically check for chip reset	*/
	currentState = 0;	/*	Always start at state 0	*/
	events.touch = false; /*	Assume no touch at first	*/
	reseed = false;
	showReset = false;

	/* Set i2c client data	*/
	i2c_set_clientdata(client, data);

  //Enabe the interrup
	//enable_irq(client->irq);

	return 0;

exit_gpio_free_rdy_pin:
	gpio_free(RDY_LINE_PIN);
err_free_mem:
	input_free_device(input_dev);
	kfree(data);
	return ret;
}


static int iqs263_sar_remove(struct i2c_client *client)
{
	struct iqs263_sar_data *data = i2c_get_clientdata(client);

	free_irq(client->irq, data);

	input_unregister_device(data->input_dev);
	input_free_device(data->input_dev);
	kfree(data);
	gpio_free(RDY_LINE_PIN);

	i2c_set_clientdata(client, NULL);

	return 0;
}

/*	Standard stucture with the device id for identification	*/
static const struct i2c_device_id iqs263_sar_id[] = {
		{ DEVICE_NAME, 0 },
		{ }
};

MODULE_DEVICE_TABLE(i2c, iqs263_sar_id);

/*
 *	Standard stucture containing the driver
 *	information and procedures
 */

#ifdef CONFIG_OF
static struct of_device_id iqs263_match_table[] = {
        { .compatible = "azoteq,iqs263",},
        { },
};
#else
#define iqs263_match_table NULL
#endif

static struct i2c_driver iqs263_sar_driver = {
	.probe = iqs263_sar_probe,
  .remove = iqs263_sar_remove,
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(iqs263_match_table),
	},
	.id_table = iqs263_sar_id,
};

/**
 *
 *	when the device I2C bus init is called
 */
static int __init iqs263_sar_init(void)
{
	/*	Add i2c driver to kernel	*/
	printk(KERN_ALERT "Installing IQS263 SAR Sensor Driver");
	return i2c_add_driver(&iqs263_sar_driver);
}

/*	Remove the driver */
static void __exit iqs263_sar_exit(void)
{
	/*	Boolean value used for the initial setup	*/
	doInitialSetup = true;

	/*	Delete driver	*/
	i2c_del_driver(&iqs263_sar_driver);
	printk(KERN_ALERT "Delete: - %s\n",
			"IQS263 SAR Sensor driver Deleted! ");
}

module_init(iqs263_sar_init);
module_exit(iqs263_sar_exit);

/* Module information */
MODULE_AUTHOR("Hai Peng Yu <yuhp@iwave.cc>");
MODULE_DESCRIPTION("SAR Sensor driver for Azoteq IQS263");
MODULE_LICENSE("GPL");

module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Enable debugging messages");
