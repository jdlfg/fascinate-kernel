/*
 *	JACK device detection driver.
 *
 *	Copyright (C) 2009 Samsung Electronics, Inc.
 *
 *	Authors:
 *		Uk Kim <w0806.kim@samsung.com>
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; version 2 of the License.
 */

#include <linux/module.h>
#include <linux/sysdev.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/wakelock.h>

#include <mach/hardware.h>
#include <mach/gpio-aries.h>
#include <mach/gpio.h>
//#include <plat/regs-gpio.h>
#include <plat/gpio-cfg.h>
#include <plat/irqs.h>
#include <asm/mach-types.h>

#include <mach/sec_jack.h>

//#define CONFIG_DEBUG_SEC_JACK
#define SUBJECT "JACK_DRIVER"

// reverse ear jack detection value - noik.heo (20100204)
#define CONFIG_EARJACK_GPIO_LOW_ACTIVE_ENABLE
#define CONFIG_SEC_HEADSETHOOK_KEY

#ifdef CONFIG_DEBUG_SEC_JACK
#define SEC_JACKDEV_DBG(format,...)\
	printk ("["SUBJECT" (%s,%d) ] " format "\n", __func__, __LINE__, ## __VA_ARGS__);
#else
#define SEC_JACKDEV_DBG(format,...)
#endif

#ifdef CONFIG_SEC_HEADSETHOOK_KEY    // for headset hook key process : noik.heo (20100222)
#define KEYCODE_HEADSETHOOK 248
#else
#define KEYCODE_SENDEND 248
#endif

#define DETECTION_CHECK_COUNT	2
#define	DETECTION_CHECK_TIME	get_jiffies_64() + (HZ/10)// 1000ms / 10 = 100ms
#define	SEND_END_ENABLE_TIME	get_jiffies_64() + (HZ*1)// 1000ms * 1 = 1sec

#define SEND_END_CHECK_COUNT	3
#define SEND_END_CHECK_TIME     get_jiffies_64() + (HZ * 2) //2000ms 

#define WAKELOCK_DET_TIMEOUT	HZ * 5 //5 sec

static struct platform_driver sec_jack_driver;

struct class *jack_class;
EXPORT_SYMBOL(jack_class);
static struct device *jack_selector_fs;				// Sysfs device, this is used for communication with Cal App.
EXPORT_SYMBOL(jack_selector_fs);
extern int s3c_adc_get_adc_data(int channel);
extern unsigned int HWREV;
extern int isVoiceCall;
extern int proximity_onoff(u8 onoff);

struct sec_jack_info {
	struct sec_jack_port port;
	struct input_dev *input;
};

static struct sec_jack_info *hi;

struct switch_dev switch_jack_detection = {
		.name = "h2w",
};

struct switch_dev switch_dock_detection = {
		.name = "dock",
};

/* To support AT+FCESTEST=1 */
struct switch_dev switch_sendend = {
		.name = "send_end",
};
static struct timer_list send_end_key_event_timer;

static unsigned int current_jack_type_status;
static unsigned int send_end_enable = 0;
static unsigned int send_end_pressed = 0;
static struct wake_lock jack_sendend_wake_lock;
static int recording_status=0;
static int send_end_irq_token=0;

unsigned int get_headset_status(void)
{
	SEC_JACKDEV_DBG(" headset_status %d", current_jack_type_status);
	return current_jack_type_status;
}
EXPORT_SYMBOL(get_headset_status);


void set_recording_status(int value)
{
	recording_status = value;
}


static int get_recording_status(void)
{
	return recording_status;
}

void set_dock_state(int value)
{
	printk(KERN_INFO "set_dock_state : 0X%x\n", value);
	switch_set_state(&switch_dock_detection, value);
}

static void jack_input_selector(int jack_type_status)
{
	SEC_JACKDEV_DBG("jack_type_status = 0X%x", jack_type_status);

	switch(jack_type_status)
	{
		case SEC_HEADSET_3_POLE_DEVICE:
		case SEC_HEADSET_4_POLE_DEVICE:	
		{
			break;
		}
		case SEC_TVOUT_DEVICE:
		{
			break;
		}
		case SEC_UNKNOWN_DEVICE:
		{
			printk("unknown jack device attached. User must select jack device type\n");
			break;
		}
		default:
		{
			printk(KERN_ERR "wrong selector value\n");
			break;			
		}
	}
}

static int jack_type_detect_change(struct work_struct *ignored)
{
	int adc = 0;
	struct sec_gpio_info   *det_jack = &hi->port.det_jack;
	int state = gpio_get_value(det_jack->gpio) ^ det_jack->low_active;

	struct sec_gpio_info   *send_end = &hi->port.send_end;
	int count_abnormal=0;
	int count_pole=0;
	bool bQuit = false;

	while(!bQuit)
	{
		state = gpio_get_value(det_jack->gpio) ^ det_jack->low_active;

		if(state)
		{
			adc = s3c_adc_get_adc_data(SEC_HEADSET_ADC_CHANNEL);
#if 1
			/* 4 pole - jig zone */
			if( (630 <= adc) && (adc < 3700) )
			{
				current_jack_type_status = SEC_HEADSET_4_POLE_DEVICE;
				printk(KERN_INFO "[ JACK_DRIVER (%s,%d) ] 4 pole headset attached : adc = %d\n",__func__,__LINE__, adc);
				bQuit = true;
				if(send_end_irq_token==0)
				{
					enable_irq(send_end->eint);
					send_end_irq_token=1;
				}
			}
			/* 3 pole zone or unstable zone */
			else
			{	
				if(!adc || count_pole == 10)
				{
					/* detect 3pole or tv-out cable */
					printk(KERN_INFO "[ JACK_DRIVER (%s,%d) ] 3 pole headset attatched : adc = %d\n", __func__,__LINE__,adc);
					count_pole = 0;
					if(send_end_irq_token==1)
					{
						disable_irq(send_end->eint);
						send_end_irq_token=0;
					}
					current_jack_type_status = SEC_HEADSET_3_POLE_DEVICE;
					gpio_set_value(GPIO_EAR_BIAS_EN, 0);
					bQuit = true;
				}
				/* If 4 pole is inserted slowly, ADC value should be lower than 250.
			 	* So, check again.
				 */
				else
				{
					++count_pole;
					/* Todo : to prevent unexpected reset bug.
					 * 		  is it msleep bug? need wakelock.
					 */
					wake_lock_timeout(&jack_sendend_wake_lock, WAKELOCK_DET_TIMEOUT);
					msleep(20);
				}
			}
#else
			/* 4 pole - jig zone */
			if( (3400 <= adc) && (adc < 3600) )
			{
				current_jack_type_status = SEC_HEADSET_4_POLE_DEVICE;
				printk(KERN_INFO "[ JACK_DRIVER (%s,%d) ] 4 pole factory jig attached : adc = %d\n",__func__,__LINE__, adc);
				bQuit = true;
				if(send_end_irq_token==0)
				{
					enable_irq(send_end->eint);
					send_end_irq_token=1;
				}
			}
			/*  unstable zone */
			else if( ((adc >= 3100) && (adc < 3550)) || (adc >= 3750) )
			{
				/* unknown cable or unknown case */
				if(count_abnormal++>100)
				{
					count_abnormal=0;
					bQuit = true;
					printk(KERN_INFO "[ JACK_DRIVER (%s,%d) ] 3 pole headset or TV-out attatched : adc = %d\n", __func__,__LINE__,adc);
					count_pole = 0;
					if(send_end_irq_token==1)
					{
						disable_irq(send_end->eint);
						send_end_irq_token=0;
					}
					current_jack_type_status = SEC_HEADSET_3_POLE_DEVICE;
					gpio_set_value(GPIO_EAR_BIAS_EN, 0);
				}
				/* Todo : to prevent unexpected reset bug.
				 * 		  is it msleep bug? need wakelock.
				 */
				wake_lock_timeout(&jack_sendend_wake_lock, WAKELOCK_DET_TIMEOUT);
				mdelay(10);
			}
			/* 4 pole zone */
			else if( (2200 <= adc) && (adc < 3100) )
			{
				current_jack_type_status = SEC_HEADSET_4_POLE_DEVICE;
				printk(KERN_INFO "[ JACK_DRIVER (%s,%d) ] 4 pole  headset attached : adc = %d\n",__func__,__LINE__, adc);
				bQuit = true;
				if(send_end_irq_token==0)
				{
					enable_irq(send_end->eint);
					send_end_irq_token=1;
				}
			}
			/* unstable zone */
			else if(adc > 600)
			{
				SEC_JACKDEV_DBG("invalid adc > 600, adc is %d\n",adc);

				/* unknown cable or unknown case */
				if(count_abnormal++>100)
				{
					count_abnormal=0;
					bQuit = true;
					printk(KERN_INFO "[ JACK_DRIVER (%s,%d) ] 3 pole headset or TV-out attatched : adc = %d\n", __func__,__LINE__,adc);
					count_pole = 0;
					if(send_end_irq_token==1)
					{
						disable_irq(send_end->eint);
						send_end_irq_token=0;
					}
					current_jack_type_status = SEC_HEADSET_3_POLE_DEVICE;
					gpio_set_value(GPIO_EAR_BIAS_EN, 0);
				}
				/* Todo : to prevent unexpected reset bug.
				 * 		  is it msleep bug? need wakelock.
				 */
				wake_lock_timeout(&jack_sendend_wake_lock, WAKELOCK_DET_TIMEOUT);
				mdelay(10);
			}
			/* 3 pole zone or unstable zone */
			else
			{	
				if(!adc || count_pole == 10)
				{
					/* detect 3pole or tv-out cable */
					printk(KERN_INFO "[ JACK_DRIVER (%s,%d) ] 3 pole headset or TV-out attatched : adc = %d\n", __func__,__LINE__,adc);
					count_pole = 0;
					if(send_end_irq_token==1)
					{
						disable_irq(send_end->eint);
						send_end_irq_token=0;
					}
					current_jack_type_status = SEC_HEADSET_3_POLE_DEVICE;
					gpio_set_value(GPIO_EAR_BIAS_EN, 0);
					bQuit=true;
				}
				/* If 4 pole is inserted slowly, ADC value should be lower than 250.
			 	* So, check again.
				 */
				else
				{
					++count_pole;
					/* Todo : to prevent unexpected reset bug.
					 * 		  is it msleep bug? need wakelock.
					 */
					wake_lock_timeout(&jack_sendend_wake_lock, WAKELOCK_DET_TIMEOUT);
					mdelay(20);
				}
			}
#endif
		} /* if(state) */
		else
		{
			bQuit = true;

			current_jack_type_status = SEC_JACK_NO_DEVICE;
			gpio_set_value(GPIO_EAR_BIAS_EN, 0);
			SEC_JACKDEV_DBG("JACK dev detached\n");
		}
		switch_set_state(&switch_jack_detection, current_jack_type_status);
		jack_input_selector(current_jack_type_status);
	}

	return 0;

}

static DECLARE_DELAYED_WORK(detect_jack_type_work, jack_type_detect_change);

static int jack_detect_change(struct work_struct *ignored)
{
	struct sec_gpio_info   *det_jack = &hi->port.det_jack;
	struct sec_gpio_info   *send_end = &hi->port.send_end;
	int state;
	int count=20;

	SEC_JACKDEV_DBG("");
	//del_timer(&jack_detect_timer);
	cancel_delayed_work_sync(&detect_jack_type_work);
	state = gpio_get_value(det_jack->gpio) ^ det_jack->low_active;

	wake_lock_timeout(&jack_sendend_wake_lock, WAKELOCK_DET_TIMEOUT);
	
	/* block send/end key event */
	mod_timer(&send_end_key_event_timer, SEND_END_CHECK_TIME);

	// There is no proximity sensor noise to ear jack connctor
#if 0
	if(state)
	{
		proximity_onoff(0);
	}
	else
	{
		// [DE14-1705] hi99.an // for sleep current when ear-jack out
		if (isVoiceCall)
		{
			proximity_onoff(1);
			gpio_direction_output(GPIO_PS_ON, 1);
			//printk("proximity on while voice call\n");
		}
	}
#endif

	if(state)
	{		
		/* check pin state repeatedly */
		while(count--)
		{
			if(state != (gpio_get_value(det_jack->gpio) ^ det_jack->low_active))
			{
				return -1;
			}
			msleep(10); // [DH24-2118] // TEST // avoid pipe error when jack inserted
			//mdelay(10);
		}

		gpio_set_value(GPIO_EAR_BIAS_EN, 1);
		schedule_delayed_work(&detect_jack_type_work,50);
	}
	else if(!state && current_jack_type_status != SEC_JACK_NO_DEVICE)
	{
		if(send_end_irq_token==1)
		{
			disable_irq(send_end->eint);
			send_end_irq_token=0;
		}
		current_jack_type_status = SEC_JACK_NO_DEVICE;
		gpio_set_value(GPIO_EAR_BIAS_EN, 0);
		switch_set_state(&switch_jack_detection, current_jack_type_status);
		SEC_JACKDEV_DBG("JACK dev detached  \n");			

	}

	return 0;
}

static int sendend_switch_change(struct work_struct *ignored)
{
	struct sec_gpio_info   *det_jack = &hi->port.det_jack;
	struct sec_gpio_info   *send_end = &hi->port.send_end;
	int state, headset_state;
	SEC_JACKDEV_DBG("");
	int count=6;
	
	headset_state = gpio_get_value(det_jack->gpio) ^ det_jack->low_active;
	state = gpio_get_value(send_end->gpio) ^ send_end->low_active;

	wake_lock_timeout(&jack_sendend_wake_lock, WAKELOCK_DET_TIMEOUT);

	/* check pin state repeatedly */
	while(count-- && !send_end_pressed)
	{
		if(state != (gpio_get_value(send_end->gpio) ^ send_end->low_active) || !headset_state)
		{
			printk(KERN_INFO "[ JACK_DRIVER] (%s,%d) ] SEND/END key is ignored. State is unstable.\n",__func__,__LINE__);
			return -1;
		}
		mdelay(10);
	}

#ifdef CONFIG_SEC_HEADSETHOOK_KEY    // for headset hook key process : noik.heo (20100222)
	input_report_key(hi->input, KEYCODE_HEADSETHOOK, state);
#else
	input_report_key(hi->input, KEYCODE_SENDEND, state);
#endif
	input_sync(hi->input);
	switch_set_state(&switch_sendend,state);
	send_end_pressed = state;
	printk(KERN_INFO "[ JACK_DRIVER (%s,%d) ] Button is %s.\n", __func__,__LINE__, state? "pressed" : "released");

	return 0;
}

static int sendend_timer_work_func(struct work_struct *ignored)
{
	struct sec_gpio_info   *det_jack = &hi->port.det_jack;
 	int headset_state;

	headset_state = gpio_get_value(det_jack->gpio) ^ det_jack->low_active;

	send_end_enable = 1;

	
	if(send_end_pressed && !headset_state)
	{
#ifdef CONFIG_SEC_HEADSETHOOK_KEY    // for headset hook key process : noik.heo (20100222)
		input_report_key(hi->input, KEYCODE_HEADSETHOOK, 0);
#else
		input_report_key(hi->input, KEYCODE_SENDEND, 0);
#endif
		input_sync(hi->input);
		switch_set_state(&switch_sendend,0);
		send_end_pressed = 0;
		SEC_JACKDEV_DBG("Button is %s forcely.\n", "released");
	}
}

static DECLARE_WORK(jack_detect_work, jack_detect_change);
static DECLARE_WORK(sendend_switch_work, sendend_switch_change);
static DECLARE_WORK(sendend_timer_work, sendend_timer_work_func);

//IRQ Handler
static irqreturn_t detect_irq_handler(int irq, void *dev_id)
{
	SEC_JACKDEV_DBG("jack isr");
	send_end_enable = 0;
	schedule_work(&jack_detect_work);
	return IRQ_HANDLED;
}

static void send_end_key_event_timer_handler(unsigned long arg)
{
#if 1
	schedule_work(&sendend_timer_work);
#else
	struct sec_gpio_info   *det_jack = &hi->port.det_jack;
	struct sec_gpio_info   *send_end = &hi->port.send_end;
 	int headset_state;

	headset_state = gpio_get_value(det_jack->gpio) ^ det_jack->low_active;
	send_end_enable = 1;

	if(send_end_pressed && !headset_state)
	{
#ifdef CONFIG_SEC_HEADSETHOOK_KEY    // for headset hook key process : noik.heo (20100222)
		input_report_key(hi->input, KEYCODE_HEADSETHOOK, 0);
#else
		input_report_key(hi->input, KEYCODE_SENDEND, 0);
#endif
		input_sync(hi->input);
		send_end_pressed = 0;
		SEC_JACKDEV_DBG("Button is %s forcely.\n", "released");
	}
#endif
}

static irqreturn_t send_end_irq_handler(int irq, void *dev_id)
{
   struct sec_gpio_info   *det_jack = &hi->port.det_jack;
   int headset_state;

	SEC_JACKDEV_DBG("sendend isr");
	headset_state = gpio_get_value(det_jack->gpio) ^ det_jack->low_active;
	SEC_JACKDEV_DBG("SEND/END isr, enable = %d , headset_state = %d",send_end_enable,headset_state);

	if (send_end_enable && headset_state && (current_jack_type_status == SEC_HEADSET_4_POLE_DEVICE))
	{
		schedule_work(&sendend_switch_work);		
	}
		  
	return IRQ_HANDLED;
}

static ssize_t select_jack_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "[JACK] %s : operate nothing\n", __FUNCTION__);

	return 0;
}

static ssize_t select_jack_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int value = 0;
	struct sec_gpio_info   *det_jack = &hi->port.det_jack;
	struct sec_gpio_info   *send_end = &hi->port.send_end;
	int state = gpio_get_value(det_jack->gpio) ^ det_jack->low_active;

	sscanf(buf, "%d", &value);
	printk(KERN_INFO "[JACK_DRIVER] User  selection : 0X%x", value);
		
	switch(value)
	{
		case SEC_HEADSET_3_POLE_DEVICE:
		{
			if(send_end_irq_token == 1)
			{
				disable_irq(send_end->eint);
				send_end_irq_token = 0;
			}
			gpio_set_value(GPIO_EAR_BIAS_EN, 0);
			current_jack_type_status = SEC_HEADSET_3_POLE_DEVICE;			
			break;
		}
		case SEC_HEADSET_4_POLE_DEVICE:
		{
			gpio_set_value(GPIO_EAR_BIAS_EN, 1);
			if(send_end_irq_token == 0)
			{
				enable_irq(send_end->eint);
				send_end_irq_token = 1;
			}
			current_jack_type_status = SEC_HEADSET_4_POLE_DEVICE;
			printk(KERN_INFO "[ JACK_DRIVER (%s,%d) ] 4 pole  headset attached\n",__func__,__LINE__);
			break;
		}
		case SEC_JACK_NO_DEVICE:
		{
			if(send_end_irq_token == 1)
			{
				disable_irq(send_end->eint);
				send_end_irq_token = 0;
			}
			gpio_set_value(GPIO_EAR_BIAS_EN, 0);
			current_jack_type_status = SEC_JACK_NO_DEVICE;
			printk(KERN_INFO "[ JACK_DRIVER (%s,%d) ] JACK dev detached  \n",__func__,__LINE__);			
			break;
		}
		default:
		{
			//current_jack_type_status = SEC_JACK_NO_DEVICE;
			gpio_set_value(GPIO_EAR_BIAS_EN, 0);
			break;
		}
	}

	switch_set_state(&switch_jack_detection, current_jack_type_status);
	jack_input_selector(current_jack_type_status);

	return size;
}


static DEVICE_ATTR(select_jack, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, select_jack_show, select_jack_store);

static int sec_jack_probe(struct platform_device *pdev)
{
	int ret;
	struct sec_jack_platform_data *pdata = pdev->dev.platform_data;
	struct sec_gpio_info   *det_jack;
	struct sec_gpio_info   *send_end;
	struct input_dev	   *input;
	current_jack_type_status = SEC_JACK_NO_DEVICE;
	
	printk(KERN_INFO "SEC JACK: Registering jack driver\n");
	
	hi = kzalloc(sizeof(struct sec_jack_info), GFP_KERNEL);
	if (!hi)
		return -ENOMEM;

	memcpy (&hi->port, pdata->port, sizeof(struct sec_jack_port));

	input = hi->input = input_allocate_device();
	if (!input)
	{
		ret = -ENOMEM;
		printk(KERN_ERR "SEC JACK: Failed to allocate input device.\n");
		goto err_request_input_dev;
	}

	input->name = "sec_jack";
	set_bit(EV_SYN, input->evbit);
	set_bit(EV_KEY, input->evbit);
#ifdef CONFIG_SEC_HEADSETHOOK_KEY    // for headset hook key process : noik.heo (20100222)
	set_bit(KEYCODE_HEADSETHOOK, input->keybit);
#else
	set_bit(KEYCODE_SENDEND, input->keybit);
#endif

	ret = input_register_device(input);
	if (ret < 0)
	{
		printk(KERN_ERR "SEC JACK: Failed to register driver\n");
		goto err_register_input_dev;
	}

	//init_timer(&jack_detect_timer);
	//jack_detect_timer.function = jack_detect_timer_handler;

	init_timer(&send_end_key_event_timer);
	send_end_key_event_timer.function = send_end_key_event_timer_handler;

	SEC_JACKDEV_DBG("registering switch_sendend switch_dev sysfs sec_jack");
	
	ret = switch_dev_register(&switch_jack_detection);
	if (ret < 0) 
	{
		printk(KERN_ERR "SEC JACK: Failed to register switch device\n");
		goto err_switch_dev_register;
	}

	ret = switch_dev_register(&switch_dock_detection);
	if (ret < 0) 
	{
		printk(KERN_ERR "SEC DOCK: Failed to register switch device\n");
		goto err_switch_dev_register;
	}

	ret = switch_dev_register(&switch_sendend);
	if (ret < 0) 
	{
		printk(KERN_ERR "SEC JACK: Failed to register switch device\n");
		goto err_switch_dev_register;
	}

	//Create JACK Device file in Sysfs
	jack_class = class_create(THIS_MODULE, "jack");
	if(IS_ERR(jack_class))
	{
		printk(KERN_ERR "Failed to create class(sec_jack)\n");
	}

	jack_selector_fs = device_create(jack_class, NULL, 0, NULL, "jack_selector");
	if (IS_ERR(jack_selector_fs))
		printk(KERN_ERR "Failed to create device(sec_jack)!= %ld\n", IS_ERR(jack_selector_fs));

	if (device_create_file(jack_selector_fs, &dev_attr_select_jack) < 0)
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_select_jack.attr.name);	

	//GPIO configuration
	send_end = &hi->port.send_end;
	s3c_gpio_cfgpin(send_end->gpio, S3C_GPIO_SFN(send_end->gpio_af));
	s3c_gpio_setpull(send_end->gpio, S3C_GPIO_PULL_NONE);
	set_irq_type(send_end->eint, IRQ_TYPE_EDGE_BOTH);

	ret = request_irq(send_end->eint, send_end_irq_handler, IRQF_DISABLED, "sec_headset_send_end", NULL);

	SEC_JACKDEV_DBG("sended isr send=0X%x, ret =%d", send_end->eint, ret);
	if (ret < 0)
	{
		printk(KERN_ERR "SEC HEADSET: Failed to register send/end interrupt.\n");
		goto err_request_send_end_irq;
	}

	disable_irq(send_end->eint);
	send_end_irq_token=0;

	det_jack = &hi->port.det_jack;
	s3c_gpio_cfgpin(det_jack->gpio, S3C_GPIO_SFN(det_jack->gpio_af));
	s3c_gpio_setpull(det_jack->gpio, S3C_GPIO_PULL_NONE);
	set_irq_type(det_jack->eint, IRQ_TYPE_EDGE_BOTH);
	
#ifdef CONFIG_EARJACK_GPIO_LOW_ACTIVE_ENABLE		// noik.heo (20100217)
	det_jack->low_active = 1; 
#else
	if(HWREV >= 0xa)
	{
		det_jack->low_active = 1; 
	}
#endif

	ret = request_irq(det_jack->eint, detect_irq_handler, IRQF_DISABLED, "sec_headset_detect", NULL);

	SEC_JACKDEV_DBG("det isr det=0X%x, ret =%d", det_jack->eint, ret);
	if (ret < 0) 
	{
		printk(KERN_ERR "SEC HEADSET: Failed to register detect interrupt.\n");
		goto err_request_detect_irq;
	}

	// EAR_SEL
    {
    	if(gpio_is_valid(GPIO_EAR_BIAS_EN))
    	{
    		if(gpio_request(GPIO_EAR_BIAS_EN, "GPJ2"))
    			printk(KERN_ERR "Failed to request GPIO_EAR_BIAS_EN!\n");
			gpio_direction_output(GPIO_EAR_BIAS_EN, 0);
    	}
		s3c_gpio_slp_cfgpin(GPIO_EAR_BIAS_EN, S3C_GPIO_SLP_PREV);
    }

	wake_lock_init(&jack_sendend_wake_lock, WAKE_LOCK_SUSPEND, "sec_jack");

	schedule_work(&jack_detect_work);
	
	return 0;

err_request_send_end_irq:
	free_irq(det_jack->eint, 0);
err_request_detect_irq:
	switch_dev_unregister(&switch_jack_detection);
	switch_dev_unregister(&switch_dock_detection);	
	switch_dev_unregister(&switch_sendend);
err_switch_dev_register:
	input_unregister_device(input);
err_register_input_dev:
	input_free_device(input);
err_request_input_dev:
	kfree (hi);

	return ret;	
}

static int sec_jack_remove(struct platform_device *pdev)
{
	SEC_JACKDEV_DBG("");
	input_unregister_device(hi->input);
	free_irq(hi->port.det_jack.eint, 0);
	free_irq(hi->port.send_end.eint, 0);
	switch_dev_unregister(&switch_jack_detection);
	switch_dev_unregister(&switch_sendend);
	return 0;
}

#ifdef CONFIG_PM
static int sec_jack_suspend(struct platform_device *pdev, pm_message_t state)
{
	//SEC_JACKDEV_DBG("");
	//flush_scheduled_work();

	// [DE11-2132] hi99.an // sleep-current 100uA down
	if (current_jack_type_status != SEC_HEADSET_4_POLE_DEVICE)
	{
		gpio_set_value(GPIO_EAR_BIAS_EN, 0);
		printk("suspend off sub bias\n");
	}

	return 0;
}
static int sec_jack_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define s3c_headset_resume	NULL
#define s3c_headset_suspend	NULL
#endif

static int __init sec_jack_init(void)
{
	SEC_JACKDEV_DBG("");
	return platform_driver_register(&sec_jack_driver);
}

static void __exit sec_jack_exit(void)
{
	platform_driver_unregister(&sec_jack_driver);
}

static struct platform_driver sec_jack_driver = {
	.probe		= sec_jack_probe,
	.remove		= sec_jack_remove,
	.suspend	= sec_jack_suspend,
	.resume		= sec_jack_resume,
	.driver		= {
		.name		= "sec_jack",
		.owner		= THIS_MODULE,
	},
};

module_init(sec_jack_init);
module_exit(sec_jack_exit);

MODULE_AUTHOR("Uk Kim <w0806.kim@samsung.com>");
MODULE_DESCRIPTION("SEC JACK detection driver");
MODULE_LICENSE("GPL");

