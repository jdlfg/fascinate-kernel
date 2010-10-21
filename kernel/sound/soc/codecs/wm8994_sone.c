/*
 * wm8994_sone.c  --  WM8994 ALSA Soc Audio driver related Aries
 *
 *  Copyright (C) 2010 Samsung Electronics.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <plat/gpio-cfg.h>
#include <plat/map-base.h>
#include <mach/regs-clock.h>
#include <mach/gpio-aries.h>
#include "wm8994.h"

#define SUBJECT "wm8994_sone.c"

////////////////////////////////////////////////////////////////////////////////
//                           SHW-M110S: WM8994 Gain                           //
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// **** DAC **** //
#define TUNING_DAC1L_VOL                        0xC0 // 610h
#define TUNING_DAC1R_VOL                        0xC0 // 611h

////////////////////////////////////////////////////////////////////////////////
// **** RCV **** //
#define TUNING_RCV_OUTMIX5_VOL                  0x00 // 31h
#define TUNING_RCV_OUTMIX6_VOL                  0x00 // 32h
#define TUNING_RCV_OPGAL_VOL                    0x3E // 20h
#define TUNING_RCV_OPGAR_VOL                    0x3E // 21h
#define TUNING_HPOUT2_VOL                       0x00 // 1Fh

////////////////////////////////////////////////////////////////////////////////
// **** EAR **** //
// Default
#define TUNING_EAR_OUTMIX5_VOL                  0x00 // 31h
#define TUNING_EAR_OUTMIX6_VOL                  0x00 // 32h

// Playback
#define TUNING_MP3_OUTPUTL_VOL                  0x36 // 1Ch // [DE30-1522] 0x30->0x34->0x38->0x35->0x36 // H/W Req.
#define TUNING_MP3_OUTPUTR_VOL                  0x36 // 1Dh // [DE30-1522] 0x30->0x34->0x38->0x35->0x36 // H/W Req.
#define TUNING_MP3_OPGAL_VOL                    0x39 // 20h
#define TUNING_MP3_OPGAR_VOL                    0x39 // 21h

// Dual Payh
#define TUNING_MP3_DUAL_OUTPUTL_VOL             0x20 // 1Ch // [DD01-1045] 0x20 // DUAL PATH TEST
#define TUNING_MP3_DUAL_OUTPUTR_VOL             0x20 // 1Dh // [DD01-1045] 0x20 // DUAL PATH TEST
#define TUNING_DUAL_OUTPUTL_VOL                 0x20 // 1Ch
#define TUNING_DUAL_OUTPUTR_VOL                 0x20 // 1Dh

// Extra_Dock_speaker
#define TUNING_MP3_EXTRA_DOCK_SPK_OPGAL_VOL     0x39 // 20h
#define TUNING_MP3_EXTRA_DOCK_SPK_OPGAR_VOL     0x39 // 21h
#define TUNING_EXTRA_DOCK_SPK_OUTMIX5_VOL       0x00 // 31h
#define TUNING_EXTRA_DOCK_SPK_OUTMIX6_VOL       0x00 // 32h
#define TUNING_MP3_EXTRA_DOCK_SPK_VOL           0x00 // 1Eh

// Voice Call
#define TUNING_CALL_OUTPUTL_VOL                 0x33 // 1Ch // -12dB
#define TUNING_CALL_OUTPUTR_VOL                 0x33 // 1Dh
#define TUNING_CALL_OPGAL_VOL                   0x39 // 20h
#define TUNING_CALL_OPGAR_VOL                   0x39 // 21h

#define TUNING_HPOUTMIX_VOL                     0x00

////////////////////////////////////////////////////////////////////////////////
// **** SPK **** //
// Default
#define TUNING_SPKMIXL_ATTEN                    0x00 // 22h
#define TUNING_SPKMIXR_ATTEN                    0x00 // 23h

// Playback
#define TUNING_MP3_SPKL_VOL                     0x3E // 26h // [DC24-1742] 0x3B->0x3E->0x3C->0x3E // H/W Req.
#define TUNING_MP3_CLASSD_VOL                   0x06 // 25h // [DC24-1742] 0x05->0x06->0x05->0x06 // H/W Req.

// Voice Call
#define TUNING_CALL_SPKL_VOL                    0x3A // 26h
#define TUNING_CALL_CLASSD_VOL                  0x06 // 25h

////////////////////////////////////////////////////////////////////////////////
// **** MIC **** //
// Main
// Normal Recording
#define TUNING_RECORD_MAIN_INPUTMIX_VOL         0x11 // 18h // [DH13-1850] 0x16(+16.5dB)->0x11(+9dB) // H/W Req.
#define TUNING_RECORD_MAIN_AIF1ADCL_VOL         0xC0 // 400h
#define TUNING_RECORD_MAIN_AIF1ADCR_VOL         0xC0 // 401h

// Voice Recognition (Google ASR Test)
#define TUNING_RECOGNITION_MAIN_INPUTLINE_VOL   0x1F // 18h
#define TUNING_RECOGNITION_MAIN_AIF1ADCL_VOL    0xC5 // 400h
#define TUNING_RECOGNITION_MAIN_AIF1ADCR_VOL    0xC5 // 401h

// Voice Call (RCV/SPK/3Pole)
#define TUNING_CALL_RCV_INPUTMIX_VOL            0x0B // 18h
#define TUNING_CALL_SPK_INPUTMIX_VOL            0x0B // 18h

// Ear
// Normal Recording
#define TUNING_RECORD_EAR_INPUTMIX_VOL          0x17 // 1Ah
#define TUNING_RECORD_SUB_AIF1ADCL_VOL          0xC0 // 400h
#define TUNING_RECORD_SUB_AIF1ADCR_VOL          0xC0 // 401h

// Call Ear MIC
#define TUNING_CALL_EAR_INPUTMIX_VOL            0x0B // 1Ah

// Voice Recognition (Google ASR Test)
#define TUNING_RECOGNITION_SUB_INPUTMIX_VOL     0x10 // 1Ah
#define TUNING_RECOGNITION_SUB_AIF1ADCL_VOL     0xC0 // 400h
#define TUNING_RECOGNITION_SUB_AIF1ADCR_VOL     0xC0 // 401

////////////////////////////////////////////////////////////////////////////////

#define JACK_NO_DEVICE                          0x00
#define JACK_4_POLE_DEVICE                      0x01
#define JACK_3_POLE_DEVICE                      0x02
#define JACK_TVOUT_DEVICE                       0x20
#define JACK_UNKNOWN_DEVICE                     0x40

extern int hw_version_check(void);
extern short int get_headset_status(void);		// For ear-jack control(MIC-Bias)
extern void set_recording_status(int value);	// For preventing MIC Bias off on using MIC.
extern unsigned int HWREV;

int audio_init(void)
{
	/* GPIO_CODEC_LDO_EN (POWER) SETTTING */
	if (gpio_is_valid(GPIO_CODEC_LDO_EN))
	{
		if (gpio_request(GPIO_CODEC_LDO_EN, "CODEC_LDO_EN"))
			DEBUG_LOG_ERR("Failed to request CODEC_LDO_EN! \n");
		gpio_direction_output(GPIO_CODEC_LDO_EN, 0);
	}
	s3c_gpio_setpull(GPIO_CODEC_LDO_EN, S3C_GPIO_PULL_NONE);
	s3c_gpio_slp_setpull_updown(GPIO_CODEC_LDO_EN, S3C_GPIO_PULL_NONE);

	/* GPIO_CODEC_XTAL_EN (CLOCK) SETTTING */
    {
        if (gpio_is_valid(GPIO_CODEC_XTAL_EN)) {
        	if (gpio_request(GPIO_CODEC_XTAL_EN, "GPIO_CODEC_XTAL_EN")) 
    			printk(KERN_ERR "Failed to request GPIO_CODEC_XTAL_EN! \n");
    		
    		gpio_direction_output(GPIO_CODEC_XTAL_EN, 1);
    	}
    	s3c_gpio_setpull(GPIO_CODEC_XTAL_EN, S3C_GPIO_PULL_NONE);
    }
    
	/* GPIO_MICBIAS_EN SETTTING */
	// (HWREV <= REV06) MAIN & EAR MIC
	// (HWREV >= REV07) MAIN MIC
	if (gpio_is_valid(GPIO_MICBIAS_EN)) {
		if (gpio_request(GPIO_MICBIAS_EN, "GPJ4")) 
			DEBUG_LOG_ERR("Failed to request GPIO_MICBIAS_EN! \n");
		gpio_direction_output(GPIO_MICBIAS_EN, 0);
	}
	s3c_gpio_setpull(GPIO_MICBIAS_EN, S3C_GPIO_PULL_NONE);
	s3c_gpio_slp_setpull_updown(GPIO_MICBIAS_EN, S3C_GPIO_PULL_NONE);

	/* GPIO_EAR_MICBIAS_EN SETTTING */
	// (HWREV >= REV07) EAR MIC // already set in sec_jack driver
#if 0
	if (HWREV >= 7)
	{
		if (gpio_is_valid(GPIO_SUB_MICBIAS_EN)) {
			if (gpio_request(GPIO_SUB_MICBIAS_EN, "GPJ2"))
			{
				DEBUG_LOG_ERR("Failed to request GPIO_SUB_MICBIAS_EN! [ %#x ]\n", gpio_get_value(GPIO_SUB_MICBIAS_EN));
			}
			gpio_direction_output(GPIO_SUB_MICBIAS_EN, 0);
		}
		s3c_gpio_setpull(GPIO_SUB_MICBIAS_EN, S3C_GPIO_PULL_NONE);

		s3c_gpio_slp_setpull_updown(GPIO_SUB_MICBIAS_EN, S3C_GPIO_PULL_NONE);
	}
#endif

	audio_ctrl_sleep_gpio(0);
	
	return 0;

}

int audio_power(int en, int check)
{
	if(check)
	{
		DEBUG_LOG(" CONTROL audio_power() check %d en %d", check, en);
		if(en)
		{
			gpio_set_value(GPIO_CODEC_LDO_EN, 1);

			mdelay(10);

			{
				s3c_gpio_cfgpin(GPIO_CODEC_XTAL_EN, S3C_GPIO_OUTPUT);
				s3c_gpio_setpin(GPIO_CODEC_XTAL_EN, 1);
			}
		}
		else
		{
			gpio_set_value(GPIO_CODEC_LDO_EN, 0);

			mdelay(125);

			{
				s3c_gpio_cfgpin(GPIO_CODEC_XTAL_EN, S3C_GPIO_OUTPUT);
				s3c_gpio_setpin(GPIO_CODEC_XTAL_EN, 0);
			}
		}

		DEBUG_LOG(" AUDIO POWER COMPLETED : %d", en);
	}

	{
		printk(SND_KERN_DEBUG "[WM8994] gpio en %d LDO %d XTAL %d\n", en, gpio_get_value(GPIO_CODEC_LDO_EN), gpio_get_value(GPIO_CODEC_XTAL_EN));
	}

	return 0;
}

void audio_ctrl_mic_bias_gpio(struct snd_soc_codec *codec)
{
	u32 enable = 0;
	struct wm8994_priv *wm8994 = codec->private_data;
	int headset_state;

	if (wm8994->codec_state & (CAPTURE_ACTIVE|CALL_ACTIVE))
		enable = 1;

	//if ( (wm8994->cur_path == BT) || (wm8994->rec_path == BT_REC))
	if ( (wm8994->cur_path == BT) )
		enable = 0;

	if (HWREV>=7)
	{
		headset_state = get_headset_status();
		if(enable)
		{
			set_recording_status(1);
			if (headset_state == JACK_4_POLE_DEVICE)
			{
				//gpio_set_value(GPIO_MICBIAS_EN, 0);
				gpio_set_value(GPIO_SUB_MICBIAS_EN, 1);
			}
			else
			{
				gpio_set_value(GPIO_SUB_MICBIAS_EN, 0);
			}

			if ((headset_state == JACK_NO_DEVICE)||(headset_state == JACK_3_POLE_DEVICE)
				||((wm8994->cur_path == SPK)&&(wm8994->codec_state & CALL_ACTIVE)))
			{
				gpio_set_value(GPIO_MICBIAS_EN, 1);
				//gpio_set_value(GPIO_SUB_MICBIAS_EN, 0);
			}
			// [DE15-2236] hi99.an // separate on/off function call for SPK + CALL_ACTIVE + JACK_4_POLE_DEVICE
		}
		else
		{
			set_recording_status(0);
			gpio_set_value(GPIO_MICBIAS_EN, 0);
			if (headset_state != JACK_4_POLE_DEVICE)
			{
				gpio_set_value(GPIO_SUB_MICBIAS_EN, 0);
			}
		}

		printk(SND_KERN_DEBUG "en/m/s %d %d %d\n", enable, gpio_get_value(GPIO_MICBIAS_EN), gpio_get_value(GPIO_SUB_MICBIAS_EN));
	}
	else
	{
		if (gpio_get_value(GPIO_MICBIAS_EN) == enable)
		{
			DEBUG_LOG("return same value %d", enable);
			return;
		}

		if(enable)
		{
			set_recording_status(1);
			gpio_set_value(GPIO_MICBIAS_EN, 1);
		}
		else
		{
			set_recording_status(0);
			headset_state = get_headset_status();
			if(headset_state != JACK_4_POLE_DEVICE)
			{
				gpio_set_value(GPIO_MICBIAS_EN, 0);
			}
		}

		printk(SND_KERN_DEBUG "en/m %d %d\n", enable, gpio_get_value(GPIO_MICBIAS_EN));
	}
}

// If enable is false, set gpio to low on sleep
void audio_ctrl_sleep_gpio(int enable)
{
	int state;

	if(enable)
	{
		DEBUG_LOG("Set gpio to low on sleep.");
		state = S3C_GPIO_SLP_OUT0;
	}
	else
	{	
		DEBUG_LOG("Set gpio to preserve on sleep.");
		state = S3C_GPIO_SLP_PREV;
	}

	// GPIO SLEEP CONFIG // preserve output state in sleep
	s3c_gpio_slp_cfgpin(GPIO_CODEC_LDO_EN, state);
		
	s3c_gpio_slp_cfgpin(GPIO_MICBIAS_EN, state);

	s3c_gpio_slp_cfgpin(GPIO_CODEC_XTAL_EN, state);
}

void wm8994_block_control_playback(int on, struct snd_soc_codec *codec)
{
	DEBUG_LOG("++ blk play %#x", on);
	DEBUG_LOG("-- blk play %#x", on);
}

void wm8994_block_control_record(int on, struct snd_soc_codec *codec)
{
	DEBUG_LOG("++ blk rec %#x", on);
	DEBUG_LOG("-- blk rec %#x", on);
}

#define FUNCTION_LIST_DISABLE

void wm8994_disable_playback_path(struct snd_soc_codec *codec, enum playback_path path)
{
	struct wm8994_priv *wm8994 = codec->private_data;
	u16 val;

	DEBUG_LOG("Path = [%d]", path);

	switch(path)
	{
		case RCV:		
			// 01H // HPOUT2_ENA off
			val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1);
			val &= ~(WM8994_HPOUT2_ENA_MASK);
			wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

			// 2DH // DAC1L_TO_MIXOUTL off
			val = wm8994_read(codec, WM8994_OUTPUT_MIXER_1);
			val &= ~(WM8994_DAC1L_TO_MIXOUTL_MASK);
			wm8994_write(codec, WM8994_OUTPUT_MIXER_1, val);

			// 2EH // DAC1L_TO_MIXOUTL off
			val = wm8994_read(codec, WM8994_OUTPUT_MIXER_2);
			val &= ~(WM8994_DAC1R_TO_MIXOUTR_MASK);
			wm8994_write(codec, WM8994_OUTPUT_MIXER_2, val);

			// 33H // MIXOUTL/RVOL_TO_HPOUT2 off
			val = wm8994_read(codec, WM8994_HPOUT2_MIXER);
			val &= ~(WM8994_MIXOUTLVOL_TO_HPOUT2_MASK|WM8994_MIXOUTRVOL_TO_HPOUT2_MASK);
			wm8994_write(codec, WM8994_HPOUT2_MIXER, val);

			// 03H // MIXOUTL/RVOL, MIXOUTL/R off
			val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_3);
			val &= ~(WM8994_MIXOUTLVOL_ENA_MASK|WM8994_MIXOUTRVOL_ENA_MASK|WM8994_MIXOUTL_ENA_MASK|WM8994_MIXOUTR_ENA_MASK);
			wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, val);			
			break;

		case SPK:
			// 01H // SPKOUTL_ENA off
			val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1);
			val &= ~(WM8994_SPKOUTL_ENA_MASK); 
			wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

			// 03H // SPKLVOL_ENA off
			val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_3);
			val &= ~(WM8994_SPKLVOL_ENA_MASK);
			wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, val);

			// 24H // SPKMIXL/R off
			val = wm8994_read(codec, WM8994_SPKOUT_MIXERS);
			val &= ~(WM8994_SPKMIXL_TO_SPKOUTL_MASK|WM8994_SPKMIXR_TO_SPKOUTL_MASK|WM8994_SPKMIXR_TO_SPKOUTR_MASK);
			wm8994_write(codec, WM8994_SPKOUT_MIXERS, val);

			// 36H // DAC1L_TO_SPKMIXL off
			val = wm8994_read(codec, WM8994_SPEAKER_MIXER);
			val &= ~(WM8994_DAC1L_TO_SPKMIXL_MASK);
			wm8994_write(codec, WM8994_SPEAKER_MIXER, val);
			break;

		case HP:
			if(wm8994->codec_state & CALL_ACTIVE)
			{
				val = wm8994_read(codec, WM8994_DAC1_LEFT_VOLUME);
				val &= ~(0x02C0);
				val |= 0x02C0;
				wm8994_write(codec, WM8994_DAC1_LEFT_VOLUME, 0x02C0);

				val = wm8994_read(codec, WM8994_DAC1_RIGHT_VOLUME);
				val &= ~(0x02C0);
				val |= 0x02C0;
				wm8994_write(codec, WM8994_DAC1_RIGHT_VOLUME, 0x02C0);

				val = wm8994_read(codec, WM8994_ANALOGUE_HP_1);
				val &= ~(0x0022);
				val |= 0x0022;
				wm8994_write(codec, WM8994_ANALOGUE_HP_1, 0x0022);

				val = wm8994_read(codec, WM8994_OUTPUT_MIXER_1);
				val &= ~(0x0);
				val |= 0x0;
				wm8994_write(codec, WM8994_OUTPUT_MIXER_1, 0x0);

				val = wm8994_read(codec, WM8994_OUTPUT_MIXER_2);
				val &= ~(0x0);
				val |= 0x0;
				wm8994_write(codec, WM8994_OUTPUT_MIXER_2, 0x0);

				val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_5);
				val &= ~(0x0300);
				val |= 0x0300;
				wm8994_write(codec, WM8994_POWER_MANAGEMENT_5, 0x0300);

				val = wm8994_read(codec, WM8994_CHARGE_PUMP_1);
				val &= ~(0x1F25);
				val |= 0x1F25;
				wm8994_write(codec, WM8994_CHARGE_PUMP_1, 0x1F25);
			}
			break;
#if 0
		case BT :
			// 06H // AIF3 interface off
			val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_6);
			val &= ~(WM8994_AIF3_ADCDAT_SRC_MASK|WM8994_AIF2_ADCDAT_SRC_MASK|WM8994_AIF2_DACDAT_SRC_MASK|WM8994_AIF1_DACDAT_SRC_MASK);
			wm8994_write(codec, WM8994_POWER_MANAGEMENT_6, val);

			// 420H // AIF1DAC1_MUTE on
			val = wm8994_read(codec, WM8994_AIF1_DAC1_FILTERS_1);
			val &= ~(WM8994_AIF1DAC1_MUTE_MASK);
			val |= (WM8994_AIF1DAC1_MUTE);
			wm8994_write(codec, WM8994_AIF1_DAC1_FILTERS_1, val); 
			break;
#endif
		case SPK_HP :
			if(wm8994->codec_state & CALL_ACTIVE)
			{
				val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1);
				val &= ~(WM8994_HPOUT1L_ENA_MASK|WM8994_HPOUT1R_ENA_MASK|WM8994_SPKOUTL_ENA_MASK);
				wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

				// ------------------ Ear path setting ------------------
				// Disable DAC1L to HPOUT1L path
				val = wm8994_read(codec, WM8994_OUTPUT_MIXER_1);
				val &= ~(WM8994_DAC1L_TO_HPOUT1L_MASK|WM8994_DAC1L_TO_MIXOUTL_MASK);
				wm8994_write(codec, WM8994_OUTPUT_MIXER_1, val);

				// Disable DAC1R to HPOUT1R path
				val = wm8994_read(codec, WM8994_OUTPUT_MIXER_2);
				val &= ~(WM8994_DAC1R_TO_HPOUT1R_MASK|WM8994_DAC1R_TO_MIXOUTR_MASK);
				wm8994_write(codec, WM8994_OUTPUT_MIXER_2, val);

				//Disable Charge Pump	
				val = wm8994_read(codec, WM8994_CHARGE_PUMP_1);
				val &= ~WM8994_CP_ENA_MASK ;
				val |=  WM8994_CP_ENA_DEFAULT ; // this is from wolfson	
				wm8994_write(codec, WM8994_CHARGE_PUMP_1, val);

				// Intermediate HP settings
				val = wm8994_read(codec, WM8994_ANALOGUE_HP_1); 	
				val &= ~(WM8994_HPOUT1R_DLY_MASK|WM8994_HPOUT1R_OUTP_MASK|WM8994_HPOUT1R_RMV_SHORT_MASK|
					WM8994_HPOUT1L_DLY_MASK|WM8994_HPOUT1L_OUTP_MASK|WM8994_HPOUT1L_RMV_SHORT_MASK);
				wm8994_write(codec, WM8994_ANALOGUE_HP_1, val);

				// ------------------ Spk path setting ------------------		
				// Disable SPKLVOL
				val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_3);
				val &= ~(WM8994_SPKLVOL_ENA_MASK);
				wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, val);

				// Disable SPKOUT mixer
				val = wm8994_read(codec, WM8994_SPKOUT_MIXERS);
				val &= ~(WM8994_SPKMIXL_TO_SPKOUTL_MASK|WM8994_SPKMIXR_TO_SPKOUTL_MASK|WM8994_SPKMIXR_TO_SPKOUTR_MASK);
				wm8994_write(codec, WM8994_SPKOUT_MIXERS, val);

				//Mute Speaker mixer
				val = wm8994_read(codec, WM8994_SPEAKER_MIXER);
				val &= ~(WM8994_DAC1L_TO_SPKMIXL_MASK);
				wm8994_write(codec, WM8994_SPEAKER_MIXER, val);
			}
			break;

		default:
			DEBUG_LOG("Path[%d] is not invaild!\n", path);
			break;
	}

	return;
}


void wm8994_disable_rec_path(struct snd_soc_codec *codec, enum mic_path rec_path)
{
	struct wm8994_priv *wm8994 = codec->private_data;

	u16 val;

	DEBUG_LOG("");

	wm8994->rec_path = MIC_OFF;

	audio_ctrl_mic_bias_gpio(codec);

	switch(rec_path)
	{
		case MAIN:
			DEBUG_LOG("Disabling MAIN Mic Path..\n");

			if (wm8994->codec_state & (CALL_ACTIVE))
			{
				//val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_2);
				//val &= ~(WM8994_IN1L_ENA_MASK|WM8994_MIXINL_ENA_MASK);
				//wm8994_write(codec, WM8994_POWER_MANAGEMENT_2, val);

				if(!wm8994->testmode_config_flag)
				{	
					// Mute IN1L PGA, update volume
					//val = wm8994_read(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME);
					//val &= ~(WM8994_IN1L_MUTE_MASK|WM8994_IN1L_VOL_MASK);
					//val |= (WM8994_IN1L_VU|WM8994_IN1L_MUTE);
					//wm8994_write(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME, val);

					//Mute the PGA
					//val = wm8994_read(codec, WM8994_INPUT_MIXER_3);
					//val&= ~(WM8994_IN1L_TO_MIXINL_MASK|WM8994_IN1L_MIXINL_VOL_MASK|WM8994_MIXOUTL_MIXINL_VOL_MASK);
					//wm8994_write(codec, WM8994_INPUT_MIXER_3, val); 
				}

#if 0 // Voice Search Tuning
				// 410H // AIF1 ADC1 Filters // AIF1 ADC1 hi-path filter on
				val = wm8994_read(codec, WM8994_AIF1_ADC1_FILTERS);
				val &= ~(WM8994_AIF1ADC1L_HPF|WM8994_AIF1ADC1R_HPF);
				wm8994_write(codec, WM8994_AIF1_ADC1_FILTERS, 0x0000);

				// 400H // AIF1 ADC1 Left Volume
				val = wm8994_read(codec, WM8994_AIF1_ADC1_LEFT_VOLUME);
				val &= ~(WM8994_AIF1ADC1L_VOL_MASK);
				val |= (WM8994_AIF1ADC1_VU|0xC0); // ADC Digital Gain
				wm8994_write(codec, WM8994_AIF1_ADC1_LEFT_VOLUME, val);
#endif
				//Disconnect IN1LN ans IN1LP to the inputs
				//val = wm8994_read(codec, WM8994_INPUT_MIXER_2);
				//val &= (WM8994_IN1LN_TO_IN1L_MASK|WM8994_IN1LP_TO_IN1L_MASK);
				//wm8994_write(codec, WM8994_INPUT_MIXER_2, val);

				// 04H // 
				val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_4);
				val &= ~(WM8994_ADCL_ENA_MASK|WM8994_AIF1ADC1L_ENA_MASK|WM8994_AIF1ADC1R_ENA_MASK);
				wm8994_write(codec, WM8994_POWER_MANAGEMENT_4, val);

				// 606H // Disaable ADC1L_TO_AIF1ADC1L (TIMESLOT 0)
				val = wm8994_read(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING);
				val &= ~(WM8994_ADC1L_TO_AIF1ADC1L_MASK);
				wm8994_write(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING, val);

				// 607H // Disaable ADC1R_TO_AIF1ADC1R (TIMESLOT 0)
				val = wm8994_read(codec, WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING);
				val &= ~(WM8994_ADC1R_TO_AIF1ADC1R_MASK);
				wm8994_write(codec, WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING, val);

			}
			else // normal
			{
				val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_2);
				val &= ~(WM8994_IN1L_ENA_MASK|WM8994_MIXINL_ENA_MASK);
				wm8994_write(codec, WM8994_POWER_MANAGEMENT_2, val);

				if(!wm8994->testmode_config_flag)
				{	
					// Mute IN1L PGA, update volume
					val = wm8994_read(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME);
					val &= ~(WM8994_IN1L_MUTE_MASK|WM8994_IN1L_VOL_MASK);
					val |= (WM8994_IN1L_VU|WM8994_IN1L_MUTE);
					wm8994_write(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME, val);

					//Mute the PGA
					val = wm8994_read(codec, WM8994_INPUT_MIXER_3);
					val&= ~(WM8994_IN1L_TO_MIXINL_MASK|WM8994_IN1L_MIXINL_VOL_MASK|WM8994_MIXOUTL_MIXINL_VOL_MASK);
					wm8994_write(codec, WM8994_INPUT_MIXER_3, val); 
				}

#if 1 // Voice Search Tuning
				// 410H // AIF1 ADC1 Filters // AIF1 ADC1 hi-path filter on
				val = wm8994_read(codec, WM8994_AIF1_ADC1_FILTERS);
				val &= ~(WM8994_AIF1ADC1_HPF_CUT_MASK|WM8994_AIF1ADC1L_HPF|WM8994_AIF1ADC1R_HPF);
				wm8994_write(codec, WM8994_AIF1_ADC1_FILTERS, 0x0000);

				// 400H // AIF1 ADC1 Left Volume
				val = wm8994_read(codec, WM8994_AIF1_ADC1_LEFT_VOLUME);
				val &= ~(WM8994_AIF1ADC1L_VOL_MASK);
				val |= (WM8994_AIF1ADC1_VU|0xC0); // ADC Digital Gain
				wm8994_write(codec, WM8994_AIF1_ADC1_LEFT_VOLUME, val);
#endif
				//Disconnect IN1LN ans IN1LP to the inputs
				val = wm8994_read(codec, WM8994_INPUT_MIXER_2);
				val &= (WM8994_IN1LN_TO_IN1L_MASK|WM8994_IN1LP_TO_IN1L_MASK);
				wm8994_write(codec, WM8994_INPUT_MIXER_2, val);

				//Digital Paths
				val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_4);
				val &= ~(WM8994_ADCL_ENA_MASK|WM8994_AIF1ADC1L_ENA_MASK);
				wm8994_write(codec, WM8994_POWER_MANAGEMENT_4, val);

				//Disable timeslots
				val = wm8994_read(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING);
				val &= ~(WM8994_ADC1L_TO_AIF1ADC1L);
				wm8994_write(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING, val);
			}

			break;

		case SUB:
			DEBUG_LOG("Disbaling EAR Mic path..\n");

			if (wm8994->codec_state & (CALL_ACTIVE))
			{
				printk("[WM8994] Disable ear mic - in call\n");
				val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_2);
				//val &= ~(WM8994_IN1R_ENA_MASK|WM8994_MIXINR_ENA_MASK);
				val &= ~(WM8994_MIXINR_ENA_MASK);
				wm8994_write(codec, WM8994_POWER_MANAGEMENT_2, val);

				if(!wm8994->testmode_config_flag)
				{	
					// Disable volume, unmute Right Line
					//val = wm8994_read(codec, WM8994_RIGHT_LINE_INPUT_1_2_VOLUME);
					//val &= ~(WM8994_IN1R_MUTE_MASK|WM8994_IN1R_VOL_MASK);
					//val |= (WM8994_IN1R_VU|WM8994_IN1R_MUTE);
					//wm8994_write(codec, WM8994_RIGHT_LINE_INPUT_1_2_VOLUME, val);

					// Mute right pga, set volume 
					val = wm8994_read(codec, WM8994_INPUT_MIXER_4);
					val&= ~(WM8994_IN1R_TO_MIXINR_MASK|WM8994_IN1R_MIXINR_VOL_MASK|WM8994_MIXOUTR_MIXINR_VOL_MASK);
					wm8994_write(codec, WM8994_INPUT_MIXER_4, val);
				}

				//Disconnect in1rn to inr1 and in1rp to inrp
				//val = wm8994_read(codec, WM8994_INPUT_MIXER_2);
				//val &= ~(WM8994_IN1RN_TO_IN1R_MASK|WM8994_IN1RP_TO_IN1R_MASK);
				//wm8994_write(codec, WM8994_INPUT_MIXER_2, val);

				//Digital Paths 
				//Disable right ADC and time slot
				val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_4);
				val &= ~(WM8994_AIF1ADC1L_ENA_MASK|WM8994_ADCR_ENA_MASK|WM8994_AIF1ADC1R_ENA_MASK);
				wm8994_write(codec, WM8994_POWER_MANAGEMENT_4, val);

				// 606H // Disaable ADC1L_TO_AIF1ADC1L (TIMESLOT 0)
				val = wm8994_read(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING);
				val &= ~(WM8994_ADC1L_TO_AIF1ADC1L_MASK);
				wm8994_write(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING, val);

				// 607H // Disaable ADC1R_TO_AIF1ADC1R (TIMESLOT 0)
				val = wm8994_read(codec, WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING);
				val &= ~(WM8994_ADC1R_TO_AIF1ADC1R_MASK);
				wm8994_write(codec, WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING, val);
			}
			else
			{
				printk("[WM8994] Disable ear mic - normal\n");
				val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_2);
				val &= ~(WM8994_IN1R_ENA_MASK|WM8994_MIXINR_ENA_MASK);
				wm8994_write(codec, WM8994_POWER_MANAGEMENT_2, val);

				if(!wm8994->testmode_config_flag)
				{	
					// Disable volume, unmute Right Line
					val = wm8994_read(codec, WM8994_RIGHT_LINE_INPUT_1_2_VOLUME);
					val &= ~(WM8994_IN1R_MUTE_MASK|WM8994_IN1R_VOL_MASK);
					val |= (WM8994_IN1R_VU|WM8994_IN1R_MUTE);
					wm8994_write(codec, WM8994_RIGHT_LINE_INPUT_1_2_VOLUME, val);

					// Mute right pga, set volume 
					val = wm8994_read(codec, WM8994_INPUT_MIXER_4);
					val&= ~(WM8994_IN1R_TO_MIXINR_MASK|WM8994_IN1R_MIXINR_VOL_MASK|WM8994_MIXOUTR_MIXINR_VOL_MASK);
					wm8994_write(codec, WM8994_INPUT_MIXER_4, val);
				}

				//Disconnect in1rn to inr1 and in1rp to inrp
				val = wm8994_read(codec, WM8994_INPUT_MIXER_2);
				val &= ~(WM8994_IN1RN_TO_IN1R_MASK|WM8994_IN1RP_TO_IN1R_MASK);
				wm8994_write(codec, WM8994_INPUT_MIXER_2, val);

				//Digital Paths 
				//Disable right ADC and time slot
				val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_4);
				val &= ~(WM8994_ADCR_ENA_MASK|WM8994_AIF1ADC1R_ENA_MASK);
				wm8994_write(codec, WM8994_POWER_MANAGEMENT_4, val);

				//ADC Right mixer routing
				val = wm8994_read(codec, WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING);
				val &= ~(WM8994_ADC1R_TO_AIF1ADC1R_MASK);
				wm8994_write(codec, WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING, val);
			}
			break;
#if 0
		case BT_REC:
			if(wm8994->codec_state & CALL_ACTIVE)
			{
				// 02H // 
				val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_2);
				val &= ~(WM8994_MIXINL_ENA_MASK|WM8994_MIXINR_ENA_MASK);
				wm8994_write(codec, WM8994_POWER_MANAGEMENT_2, val);

				// 2CH // [DE03-1723] // IN2LRP_MIXINR_VOL 0x01(-12dB)
				val = wm8994_read(codec, WM8994_INPUT_MIXER_6);
				val &= ~(WM8994_IN2LP_MIXINR_VOL_MASK);
				wm8994_write(codec, WM8994_INPUT_MIXER_6, val);

				// 04H // AIF1ADC1L_ENA, ADCL_ENA // BT CALL = WM8994_AIF2ADCL_ENA|WM8994_ADCL_ENA
				val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_4);
				val &= ~(WM8994_AIF1ADC1L_ENA_MASK|WM8994_AIF1ADC1R_ENA_MASK|WM8994_ADCL_ENA_MASK|WM8994_ADCR_ENA_MASK);
				val |= (WM8994_AIF2ADCL_ENA|WM8994_ADCL_ENA);
				wm8994_write(codec, WM8994_POWER_MANAGEMENT_4, val);

				//R1542(606h) - 0x0000
				val = wm8994_read(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING);
				val &= ~(WM8994_AIF2DACL_TO_AIF1ADC1L_MASK|WM8994_ADC1L_TO_AIF1ADC1L_MASK);
				wm8994_write(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING, val);

				//R1543(607h) - 0x0000
				val = wm8994_read(codec, WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING);
				val &= ~(WM8994_AIF2DACR_TO_AIF1ADC1R_MASK|WM8994_ADC1R_TO_AIF1ADC1R_MASK);
				wm8994_write(codec, WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING, val);

				//R1312(520h) - 0x0200
				//val = wm8994_read(codec, WM8994_AIF2_DAC_FILTERS_1);
				//val &= ~(WM8994_AIF2DAC_MUTE_MASK);
				//val |= (WM8994_AIF2DAC_MUTE);
				//wm8994_write(codec, WM8994_AIF2_DAC_FILTERS_1, val); 
			}
			break;
#endif
		default:
			DEBUG_LOG("Path[%d] is not invaild!\n", rec_path);
			break;
	}
}

#define FUNCTION_LIST_RECORD

void wm8994_record_headset_mic(struct snd_soc_codec *codec) 
{
	struct wm8994_priv *wm8994 = codec->private_data;
	
	u16 val;

	DEBUG_LOG("");

	DEBUG_LOG("Recording through Headset Mic\n");

	if (wm8994->codec_state & (CALL_ACTIVE))
	{
		printk("[WM8994] Recording ear mic - in call\n");
		// 300H // Mixing left channel output to right channel // val: 0x0010
		//val = wm8994_read(codec, WM8994_AIF1_CONTROL_1);
		//val &= ~(WM8994_AIF1ADCL_SRC_MASK|WM8994_AIF1ADCR_SRC_MASK);
		//val |= (WM8994_AIF1ADCL_SRC|WM8994_AIF1ADCR_SRC);
		//wm8994_write(codec, WM8994_AIF1_CONTROL_1, val);

		wm8994_write(codec, WM8994_ANTIPOP_2, 0x68); // Ear mic volume issue fix		

		// 01H // VMID_SEL, BIAS_ENA
		val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1);
		val &= ~(WM8994_VMID_SEL_MASK|WM8994_BIAS_ENA_MASK);
		val |= (WM8994_VMID_SEL_NORMAL|WM8994_BIAS_ENA);
		wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

		// 700H // GPIO1 // GP1_DIR IN, GP1_PD EN, GP1_DB DE-BOUNCE, GP1_FN = LOGIC LVL 0
		wm8994_write(codec, WM8994_GPIO_1, 0xA101);

		// 02H // MIXINR_ENA, IN1R_ENA
		val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_2);
		val &= ~(WM8994_MIXINL_ENA_MASK|WM8994_MIXINR_ENA_MASK|WM8994_IN1R_ENA_MASK);
		val |= (WM8994_MIXINL_ENA|WM8994_MIXINR_ENA|WM8994_IN1R_ENA);
		wm8994_write(codec, WM8994_POWER_MANAGEMENT_2, val);

		if(!wm8994->testmode_config_flag)
		{	
			// 1AH // IN1R PGA // IN1R UNMUTE, SET VOL
			val = wm8994_read(codec, WM8994_RIGHT_LINE_INPUT_1_2_VOLUME);
			val &= ~(WM8994_IN1R_MUTE_MASK|WM8994_IN1R_VOL_MASK);
			val |= (WM8994_IN1R_VU|TUNING_CALL_EAR_INPUTMIX_VOL);
			wm8994_write(codec, WM8994_RIGHT_LINE_INPUT_1_2_VOLUME, val);

			// 2AH // MIXINR PGA // IN2R_TO_MIXINR MUTE, IN1R_TO_MIXINR UNMUTE, 0dB
			val = wm8994_read(codec, WM8994_INPUT_MIXER_4);
			val &= ~(WM8994_IN1R_TO_MIXINR_MASK|WM8994_IN1R_MIXINR_VOL_MASK|WM8994_MIXOUTR_MIXINR_VOL_MASK);
			//val |= (WM8994_IN1R_TO_MIXINR); //0db
			val |= (WM8994_IN1R_TO_MIXINR|WM8994_IN1R_MIXINR_VOL); // Boost On(+30dB)
			wm8994_write(codec, WM8994_INPUT_MIXER_4, val);
		}

		// 04H // AIF1ADC1L/R_ENA, ADCL/R_ENA
		val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_4);
		val &= ~(WM8994_AIF1ADC1L_ENA_MASK|WM8994_ADCL_ENA_MASK|WM8994_AIF1ADC1R_ENA_MASK|WM8994_ADCR_ENA_MASK);
		val |= (WM8994_AIF1ADC1L_ENA|WM8994_ADCL_ENA|WM8994_AIF1ADC1R_ENA|WM8994_ADCR_ENA);
		wm8994_write(codec, WM8994_POWER_MANAGEMENT_4, val);

		// 606H // ADC1L_TO_AIF1ADC1L (TIMESLOT 0) ASSIGN
		val = wm8994_read(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING);
		val &= ~(WM8994_ADC1L_TO_AIF1ADC1L_MASK);
		val |= (WM8994_ADC1L_TO_AIF1ADC1L);
		wm8994_write(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING, val);

		// 607H // ADC1R_TO_AIF1ADC1R (TIMESLOT 0) ASSIGN
		val = wm8994_read(codec, WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING);
		val &= ~(WM8994_ADC1R_TO_AIF1ADC1R_MASK);
		val |= (WM8994_ADC1R_TO_AIF1ADC1R);
		wm8994_write(codec, WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING, val);
	}
	else
	{
		printk("[WM8994] Recording ear mic - normal\n");
		// 300H // Mixing left channel output to right channel // val: 0x0010
		val = wm8994_read(codec, WM8994_AIF1_CONTROL_1);
		val &= ~(WM8994_AIF1ADCL_SRC_MASK|WM8994_AIF1ADCR_SRC_MASK);
		val |= (WM8994_AIF1ADCL_SRC|WM8994_AIF1ADCR_SRC);
		wm8994_write(codec, WM8994_AIF1_CONTROL_1, val);

		wm8994_write(codec, WM8994_ANTIPOP_2, 0x68); // Ear mic volume issue fix

		// 01H // VMID_SEL, BIAS_ENA
		val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1);
		val &= ~(WM8994_VMID_SEL_MASK|WM8994_BIAS_ENA_MASK);
		val |= (WM8994_VMID_SEL_NORMAL|WM8994_BIAS_ENA);
		wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

		// 700H // GPIO1 // GP1_DIR IN, GP1_PD EN, GP1_DB DE-BOUNCE, GP1_FN = LOGIC LVL 0
		wm8994_write(codec, WM8994_GPIO_1, 0xA101);

		// 02H // MIXINR_ENA, IN1R_ENA
		val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_2);
		val &= ~(WM8994_MIXINR_ENA_MASK|WM8994_IN1R_ENA_MASK);
		val |= (WM8994_MIXINR_ENA|WM8994_IN1R_ENA);
		wm8994_write(codec, WM8994_POWER_MANAGEMENT_2, val);

		if(!wm8994->testmode_config_flag)
		{	
			// 1AH // IN1R PGA // IN1R UNMUTE, SET VOL
			val = wm8994_read(codec, WM8994_RIGHT_LINE_INPUT_1_2_VOLUME);
			val &= ~(WM8994_IN1R_MUTE_MASK|WM8994_IN1R_VOL_MASK);
			if(wm8994->recognition_active == REC_ON)
				val |= (WM8994_IN1R_VU|TUNING_RECOGNITION_SUB_INPUTMIX_VOL);
			else
				val |= (WM8994_IN1R_VU|TUNING_RECORD_EAR_INPUTMIX_VOL);
			wm8994_write(codec, WM8994_RIGHT_LINE_INPUT_1_2_VOLUME, val);

			// 2AH // MIXINR PGA // IN2R_TO_MIXINR MUTE, IN1R_TO_MIXINR UNMUTE, 0dB
			val = wm8994_read(codec, WM8994_INPUT_MIXER_4);
			val &= ~(WM8994_IN1R_TO_MIXINR_MASK|WM8994_IN1R_MIXINR_VOL_MASK|WM8994_MIXOUTR_MIXINR_VOL_MASK);
			if(wm8994->recognition_active == REC_ON)
				val |= (WM8994_IN1R_TO_MIXINR|WM8994_IN1R_MIXINR_VOL); //30db
			else
				val |= (WM8994_IN1R_TO_MIXINR); //0db
			wm8994_write(codec, WM8994_INPUT_MIXER_4, val);
		}

		// 28H // INPUT MIXER // IN1RP/N_TO_IN1R PGA
		val = wm8994_read(codec, WM8994_INPUT_MIXER_2);
		val &= ~(WM8994_IN1RP_TO_IN1R_MASK|WM8994_IN1RN_TO_IN1R_MASK);
		val |= (WM8994_IN1RP_TO_IN1R|WM8994_IN1RN_TO_IN1R);
		wm8994_write(codec, WM8994_INPUT_MIXER_2, val);

		// 04H // AIF1ADC1R_ENA, ADCR_ENA
		val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_4);
		val &= ~(WM8994_AIF1ADC1R_ENA_MASK|WM8994_ADCR_ENA_MASK);
		val |= (WM8994_AIF1ADC1R_ENA|WM8994_ADCR_ENA);
		wm8994_write(codec, WM8994_POWER_MANAGEMENT_4, val);

		// 607H // ADC1R_TO_AIF1ADC1R (TIMESLOT 0) ASSIGN
		val = wm8994_read(codec, WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING);
		val &= ~(WM8994_ADC1R_TO_AIF1ADC1R_MASK);
		val |= (WM8994_ADC1R_TO_AIF1ADC1R);
		wm8994_write(codec, WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING, val);
	}

	printk(SND_KERN_DEBUG "[WM8994] wm8994_record_headset_mic()\n");
}


void wm8994_record_main_mic(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = codec->private_data;

	u16 val;

	DEBUG_LOG("");

	DEBUG_LOG("Recording through Main Mic\n");

	if (wm8994->codec_state & (VOICE_CALL_ACTIVE))
	{
		printk("[WM8994] Recording Main Mic - in voice call\n");
		// 300H // Mixing left channel output to right channel // val: 0x0010
		//val = wm8994_read(codec, WM8994_AIF1_CONTROL_1);
		//val &= ~(WM8994_AIF1ADCL_SRC_MASK|WM8994_AIF1ADCR_SRC_MASK);
		//wm8994_write(codec, WM8994_AIF1_CONTROL_1, val);

		wm8994_write(codec, WM8994_ANTIPOP_2, 0x68); // Main mic volume issue fix

		// 01H // VMID_SEL_NORMAL, BIAS_ENA, MICB1_ENA
		val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1);
		val &= ~(WM8994_VMID_SEL_MASK|WM8994_BIAS_ENA_MASK);
		val |= (WM8994_VMID_SEL_NORMAL|WM8994_BIAS_ENA);
		wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

		// 700H // GPIO1 // GP1_DIR IN, GP1_PD EN, GP1_DB DE-BOUNCE, GP1_FN = LOGIC LVL 0
		wm8994_write(codec, WM8994_GPIO_1, 0xA101);

		// 02H // MIXINL_ENA, IN1L_ENA
		val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_2);
		val &= ~(WM8994_MIXINL_ENA_MASK|WM8994_MIXINR_ENA_MASK|WM8994_IN1L_ENA_MASK);
		val |= (WM8994_MIXINL_ENA|WM8994_MIXINR_ENA|WM8994_IN1L_ENA);
		wm8994_write(codec, WM8994_POWER_MANAGEMENT_2, val);

		if(!wm8994->testmode_config_flag)
		{	
			// 18H // IN1L PGA // IN1L UNMUTE, SET VOL
			val = wm8994_read(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME);
			val &= ~(WM8994_IN1L_MUTE_MASK|WM8994_IN1L_VOL_MASK);
			val |= (WM8994_IN1L_VU|TUNING_CALL_SPK_INPUTMIX_VOL); //volume
			wm8994_write(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME, val);

			// 29H // MIXINL PGA // IN2L_TO_MIXINL MUTE, IN1L_TO_MIXINL UNMUTE, 0dB
			val = wm8994_read(codec, WM8994_INPUT_MIXER_3);
			val &= ~(WM8994_IN1L_TO_MIXINL_MASK|WM8994_IN1L_MIXINL_VOL_MASK|WM8994_MIXOUTL_MIXINL_VOL_MASK);
			//val |= (WM8994_IN1L_TO_MIXINL);	// 0db
			val |= (WM8994_IN1L_TO_MIXINL|WM8994_IN1L_MIXINL_VOL); // Boost On(+30dB)
			wm8994_write(codec, WM8994_INPUT_MIXER_3, val);
		}

		// 28H // INPUT MIXER // IN1LP/N_TO_IN1L PGA
		val = wm8994_read(codec, WM8994_INPUT_MIXER_2);	
		val &= (WM8994_IN1LP_TO_IN1L_MASK|WM8994_IN1LN_TO_IN1L_MASK);
		val |= (WM8994_IN1LP_TO_IN1L|WM8994_IN1LN_TO_IN1L);
		wm8994_write(codec, WM8994_INPUT_MIXER_2, val);

		// 04H // AIF1ADC1L_ENA, ADCL_ENA 
		val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_4);
		val &= ~(WM8994_AIF1ADC1R_ENA_MASK|WM8994_ADCR_ENA_MASK|WM8994_AIF1ADC1L_ENA_MASK|WM8994_ADCL_ENA_MASK);
		val |= (WM8994_AIF1ADC1R_ENA|WM8994_ADCR_ENA|WM8994_AIF1ADC1L_ENA|WM8994_ADCL_ENA);
		wm8994_write(codec, WM8994_POWER_MANAGEMENT_4, val);

		// 606H // ADC1L_TO_AIF1ADC1L (TIMESLOT 0) ASSIGN
		val = wm8994_read(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING);
		val &= ~(WM8994_ADC1L_TO_AIF1ADC1L_MASK);
		val |= (WM8994_ADC1L_TO_AIF1ADC1L);
		wm8994_write(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING, val);

		// 607H // ADC1R_TO_AIF1ADC1R (TIMESLOT 0) ASSIGN
		val = wm8994_read(codec, WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING);
		val &= ~(WM8994_ADC1R_TO_AIF1ADC1R_MASK);
		val |= (WM8994_ADC1R_TO_AIF1ADC1R);
		wm8994_write(codec, WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING, val);
	}
	else if (wm8994->codec_state & (VOIP_CALL_ACTIVE))
	{
		printk("[WM8994] Recording Main Mic - in voip call\n");
#if 0
		// 300H // Mixing left channel output to right channel // val: 0x0010
		//val = wm8994_read(codec, WM8994_AIF1_CONTROL_1);
		//val &= ~(WM8994_AIF1ADCL_SRC_MASK|WM8994_AIF1ADCR_SRC_MASK);
		//wm8994_write(codec, WM8994_AIF1_CONTROL_1, val);

		wm8994_write(codec, WM8994_ANTIPOP_2, 0x68); // Main mic volume issue fix

		// 01H // VMID_SEL_NORMAL, BIAS_ENA, MICB1_ENA
		val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1);
		val &= ~(WM8994_VMID_SEL_MASK|WM8994_BIAS_ENA_MASK);
		val |= (WM8994_VMID_SEL_NORMAL|WM8994_BIAS_ENA);
		wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

		// 700H // GPIO1 // GP1_DIR IN, GP1_PD EN, GP1_DB DE-BOUNCE, GP1_FN = LOGIC LVL 0
		wm8994_write(codec, WM8994_GPIO_1, 0xA101);

		// 02H // MIXINL_ENA, IN1L_ENA
		val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_2);
		val &= ~(WM8994_MIXINL_ENA_MASK|WM8994_MIXINR_ENA_MASK|WM8994_IN1L_ENA_MASK);
		val |= (WM8994_MIXINL_ENA|WM8994_MIXINR_ENA|WM8994_IN1L_ENA);
		wm8994_write(codec, WM8994_POWER_MANAGEMENT_2, val);

		if(!wm8994->testmode_config_flag)
		{	
			// 18H // IN1L PGA // IN1L UNMUTE, SET VOL
			val = wm8994_read(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME);
			val &= ~(WM8994_IN1L_MUTE_MASK|WM8994_IN1L_VOL_MASK);
			val |= (WM8994_IN1L_VU|TUNING_CALL_SPK_INPUTMIX_VOL); //volume
			wm8994_write(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME, val);

			// 29H // MIXINL PGA // IN2L_TO_MIXINL MUTE, IN1L_TO_MIXINL UNMUTE, 0dB
			val = wm8994_read(codec, WM8994_INPUT_MIXER_3);
			val &= ~(WM8994_IN1L_TO_MIXINL_MASK|WM8994_IN1L_MIXINL_VOL_MASK|WM8994_MIXOUTL_MIXINL_VOL_MASK);
			//val |= (WM8994_IN1L_TO_MIXINL);	// 0db
			val |= (WM8994_IN1L_TO_MIXINL|WM8994_IN1L_MIXINL_VOL); // Boost On(+30dB)
			wm8994_write(codec, WM8994_INPUT_MIXER_3, val);
		}

		// 28H // INPUT MIXER // IN1LP/N_TO_IN1L PGA
		val = wm8994_read(codec, WM8994_INPUT_MIXER_2);	
		val &= (WM8994_IN1LP_TO_IN1L_MASK|WM8994_IN1LN_TO_IN1L_MASK);
		val |= (WM8994_IN1LP_TO_IN1L|WM8994_IN1LN_TO_IN1L);
		wm8994_write(codec, WM8994_INPUT_MIXER_2, val);

		// 04H // AIF1ADC1L_ENA, ADCL_ENA 
		val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_4);
		val &= ~(WM8994_AIF1ADC1R_ENA_MASK|WM8994_ADCR_ENA_MASK|WM8994_AIF1ADC1L_ENA_MASK|WM8994_ADCL_ENA_MASK);
		val |= (WM8994_AIF1ADC1R_ENA|WM8994_ADCR_ENA|WM8994_AIF1ADC1L_ENA|WM8994_ADCL_ENA);
		wm8994_write(codec, WM8994_POWER_MANAGEMENT_4, val);

		// 606H // ADC1L_TO_AIF1ADC1L (TIMESLOT 0) ASSIGN
		val = wm8994_read(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING);
		val &= ~(WM8994_ADC1L_TO_AIF1ADC1L_MASK);
		val |= (WM8994_ADC1L_TO_AIF1ADC1L);
		wm8994_write(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING, val);

		// 607H // ADC1R_TO_AIF1ADC1R (TIMESLOT 0) ASSIGN
		val = wm8994_read(codec, WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING);
		val &= ~(WM8994_ADC1R_TO_AIF1ADC1R_MASK);
		val |= (WM8994_ADC1R_TO_AIF1ADC1R);
		wm8994_write(codec, WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING, val);
#endif
	}
	else
	{
		printk("[WM8994] Recording Main Mic - normal\n");

		// 300H // Mixing left channel output to right channel // val: 0x0010
		val = wm8994_read(codec, WM8994_AIF1_CONTROL_1);
		val &= ~(WM8994_AIF1ADCL_SRC_MASK|WM8994_AIF1ADCR_SRC_MASK);
		wm8994_write(codec, WM8994_AIF1_CONTROL_1, val);

		wm8994_write(codec, WM8994_ANTIPOP_2, 0x68); // Main mic volume issue fix

		// 01H // VMID_SEL_NORMAL, BIAS_ENA, MICB1_ENA
		val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1);
		val &= ~(WM8994_VMID_SEL_MASK|WM8994_BIAS_ENA_MASK);
		val |= (WM8994_VMID_SEL_NORMAL|WM8994_BIAS_ENA);
		wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

		// 700H // GPIO1 // GP1_DIR IN, GP1_PD EN, GP1_DB DE-BOUNCE, GP1_FN = LOGIC LVL 0
		wm8994_write(codec, WM8994_GPIO_1, 0xA101);

		// 02H // MIXINL_ENA, IN1L_ENA
		val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_2);
		val &= ~(WM8994_MIXINL_ENA_MASK|WM8994_IN1L_ENA_MASK);
		val |= (WM8994_MIXINL_ENA|WM8994_IN1L_ENA);
		wm8994_write(codec, WM8994_POWER_MANAGEMENT_2, val);

		if(!wm8994->testmode_config_flag)
		{	
			// 18H // IN1L PGA // IN1L UNMUTE, SET VOL
			val = wm8994_read(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME);
			val &= ~(WM8994_IN1L_MUTE_MASK|WM8994_IN1L_VOL_MASK);
			if(wm8994->recognition_active == REC_ON)
				val |= (WM8994_IN1L_VU|TUNING_RECOGNITION_MAIN_INPUTLINE_VOL); //volume
			else
				val |= (WM8994_IN1L_VU|TUNING_RECORD_MAIN_INPUTMIX_VOL);
			wm8994_write(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME, val);

			// 29H // MIXINL PGA // IN2L_TO_MIXINL MUTE, IN1L_TO_MIXINL UNMUTE, 0dB
			val = wm8994_read(codec, WM8994_INPUT_MIXER_3);
			val &= ~(WM8994_IN1L_TO_MIXINL_MASK|WM8994_IN1L_MIXINL_VOL_MASK|WM8994_MIXOUTL_MIXINL_VOL_MASK);
			if(wm8994->recognition_active == REC_ON)		
				val |= (WM8994_IN1L_TO_MIXINL);	// 0db
			else
				val |= (WM8994_IN1L_TO_MIXINL|WM8994_IN1L_MIXINL_VOL); // Boost On(+30dB)
			wm8994_write(codec, WM8994_INPUT_MIXER_3, val);
		}

		if(wm8994->recognition_active == REC_ON)
		{
			// Voice Search Tuning
			// 400H // AIF1 ADC1 Left Volume
			val = wm8994_read(codec, WM8994_AIF1_ADC1_LEFT_VOLUME);
			val &= ~(WM8994_AIF1ADC1L_VOL_MASK);
			val |= (WM8994_AIF1ADC1_VU|TUNING_RECOGNITION_MAIN_AIF1ADCL_VOL); // ADC Digital Gain
			wm8994_write(codec, WM8994_AIF1_ADC1_LEFT_VOLUME, val);
		}

		// 28H // INPUT MIXER // IN1LP/N_TO_IN1L PGA
		val = wm8994_read(codec, WM8994_INPUT_MIXER_2);	
		val &= (WM8994_IN1LP_TO_IN1L_MASK|WM8994_IN1LN_TO_IN1L_MASK);
		val |= (WM8994_IN1LP_TO_IN1L|WM8994_IN1LN_TO_IN1L);
		wm8994_write(codec, WM8994_INPUT_MIXER_2, val);

		if(wm8994->recognition_active == REC_ON)
		{
			// 410H // AIF1 ADC1 Filters // AIF1 ADC1 hi-path filter on
			val = wm8994_read(codec, WM8994_AIF1_ADC1_FILTERS);
			val &= ~(WM8994_AIF1ADC1L_HPF|WM8994_AIF1ADC1R_HPF);
			val |= (WM8994_AIF1ADC1L_HPF|WM8994_AIF1ADC1R_HPF); // hi-path filter on (L/R)
			wm8994_write(codec, WM8994_AIF1_ADC1_FILTERS, 0x3000);
		}

		// 04H // AIF1ADC1L_ENA, ADCL_ENA 
		val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_4);
		val &= ~(WM8994_AIF1ADC1L_ENA_MASK|WM8994_ADCL_ENA_MASK);
		val |= (WM8994_AIF1ADC1L_ENA|WM8994_ADCL_ENA);
		wm8994_write(codec, WM8994_POWER_MANAGEMENT_4, val);

		// 606H // ADC1L_TO_AIF1ADC1L (TIMESLOT 0) ASSIGN
		val = wm8994_read(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING);
		val &= ~(WM8994_ADC1L_TO_AIF1ADC1L_MASK);
		val |= (WM8994_ADC1L_TO_AIF1ADC1L);
		wm8994_write(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING, val);
	}

	printk(SND_KERN_DEBUG "[WM8994] wm8994_record_main_mic()\n");
}

void wm8994_record_bluetooth_mic(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = codec->private_data;

	u16 val;

	DEBUG_LOG("");

	DEBUG_LOG("Recording through Bluetooth mic\n");

	if (wm8994->codec_state & (CALL_ACTIVE))
	{
		printk("[WM8994] Recording Bluetooth mic - in call\n");

		// 01H // VMID_SEL_NORMAL, BIAS_ENA, MICB1_ENA
		val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1);
		val &= ~(WM8994_VMID_SEL_MASK|WM8994_BIAS_ENA_MASK);
		val |= (WM8994_VMID_SEL_NORMAL|WM8994_BIAS_ENA);
		wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

		// 700H // GPIO1 // GP1_DIR IN, GP1_PD EN, GP1_DB DE-BOUNCE, GP1_FN = LOGIC LVL 0
		wm8994_write(codec, WM8994_GPIO_1, 0xA101);

		// 02H // 
		val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_2);
		val &= ~(WM8994_MIXINL_ENA_MASK|WM8994_MIXINR_ENA_MASK);
		val |= (WM8994_MIXINL_ENA|WM8994_MIXINR_ENA);
		wm8994_write(codec, WM8994_POWER_MANAGEMENT_2, val);

		// 04H // AIF1ADC1L_ENA, ADCL_ENA // BT CALL = WM8994_AIF2ADCL_ENA|WM8994_ADCL_ENA
		val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_4);
		val &= ~(WM8994_AIF1ADC1L_ENA_MASK|WM8994_AIF1ADC1R_ENA_MASK|WM8994_ADCL_ENA_MASK|WM8994_ADCR_ENA_MASK);
		val |= (WM8994_AIF1ADC1L_ENA|WM8994_AIF1ADC1R_ENA|WM8994_ADCL_ENA|WM8994_ADCR_ENA);
		wm8994_write(codec, WM8994_POWER_MANAGEMENT_4, val);

		//R1542(606h) - 0x0000
		val = wm8994_read(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING);
		val &= ~(WM8994_AIF2DACL_TO_AIF1ADC1L_MASK|WM8994_ADC1L_TO_AIF1ADC1L_MASK);
		val |= (WM8994_AIF2DACL_TO_AIF1ADC1L|WM8994_ADC1L_TO_AIF1ADC1L);
		wm8994_write(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING, val);

		//R1543(607h) - 0x0000
		val = wm8994_read(codec, WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING);
		val &= ~(WM8994_AIF2DACR_TO_AIF1ADC1R_MASK|WM8994_ADC1R_TO_AIF1ADC1R_MASK);
		val |= (WM8994_AIF2DACR_TO_AIF1ADC1R|WM8994_ADC1R_TO_AIF1ADC1R);
		wm8994_write(codec, WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING, val);
	}
	else
	{
		printk("[WM8994] Recording Bluetooth mic - normal\n");
	}

	printk(SND_KERN_DEBUG "[WM8994] wm8994_record_bluetooth_mic()\n");
}

/*
void wm8994_record_bluetooth(struct snd_soc_codec *codec)
{
	u16 val;

	DEBUG_LOG("BT Record Path for Voice Command\n");

	//R1(1h) - 0x0003 -normal vmid
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1);
	val &= ~(WM8994_BIAS_ENA_MASK|WM8994_VMID_SEL_MASK);
	val |= (WM8994_BIAS_ENA|WM8994_VMID_SEL_NORMAL);	
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);
	
	// Digital Path Enables and Unmutes
	//R2(2h) - 0x0000
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_2, 0x0000);

	//R3(3h) - 0x0000
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, 0x0000);

	//R4(4h) - 0x0300
	//AIF1ADC1(Left/Right) Output
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_4);
	val &= ~(WM8994_AIF1ADC1L_ENA_MASK |WM8994_AIF1ADC1R_ENA_MASK);
	val |= ( WM8994_AIF1ADC1L_ENA|WM8994_AIF1ADC1R_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_4, val);

	//R5(5h) - 0x3000
	//AIF2DAC(Left/Right) Input
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_5);
	val &= ~(WM8994_AIF2DACL_ENA_MASK|WM8994_AIF2DACR_ENA_MASK);
	val |= (WM8994_AIF2DACL_ENA|WM8994_AIF2DACR_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_5, val);
	
	//R6(6h) - 0x0002
	//AIF2_DACDAT_SRC(GPIO8/DACDAT3)
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_6);
	val &= ~(WM8994_AIF2_DACDAT_SRC_MASK);
	val |= (WM8994_AIF2_DACDAT_SRC);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_6, val);

	// Un-Mute
	//R1312(520h) - 0x0000
	//AIF2DAC_MUTE(Un-mute)
	val = wm8994_read(codec, WM8994_AIF2_DAC_FILTERS_1);
	val &= ~(WM8994_AIF2DAC_MUTE_MASK);
	wm8994_write(codec, WM8994_AIF2_DAC_FILTERS_1, val); 

	// Mixer Routing
	//R1542(606h) - 0x0002
	//AIF2DACL_TO_AIF1ADC1L
	val = wm8994_read(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING);
	val &= ~(WM8994_AIF2DACL_TO_AIF1ADC1L_MASK);
	val |= (WM8994_AIF2DACL_TO_AIF1ADC1L);
	wm8994_write(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING, val);
	
	//R1543(607h) - 0x0002
	//AIF2DACR_TO_AIF1ADC1R
	val = wm8994_read(codec, WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING);
	val &= ~(WM8994_AIF2DACR_TO_AIF1ADC1R_MASK);
	val |= (WM8994_AIF2DACR_TO_AIF1ADC1R);
	wm8994_write(codec, WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING, val);

	wm8994_write(codec, WM8994_OVERSAMPLING, 0X0000);

	wm8994_set_voicecall_common_setting(codec);

	// GPIO Configuration
	wm8994_write(codec, WM8994_GPIO_8, WM8994_GP8_DIR|WM8994_GP8_DB);
	wm8994_write(codec, WM8994_GPIO_9, WM8994_GP9_DB);
	wm8994_write(codec, WM8994_GPIO_10, WM8994_GP10_DB);
	wm8994_write(codec, WM8994_GPIO_11, WM8994_GP11_DB); 
}
*/

#define FUNCTION_LIST_PLAYBACK

void wm8994_set_playback_receiver(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = codec->private_data;

	u16 val;

	DEBUG_LOG("");

	if(!wm8994->testmode_config_flag)
	{
	    val = wm8994_read(codec, WM8994_OUTPUT_MIXER_5);
		val &= ~(WM8994_DACL_MIXOUTL_VOL_MASK);
		val |= (TUNING_RCV_OUTMIX5_VOL<<WM8994_DACL_MIXOUTL_VOL_SHIFT);
		wm8994_write(codec, WM8994_OUTPUT_MIXER_5, val);
	
		val = wm8994_read(codec, WM8994_OUTPUT_MIXER_6);
		val &= ~(WM8994_DACR_MIXOUTR_VOL_MASK);
		val |= (TUNING_RCV_OUTMIX6_VOL<<WM8994_DACR_MIXOUTR_VOL_SHIFT);
		wm8994_write(codec, WM8994_OUTPUT_MIXER_6, val);

		val = wm8994_read(codec, WM8994_LEFT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTL_MUTE_N_MASK|WM8994_MIXOUTL_VOL_MASK);
		val |= (WM8994_MIXOUT_VU|WM8994_MIXOUTL_MUTE_N|TUNING_RCV_OPGAL_VOL);
		wm8994_write(codec, WM8994_LEFT_OPGA_VOLUME, val);

		val = wm8994_read(codec, WM8994_RIGHT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTR_MUTE_N_MASK|WM8994_MIXOUTR_VOL_MASK);
		val |= (WM8994_MIXOUT_VU|WM8994_MIXOUTR_MUTE_N|TUNING_RCV_OPGAR_VOL);
		wm8994_write(codec, WM8994_RIGHT_OPGA_VOLUME, val);
	
        val = wm8994_read(codec, WM8994_HPOUT2_VOLUME);
		val &= ~(WM8994_HPOUT2_MUTE_MASK|WM8994_HPOUT2_VOL_MASK);
        val |= (TUNING_HPOUT2_VOL<<WM8994_HPOUT2_VOL_SHIFT);
		wm8994_write(codec, WM8994_HPOUT2_VOLUME, val);

		//Unmute DAC1 left
		val = wm8994_read(codec, WM8994_DAC1_LEFT_VOLUME);
		val &= ~(WM8994_DAC1L_MUTE_MASK|WM8994_DAC1L_VOL_MASK);
		val |= (WM8994_DAC1_VU|TUNING_DAC1L_VOL);
		wm8994_write(codec, WM8994_DAC1_LEFT_VOLUME, val);
	
		//Unmute and volume ctrl RightDAC
		val = wm8994_read(codec, WM8994_DAC1_RIGHT_VOLUME); 
		val &= ~(WM8994_DAC1R_MUTE_MASK|WM8994_DAC1R_VOL_MASK);
		val |= (WM8994_DAC1_VU|TUNING_DAC1R_VOL);
		wm8994_write(codec, WM8994_DAC1_RIGHT_VOLUME, val);
	}

	val = wm8994_read(codec, WM8994_OUTPUT_MIXER_1);
	val &= ~(WM8994_DAC1L_TO_MIXOUTL_MASK);
	val |= (WM8994_DAC1L_TO_MIXOUTL);
	wm8994_write(codec, WM8994_OUTPUT_MIXER_1, val);

	val = wm8994_read(codec, WM8994_OUTPUT_MIXER_2);
	val &= ~(WM8994_DAC1R_TO_MIXOUTR_MASK);
	val |= (WM8994_DAC1R_TO_MIXOUTR);
	wm8994_write(codec, WM8994_OUTPUT_MIXER_2, val);

	val = wm8994_read(codec, WM8994_HPOUT2_MIXER);
	val &= ~(WM8994_MIXOUTLVOL_TO_HPOUT2_MASK|WM8994_MIXOUTRVOL_TO_HPOUT2_MASK);
	val |= (WM8994_MIXOUTRVOL_TO_HPOUT2|WM8994_MIXOUTLVOL_TO_HPOUT2);
	wm8994_write(codec, WM8994_HPOUT2_MIXER, val);

	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_5);
	//val &= ~(WM8994_DAC1R_ENA_MASK|WM8994_DAC1L_ENA_MASK|WM8994_AIF1DAC1R_ENA_MASK|WM8994_AIF1DAC1L_ENA_MASK);
	//val |= (WM8994_AIF1DAC1L_ENA|WM8994_AIF1DAC1R_ENA|WM8994_DAC1L_ENA|WM8994_DAC1R_ENA);
	val &= ~(WM8994_AIF1DAC1L_ENA_MASK|WM8994_AIF1DAC1R_ENA_MASK|WM8994_DAC1L_ENA_MASK);
	val |= (WM8994_AIF1DAC1L_ENA|WM8994_AIF1DAC1R_ENA|WM8994_DAC1L_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_5, val);

	//val = wm8994_read(codec, WM8994_AIF1_DAC1_FILTERS_1);
	//val &= ~(WM8994_AIF1DAC1_MUTE_MASK);
	val = wm8994_read(codec, WM8994_AIF1_DAC1_FILTERS_1);
	val &= ~(WM8994_AIF1DAC1_MUTE_MASK|WM8994_AIF1DAC1_MONO_MASK);
	val |= (WM8994_AIF1DAC1_UNMUTE|WM8994_AIF1DAC1_MONO);
	wm8994_write(codec, WM8994_AIF1_DAC1_FILTERS_1, val);

	val = wm8994_read(codec, WM8994_DAC1_LEFT_MIXER_ROUTING);
	val &= ~(WM8994_AIF1DAC1L_TO_DAC1L_MASK);
	val |= (WM8994_AIF1DAC1L_TO_DAC1L);
	wm8994_write(codec, WM8994_DAC1_LEFT_MIXER_ROUTING, val);

	//val = wm8994_read(codec, WM8994_DAC1_RIGHT_MIXER_ROUTING);
	//val &= ~(WM8994_AIF1DAC1R_TO_DAC1R_MASK);
	//val |= (WM8994_AIF1DAC1R_TO_DAC1R);
	//wm8994_write(codec, WM8994_DAC1_RIGHT_MIXER_ROUTING, val);

	val = wm8994_read(codec, WM8994_CLOCKING_1);
	val &= ~(WM8994_DSP_FS1CLK_ENA_MASK|WM8994_DSP_FSINTCLK_ENA_MASK);
	val |= (WM8994_DSP_FS1CLK_ENA|WM8994_DSP_FSINTCLK_ENA);
	wm8994_write(codec, WM8994_CLOCKING_1, val);

	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_3);
	val &= ~(WM8994_MIXOUTLVOL_ENA_MASK|WM8994_MIXOUTRVOL_ENA_MASK|WM8994_MIXOUTL_ENA_MASK|WM8994_MIXOUTR_ENA_MASK);
	val |= (WM8994_MIXOUTL_ENA|WM8994_MIXOUTR_ENA|WM8994_MIXOUTRVOL_ENA|WM8994_MIXOUTLVOL_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, val);

	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1);
	val &= ~(WM8994_BIAS_ENA_MASK|WM8994_VMID_SEL_MASK|WM8994_HPOUT2_ENA_MASK);
	val |= (WM8994_BIAS_ENA|WM8994_VMID_SEL_NORMAL|WM8994_HPOUT2_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

	printk(SND_KERN_DEBUG "[WM8994] wm8994_set_playback_receiver()\n");
}


void wm8994_set_playback_headset(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = codec->private_data;

	u16 val;

#if 1
	u16 TestReturn1 = 0;
	u16 TestReturn2 = 0;
	u16 TestLow1 = 0;
	u16 TestHigh1 = 0;
	u8 TestLow = 0;
	u8 TestHigh = 0;

	DEBUG_LOG("");

	//Configuring the Digital Paths
	// Enable the Timeslot0 to DAC1L
	val = wm8994_read(codec, WM8994_DAC1_LEFT_MIXER_ROUTING);
	val &= ~(WM8994_AIF1DAC1L_TO_DAC1L_MASK);
	val |= WM8994_AIF1DAC1L_TO_DAC1L;
	wm8994_write(codec, WM8994_DAC1_LEFT_MIXER_ROUTING, val);

	//Enable the Timeslot0 to DAC1R
	val = wm8994_read(codec, WM8994_DAC1_RIGHT_MIXER_ROUTING);
	val &= ~(WM8994_AIF1DAC1R_TO_DAC1R_MASK);
	val |= WM8994_AIF1DAC1R_TO_DAC1R;
	wm8994_write(codec, WM8994_DAC1_RIGHT_MIXER_ROUTING, val);

	val = wm8994_read(codec, 0x102);
	val &= ~(0x0003);
	val = (0x0003);
	wm8994_write(codec,0x102, val);
	
	val = wm8994_read(codec, 0x56);
	val &= ~(0x0003);
	val = (0x0003);
	wm8994_write(codec,0x56, val);
	
	val = wm8994_read(codec, 0x102);
	val &= ~(0x0000);
	val = (0x0000);
	wm8994_write(codec,0x102, val);

	val = wm8994_read(codec, WM8994_CLASS_W_1);
	val &= ~(0x0005);
	val |= (0x0005);
	wm8994_write(codec, WM8994_CLASS_W_1, val);

	// Headset Control
	if(!wm8994->testmode_config_flag)
	{
		val = wm8994_read(codec, WM8994_LEFT_OUTPUT_VOLUME);
		val &= ~(WM8994_HPOUT1L_MUTE_N_MASK|WM8994_HPOUT1L_VOL_MASK);
		val |= (WM8994_HPOUT1_VU|WM8994_HPOUT1L_MUTE_N|TUNING_MP3_OUTPUTL_VOL);
		wm8994_write(codec, WM8994_LEFT_OUTPUT_VOLUME, val);

		val = wm8994_read(codec, WM8994_RIGHT_OUTPUT_VOLUME);
		val &= ~(WM8994_HPOUT1R_MUTE_N_MASK|WM8994_HPOUT1R_VOL_MASK);
		val |= (WM8994_HPOUT1_VU|WM8994_HPOUT1R_MUTE_N|TUNING_MP3_OUTPUTR_VOL);
		wm8994_write(codec, WM8994_RIGHT_OUTPUT_VOLUME, val);

		val = wm8994_read(codec, WM8994_LEFT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTL_MUTE_N_MASK|WM8994_MIXOUTL_VOL_MASK);
		val |= (WM8994_MIXOUT_VU|WM8994_MIXOUTL_MUTE_N|TUNING_MP3_OPGAL_VOL);
		wm8994_write(codec, WM8994_LEFT_OPGA_VOLUME, val);
	
		val = wm8994_read(codec, WM8994_RIGHT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTR_MUTE_N_MASK|WM8994_MIXOUTR_VOL_MASK);
		val |= (WM8994_MIXOUT_VU|WM8994_MIXOUTR_MUTE_N|TUNING_MP3_OPGAR_VOL);
		wm8994_write(codec, WM8994_RIGHT_OPGA_VOLUME, val);
	}
	
	val = wm8994_read(codec, WM8994_DC_SERVO_2);
	val &= ~(0x03E0);
	val = (0x03E0);
	wm8994_write(codec, WM8994_DC_SERVO_2, val);

	//Enable vmid,bias, hp left and right
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1);
	val &= ~(WM8994_BIAS_ENA_MASK|WM8994_VMID_SEL_MASK|WM8994_HPOUT1L_ENA_MASK|WM8994_HPOUT1R_ENA_MASK);
	val |= (WM8994_BIAS_ENA|WM8994_VMID_SEL_NORMAL|WM8994_HPOUT1R_ENA|WM8994_HPOUT1L_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

	val = wm8994_read(codec, WM8994_ANALOGUE_HP_1);
	val &= ~(0x0022);
	val = 0x0022;
	wm8994_write(codec, WM8994_ANALOGUE_HP_1, val);

	//Enable Charge Pump	
	val = wm8994_read(codec, WM8994_CHARGE_PUMP_1);
	val &= ~(WM8994_CP_ENA_MASK);
	val |= (WM8994_CP_ENA|WM8994_CP_ENA_DEFAULT); // this is from wolfson
	wm8994_write(codec, WM8994_CHARGE_PUMP_1, val);

	msleep(5);// 20ms delay

	//Enable Dac1 and DAC2 and the Timeslot0 for AIF1	
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_5);
	val &= ~(WM8994_DAC1R_ENA_MASK|WM8994_DAC1L_ENA_MASK|WM8994_AIF1DAC1R_ENA_MASK| WM8994_AIF1DAC1L_ENA_MASK);
	val |= (WM8994_AIF1DAC1L_ENA|WM8994_AIF1DAC1R_ENA|WM8994_DAC1L_ENA|WM8994_DAC1R_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_5, val);

	// Enable DAC1L to HPOUT1L path
	val = wm8994_read(codec, WM8994_OUTPUT_MIXER_1);
	val &=  ~(WM8994_DAC1L_TO_HPOUT1L_MASK|WM8994_DAC1L_TO_MIXOUTL_MASK);
	val |= (WM8994_DAC1L_TO_MIXOUTL);  	
	wm8994_write(codec, WM8994_OUTPUT_MIXER_1, val);

	// Enable DAC1R to HPOUT1R path
	val = wm8994_read(codec, WM8994_OUTPUT_MIXER_2);
	val &= ~(WM8994_DAC1R_TO_HPOUT1R_MASK|WM8994_DAC1R_TO_MIXOUTR_MASK);
	val |= (WM8994_DAC1R_TO_MIXOUTR);
	wm8994_write(codec, WM8994_OUTPUT_MIXER_2, val);

	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_3);
	val &= ~(WM8994_MIXOUTLVOL_ENA_MASK|WM8994_MIXOUTRVOL_ENA_MASK|WM8994_MIXOUTL_ENA_MASK|WM8994_MIXOUTR_ENA_MASK);
	val |= (WM8994_MIXOUTLVOL_ENA|WM8994_MIXOUTRVOL_ENA|WM8994_MIXOUTL_ENA|WM8994_MIXOUTR_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, 0x0030);

	val = wm8994_read(codec, WM8994_DC_SERVO_1);
	val &= ~(0x0303);
	val = (0x0303);
	wm8994_write(codec, WM8994_DC_SERVO_1, val);

	msleep(160); // 160ms delay

	TestReturn1 = wm8994_read(codec, WM8994_DC_SERVO_4);

	TestLow=(signed char)(TestReturn1 & 0xff);
	TestHigh=(signed char)((TestReturn1>>8)&0xff);
	TestLow1 = ((signed short)(TestLow-5))&0x00ff;
	TestHigh1 = (((signed short)(TestHigh-5))<<8)&0xff00;
	TestReturn2 = (TestLow1|TestHigh1);
	wm8994_write(codec, WM8994_DC_SERVO_4, TestReturn2);

	val = wm8994_read(codec, WM8994_DC_SERVO_1);
	val &= ~(0x000F);
	val = (0x000F);
	wm8994_write(codec, WM8994_DC_SERVO_1, val);

	msleep(20);

	// Intermediate HP settings
	val = wm8994_read(codec, WM8994_ANALOGUE_HP_1);
	val &= ~(WM8994_HPOUT1R_DLY_MASK|WM8994_HPOUT1R_OUTP_MASK|WM8994_HPOUT1R_RMV_SHORT_MASK|
		WM8994_HPOUT1L_DLY_MASK|WM8994_HPOUT1L_OUTP_MASK|WM8994_HPOUT1L_RMV_SHORT_MASK);
	val = (WM8994_HPOUT1L_RMV_SHORT|WM8994_HPOUT1L_OUTP|WM8994_HPOUT1L_DLY|WM8994_HPOUT1R_RMV_SHORT|
		WM8994_HPOUT1R_OUTP|WM8994_HPOUT1R_DLY);
	wm8994_write(codec, WM8994_ANALOGUE_HP_1, val);

	//Unmute DAC1 left
	val = wm8994_read(codec, WM8994_DAC1_LEFT_VOLUME);
	val &= ~(WM8994_DAC1L_MUTE_MASK|WM8994_DAC1L_VOL_MASK);
	val |= (TUNING_DAC1L_VOL);
	wm8994_write(codec, WM8994_DAC1_LEFT_VOLUME, val);

	//Unmute and volume ctrl RightDAC
	val = wm8994_read(codec, WM8994_DAC1_RIGHT_VOLUME);
	val &= ~(WM8994_DAC1R_MUTE_MASK|WM8994_DAC1R_VOL_MASK);
	val |= (TUNING_DAC1R_VOL); //0 db volume
	wm8994_write(codec, WM8994_DAC1_RIGHT_VOLUME, val);

	// Unmute the AF1DAC1	
	val = wm8994_read(codec, WM8994_AIF1_DAC1_FILTERS_1);
	val &= ~(WM8994_AIF1DAC1_MUTE_MASK|WM8994_AIF1DAC1_MONO_MASK);
	val |= (WM8994_AIF1DAC1_UNMUTE);
	wm8994_write(codec, WM8994_AIF1_DAC1_FILTERS_1, val);
#endif
	printk(SND_KERN_DEBUG "[WM8994] wm8994_set_playback_headset()\n");
}

void wm8994_set_playback_speaker(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = codec->private_data;

	u16 val;

	DEBUG_LOG("");
	DEBUG_LOG("testmode_config_flag = [%d]",wm8994->testmode_config_flag);

	//Disable end point for preventing pop up noise.
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1);
	val &= ~(WM8994_SPKOUTL_ENA_MASK);
	//wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, 0x0003);

	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_3);
	val &= ~(WM8994_SPKLVOL_ENA_MASK);
	val |= (WM8994_SPKLVOL_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, val);

	// Speaker Volume Control
	if(!wm8994->testmode_config_flag)
	{
		// Unmute the SPKMIXVOLUME
		val = wm8994_read(codec, WM8994_SPKMIXL_ATTENUATION);
		val &= ~(WM8994_SPKMIXL_VOL_MASK);
		val |= (TUNING_SPKMIXL_ATTEN);
		wm8994_write(codec, WM8994_SPKMIXL_ATTENUATION, val);
		
		val = wm8994_read(codec, WM8994_SPKMIXR_ATTENUATION);
		val &= ~(WM8994_SPKMIXR_VOL_MASK);
		//val |= (TUNING_SPKMIXR_ATTEN);
		wm8994_write(codec, WM8994_SPKMIXR_ATTENUATION, val);

		val = wm8994_read(codec, WM8994_SPEAKER_VOLUME_LEFT);
		val &= ~(WM8994_SPKOUTL_MUTE_N_MASK|WM8994_SPKOUTL_VOL_MASK);
		val |= (WM8994_SPKOUT_VU|WM8994_SPKOUTL_MUTE_N|TUNING_MP3_SPKL_VOL);
		wm8994_write(codec, WM8994_SPEAKER_VOLUME_LEFT, val);

		val = wm8994_read(codec, WM8994_SPEAKER_VOLUME_RIGHT);
		val &= ~(WM8994_SPKOUTR_MUTE_N_MASK|WM8994_SPKOUTR_VOL_MASK);
		//val |= (WM8994_SPKOUT_VU|WM8994_SPKOUTR_MUTE_N|TUNING_MP3_SPKL_VOL);
		wm8994_write(codec, WM8994_SPEAKER_VOLUME_RIGHT, val);

		val = wm8994_read(codec, WM8994_CLASSD);
		val &= ~(WM8994_SPKOUTL_BOOST_MASK);
		val |= (TUNING_MP3_CLASSD_VOL<<WM8994_SPKOUTL_BOOST_SHIFT);
		wm8994_write(codec, WM8994_CLASSD, val);
	
		//Unmute DAC1 left
		val = wm8994_read(codec, WM8994_DAC1_LEFT_VOLUME );
		val &= ~(WM8994_DAC1L_MUTE_MASK|WM8994_DAC1L_VOL_MASK);
		val |= (TUNING_DAC1L_VOL); 
		wm8994_write(codec, WM8994_DAC1_LEFT_VOLUME, val);
	
		//Unmute and volume ctrl RightDAC
		val = wm8994_read(codec, WM8994_DAC1_RIGHT_VOLUME ); 
		val &= ~(WM8994_DAC1R_MUTE_MASK|WM8994_DAC1R_VOL_MASK);
		val |= (WM8994_DAC1_VU|TUNING_DAC1R_VOL); //0 db volume	
		wm8994_write(codec, WM8994_DAC1_RIGHT_VOLUME, val);

	}

	val = wm8994_read(codec, WM8994_SPKOUT_MIXERS);
	val &= ~(WM8994_SPKMIXL_TO_SPKOUTL_MASK|
		WM8994_SPKMIXL_TO_SPKOUTR_MASK|WM8994_SPKMIXR_TO_SPKOUTR_MASK);
	val |= (WM8994_SPKMIXL_TO_SPKOUTL);
	wm8994_write(codec, WM8994_SPKOUT_MIXERS, val);

	//Unmute the DAC path
	val = wm8994_read(codec, WM8994_SPEAKER_MIXER);
	val &= ~(WM8994_DAC1L_TO_SPKMIXL_MASK);
	val |= (WM8994_DAC1L_TO_SPKMIXL);
	wm8994_write(codec, WM8994_SPEAKER_MIXER, val);

	// Eable DAC1 Left and timeslot left
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_5);
	val &= ~(WM8994_AIF1DAC1L_ENA_MASK|WM8994_AIF1DAC1R_ENA_MASK|WM8994_DAC1L_ENA_MASK);
	val |= (WM8994_AIF1DAC1L_ENA|WM8994_AIF1DAC1R_ENA|WM8994_DAC1L_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_5, val);

	//Unmute
	val = wm8994_read(codec, WM8994_AIF1_DAC1_FILTERS_1);
	val &= ~(WM8994_AIF1DAC1_MUTE_MASK|WM8994_AIF1DAC1_MONO_MASK);
	val |= (WM8994_AIF1DAC1_UNMUTE|WM8994_AIF1DAC1_MONO);
	wm8994_write(codec, WM8994_AIF1_DAC1_FILTERS_1, val);

	//enable timeslot0 to left dac
	val = wm8994_read(codec, WM8994_DAC1_LEFT_MIXER_ROUTING);
	val &= ~(WM8994_AIF1DAC1L_TO_DAC1L_MASK);
	val |= (WM8994_AIF1DAC1L_TO_DAC1L);
	wm8994_write(codec, WM8994_DAC1_LEFT_MIXER_ROUTING, val);
	
	//Enbale bias,vmid and Left speaker
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1);
	val &= ~(WM8994_SPKOUTL_ENA_MASK|WM8994_VMID_SEL_MASK|WM8994_BIAS_ENA_MASK);
	val |= (WM8994_SPKOUTL_ENA|WM8994_VMID_SEL_NORMAL|WM8994_BIAS_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

	printk(SND_KERN_DEBUG "[WM8994] wm8994_set_playback_speaker()\n");
}

void wm8994_set_playback_speaker_headset(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = codec->private_data;

	u16 val;
	int ret;

#if 1
	u16 nReadServo4Val = NULL;
	u16 ncompensationResult = NULL;
	u16 nCompensationResultLow = NULL;
	u16 nCompensationResultHigh = NULL;
	u8 nServo4Low = 0;
	u8 nServo4High = 0;

	DEBUG_LOG("");
	DEBUG_LOG("testmode_config_flag = [%d]",wm8994->testmode_config_flag);

	//------------------  Common Settings ------------------
	// Enable the Timeslot0 to DAC1L
	val = wm8994_read(codec, WM8994_DAC1_LEFT_MIXER_ROUTING);
	val &= ~(WM8994_AIF1DAC1L_TO_DAC1L_MASK);
	val |= (WM8994_AIF1DAC1L_TO_DAC1L);
	wm8994_write(codec, WM8994_DAC1_LEFT_MIXER_ROUTING, val);
	
	//Enable the Timeslot0 to DAC1R
	val = wm8994_read(codec, WM8994_DAC1_RIGHT_MIXER_ROUTING);
	val &= ~(WM8994_AIF1DAC1R_TO_DAC1R_MASK);
	val |= (WM8994_AIF1DAC1R_TO_DAC1R);
	wm8994_write(codec, WM8994_DAC1_RIGHT_MIXER_ROUTING, val);

	//------------------  Speaker Path Settings ------------------

	// Speaker Volume Control
	if(!wm8994->testmode_config_flag)
	{
		val = wm8994_read(codec, WM8994_SPEAKER_VOLUME_LEFT);
		val &= ~(WM8994_SPKOUTL_MUTE_N_MASK|WM8994_SPKOUTL_VOL_MASK);
		val |= (WM8994_SPKOUTL_MUTE_N|TUNING_MP3_SPKL_VOL);
		wm8994_write(codec, WM8994_SPEAKER_VOLUME_LEFT, val);

		val = wm8994_read(codec, WM8994_SPEAKER_VOLUME_RIGHT);
		val &= ~(WM8994_SPKOUTR_MUTE_N_MASK|WM8994_SPKOUTR_VOL_MASK);
		wm8994_write(codec, WM8994_SPEAKER_VOLUME_RIGHT, val);

		val = wm8994_read(codec, WM8994_CLASSD);
		val &= ~(WM8994_SPKOUTL_BOOST_MASK);
		val |= (TUNING_MP3_CLASSD_VOL<<WM8994_SPKOUTL_BOOST_SHIFT);
		wm8994_write(codec, WM8994_CLASSD, val);
	}

	val = wm8994_read(codec, WM8994_SPKOUT_MIXERS);
	val &= ~(WM8994_SPKMIXL_TO_SPKOUTL_MASK|WM8994_SPKMIXR_TO_SPKOUTL_MASK|WM8994_SPKMIXR_TO_SPKOUTR_MASK);
	val |= (WM8994_SPKMIXL_TO_SPKOUTL);
	wm8994_write(codec, WM8994_SPKOUT_MIXERS, val);

	//Unmute the DAC path
	val = wm8994_read(codec, WM8994_SPEAKER_MIXER);
	val &= ~(WM8994_DAC1L_TO_SPKMIXL_MASK);
	val |= (WM8994_DAC1L_TO_SPKMIXL);
	wm8994_write(codec, WM8994_SPEAKER_MIXER, val);

	//------------------  Ear Path Settings ------------------
	//Configuring the Digital Paths
	
	val = wm8994_read(codec, 0x102);
	val &= ~(0x0003);
	val = (0x0003);
	wm8994_write(codec,0x102, val);
	
	val = wm8994_read(codec, 0x56);
	val &= ~(0x0003);
	val = (0x0003);
	wm8994_write(codec,0x56, val);
	
	val = wm8994_read(codec, 0x102);
	val &= ~(0x0000);
	val = (0x0000);
	wm8994_write(codec,0x102, val);

	val = wm8994_read(codec, WM8994_CLASS_W_1);
	val &= ~(0x0005);
	val = (0x0005);
	wm8994_write(codec, WM8994_CLASS_W_1, val);

	if(!wm8994->testmode_config_flag)
	{
		val = wm8994_read(codec, WM8994_LEFT_OUTPUT_VOLUME);
		val &= ~(WM8994_HPOUT1L_MUTE_N_MASK|WM8994_HPOUT1L_VOL_MASK);
		val |= (WM8994_HPOUT1_VU|WM8994_HPOUT1L_MUTE_N|TUNING_DUAL_OUTPUTL_VOL);
		wm8994_write(codec, WM8994_LEFT_OUTPUT_VOLUME, val);

		val = wm8994_read(codec, WM8994_RIGHT_OUTPUT_VOLUME);
		val &= ~(WM8994_HPOUT1R_MUTE_N_MASK|WM8994_HPOUT1R_VOL_MASK);
		val |= (WM8994_HPOUT1_VU|WM8994_HPOUT1R_MUTE_N|TUNING_DUAL_OUTPUTR_VOL);
		wm8994_write(codec, WM8994_RIGHT_OUTPUT_VOLUME, val);
	}

	// DC Servo Series Count
	val = 0x03E0;
	wm8994_write(codec, WM8994_DC_SERVO_2, val);

	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1);
	val &= ~(WM8994_BIAS_ENA_MASK|WM8994_VMID_SEL_MASK|WM8994_HPOUT1L_ENA_MASK|WM8994_HPOUT1R_ENA_MASK|WM8994_SPKOUTL_ENA_MASK);
	val |= (WM8994_BIAS_ENA|WM8994_VMID_SEL_NORMAL|WM8994_HPOUT1R_ENA|WM8994_HPOUT1L_ENA|WM8994_SPKOUTL_ENA);
	ret |= wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

	val = (WM8994_HPOUT1L_DLY|WM8994_HPOUT1R_DLY);
	wm8994_write(codec, WM8994_ANALOGUE_HP_1, val);

	// Enable Charge Pump
	val = wm8994_read(codec, WM8994_CHARGE_PUMP_1);
	val &= ~(WM8994_CP_ENA_MASK);
	val |= (WM8994_CP_ENA|WM8994_CP_ENA_DEFAULT); // this is from wolfson 
	wm8994_write(codec, WM8994_CHARGE_PUMP_1, val);

	msleep(5);

	// Enable DAC1 and DAC2 and the Timeslot0 for AIF1
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_5);
	val &= ~(WM8994_DAC1R_ENA_MASK|WM8994_DAC1L_ENA_MASK|WM8994_AIF1DAC1R_ENA_MASK|WM8994_AIF1DAC1L_ENA_MASK);
	val |= (WM8994_AIF1DAC1L_ENA|WM8994_AIF1DAC1R_ENA|WM8994_DAC1L_ENA|WM8994_DAC1R_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_5, val);

	// Enbale DAC1L to HPOUT1L path
	val = wm8994_read(codec, WM8994_OUTPUT_MIXER_1);
	val &= ~(WM8994_DAC1L_TO_HPOUT1L_MASK|WM8994_DAC1L_TO_MIXOUTL_MASK);
	val |= (WM8994_DAC1L_TO_MIXOUTL);
	wm8994_write(codec, WM8994_OUTPUT_MIXER_1, val);

	// Enbale DAC1R to HPOUT1R path
	val = wm8994_read(codec, WM8994_OUTPUT_MIXER_2);
	val &= ~(WM8994_DAC1R_TO_HPOUT1R_MASK|WM8994_DAC1R_TO_MIXOUTR_MASK);
	val |= (WM8994_DAC1R_TO_MIXOUTR);
	wm8994_write(codec, WM8994_OUTPUT_MIXER_2, val);

	//Enbale bias,vmid, hp left and right and Left speaker
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_3);
	val &= ~(WM8994_MIXOUTLVOL_ENA_MASK|WM8994_MIXOUTRVOL_ENA_MASK|WM8994_MIXOUTL_ENA_MASK|WM8994_MIXOUTR_ENA_MASK|WM8994_SPKLVOL_ENA_MASK);
	val |= (WM8994_MIXOUTL_ENA|WM8994_MIXOUTR_ENA|WM8994_SPKLVOL_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, val);

	//* DC Servo 
	val = (WM8994_DCS_TRIG_SERIES_1|WM8994_DCS_TRIG_SERIES_0|WM8994_DCS_ENA_CHAN_1|WM8994_DCS_ENA_CHAN_0);
	wm8994_write(codec, WM8994_DC_SERVO_1, 0x0303);

	msleep(160);

	nReadServo4Val = wm8994_read(codec, WM8994_DC_SERVO_4);
	nServo4Low = (signed char)(nReadServo4Val & 0xff);
	nServo4High = (signed char)((nReadServo4Val>>8)&0xff);

	nCompensationResultLow = ((signed short)nServo4Low-5)&0x00ff;
	nCompensationResultHigh = (((signed short)(nServo4High-5))<<8)&0xff00;
	ncompensationResult = (nCompensationResultLow|nCompensationResultHigh);
	wm8994_write(codec, WM8994_DC_SERVO_4, ncompensationResult);

	val = (WM8994_DCS_TRIG_DAC_WR_1|WM8994_DCS_TRIG_DAC_WR_0|WM8994_DCS_ENA_CHAN_1|WM8994_DCS_ENA_CHAN_0);
	wm8994_write(codec, WM8994_DC_SERVO_1, val);

	msleep(15);

	// Intermediate HP settings
	val = wm8994_read(codec, WM8994_ANALOGUE_HP_1);
	val &= ~(WM8994_HPOUT1R_DLY_MASK|WM8994_HPOUT1R_OUTP_MASK|WM8994_HPOUT1R_RMV_SHORT_MASK|
		WM8994_HPOUT1L_DLY_MASK|WM8994_HPOUT1L_OUTP_MASK|WM8994_HPOUT1L_RMV_SHORT_MASK);
	val |= (WM8994_HPOUT1L_RMV_SHORT|WM8994_HPOUT1L_OUTP|WM8994_HPOUT1L_DLY|WM8994_HPOUT1R_RMV_SHORT|
		WM8994_HPOUT1R_OUTP|WM8994_HPOUT1R_DLY);
	wm8994_write(codec, WM8994_ANALOGUE_HP_1, val);

	if(!wm8994->testmode_config_flag)
	{
		//Unmute DAC1 left
		val = wm8994_read(codec, WM8994_DAC1_LEFT_VOLUME);
		val &= ~(WM8994_DAC1L_MUTE_MASK|WM8994_DAC1L_VOL_MASK);
		val |= (TUNING_DAC1L_VOL);
		wm8994_write(codec, WM8994_DAC1_LEFT_VOLUME, val);

		//Unmute and volume ctrl RightDAC
		val = wm8994_read(codec, WM8994_DAC1_RIGHT_VOLUME);
		val &= ~(WM8994_DAC1R_MUTE_MASK|WM8994_DAC1R_VOL_MASK);
		val |= (TUNING_DAC1R_VOL);
		wm8994_write(codec, WM8994_DAC1_RIGHT_VOLUME, val);
	}

	//------------------  Common Settings ------------------
	// Unmute the AIF1DAC1	
	val = wm8994_read(codec, WM8994_AIF1_DAC1_FILTERS_1);
	val &= ~(WM8994_AIF1DAC1_MUTE_MASK|WM8994_AIF1DAC1_MONO_MASK);
	val |= (WM8994_AIF1DAC1_UNMUTE|WM8994_AIF1DAC1_MONO);
	wm8994_write(codec, WM8994_AIF1_DAC1_FILTERS_1, val);

	if(!wm8994->testmode_config_flag)
	{
		// Unmute the SPKMIXVOLUME
		val = wm8994_read(codec, WM8994_SPKMIXL_ATTENUATION);
		val &= ~(WM8994_SPKMIXL_VOL_MASK);
		val |= (TUNING_SPKMIXL_ATTEN);
		wm8994_write(codec, WM8994_SPKMIXL_ATTENUATION, val);

		val = wm8994_read(codec, WM8994_SPKMIXR_ATTENUATION);
		val &= ~(WM8994_SPKMIXR_VOL_MASK);
		wm8994_write(codec, WM8994_SPKMIXR_ATTENUATION, val);
	}
#endif
	printk(SND_KERN_DEBUG "[WM8994] wm8994_set_playback_speaker_headset()\n");
}                                                      


void wm8994_set_playback_extra_dock_speaker(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = codec->private_data;

	u16 val;

	DEBUG_LOG("");

	//OUTPUT mute
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_3);
	val &= ~(WM8994_LINEOUT2N_ENA_MASK|WM8994_LINEOUT2P_ENA_MASK);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, val);
	
	if(!wm8994->testmode_config_flag)
	{
        val = wm8994_read(codec, WM8994_OUTPUT_MIXER_5);
		val &= ~(WM8994_DACL_MIXOUTL_VOL_MASK);
        val |= (TUNING_EXTRA_DOCK_SPK_OUTMIX5_VOL<<WM8994_DACL_MIXOUTL_VOL_SHIFT);
        wm8994_write(codec, WM8994_OUTPUT_MIXER_5, val);
	
		val = wm8994_read(codec, WM8994_OUTPUT_MIXER_6);
		val &= ~(WM8994_DACR_MIXOUTR_VOL_MASK);
		val |= (TUNING_EXTRA_DOCK_SPK_OUTMIX6_VOL<<WM8994_DACR_MIXOUTR_VOL_SHIFT);
		wm8994_write(codec, WM8994_OUTPUT_MIXER_6, val);

		val = wm8994_read(codec, WM8994_LEFT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTL_MUTE_N_MASK|WM8994_MIXOUTL_VOL_MASK);
		val |= (WM8994_MIXOUT_VU|WM8994_MIXOUTL_MUTE_N|TUNING_MP3_EXTRA_DOCK_SPK_OPGAL_VOL);
		wm8994_write(codec, WM8994_LEFT_OPGA_VOLUME, val);

		val = wm8994_read(codec, WM8994_RIGHT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTR_MUTE_N_MASK|WM8994_MIXOUTR_VOL_MASK);
		val |= (WM8994_MIXOUT_VU|WM8994_MIXOUTR_MUTE_N|TUNING_MP3_EXTRA_DOCK_SPK_OPGAR_VOL);
		wm8994_write(codec, WM8994_RIGHT_OPGA_VOLUME, val);

		val = wm8994_read(codec, WM8994_LINE_OUTPUTS_VOLUME);
		val &= ~(WM8994_LINEOUT2_VOL_MASK);
		val |= (TUNING_MP3_EXTRA_DOCK_SPK_VOL<<WM8994_LINEOUT2_VOL_SHIFT);
		wm8994_write(codec, WM8994_LINE_OUTPUTS_VOLUME, val);

		//Unmute DAC1 left
		val = wm8994_read(codec, WM8994_DAC1_LEFT_VOLUME );
		val &= ~(WM8994_DAC1L_MUTE_MASK|WM8994_DAC1L_VOL_MASK);
		val |= (TUNING_DAC1L_VOL);
		wm8994_write(codec, WM8994_DAC1_LEFT_VOLUME, val);
	
		//Unmute and volume ctrl RightDAC
		val = wm8994_read(codec, WM8994_DAC1_RIGHT_VOLUME ); 
		val &= ~(WM8994_DAC1R_MUTE_MASK|WM8994_DAC1R_VOL_MASK);
		val |= (TUNING_DAC1R_VOL);
		wm8994_write(codec, WM8994_DAC1_RIGHT_VOLUME, val);
		
		//val = wm8994_read(codec, WM8994_AIF1_DAC1_LEFT_VOLUME);
		//val &= ~(WM8994_AIF1DAC1L_VOL_MASK);
		//val |= (WM8994_AIF1DAC1_VU|TUNING_DAC1L_VOL);
		//wm8994_write(codec, WM8994_AIF1_DAC1_LEFT_VOLUME, val);

		//val = wm8994_read(codec, WM8994_AIF1_DAC1_RIGHT_VOLUME);
		//val &= ~(WM8994_AIF1DAC1R_VOL_MASK);
		//val |= (WM8994_AIF1DAC1_VU|TUNING_DAC1R_VOL);
		//wm8994_write(codec, WM8994_AIF1_DAC1_RIGHT_VOLUME, val);
	}

	val = wm8994_read(codec, WM8994_OUTPUT_MIXER_1);
	val &= ~(WM8994_DAC1L_TO_MIXOUTL_MASK);
	val |= (WM8994_DAC1L_TO_MIXOUTL);
	wm8994_write(codec, WM8994_OUTPUT_MIXER_1, val);

	val = wm8994_read(codec, WM8994_OUTPUT_MIXER_2);
	val &= ~(WM8994_DAC1R_TO_MIXOUTR_MASK);
	val |= (WM8994_DAC1R_TO_MIXOUTR);
	wm8994_write(codec, WM8994_OUTPUT_MIXER_2, val);

	val = wm8994_read(codec, WM8994_LINE_MIXER_2);
	val &= ~(WM8994_MIXOUTR_TO_LINEOUT2N_MASK|WM8994_MIXOUTL_TO_LINEOUT2N_MASK|WM8994_LINEOUT2_MODE_MASK|WM8994_MIXOUTR_TO_LINEOUT2P_MASK);
	//val |= (WM8994_MIXOUTL_TO_LINEOUT2N|WM8994_LINEOUT2_MODE|WM8994_MIXOUTR_TO_LINEOUT2P); // singled-ended output
	val |= (WM8994_MIXOUTL_TO_LINEOUT2N|WM8994_MIXOUTR_TO_LINEOUT2P); // differential output
	//val |= (WM8994_MIXOUTR_TO_LINEOUT2P); // differential output
	wm8994_write(codec, WM8994_LINE_MIXER_2, val);

	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_5);
	val &= ~(WM8994_DAC1R_ENA_MASK|WM8994_DAC1L_ENA_MASK|WM8994_AIF1DAC1R_ENA_MASK|WM8994_AIF1DAC1L_ENA_MASK);
	val |= (WM8994_AIF1DAC1L_ENA|WM8994_AIF1DAC1R_ENA|WM8994_DAC1L_ENA|WM8994_DAC1R_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_5, val);

	val = wm8994_read(codec, WM8994_AIF1_DAC1_FILTERS_1);
	val &= ~(WM8994_AIF1DAC1_MUTE_MASK|WM8994_AIF1DAC1_MONO_MASK);
	val |= (WM8994_AIF1DAC1_UNMUTE|WM8994_AIF1DAC1_MONO);
	wm8994_write(codec, WM8994_AIF1_DAC1_FILTERS_1, val);

	val = wm8994_read(codec, WM8994_LINE_OUTPUTS_VOLUME);
	val &= ~(WM8994_LINEOUT2N_MUTE_MASK|WM8994_LINEOUT2P_MUTE_MASK);
	wm8994_write(codec, WM8994_LINE_OUTPUTS_VOLUME, val);

	val = wm8994_read(codec, WM8994_DAC1_LEFT_MIXER_ROUTING);
	val &= ~(WM8994_AIF1DAC1L_TO_DAC1L_MASK);
	val |= (WM8994_AIF1DAC1L_TO_DAC1L);
	wm8994_write(codec, WM8994_DAC1_LEFT_MIXER_ROUTING, val);

	val = wm8994_read(codec, WM8994_DAC1_RIGHT_MIXER_ROUTING);
	val &= ~(WM8994_AIF1DAC1R_TO_DAC1R_MASK);
	val |= (WM8994_AIF1DAC1R_TO_DAC1R);
	wm8994_write(codec, WM8994_DAC1_RIGHT_MIXER_ROUTING, val);

	//val = wm8994_read(codec, WM8994_CLOCKING_1);
	//val &= ~(WM8994_DSP_FS1CLK_ENA_MASK|WM8994_DSP_FSINTCLK_ENA_MASK);
	//val |= (WM8994_DSP_FS1CLK_ENA|WM8994_DSP_FSINTCLK_ENA);
	//wm8994_write(codec, WM8994_CLOCKING_1, val);

	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_3);
	val &= ~(WM8994_LINEOUT2N_ENA_MASK|WM8994_LINEOUT2P_ENA_MASK|WM8994_MIXOUTLVOL_ENA_MASK|WM8994_MIXOUTRVOL_ENA_MASK|WM8994_MIXOUTL_ENA_MASK|WM8994_MIXOUTR_ENA_MASK);
	val |= (WM8994_LINEOUT2N_ENA|WM8994_LINEOUT2P_ENA|WM8994_MIXOUTL_ENA|WM8994_MIXOUTR_ENA|WM8994_MIXOUTRVOL_ENA|WM8994_MIXOUTLVOL_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, val);
		
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1 );
	val &= ~(WM8994_BIAS_ENA_MASK|WM8994_VMID_SEL_MASK|WM8994_HPOUT2_ENA_MASK);
	val |= (WM8994_BIAS_ENA|WM8994_VMID_SEL_NORMAL);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

}

#define FUNCTION_LIST_VOICECALL

void wm8994_set_voicecall_receiver(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = codec->private_data;

	int val;
	
	DEBUG_LOG("");

	audio_ctrl_mic_bias_gpio(codec);

#if 1
	//wm8994_write(codec, 0x39, 0x006C);
	//mdelay(50);
	//wm8994_write(codec, 0x38, 0x0080); // LINEOUT1FB : use only single-ended mode
	//wm8994_write(codec, 0x37, 0x0081);

	/* CHARGE PUMP */
	wm8994_write(codec, 0x4C, 0x1F25); // Charge Pump
	
	wm8994_write(codec, 0x02, 0x6240); // PM_2 // TSHUT_ENA, TSHUT_OPDIS, MIXINL_ENA, IN1L_ENA

	/* DIGITAL - AIF1DAC1 */
	wm8994_write(codec, 0x05, 0x0303); // AIF1DAC1L/R_ENA, DAC1L/R_ENA
	wm8994_write(codec,0x420, 0x0000); // AIF1DAC1 On
	wm8994_write(codec,0x601, 0x0001); // AIF1DAC1L_TO_DAC1L
	wm8994_write(codec,0x602, 0x0001); // AIF1DAC1R_TO_DAC1R

	if(!wm8994->testmode_config_flag)
	{
		/* ANALOG INPUT PATH */
		wm8994_write(codec, 0x1A, 0x018B); // IN1RN/P(EAR_MIC_P/N) DISABLE // 0x0B(0.0dB)
		wm8994_write(codec, 0x18, 0x010B); // IN1LN/P(MAIN_MIC_P/N) // 0x0B(0.0dB)
		wm8994_write(codec, 0x28, 0x0030); // IN1LN/P(MAIN_MIC_P/N)_TO_IN1L
		//wm8994_write(codec, 0x29, 0x0020); // IN1L_TO_MIXINL, 0dB

		wm8994_write(codec, 0x2C, 0x0001); // 2Ch // [DE03-1723] // IN2LRP_MIXINR_VOL 0x03(-6dB)

		// DAC TO MIXOUT
		//wm8994_write(codec, 0x31, 0x0000); // DACL_MIXTOUTL_VOL, 0dB
		//wm8994_write(codec, 0x32, 0x0000); // DACL_MIXTOUTL_VOL, 0dB

		// MIXOUT
		//wm8994_write(codec, 0x20, 0x0079); // MIXOUTL_VOL, 0dB
		//wm8994_write(codec, 0x21, 0x0179); // MIXOUTR_VOL, 0dB

		// HPOUT2
		wm8994_write(codec, 0x1F, 0x0000); // HPOUT2(RCV) Unmute, 0x0(0dB, not -6dB)

		// DAC1L/RVOL
		wm8994_write(codec,0x610, 0x00C0); // DAC1L Digital Vol 0xC0(0.0dB)
		wm8994_write(codec,0x611, 0x01C0); // DAC1R Digital Vol 0xC0(0.0dB), update!
	}

	/* ANALOG OUTPUT - LINEOUT1N/P(To BaseBand) */
	wm8994_write(codec, 0x1E, 0x0006); // LINEOUT1N/P(CP_MIC_N/P) 0x0(0dB)
	wm8994_write(codec, 0x34, 0x0002); // IN1L_TO_LINEOUT1P(diff.)

	/* OUTPUT MIXER */
	wm8994_write(codec, 0x2D, 0x0001); // DAC1L_TO_MIXOUTL
	wm8994_write(codec, 0x2E, 0x0001); // DAC1R_TO_MIXOUTR

	/* ANALOG OUTPUT CONFIGRATION */
	//wm8994_write(codec, 0x03, 0x30F0); // LINEOUT1N/P_ENA, MIXOUTL/RVOL_ENA, MIXOUTL/R_ENA
	wm8994_write(codec, 0x03, 0x3030); // LINEOUT1N/P_ENA, MIXOUTL/R_ENA
	wm8994_write(codec, 0x33, 0x0038); // IN2LRP_TO_HPOUT2(direct), MIXOUTL/RVOL_TO_HPOUT2
	wm8994_write(codec, 0x01, 0x0803); // HPOUT2_ENA, VMID_SEL_NORMAL, BIAS_ENA
#endif
#if 0
	/* POWER UP */
	wm8994_write(codec, 0x39, 0x006C);
	msleep(5);
	wm8994_write(codec, 0x01, 0x0003);
	msleep(11);//mdelay(50);
	wm8994_write(codec, 0x01, 0x0803);

	/* POWER MANAGEMENT */
	wm8994_write(codec, 0x02, 0x6240); // TSHUT_ENA, TSHUT_OPDIS, MIXINL_ENA, IN1L_ENA
	wm8994_write(codec, 0x18, 0x010B); // IN1LN/P(MAIN_MIC_P/N) // [DB16-2232] 0x0B(0.0dB) HW req.
	wm8994_write(codec, 0x28, 0x0030); // IN1LN/P(MAIN_MIC_P/N)_TO_IN1L
	//wm8994_write(codec, 0x29, 0x0020); // IN1L_TO_MIXINL, 0dB

	/* ANALOG OUTPUT */	
	wm8994_write(codec, 0x03, 0x30F0); // LINEOUT1N/P_ENA, MIXOUTL/RVOL_ENA, MIXOUTL/R_ENA
	wm8994_write(codec, 0x1E, 0x0006); // LINEOUT1N/P(CP_MIC_N/P) 0x0(0dB)
	wm8994_write(codec, 0x2D, 0x0001); // DAC1L_TO_MIXOUTL
	wm8994_write(codec, 0x2E, 0x0001); // DAC1R_TO_MIXOUTR
	wm8994_write(codec, 0x33, 0x0038); // IN2LRP_TO_HPOUT2(direct), MIXOUTL/RVOL_TO_HPOUT2

	/* DIGITAL PATH - AIF1DAC1 */
	wm8994_write(codec, 0x34, 0x0002); // IN1L_TO_LINEOUT1P(diff.)
	wm8994_write(codec, 0x05, 0x0303); // AIF1DAC1L/R_ENA, DAC1L/R_ENA
	wm8994_write(codec,0x420, 0x0000); // AIF1DAC1 On
	wm8994_write(codec,0x601, 0x0001); // AIF1DAC1L_TO_DAC1L
	wm8994_write(codec,0x602, 0x0001); // AIF1DAC1R_TO_DAC1R
	wm8994_write(codec,0x610, 0x00C0); // DAC1L Digital Vol 0xC0(0.0dB)
	wm8994_write(codec,0x611, 0x01C0); // DAC1R Digital Vol 0xC0(0.0dB), update!

	/* HPOUT2 UNMUTE */
	wm8994_write(codec, 0x1F, 0x0000); // HPOUT2(RCV) Unmute, 0x0(0dB, not -6dB)
#endif
#if 0
	/* POWER UP */
	wm8994_write(codec, 0x39, 0x006C);
	mdelay(5);
	wm8994_write(codec, 0x01, 0x0003);
	mdelay(11);//mdelay(50);
	wm8994_write(codec, 0x01, 0x0803);

	/* POWER MANAGEMENT */
	wm8994_write(codec, 0x02, 0x6340); // TSHUT_ENA, TSHUT_OPDIS, MIXINL/R_ENA, IN1L_ENA
	wm8994_write(codec, 0x18, 0x010B); // IN1LN/P(MAIN_MIC_P/N) // [DB16-2232] 0x0B(0.0dB) HW req.
	wm8994_write(codec, 0x28, 0x0030); // IN1LN/P(MAIN_MIC_P/N)_TO_IN1L
	//wm8994_write(codec, 0x29, 0x0020); // IN1L_TO_MIXINL, 0dB
	wm8994_write(codec, 0x2C, 0x0005);

	/* ANALOG OUTPUT */	
	wm8994_write(codec, 0x03, 0x30F0); // LINEOUT1N/P_ENA, MIXOUTL/RVOL_ENA, MIXOUTL/R_ENA
	wm8994_write(codec, 0x1E, 0x0006); // LINEOUT1N/P(CP_MIC_N/P) 0x0(0dB)
	wm8994_write(codec, 0x2D, 0x0081); // MIXINR_TO_MIXOUTL, DAC1L_TO_MIXOUTL
	wm8994_write(codec, 0x2E, 0x0041); // MIXINR_TO_MIXOUTR, DAC1R_TO_MIXOUTR
	wm8994_write(codec, 0x33, 0x0018); // MIXOUTL/RVOL_TO_HPOUT2
	wm8994_write(codec, 0x34, 0x0002); // IN1L_TO_LINEOUT1P(diff.)

	wm8994_write(codec, 0x20, 0x0079);
	wm8994_write(codec, 0x21, 0x0179);

	/* DIGITAL PATH - AIF1DAC1 */
	wm8994_write(codec, 0x05, 0x0303); // AIF1DAC1L/R_ENA, DAC1L/R_ENA
	wm8994_write(codec,0x420, 0x0010); // AIF1DAC1 On
	wm8994_write(codec,0x601, 0x0001); // AIF1DAC1L_TO_DAC1L
	wm8994_write(codec,0x602, 0x0001); // AIF1DAC1R_TO_DAC1R
	wm8994_write(codec,0x610, 0x00C0); // DAC1L Digital Vol 0xC0(0.0dB)
	wm8994_write(codec,0x611, 0x01C0); // DAC1R Digital Vol 0xC0(0.0dB), update!

	/* HPOUT2 UNMUTE */
	wm8994_write(codec, 0x1F, 0x0000); // HPOUT2(RCV) Unmute, 0x0(0dB, not -6dB)
#endif
	printk(SND_KERN_DEBUG "[WM8994] wm8994_set_voicecall_receiver()\n");
}


void wm8994_set_voicecall_headset(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = codec->private_data;

	int val;

	DEBUG_LOG("");

	audio_ctrl_mic_bias_gpio(codec);

#if 1
	u16 TestReturn1 = 0;
	u16 TestReturn2 = 0;
	u16 TestLow1 = 0;
	u16 TestHigh1 = 0;
	u8 TestLow = 0;
	u8 TestHigh = 0;

	wm8994_write(codec, 0x01, 0x0003);

	wm8994_write(codec, 0x1F, 0x0020); // HPOUT2_MUTE

	wm8994_write(codec, 0x03, 0x0000); // MIXOUTL/R_VOL // OFF

	/*Digital Path Enables and Unmutes*/
	//wm8994_write(codec, WM8994_DAC1_LEFT_MIXER_ROUTING, 0x0021);
	//wm8994_write(codec, WM8994_DAC1_RIGHT_MIXER_ROUTING, 0x0021);
	wm8994_write(codec, WM8994_DAC1_LEFT_MIXER_ROUTING, 0x0011);
	wm8994_write(codec, WM8994_DAC1_RIGHT_MIXER_ROUTING, 0x0011);
	//wm8994_write(codec, WM8994_DAC1_LEFT_MIXER_ROUTING, 0x0001);
	//wm8994_write(codec, WM8994_DAC1_RIGHT_MIXER_ROUTING, 0x0001);

	/*Analogue Input Configuration*/
	//wm8994_write(codec, WM8994_POWER_MANAGEMENT_2, 0x6110); // MIXINR_ENA, IN1R_ENA
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_2, 0x6310); // MIXINL/R_ENA, IN1R_ENA

	// test - DD24-0226
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_4, 0x0002); // 04h // ADCL_ENA

	val = wm8994_read(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME); // IN1LN/P(MAIN_MIC_P/N)
	val &= ~(WM8994_IN1L_VOL_MASK); // Mute IN1L
	val |= (WM8994_IN1L_VU|WM8994_IN1L_MUTE|TUNING_CALL_EAR_INPUTMIX_VOL);
	wm8994_write(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME, val);

	if(!wm8994->testmode_config_flag)
	{
		val = wm8994_read(codec, WM8994_RIGHT_LINE_INPUT_1_2_VOLUME); // IN1RN/P(EAR_MIC_P/N) // [DB16-2232] 0x0B(0.0dB) HW req.
		val &= ~(WM8994_IN1R_MUTE_MASK|WM8994_IN1R_VOL_MASK); // Unmute IN1R
		val |= (WM8994_IN1R_VU|TUNING_CALL_EAR_INPUTMIX_VOL);
		wm8994_write(codec, WM8994_RIGHT_LINE_INPUT_1_2_VOLUME, val);

		// unmute right pga, set volume 
		val = wm8994_read(codec, WM8994_INPUT_MIXER_4);
		val&= ~(WM8994_IN1R_TO_MIXINR_MASK|WM8994_IN1R_MIXINR_VOL_MASK|WM8994_MIXOUTR_MIXINR_VOL_MASK);
		val |= (WM8994_IN1R_TO_MIXINR);//0db
		wm8994_write(codec, WM8994_INPUT_MIXER_4, val);

		//wm8994_write(codec, WM8994_INPUT_MIXER_6, 0x0001); // 2Ch // [DE03-1723] // IN2LRP_MIXINR_VOL 0x03(-6dB)
		wm8994_write(codec, WM8994_INPUT_MIXER_5, 0x0001); // 2Bh // [DF29-1315] // IN2LRP_MIXINL_VOL 0x03(-6dB)
	}

	wm8994_write(codec, WM8994_INPUT_MIXER_2, 0x0001); // IN1RN(EAR_MIC_P)_TO_IN1R

	// code add
	wm8994_write(codec, 0x34, 0x0004); // IN1R_TO_LINEOUT1P(diff)

	wm8994_write(codec, 0x1E, 0x0006); // WM8994_LINE_OUTPUTS_VOLUME // LINEOUT1N/P On, LINEOUT1_VOL 0dB(MAX)
	//~code add

	//wm8994_set_voicecall_common_setting(codec);

	// test - DD24-0226
	//wm8994_write(codec, 0x600, 0x0180); // Right
	wm8994_write(codec, 0x600, 0x000C); // Left

	/* Unmute*/
	if(!wm8994->testmode_config_flag)
	{
		val = wm8994_read(codec, WM8994_LEFT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTL_MUTE_N_MASK|WM8994_MIXOUTL_VOL_MASK);
		val |= (WM8994_MIXOUT_VU|WM8994_MIXOUTL_MUTE_N|TUNING_CALL_OPGAL_VOL);
		wm8994_write(codec, WM8994_LEFT_OPGA_VOLUME, val);

		val = wm8994_read(codec, WM8994_RIGHT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTR_MUTE_N_MASK|WM8994_MIXOUTR_VOL_MASK);
		val |= (WM8994_MIXOUT_VU|WM8994_MIXOUTR_MUTE_N|TUNING_CALL_OPGAR_VOL);
		wm8994_write(codec, WM8994_RIGHT_OPGA_VOLUME, val);
	}

	val = wm8994_read(codec, 0x102);
	val &= ~(0x0003);
	val = 0x0003;
	wm8994_write(codec,0x102, val);

	val = wm8994_read(codec, 0x56);
	val &= ~(0x0003);
	val = 0x0003;
	wm8994_write(codec,0x56, val);

	val = wm8994_read(codec, 0x102);
	val &= ~(0x0000);
	val = 0x0000;
	wm8994_write(codec,0x102, val);

	val = wm8994_read(codec, WM8994_CLASS_W_1);
	val &= ~(0x0005);
	val |= 0x0005;
	wm8994_write(codec, WM8994_CLASS_W_1, val);

	if(!wm8994->testmode_config_flag)
	{
		val = wm8994_read(codec, WM8994_LEFT_OUTPUT_VOLUME);
		val &= ~(WM8994_HPOUT1L_MUTE_N_MASK|WM8994_HPOUT1L_VOL_MASK);
		//val |= (WM8994_HPOUT1_VU|WM8994_HPOUT1L_MUTE_N|TUNING_CALL_OUTPUTL_VOL);
		val |= (WM8994_HPOUT1L_MUTE_N|TUNING_CALL_OUTPUTL_VOL);
		wm8994_write(codec, WM8994_LEFT_OUTPUT_VOLUME, val);

		val = wm8994_read(codec, WM8994_RIGHT_OUTPUT_VOLUME);
		val &= ~(WM8994_HPOUT1R_MUTE_N_MASK|WM8994_HPOUT1R_VOL_MASK);
		val |= (WM8994_HPOUT1_VU|WM8994_HPOUT1R_MUTE_N|TUNING_CALL_OUTPUTR_VOL);
		wm8994_write(codec, WM8994_RIGHT_OUTPUT_VOLUME, val);
	}

	val = wm8994_read(codec, WM8994_DC_SERVO_2);
	val &= ~(0x03E0);
	val = 0x03E0;
	wm8994_write(codec, WM8994_DC_SERVO_2, val);

	wm8994_write(codec, 0x01, 0x0303);
	wm8994_write(codec, 0x60, 0x0022);
	wm8994_write(codec, 0x4C, 0x9F25);

	msleep(5);

	wm8994_write(codec, 0x05, 0x0303);

	/*Analogue Output Configuration*/	
	wm8994_write(codec, WM8994_OUTPUT_MIXER_1, 0x0001); // 2Dh //
	wm8994_write(codec, WM8994_OUTPUT_MIXER_2, 0x0001); // 2Eh //

	wm8994_write(codec, 0x03, 0x3030); // LINEOUTL/R_ENA, MIXOUTL/R_ENA

	wm8994_write(codec, 0x54, 0x0303);

	msleep(160);// 160ms delay

	TestReturn1 = wm8994_read(codec, WM8994_DC_SERVO_4);
	TestLow = (signed char)(TestReturn1 & 0xff);
	TestHigh = (signed char)((TestReturn1>>8) & 0xff);
	TestLow1 = ((signed short)TestLow-5)&0x00ff;
	TestHigh1 = (((signed short)(TestHigh-5))<<8)&0xff00;
	TestReturn2 = TestLow1|TestHigh1;
	wm8994_write(codec, WM8994_DC_SERVO_4, TestReturn2);

	val = wm8994_read(codec, WM8994_DC_SERVO_1);
	val &= ~(0x000F);
	val = (0x000F);
	wm8994_write(codec, WM8994_DC_SERVO_1, val);

	msleep(15);

	wm8994_write(codec, 0x60, 0x00EE);

	//Unmute DAC1 left
	val = wm8994_read(codec, WM8994_DAC1_LEFT_VOLUME);
	val &= ~(WM8994_DAC1L_MUTE_MASK|WM8994_DAC1L_VOL_MASK);
	val |= (WM8994_DAC1_VU|TUNING_DAC1L_VOL);
	wm8994_write(codec, WM8994_DAC1_LEFT_VOLUME, val);

	//Unmute and volume ctrl RightDAC
	val = wm8994_read(codec, WM8994_DAC1_RIGHT_VOLUME);
	val &= ~(WM8994_DAC1R_MUTE_MASK|WM8994_DAC1R_VOL_MASK);
	val |= (WM8994_DAC1_VU|TUNING_DAC1R_VOL); //0 db volume
	wm8994_write(codec, WM8994_DAC1_RIGHT_VOLUME, val);

	wm8994_write(codec, WM8994_AIF1_DAC1_FILTERS_1, 0x0000); // 420h //
#endif
	printk(SND_KERN_DEBUG "[WM8994] wm8994_set_voicecall_headset()\n");
}

void wm8994_set_voicecall_headphone(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = codec->private_data;

	int val;

	DEBUG_LOG("");

	audio_ctrl_mic_bias_gpio(codec);

#if 1
	u16 TestReturn1 = 0;
	u16 TestReturn2 = 0;
	u16 TestLow1 = 0;
	u16 TestHigh1 = 0;
	u8 TestLow = 0;
	u8 TestHigh = 0;

	wm8994_write(codec, 0x01, 0x0003);

	wm8994_write(codec, 0x1F, 0x0020); // HPOUT2_MUTE

	wm8994_write(codec, 0x03, 0x0000); // MIXOUTL/R_VOL // OFF

	/*Digital Path Enables and Unmutes*/	
	wm8994_write(codec, WM8994_DAC1_LEFT_MIXER_ROUTING, 0x0021);
	wm8994_write(codec, WM8994_DAC1_RIGHT_MIXER_ROUTING, 0x0021);
	//wm8994_write(codec, WM8994_DAC1_LEFT_MIXER_ROUTING, 0x0001);
	//wm8994_write(codec, WM8994_DAC1_RIGHT_MIXER_ROUTING, 0x0001);

	/*Analogue Input Configuration*/
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_2, 0x6340); // MIXINL/R_ENA, IN1L_ENA // MAIN MIC

	// test - DD24-0226
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_4, 0x0001); // 04h // ADCR_ENA

	if(!wm8994->testmode_config_flag)
	{
		val = wm8994_read(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME); // IN1LN/P(MAIN_MIC_P/N) // [DB16-2232] 0x0B(0.0dB) HW req.
		val &= ~(WM8994_IN1L_MUTE_MASK|WM8994_IN1L_VOL_MASK); // Unmute IN1L
		val |= (WM8994_IN1L_VU|TUNING_CALL_SPK_INPUTMIX_VOL);
		wm8994_write(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME, val);

		// unmute left pga, set volume 
		val = wm8994_read(codec, WM8994_INPUT_MIXER_3);
		val&= ~(WM8994_IN1L_TO_MIXINL_MASK|WM8994_IN1L_MIXINL_VOL_MASK|WM8994_MIXOUTL_MIXINL_VOL_MASK);
		val |= (WM8994_IN1L_TO_MIXINL); //0db
		wm8994_write(codec, WM8994_INPUT_MIXER_3, val);

		wm8994_write(codec, WM8994_INPUT_MIXER_6, 0x0001); // 2Ch // [DE03-1723] // IN2LRP_MIXINR_VOL 0x03(-6dB)
	}

	wm8994_write(codec, WM8994_INPUT_MIXER_2, 0x0030); // IN1LN/P(MAIN_MIC_P/N)_TO_IN1L

	// code add
	// 3-POLE Path
	wm8994_write(codec, 0x34, 0x0002); // IN1L_TO_LINEOUT1P(diff.)
	// 4-POLE Path
	//wm8994_write(codec, 0x34, 0x0004); // IN1R_TO_LINEOUT1P(diff)

	wm8994_write(codec, 0x1E, 0x0006); // LINEOUT1N/P On, LINEOUT1_VOL 0dB(MAX)
	//~code add

	//wm8994_set_voicecall_common_setting(codec);

	// test - DD24-0226
	wm8994_write(codec, 0x600, 0x0180);

	/* Unmute*/
	if(!wm8994->testmode_config_flag)
	{
		val = wm8994_read(codec, WM8994_LEFT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTL_MUTE_N_MASK|WM8994_MIXOUTL_VOL_MASK);
		val |= (WM8994_MIXOUT_VU|WM8994_MIXOUTL_MUTE_N|TUNING_CALL_OPGAL_VOL);
		wm8994_write(codec, WM8994_LEFT_OPGA_VOLUME, val);

		val = wm8994_read(codec, WM8994_RIGHT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTR_MUTE_N_MASK|WM8994_MIXOUTR_VOL_MASK);
		val |= (WM8994_MIXOUT_VU|WM8994_MIXOUTR_MUTE_N|TUNING_CALL_OPGAR_VOL);
		wm8994_write(codec, WM8994_RIGHT_OPGA_VOLUME, val);
	}

	//wm8994_write(codec, WM8994_POWER_MANAGEMENT_4, 0x2001);

	val = wm8994_read(codec, 0x102);
	val &= ~(0x0003);
	val = 0x0003;
	wm8994_write(codec,0x102, val);

	val = wm8994_read(codec, 0x56);
	val &= ~(0x0003);
	val = 0x0003;
	wm8994_write(codec,0x56, val);

	val = wm8994_read(codec, 0x102);
	val &= ~(0x0000);
	val = 0x0000;
	wm8994_write(codec,0x102, val);

	val = wm8994_read(codec, WM8994_CLASS_W_1);
	val &= ~(0x0005);
	val |= 0x0005;
	wm8994_write(codec, WM8994_CLASS_W_1, val);

	if(!wm8994->testmode_config_flag)
	{
		val = wm8994_read(codec, WM8994_LEFT_OUTPUT_VOLUME);
		val &= ~(WM8994_HPOUT1L_MUTE_N_MASK|WM8994_HPOUT1L_VOL_MASK);
		//val |= (WM8994_HPOUT1_VU|WM8994_HPOUT1L_MUTE_N|TUNING_CALL_OUTPUTL_VOL);
		val |= (WM8994_HPOUT1L_MUTE_N|TUNING_CALL_OUTPUTL_VOL);
		wm8994_write(codec, WM8994_LEFT_OUTPUT_VOLUME, val);

		val = wm8994_read(codec, WM8994_RIGHT_OUTPUT_VOLUME);
		val &= ~(WM8994_HPOUT1R_MUTE_N_MASK|WM8994_HPOUT1R_VOL_MASK);
		val |= (WM8994_HPOUT1_VU|WM8994_HPOUT1R_MUTE_N|TUNING_CALL_OUTPUTR_VOL);
		wm8994_write(codec, WM8994_RIGHT_OUTPUT_VOLUME, val);
	}

	val = wm8994_read(codec, WM8994_DC_SERVO_2);
	val &= ~(0x03E0);
	val = 0x03E0;
	wm8994_write(codec, WM8994_DC_SERVO_2, val);

	wm8994_write(codec, 0x01, 0x0303);
	wm8994_write(codec, 0x60, 0x0022);
	wm8994_write(codec, 0x4C, 0x9F25);

	msleep(5);

	wm8994_write(codec, 0x05, 0x0303);

	/*Analogue Output Configuration*/	
	wm8994_write(codec, WM8994_OUTPUT_MIXER_1, 0x0001); // 2Dh //
	wm8994_write(codec, WM8994_OUTPUT_MIXER_2, 0x0001); // 2Eh //

	wm8994_write(codec, 0x03, 0x3030); // LINEOUTL/R_ENA, MIXOUTL/R_ENA

	wm8994_write(codec, 0x54, 0x0303);

	msleep(160);// 160ms delay

	TestReturn1 = wm8994_read(codec, WM8994_DC_SERVO_4);
	TestLow = (signed char)(TestReturn1 & 0xff);
	TestHigh = (signed char)((TestReturn1>>8) & 0xff);
	TestLow1 = ((signed short)TestLow-5)&0x00ff;
	TestHigh1 = (((signed short)(TestHigh-5)<<8)&0xff00);
	TestReturn2 = TestLow1|TestHigh1;
	wm8994_write(codec, WM8994_DC_SERVO_4, TestReturn2);

	val = wm8994_read(codec, WM8994_DC_SERVO_1);
	val &= ~(0x000F);
	val = (0x000F);
	wm8994_write(codec, WM8994_DC_SERVO_1, val);

	msleep(15);

	wm8994_write(codec, 0x60, 0x00EE);

	//Unmute DAC1 left
	val = wm8994_read(codec, WM8994_DAC1_LEFT_VOLUME);
	val &= ~(WM8994_DAC1L_MUTE_MASK|WM8994_DAC1L_VOL_MASK);
	val |= (WM8994_DAC1_VU|TUNING_DAC1L_VOL);
	wm8994_write(codec, WM8994_DAC1_LEFT_VOLUME, val);

	//Unmute and volume ctrl RightDAC
	val = wm8994_read(codec, WM8994_DAC1_RIGHT_VOLUME);
	val &= ~(WM8994_DAC1R_MUTE_MASK|WM8994_DAC1R_VOL_MASK);
	val |= (WM8994_DAC1_VU|TUNING_DAC1R_VOL); //0 db volume
	wm8994_write(codec, WM8994_DAC1_RIGHT_VOLUME, val);

	wm8994_write(codec, WM8994_AIF1_DAC1_FILTERS_1, 0x0000); // 420h //
#endif
	printk(SND_KERN_DEBUG "[WM8994] wm8994_set_voicecall_headphone()\n");
}

void wm8994_set_voicecall_speaker(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = codec->private_data;

	int val;

	DEBUG_LOG("");

	audio_ctrl_mic_bias_gpio(codec);

#if 1
	// [DE15-2239] hi99.an // reduction path change noise from (CALL_ACTIVE + JACK_4_POLE_DEVICE) to (CALL_ACTIVE + JACK_4_POLE_DEVICE + SPK)
	wm8994_write(codec, 0x01, 0x0003);

	wm8994_write(codec, 0x1F, 0x0020); // HPOUT2_MUTE

	wm8994_write(codec, 0x03, 0x0000); // MIXOUTL/R_VOL // OFF
	//~[DE15-2239] hi99.an //

	wm8994_write(codec, 0x601, 0x0001);
	wm8994_write(codec, 0x602, 0x0001);
	//wm8994_write(codec,0x603, 0x000C);

	// Tx -> AIF2 Path
	//wm8994_write(codec, WM8994_DAC2_LEFT_MIXER_ROUTING, WM8994_ADC1_TO_DAC2L);

	/*Analogue Input Configuration*/
	// code add
	wm8994_write(codec, 0x02, 0x6340);
	//~code add
	wm8994_write(codec, 0x28, 0x0030); // IN1LN/P(MAIN_MIC_P/N)_TO_IN1L // IN1LN/P(MAIN_MIC_P/N)_TO_IN1L, IN2RN/P_TO_IN2R (0x3C)
	
	if(!wm8994->testmode_config_flag)
	{
		// Volume Control - Input
		val = wm8994_read(codec, WM8994_INPUT_MIXER_3); // IN1L_TO_MIXINL 0dB
		val &= ~(WM8994_IN1L_TO_MIXINL_MASK|WM8994_IN1L_MIXINL_VOL_MASK|WM8994_MIXOUTL_MIXINL_VOL_MASK);
		val |= (WM8994_IN1L_TO_MIXINL); //0db
		wm8994_write(codec, WM8994_INPUT_MIXER_3, val);
	
		val = wm8994_read(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME); // IN1LN/P(MAIN_MIC_P/N) // [DB16-2232] 0x0B(0.0dB) HW req.
		val &= ~(WM8994_IN1L_MUTE_MASK|WM8994_IN1L_VOL_MASK); // Unmute IN1L
		val |= (WM8994_IN1L_VU|TUNING_CALL_SPK_INPUTMIX_VOL);
		wm8994_write(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME, val);

		//wm8994_write(codec, 0x2A, 0x0100); // IN2R_TO_MIXINL 0dB // if 28h reg == 0x3C, uncomment
		//wm8994_write(codec, 0x1B, 0x0111); // IN2R PGA Control // if 28h reg == 0x3C, uncomment
	}
	
	//wm8994_set_voicecall_common_setting(codec);

	/*Analogue Output Configuration*/
	// code add
	wm8994_write(codec, 0x34, 0x0002); // IN1L_TO_LINEOUT1P(diff)

	wm8994_write(codec, 0x1E, 0x0006); // LINEOUT1N/P(CP_MIC_N/P) 0x0(0dB, not -6dB)

	wm8994_write(codec, 0x2C, 0x0005); // IN2LRP_MIXINR_VOL[2:0] = b101 // RxVoice to MIXINR

	if(!wm8994->testmode_config_flag)
	{
		wm8994_write(codec, 0x31, 0x0000); // MIXINR_MIXOUTL_VOL 0dB(0x0)
		wm8994_write(codec, 0x32, 0x0000); // MIXINR_MIXOUTR_VOL 0dB(0x0)
	}

	wm8994_write(codec, 0x2D, 0x0081); // MIXINR_TO_MIXOUTL, DAC1L_TO_MIXOUTL
	wm8994_write(codec, 0x2E, 0x0041); // MIXINR_TO_MIXOUTR, DAC1R_TO_MIXOUTR
	//~code add

	wm8994_write(codec, 0x03, 0x33F0); // LINEOUT1N/P_ENA, SPKl/RVOL_ENA, MIXOUTL/RVOL_ENA, MIXOUTL/R_ENA
	//wm8994_write(codec, 0x03, 0x3330); // LINEOUT1N/P_ENA, SPKl/RVOL_ENA, MIXOUTL/R_ENA
	
	wm8994_write(codec, 0x05, 0X0303); // AIF1DAC1L/R_ENA, DAC1L/R_ENA

	if(!wm8994->testmode_config_flag)
	{
		// Volume Control - Output
		// Unmute the SPKMIXVOLUME
		val = wm8994_read(codec, WM8994_SPKMIXL_ATTENUATION);
		val &= ~(WM8994_SPKMIXL_VOL_MASK);
		val |= (TUNING_SPKMIXL_ATTEN);
		wm8994_write(codec, WM8994_SPKMIXL_ATTENUATION, val);

		val = wm8994_read(codec, WM8994_SPKMIXR_ATTENUATION);
		val &= ~(WM8994_SPKMIXR_VOL_MASK);
		val |= (WM8994_SPKOUT_CLASSAB|TUNING_SPKMIXR_ATTEN);
		wm8994_write(codec, WM8994_SPKMIXR_ATTENUATION, val);

		val = wm8994_read(codec, WM8994_SPEAKER_VOLUME_LEFT);
		val &= ~(WM8994_SPKOUTL_MUTE_N_MASK|WM8994_SPKOUTL_VOL_MASK);
		val |= (WM8994_SPKOUT_VU|WM8994_SPKOUTL_MUTE_N|TUNING_CALL_SPKL_VOL);
		wm8994_write(codec, WM8994_SPEAKER_VOLUME_LEFT, val);

		val = wm8994_read(codec, WM8994_SPEAKER_VOLUME_RIGHT);
		val &= ~(WM8994_SPKOUTR_MUTE_N_MASK|WM8994_SPKOUTR_VOL_MASK);
		val |= (WM8994_SPKOUT_VU|WM8994_SPKOUTL_MUTE_N|TUNING_CALL_SPKL_VOL);
		wm8994_write(codec, WM8994_SPEAKER_VOLUME_RIGHT, val);

		val = wm8994_read(codec, WM8994_CLASSD);
		val &= ~(WM8994_SPKOUTL_BOOST_MASK);
		val |= (TUNING_CALL_CLASSD_VOL<<WM8994_SPKOUTL_BOOST_SHIFT);
		wm8994_write(codec, WM8994_CLASSD, val);
	}

	// code add
	wm8994_write(codec, 0x36, 0x000C); // MIXOUTL/R_TO_SPKMIXL/R

	val = wm8994_read(codec, WM8994_SPKOUT_MIXERS);
	val &= ~(WM8994_SPKMIXL_TO_SPKOUTL_MASK|WM8994_SPKMIXL_TO_SPKOUTR_MASK|WM8994_SPKMIXR_TO_SPKOUTL_MASK|WM8994_SPKMIXR_TO_SPKOUTR_MASK);
	val |= (WM8994_SPKMIXL_TO_SPKOUTL|WM8994_SPKMIXR_TO_SPKOUTL); // SPKMIXL/R_TO_SPKOUTL On
	wm8994_write(codec, WM8994_SPKOUT_MIXERS, val);
	//~code add

	// code add
	//wm8994_write(codec, 0x36, 0x000C); // MIXOUTL/R_TO_SPKMIXL/R
	//~code add

	/* Digital Path Enables and Unmutes*/
	//wm8994_write(codec, WM8994_SIDETONE, 0x01C0);

	wm8994_write(codec, 0x60, 0x0000);
	wm8994_write(codec, 0x54, 0x0000); // DC Servo 1

	wm8994_write(codec, 0x01, WM8994_SPKOUTL_ENA|WM8994_VMID_SEL_NORMAL|WM8994_BIAS_ENA);

	if(!wm8994->testmode_config_flag)
	{
		//Unmute DAC1 left
		val = wm8994_read(codec, WM8994_DAC1_LEFT_VOLUME);
		val &= ~(WM8994_DAC1L_MUTE_MASK|WM8994_DAC1L_VOL_MASK);
		val |= (WM8994_DAC1_VU|TUNING_DAC1L_VOL);
		wm8994_write(codec, WM8994_DAC1_LEFT_VOLUME, val);
	
		//Unmute and volume ctrl RightDAC
		val = wm8994_read(codec, WM8994_DAC1_RIGHT_VOLUME);
		val &= ~(WM8994_DAC1R_MUTE_MASK|WM8994_DAC1R_VOL_MASK);
		val |= (WM8994_DAC1_VU|TUNING_DAC1R_VOL); //0 db volume
		wm8994_write(codec, WM8994_DAC1_RIGHT_VOLUME, val);
	}

	//wm8994_write(codec, WM8994_AIF2_DAC_FILTERS_1, WM8994_AIF1DAC1_UNMUTE);
#endif
	printk(SND_KERN_DEBUG "[WM8994] wm8994_set_voicecall_speaker()\n");
}

void wm8994_set_voicecall_bluetooth(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = codec->private_data;

	int val;

	DEBUG_LOG("");

	audio_ctrl_mic_bias_gpio(codec);
#if 1
	/***** from S1-EUR: voice_common_settings() *****/

	/* GPIO Configuration */
	wm8994_write(codec, WM8994_GPIO_1, 0xA101);
	wm8994_write(codec, WM8994_GPIO_2, 0x8100);
	wm8994_write(codec, WM8994_GPIO_3, 0x0100);
	wm8994_write(codec, WM8994_GPIO_4, 0x0100);
	wm8994_write(codec, WM8994_GPIO_5, 0x8100);
	wm8994_write(codec, WM8994_GPIO_6, 0xA101);
	wm8994_write(codec, WM8994_GPIO_7, 0x0100);
	wm8994_write(codec, WM8994_GPIO_8, 0xA101);
	wm8994_write(codec, WM8994_GPIO_9, 0xA101);
	wm8994_write(codec, WM8994_GPIO_10, 0xA101);
	wm8994_write(codec, WM8994_GPIO_11, 0xA101);

	/* FLL2	Setting */
	wm8994_write(codec, WM8994_FLL2_CONTROL_2, 0x2F00); // FLL1 Ctrl2, FLL1 Setting
	wm8994_write(codec, WM8994_FLL2_CONTROL_3, 0x3126); // FLL1 Ctrl3, K Value
	wm8994_write(codec, WM8994_FLL2_CONTROL_4, 0x0100); // FLL1 Ctrl4, N Value
	wm8994_write(codec, WM8994_FLL2_CONTROL_5, 0x0C88); // FLL1 Ctrl5
	wm8994_write(codec, WM8994_FLL2_CONTROL_1, (WM8994_FLL2_FRACN_ENA|WM8994_FLL2_ENA));

	wm8994_write(codec, WM8994_AIF2_CLOCKING_1, 0x0018); // AIF2 Clock Source = FLL2
	//wm8994_write(codec, WM8994_AIF2_CLOCKING_1, 0x0009); // Enable AIF2 Clock, AIF2 Clock Source = MCLK2 (from CP)

	/* Clocking - 8KHz */
	wm8994_write(codec, WM8994_AIF2_RATE, (0x3<<WM8994_AIF2CLK_RATE_SHIFT));

	/* AIF2 Interface - PCM Stereo mode */
	wm8994_write(codec, WM8994_AIF2_CONTROL_1, 0x4118); // DSP Mode, BCLK invert, LRCLK normal
	wm8994_write(codec, WM8994_AIF2_BCLK, 0x70);
	wm8994_write(codec, WM8994_AIF2_CONTROL_2, 0x0000);	// Left & Right DAC receives left interface data
	//wm8994_write(codec, WM8994_AIF2_CONTROL_2, 0x4000); // 
	wm8994_write(codec, WM8994_AIF2_MASTER_SLAVE, 0x0000);

	/* Digital Path Enables and Unmutes */
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_5, (WM8994_AIF2DACL_ENA|WM8994_AIF2DACR_ENA|WM8994_AIF1DAC1L_ENA|WM8994_AIF1DAC1R_ENA));

	/* Clocking */
	val = wm8994_read(codec, WM8994_CLOCKING_1);
	val |= (WM8994_DSP_FS2CLK_ENA);
	wm8994_write(codec, WM8994_CLOCKING_1, val);

	wm8994_write(codec, WM8994_POWER_MANAGEMENT_6, 0x0);
#endif
#if 1
	wm8994_write(codec, WM8994_GPIO_3, (WM8994_GP3_DIR|WM8994_GP3_DB));
	wm8994_write(codec, WM8994_GPIO_4, (WM8994_GP4_DIR|WM8994_GP4_DB));
	wm8994_write(codec, WM8994_GPIO_5, (WM8994_GP5_DIR|WM8994_GP5_DB));	
	//wm8994_write(codec, WM8994_GPIO_6, (WM8994_GP6_DIR|WM8994_GP6_PD|WM8994_GP6_DB|(0x1<<WM8994_GP6_FN_SHIFT)));
	wm8994_write(codec, WM8994_GPIO_7, WM8994_GP7_DB);

	wm8994_write(codec, WM8994_GPIO_8, (WM8994_GP8_DIR|WM8994_GP8_DB));
	wm8994_write(codec, WM8994_GPIO_9, WM8994_GP9_DB);
	wm8994_write(codec, WM8994_GPIO_10, WM8994_GP10_DB);
	wm8994_write(codec, WM8994_GPIO_11, WM8994_GP11_DB);

	/* Analog Output Power Management */
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1);
	val &= ~(WM8994_SPKOUTL_ENA_MASK|WM8994_HPOUT2_ENA_MASK|WM8994_HPOUT1L_ENA_MASK|WM8994_HPOUT1R_ENA_MASK);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

	/* Digital Path Enables and Unmutes */
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_4, WM8994_AIF2ADCL_ENA|WM8994_ADCL_ENA);

	// If Input MIC is enabled, bluetooth Rx is muted.
	wm8994_write(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME, WM8994_IN1L_MUTE);
	wm8994_write(codec, WM8994_RIGHT_LINE_INPUT_1_2_VOLUME, WM8994_IN1R_MUTE);
	wm8994_write(codec, WM8994_INPUT_MIXER_2, 0x0000);
	wm8994_write(codec, WM8994_INPUT_MIXER_3, 0x0000);
	wm8994_write(codec, WM8994_INPUT_MIXER_4, 0x0000);

	//for BT DTMF Play
	//Rx Path: AIF2ADCDAT2 select
	//CP(CALL) Path:GPIO5/DACDAT2 select
	//AP(DTMF) Path: DACDAT1 select
	//Tx Path: GPIO8/DACDAT3 select
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_6, 0x000C);

	// AIF1 & AIF2 Output is connected to DAC1	
	wm8994_write(codec, WM8994_DAC2_LEFT_MIXER_ROUTING, WM8994_AIF2DACL_TO_DAC2L | WM8994_AIF1DAC1L_TO_DAC2L);
	wm8994_write(codec, WM8994_DAC2_RIGHT_MIXER_ROUTING, WM8994_AIF2DACR_TO_DAC2R | WM8994_AIF1DAC1R_TO_DAC2R);

	wm8994_write(codec, WM8994_AIF2_CLOCKING_1, 0x0019); // AIF2 Clock Source = FLL2

	wm8994_write(codec, WM8994_DAC2_MIXER_VOLUMES, 0x000C); 

	wm8994_write(codec, WM8994_DAC2_LEFT_VOLUME, 0x01C0);
	wm8994_write(codec, WM8994_DAC2_RIGHT_VOLUME, 0x01C0); 

	wm8994_write(codec, WM8994_OVERSAMPLING, 0x0000);

	//Unmute DAC1 left
	val = wm8994_read(codec, WM8994_DAC1_LEFT_VOLUME);
	val &= ~(WM8994_DAC1L_MUTE_MASK|WM8994_DAC1L_VOL_MASK);
	val |= (TUNING_DAC1L_VOL);
	wm8994_write(codec, WM8994_DAC1_LEFT_VOLUME, val);
	
	//Unmute and volume ctrl RightDAC
	val = wm8994_read(codec, WM8994_DAC1_RIGHT_VOLUME); 
	val &= ~(WM8994_DAC1R_MUTE_MASK|WM8994_DAC1R_VOL_MASK);
	val |= (TUNING_DAC1R_VOL);
	wm8994_write(codec, WM8994_DAC1_RIGHT_VOLUME, val);
	
	wm8994_write(codec, WM8994_AIF1_DAC1_FILTERS_1, 0x0000);  
	wm8994_write(codec, WM8994_AIF2_DAC_FILTERS_1, 0x0080);
#endif
	printk(SND_KERN_DEBUG "[WM8994] wm8994_set_voicecall_bluetooth()\n");
}

#define FUNCTION_LIST_VOIPCALL

void close_output_path_all(struct snd_soc_codec *codec)
{
	int val;

	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1);
	val &= ~(WM8994_SPKOUTR_ENA|WM8994_SPKOUTL_ENA|WM8994_HPOUT2_ENA|WM8994_HPOUT1L_ENA|WM8994_HPOUT1R_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

	wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, 0x0000);

	val = wm8994_read(codec, WM8994_AIF1_CONTROL_2); 
	val &= ~(WM8994_AIF1DAC_BOOST_MASK);
	wm8994_write(codec, WM8994_AIF1_CONTROL_2, val);
}

void wm8994_set_voipcall_receiver(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = codec->private_data;

	u16 val;

    close_output_path_all(codec);
	DEBUG_LOG("");

    val = wm8994_read(codec, WM8994_OUTPUT_MIXER_5);
	val &= ~(WM8994_DACL_MIXOUTL_VOL_MASK);
	val |= (TUNING_RCV_OUTMIX5_VOL<<WM8994_DACL_MIXOUTL_VOL_SHIFT);
	wm8994_write(codec, WM8994_OUTPUT_MIXER_5, val);

	val = wm8994_read(codec, WM8994_OUTPUT_MIXER_6);
	val &= ~(WM8994_DACR_MIXOUTR_VOL_MASK);
	val |= (TUNING_RCV_OUTMIX6_VOL<<WM8994_DACR_MIXOUTR_VOL_SHIFT);
	wm8994_write(codec, WM8994_OUTPUT_MIXER_6, val);

	if(!wm8994->testmode_config_flag)
	{
		val = wm8994_read(codec, WM8994_LEFT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTL_MUTE_N_MASK|WM8994_MIXOUTL_VOL_MASK);
		val |= (WM8994_MIXOUT_VU|WM8994_MIXOUTL_MUTE_N|0x3F);//TUNING_RCV_OPGAL_VOL
		wm8994_write(codec, WM8994_LEFT_OPGA_VOLUME, val);

		val = wm8994_read(codec, WM8994_RIGHT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTR_MUTE_N_MASK|WM8994_MIXOUTR_VOL_MASK);
		val |= (WM8994_MIXOUT_VU|WM8994_MIXOUTR_MUTE_N|0x3F);//TUNING_RCV_OPGAR_VOL
		wm8994_write(codec, WM8994_RIGHT_OPGA_VOLUME, val);
	}

    val = wm8994_read(codec, WM8994_HPOUT2_VOLUME);
	val &= ~(WM8994_HPOUT2_MUTE_MASK|WM8994_HPOUT2_VOL_MASK);
    val |= (TUNING_HPOUT2_VOL<<WM8994_HPOUT2_VOL_SHIFT);
	wm8994_write(codec, WM8994_HPOUT2_VOLUME, val);

	val = wm8994_read(codec, WM8994_DAC1_LEFT_VOLUME);
	val &= ~(WM8994_DAC1L_MUTE_MASK|WM8994_DAC1L_VOL_MASK);
	val |= (WM8994_DAC1_VU|TUNING_DAC1L_VOL); // 0xC0 = 0dB (MAX)
	wm8994_write(codec, WM8994_DAC1_LEFT_VOLUME, val);

	val = wm8994_read(codec, WM8994_DAC1_RIGHT_VOLUME); 
	val &= ~(WM8994_DAC1R_MUTE_MASK|WM8994_DAC1R_VOL_MASK);
	val |= (WM8994_DAC1_VU|TUNING_DAC1R_VOL);
	wm8994_write(codec, WM8994_DAC1_RIGHT_VOLUME, val);

	if(!wm8994->testmode_config_flag)
	{
		val = wm8994_read(codec, WM8994_AIF1_CONTROL_2);
		val &= ~(WM8994_AIF1DAC_BOOST_MASK);
		val |= (0x02<<WM8994_AIF1DAC_BOOST_SHIFT); // 00 = 0dB, 01 = +6dB, 02 = +12dB, 03 = +18dB
		wm8994_write(codec, WM8994_AIF1_CONTROL_2, val);
	}

	val = wm8994_read(codec, WM8994_OUTPUT_MIXER_1);
	val &= ~(WM8994_DAC1L_TO_MIXOUTL_MASK);
	val |= (WM8994_DAC1L_TO_MIXOUTL);
	wm8994_write(codec, WM8994_OUTPUT_MIXER_1, val);

	val = wm8994_read(codec, WM8994_OUTPUT_MIXER_2);
	val &= ~(WM8994_DAC1R_TO_MIXOUTR_MASK);
	val |= (WM8994_DAC1R_TO_MIXOUTR);
	wm8994_write(codec, WM8994_OUTPUT_MIXER_2, val);

	val = wm8994_read(codec, WM8994_HPOUT2_MIXER);
	val &= ~(WM8994_MIXOUTLVOL_TO_HPOUT2_MASK|WM8994_MIXOUTRVOL_TO_HPOUT2_MASK);
	val |= (WM8994_MIXOUTRVOL_TO_HPOUT2|WM8994_MIXOUTLVOL_TO_HPOUT2);
	wm8994_write(codec, WM8994_HPOUT2_MIXER, val);

	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_5);
	//val &= ~(WM8994_AIF1DAC1L_ENA_MASK|WM8994_AIF1DAC1R_ENA_MASK|WM8994_DAC1L_ENA_MASK|WM8994_DAC1R_ENA_MASK);
	//val |= (WM8994_AIF1DAC1L_ENA|WM8994_AIF1DAC1R_ENA|WM8994_DAC1L_ENA|WM8994_DAC1R_ENA);
	val &= ~(WM8994_AIF1DAC1L_ENA_MASK|WM8994_AIF1DAC1R_ENA_MASK|WM8994_DAC1L_ENA_MASK);
	val |= (WM8994_AIF1DAC1L_ENA|WM8994_AIF1DAC1R_ENA|WM8994_DAC1L_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_5, val);

	val = wm8994_read(codec, WM8994_AIF1_DAC1_FILTERS_1);
	val &= ~(WM8994_AIF1DAC1_MUTE_MASK|WM8994_AIF1DAC1_MONO_MASK);
	val |= (WM8994_AIF1DAC1_UNMUTE|WM8994_AIF1DAC1_MONO);
	wm8994_write(codec, WM8994_AIF1_DAC1_FILTERS_1, val);

	val = wm8994_read(codec, WM8994_DAC1_LEFT_MIXER_ROUTING);
	val &= ~(WM8994_AIF1DAC1L_TO_DAC1L_MASK);
	val |= (WM8994_AIF1DAC1L_TO_DAC1L);
	wm8994_write(codec, WM8994_DAC1_LEFT_MIXER_ROUTING, val);

	//val = wm8994_read(codec, WM8994_DAC1_RIGHT_MIXER_ROUTING);
	//val &= ~(WM8994_AIF1DAC1R_TO_DAC1R_MASK);
	//val |= (WM8994_AIF1DAC1R_TO_DAC1R);
	//wm8994_write(codec, WM8994_DAC1_RIGHT_MIXER_ROUTING, val);

	val = wm8994_read(codec, WM8994_CLOCKING_1);
	val &= ~(WM8994_DSP_FS1CLK_ENA_MASK|WM8994_DSP_FSINTCLK_ENA_MASK);
	val |= (WM8994_DSP_FS1CLK_ENA|WM8994_DSP_FSINTCLK_ENA);
	wm8994_write(codec, WM8994_CLOCKING_1, val);

	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_3);
	val &= ~(WM8994_MIXOUTLVOL_ENA_MASK|WM8994_MIXOUTRVOL_ENA_MASK|WM8994_MIXOUTL_ENA_MASK|WM8994_MIXOUTR_ENA_MASK);
	val |= (WM8994_MIXOUTL_ENA|WM8994_MIXOUTR_ENA|WM8994_MIXOUTRVOL_ENA|WM8994_MIXOUTLVOL_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, val);

	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1);
	val &= ~(WM8994_BIAS_ENA_MASK|WM8994_VMID_SEL_MASK|WM8994_HPOUT2_ENA_MASK);
	val |= (WM8994_BIAS_ENA|WM8994_VMID_SEL_NORMAL|WM8994_HPOUT2_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);
#if 1
	printk("[WM8994] wm8994_set_voipcall_receiver - rec main mic\n");

	// 300H // Mixing left channel output to right channel // val: 0x0010
	val = wm8994_read(codec, WM8994_AIF1_CONTROL_1);
	val &= ~(WM8994_AIF1ADCL_SRC_MASK|WM8994_AIF1ADCR_SRC_MASK);
	wm8994_write(codec, WM8994_AIF1_CONTROL_1, val);

	wm8994_write(codec, WM8994_ANTIPOP_2, 0x68); // Main mic volume issue fix

	// 01H // VMID_SEL_NORMAL, BIAS_ENA, MICB1_ENA
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1);
	val &= ~(WM8994_VMID_SEL_MASK|WM8994_BIAS_ENA_MASK);
	val |= (WM8994_VMID_SEL_NORMAL|WM8994_BIAS_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

	// 700H // GPIO1 // GP1_DIR IN, GP1_PD EN, GP1_DB DE-BOUNCE, GP1_FN = LOGIC LVL 0
	wm8994_write(codec, WM8994_GPIO_1, 0xA101);

	// 02H // MIXINL_ENA, IN1L_ENA
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_2);
	val &= ~(WM8994_MIXINL_ENA_MASK|WM8994_IN1L_ENA_MASK);
	val |= (WM8994_MIXINL_ENA|WM8994_IN1L_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_2, val);

	if(!wm8994->testmode_config_flag)
	{	
		// 18H // IN1L PGA // IN1L UNMUTE, SET VOL
		val = wm8994_read(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME);
		val &= ~(WM8994_IN1L_MUTE_MASK|WM8994_IN1L_VOL_MASK);
		val |= (WM8994_IN1L_VU|0x1A);
		wm8994_write(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME, val);

		// 29H // MIXINL PGA // IN2L_TO_MIXINL MUTE, IN1L_TO_MIXINL UNMUTE, 0dB
		val = wm8994_read(codec, WM8994_INPUT_MIXER_3);
		val &= ~(WM8994_IN1L_TO_MIXINL_MASK|WM8994_IN1L_MIXINL_VOL_MASK|WM8994_MIXOUTL_MIXINL_VOL_MASK);
		//val |= (WM8994_IN1L_TO_MIXINL|WM8994_IN1L_MIXINL_VOL); // Boost On(+30dB)
		val |= (WM8994_IN1L_TO_MIXINL);	// 0db
		wm8994_write(codec, WM8994_INPUT_MIXER_3, val);
	}
#if 0
	if(wm8994->recognition_active == REC_ON)
	{
		// Voice Search Tuning
		// 400H // AIF1 ADC1 Left Volume
		val = wm8994_read(codec, WM8994_AIF1_ADC1_LEFT_VOLUME);
		val &= ~(WM8994_AIF1ADC1L_VOL_MASK);
		val |= (WM8994_AIF1ADC1_VU|TUNING_RECOGNITION_MAIN_AIF1ADCL_VOL); // ADC Digital Gain
		wm8994_write(codec, WM8994_AIF1_ADC1_LEFT_VOLUME, val);
	}
#endif
	// 28H // INPUT MIXER // IN1LP/N_TO_IN1L PGA
	val = wm8994_read(codec, WM8994_INPUT_MIXER_2);	
	val &= (WM8994_IN1LP_TO_IN1L_MASK|WM8994_IN1LN_TO_IN1L_MASK);
	val |= (WM8994_IN1LP_TO_IN1L|WM8994_IN1LN_TO_IN1L);
	wm8994_write(codec, WM8994_INPUT_MIXER_2, val);
#if 0
	if(wm8994->recognition_active == REC_ON)
	{
		// 410H // AIF1 ADC1 Filters // AIF1 ADC1 hi-path filter on
		val = wm8994_read(codec, WM8994_AIF1_ADC1_FILTERS);
		val &= ~(WM8994_AIF1ADC1L_HPF|WM8994_AIF1ADC1R_HPF);
		val |= (WM8994_AIF1ADC1L_HPF|WM8994_AIF1ADC1R_HPF); // hi-path filter on (L/R)
		wm8994_write(codec, WM8994_AIF1_ADC1_FILTERS, 0x3000);
	}
#endif
	// 04H // AIF1ADC1L_ENA, ADCL_ENA 
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_4);
	val &= ~(WM8994_AIF1ADC1L_ENA_MASK|WM8994_ADCL_ENA_MASK);
	val |= (WM8994_AIF1ADC1L_ENA|WM8994_ADCL_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_4, val);

	// 606H // ADC1L_TO_AIF1ADC1L (TIMESLOT 0) ASSIGN
	val = wm8994_read(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING);
	val &= ~(WM8994_ADC1L_TO_AIF1ADC1L_MASK);
	val |= (WM8994_ADC1L_TO_AIF1ADC1L);
	wm8994_write(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING, val);
#endif    
    printk(SND_KERN_DEBUG "[WM8994] wm8994_set_voipcall_receiver()\n");
}

void wm8994_set_voipcall_headset(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = codec->private_data;

	u16 val;

    close_output_path_all(codec);
	DEBUG_LOG("");
    
	u16 TestReturn1 = 0;
	u16 TestReturn2 = 0;
	u16 TestLow1 = 0;
	u16 TestHigh1 = 0;
	u8 TestLow = 0;
	u8 TestHigh = 0;

	//Configuring the Digital Paths
	// Enable the Timeslot0 to DAC1L
	val = wm8994_read(codec, WM8994_DAC1_LEFT_MIXER_ROUTING);
	val &= ~(WM8994_AIF1DAC1L_TO_DAC1L_MASK);
	val |= WM8994_AIF1DAC1L_TO_DAC1L;
	wm8994_write(codec, WM8994_DAC1_LEFT_MIXER_ROUTING, val);

	//Enable the Timeslot0 to DAC1R
	val = wm8994_read(codec, WM8994_DAC1_RIGHT_MIXER_ROUTING);
	val &= ~(WM8994_AIF1DAC1R_TO_DAC1R_MASK);
	val |= WM8994_AIF1DAC1R_TO_DAC1R;
	wm8994_write(codec, WM8994_DAC1_RIGHT_MIXER_ROUTING, val);

	val = wm8994_read(codec, 0x102);
	val &= ~(0x0003);
	val = (0x0003);
	wm8994_write(codec,0x102, val);
	
	val = wm8994_read(codec, 0x56);
	val &= ~(0x0003);
	val = (0x0003);
	wm8994_write(codec,0x56, val);
	
	val = wm8994_read(codec, 0x102);
	val &= ~(0x0000);
	val = (0x0000);
	wm8994_write(codec,0x102, val);

	val = wm8994_read(codec, WM8994_CLASS_W_1);
	val &= ~(0x0005);
	val |= (0x0005);
	wm8994_write(codec, WM8994_CLASS_W_1, val);

	// Headset Control
	if(!wm8994->testmode_config_flag)
	{
		val = wm8994_read(codec, WM8994_LEFT_OUTPUT_VOLUME);
		val &= ~(WM8994_HPOUT1L_MUTE_N_MASK|WM8994_HPOUT1L_VOL_MASK);
		val |= (WM8994_HPOUT1_VU|WM8994_HPOUT1L_MUTE_N|TUNING_MP3_OUTPUTL_VOL);
		wm8994_write(codec, WM8994_LEFT_OUTPUT_VOLUME, val);

		val = wm8994_read(codec, WM8994_RIGHT_OUTPUT_VOLUME);
		val &= ~(WM8994_HPOUT1R_MUTE_N_MASK|WM8994_HPOUT1R_VOL_MASK);
		val |= (WM8994_HPOUT1_VU|WM8994_HPOUT1R_MUTE_N|TUNING_MP3_OUTPUTR_VOL);
		wm8994_write(codec, WM8994_RIGHT_OUTPUT_VOLUME, val);

		val = wm8994_read(codec, WM8994_LEFT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTL_MUTE_N_MASK|WM8994_MIXOUTL_VOL_MASK);
		val |= (WM8994_MIXOUT_VU|WM8994_MIXOUTL_MUTE_N|TUNING_MP3_OPGAL_VOL);
		wm8994_write(codec, WM8994_LEFT_OPGA_VOLUME, val);
	
		val = wm8994_read(codec, WM8994_RIGHT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTR_MUTE_N_MASK|WM8994_MIXOUTR_VOL_MASK);
		val |= (WM8994_MIXOUT_VU|WM8994_MIXOUTR_MUTE_N|TUNING_MP3_OPGAR_VOL);
		wm8994_write(codec, WM8994_RIGHT_OPGA_VOLUME, val);
	}
	
	val = wm8994_read(codec, WM8994_DC_SERVO_2);
	val &= ~(0x03E0);
	val = (0x03E0);
	wm8994_write(codec, WM8994_DC_SERVO_2, val);

	//Enable vmid,bias, hp left and right
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1);
	val &= ~(WM8994_BIAS_ENA_MASK|WM8994_VMID_SEL_MASK|WM8994_HPOUT1L_ENA_MASK|WM8994_HPOUT1R_ENA_MASK);
	val |= (WM8994_BIAS_ENA|WM8994_VMID_SEL_NORMAL|WM8994_HPOUT1R_ENA|WM8994_HPOUT1L_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

	val = wm8994_read(codec, WM8994_ANALOGUE_HP_1);
	val &= ~(0x0022);
	val = 0x0022;
	wm8994_write(codec, WM8994_ANALOGUE_HP_1, val);

	//Enable Charge Pump	
	val = wm8994_read(codec, WM8994_CHARGE_PUMP_1);
	val &= ~(WM8994_CP_ENA_MASK);
	val |= (WM8994_CP_ENA|WM8994_CP_ENA_DEFAULT); // this is from wolfson
	wm8994_write(codec, WM8994_CHARGE_PUMP_1, val);

	msleep(5);// 20ms delay

	//Enable Dac1 and DAC2 and the Timeslot0 for AIF1	
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_5);
	val &= ~(WM8994_DAC1R_ENA_MASK|WM8994_DAC1L_ENA_MASK|WM8994_AIF1DAC1R_ENA_MASK| WM8994_AIF1DAC1L_ENA_MASK);
	val |= (WM8994_AIF1DAC1L_ENA|WM8994_AIF1DAC1R_ENA|WM8994_DAC1L_ENA|WM8994_DAC1R_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_5, val);

	// Enable DAC1L to HPOUT1L path
	val = wm8994_read(codec, WM8994_OUTPUT_MIXER_1);
	val &=  ~(WM8994_DAC1L_TO_HPOUT1L_MASK|WM8994_DAC1L_TO_MIXOUTL_MASK);
	val |= (WM8994_DAC1L_TO_MIXOUTL);  	
	wm8994_write(codec, WM8994_OUTPUT_MIXER_1, val);

	// Enable DAC1R to HPOUT1R path
	val = wm8994_read(codec, WM8994_OUTPUT_MIXER_2);
	val &= ~(WM8994_DAC1R_TO_HPOUT1R_MASK|WM8994_DAC1R_TO_MIXOUTR_MASK);
	val |= (WM8994_DAC1R_TO_MIXOUTR);
	wm8994_write(codec, WM8994_OUTPUT_MIXER_2, val);

	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_3);
	val &= ~(WM8994_MIXOUTLVOL_ENA_MASK|WM8994_MIXOUTRVOL_ENA_MASK|WM8994_MIXOUTL_ENA_MASK|WM8994_MIXOUTR_ENA_MASK);
	val |= (WM8994_MIXOUTLVOL_ENA|WM8994_MIXOUTRVOL_ENA|WM8994_MIXOUTL_ENA|WM8994_MIXOUTR_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, 0x0030);

	val = wm8994_read(codec, WM8994_DC_SERVO_1);
	val &= ~(0x0303);
	val = (0x0303);
	wm8994_write(codec, WM8994_DC_SERVO_1, val);

	msleep(160); // 160ms delay

	TestReturn1 = wm8994_read(codec, WM8994_DC_SERVO_4);

	TestLow=(signed char)(TestReturn1 & 0xff);
	TestHigh=(signed char)((TestReturn1>>8)&0xff);
	TestLow1 = ((signed short)(TestLow-5))&0x00ff;
	TestHigh1 = (((signed short)(TestHigh-5))<<8)&0xff00;
	TestReturn2 = (TestLow1|TestHigh1);
	wm8994_write(codec, WM8994_DC_SERVO_4, TestReturn2);

	val = wm8994_read(codec, WM8994_DC_SERVO_1);
	val &= ~(0x000F);
	val = (0x000F);
	wm8994_write(codec, WM8994_DC_SERVO_1, val);

	msleep(20);

	// Intermediate HP settings
	val = wm8994_read(codec, WM8994_ANALOGUE_HP_1);
	val &= ~(WM8994_HPOUT1R_DLY_MASK|WM8994_HPOUT1R_OUTP_MASK|WM8994_HPOUT1R_RMV_SHORT_MASK|
		WM8994_HPOUT1L_DLY_MASK|WM8994_HPOUT1L_OUTP_MASK|WM8994_HPOUT1L_RMV_SHORT_MASK);
	val = (WM8994_HPOUT1L_RMV_SHORT|WM8994_HPOUT1L_OUTP|WM8994_HPOUT1L_DLY|WM8994_HPOUT1R_RMV_SHORT|
		WM8994_HPOUT1R_OUTP|WM8994_HPOUT1R_DLY);
	wm8994_write(codec, WM8994_ANALOGUE_HP_1, val);

	//Unmute DAC1 left
	val = wm8994_read(codec, WM8994_DAC1_LEFT_VOLUME);
	val &= ~(WM8994_DAC1L_MUTE_MASK|WM8994_DAC1L_VOL_MASK);
	val |= (TUNING_DAC1L_VOL);
	wm8994_write(codec, WM8994_DAC1_LEFT_VOLUME, val);

	//Unmute and volume ctrl RightDAC
	val = wm8994_read(codec, WM8994_DAC1_RIGHT_VOLUME);
	val &= ~(WM8994_DAC1R_MUTE_MASK|WM8994_DAC1R_VOL_MASK);
	val |= (TUNING_DAC1R_VOL); //0 db volume
	wm8994_write(codec, WM8994_DAC1_RIGHT_VOLUME, val);

	// Unmute the AF1DAC1	
	val = wm8994_read(codec, WM8994_AIF1_DAC1_FILTERS_1);
	val &= ~(WM8994_AIF1DAC1_MUTE_MASK|WM8994_AIF1DAC1_MONO_MASK);
	val |= (WM8994_AIF1DAC1_UNMUTE);
	wm8994_write(codec, WM8994_AIF1_DAC1_FILTERS_1, val);

	printk("[WM8994] wm8994_set_voipcall_headset - rec ear mic\n");
	// 300H // Mixing left channel output to right channel // val: 0x0010
	val = wm8994_read(codec, WM8994_AIF1_CONTROL_1);
	val &= ~(WM8994_AIF1ADCL_SRC_MASK|WM8994_AIF1ADCR_SRC_MASK);
	val |= (WM8994_AIF1ADCL_SRC|WM8994_AIF1ADCR_SRC);
	wm8994_write(codec, WM8994_AIF1_CONTROL_1, val);

	wm8994_write(codec, WM8994_ANTIPOP_2, 0x68); // Ear mic volume issue fix

	// 01H // VMID_SEL, BIAS_ENA
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1);
	val &= ~(WM8994_VMID_SEL_MASK|WM8994_BIAS_ENA_MASK);
	val |= (WM8994_VMID_SEL_NORMAL|WM8994_BIAS_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

	// 700H // GPIO1 // GP1_DIR IN, GP1_PD EN, GP1_DB DE-BOUNCE, GP1_FN = LOGIC LVL 0
	wm8994_write(codec, WM8994_GPIO_1, 0xA101);

	// 02H // MIXINR_ENA, IN1R_ENA
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_2);
	val &= ~(WM8994_MIXINR_ENA_MASK|WM8994_IN1R_ENA_MASK);
	val |= (WM8994_MIXINR_ENA|WM8994_IN1R_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_2, val);

	if(!wm8994->testmode_config_flag)
	{	
		// 1AH // IN1R PGA // IN1R UNMUTE, SET VOL
		val = wm8994_read(codec, WM8994_RIGHT_LINE_INPUT_1_2_VOLUME);
		val &= ~(WM8994_IN1R_MUTE_MASK|WM8994_IN1R_VOL_MASK);
		//val |= (WM8994_IN1R_VU|TUNING_RECOGNITION_SUB_INPUTMIX_VOL);
		val |= (WM8994_IN1R_VU|TUNING_RECORD_EAR_INPUTMIX_VOL);
		wm8994_write(codec, WM8994_RIGHT_LINE_INPUT_1_2_VOLUME, val);

		// 2AH // MIXINR PGA // IN2R_TO_MIXINR MUTE, IN1R_TO_MIXINR UNMUTE, 0dB
		val = wm8994_read(codec, WM8994_INPUT_MIXER_4);
		val &= ~(WM8994_IN1R_TO_MIXINR_MASK|WM8994_IN1R_MIXINR_VOL_MASK|WM8994_MIXOUTR_MIXINR_VOL_MASK);
		//val |= (WM8994_IN1R_TO_MIXINR|WM8994_IN1R_MIXINR_VOL); //30db
		val |= (WM8994_IN1R_TO_MIXINR); //0db
		wm8994_write(codec, WM8994_INPUT_MIXER_4, val);
	}

	// 28H // INPUT MIXER // IN1RP/N_TO_IN1R PGA
	val = wm8994_read(codec, WM8994_INPUT_MIXER_2);
	val &= ~(WM8994_IN1RP_TO_IN1R_MASK|WM8994_IN1RN_TO_IN1R_MASK);
	val |= (WM8994_IN1RP_TO_IN1R|WM8994_IN1RN_TO_IN1R);
	wm8994_write(codec, WM8994_INPUT_MIXER_2, val);

	// 04H // AIF1ADC1R_ENA, ADCR_ENA
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_4);
	val &= ~(WM8994_AIF1ADC1R_ENA_MASK|WM8994_ADCR_ENA_MASK);
	val |= (WM8994_AIF1ADC1R_ENA|WM8994_ADCR_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_4, val);

	// 607H // ADC1R_TO_AIF1ADC1R (TIMESLOT 0) ASSIGN
	val = wm8994_read(codec, WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING);
	val &= ~(WM8994_ADC1R_TO_AIF1ADC1R_MASK);
	val |= (WM8994_ADC1R_TO_AIF1ADC1R);
	wm8994_write(codec, WM8994_AIF1_ADC1_RIGHT_MIXER_ROUTING, val);
    
    printk(SND_KERN_DEBUG "[WM8994] wm8994_set_voipcall_headset()\n");
}

void wm8994_set_voipcall_headphone(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = codec->private_data;

	u16 val;

    close_output_path_all(codec);
	DEBUG_LOG("");
    
	u16 TestReturn1 = 0;
	u16 TestReturn2 = 0;
	u16 TestLow1 = 0;
	u16 TestHigh1 = 0;
	u8 TestLow = 0;
	u8 TestHigh = 0;

	//Configuring the Digital Paths
	// Enable the Timeslot0 to DAC1L
	val = wm8994_read(codec, WM8994_DAC1_LEFT_MIXER_ROUTING);
	val &= ~(WM8994_AIF1DAC1L_TO_DAC1L_MASK);
	val |= WM8994_AIF1DAC1L_TO_DAC1L;
	wm8994_write(codec, WM8994_DAC1_LEFT_MIXER_ROUTING, val);

	//Enable the Timeslot0 to DAC1R
	val = wm8994_read(codec, WM8994_DAC1_RIGHT_MIXER_ROUTING);
	val &= ~(WM8994_AIF1DAC1R_TO_DAC1R_MASK);
	val |= WM8994_AIF1DAC1R_TO_DAC1R;
	wm8994_write(codec, WM8994_DAC1_RIGHT_MIXER_ROUTING, val);

	val = wm8994_read(codec, 0x102);
	val &= ~(0x0003);
	val = (0x0003);
	wm8994_write(codec,0x102, val);
	
	val = wm8994_read(codec, 0x56);
	val &= ~(0x0003);
	val = (0x0003);
	wm8994_write(codec,0x56, val);
	
	val = wm8994_read(codec, 0x102);
	val &= ~(0x0000);
	val = (0x0000);
	wm8994_write(codec,0x102, val);

	val = wm8994_read(codec, WM8994_CLASS_W_1);
	val &= ~(0x0005);
	val |= (0x0005);
	wm8994_write(codec, WM8994_CLASS_W_1, val);

	// Headset Control
	if(!wm8994->testmode_config_flag)
	{
		val = wm8994_read(codec, WM8994_LEFT_OUTPUT_VOLUME);
		val &= ~(WM8994_HPOUT1L_MUTE_N_MASK|WM8994_HPOUT1L_VOL_MASK);
		val |= (WM8994_HPOUT1_VU|WM8994_HPOUT1L_MUTE_N|TUNING_MP3_OUTPUTL_VOL);
		wm8994_write(codec, WM8994_LEFT_OUTPUT_VOLUME, val);

		val = wm8994_read(codec, WM8994_RIGHT_OUTPUT_VOLUME);
		val &= ~(WM8994_HPOUT1R_MUTE_N_MASK|WM8994_HPOUT1R_VOL_MASK);
		val |= (WM8994_HPOUT1_VU|WM8994_HPOUT1R_MUTE_N|TUNING_MP3_OUTPUTR_VOL);
		wm8994_write(codec, WM8994_RIGHT_OUTPUT_VOLUME, val);

		val = wm8994_read(codec, WM8994_LEFT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTL_MUTE_N_MASK|WM8994_MIXOUTL_VOL_MASK);
		val |= (WM8994_MIXOUT_VU|WM8994_MIXOUTL_MUTE_N|TUNING_MP3_OPGAL_VOL);
		wm8994_write(codec, WM8994_LEFT_OPGA_VOLUME, val);
	
		val = wm8994_read(codec, WM8994_RIGHT_OPGA_VOLUME);
		val &= ~(WM8994_MIXOUTR_MUTE_N_MASK|WM8994_MIXOUTR_VOL_MASK);
		val |= (WM8994_MIXOUT_VU|WM8994_MIXOUTR_MUTE_N|TUNING_MP3_OPGAR_VOL);
		wm8994_write(codec, WM8994_RIGHT_OPGA_VOLUME, val);
	}
	
	val = wm8994_read(codec, WM8994_DC_SERVO_2);
	val &= ~(0x03E0);
	val = (0x03E0);
	wm8994_write(codec, WM8994_DC_SERVO_2, val);

	//Enable vmid,bias, hp left and right
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1);
	val &= ~(WM8994_BIAS_ENA_MASK|WM8994_VMID_SEL_MASK|WM8994_HPOUT1L_ENA_MASK|WM8994_HPOUT1R_ENA_MASK);
	val |= (WM8994_BIAS_ENA|WM8994_VMID_SEL_NORMAL|WM8994_HPOUT1R_ENA|WM8994_HPOUT1L_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

	val = wm8994_read(codec, WM8994_ANALOGUE_HP_1);
	val &= ~(0x0022);
	val = 0x0022;
	wm8994_write(codec, WM8994_ANALOGUE_HP_1, val);

	//Enable Charge Pump	
	val = wm8994_read(codec, WM8994_CHARGE_PUMP_1);
	val &= ~(WM8994_CP_ENA_MASK);
	val |= (WM8994_CP_ENA|WM8994_CP_ENA_DEFAULT); // this is from wolfson
	wm8994_write(codec, WM8994_CHARGE_PUMP_1, val);

	msleep(5);// 20ms delay

	//Enable Dac1 and DAC2 and the Timeslot0 for AIF1	
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_5);
	val &= ~(WM8994_DAC1R_ENA_MASK|WM8994_DAC1L_ENA_MASK|WM8994_AIF1DAC1R_ENA_MASK| WM8994_AIF1DAC1L_ENA_MASK);
	val |= (WM8994_AIF1DAC1L_ENA|WM8994_AIF1DAC1R_ENA|WM8994_DAC1L_ENA|WM8994_DAC1R_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_5, val);

	// Enable DAC1L to HPOUT1L path
	val = wm8994_read(codec, WM8994_OUTPUT_MIXER_1);
	val &=  ~(WM8994_DAC1L_TO_HPOUT1L_MASK|WM8994_DAC1L_TO_MIXOUTL_MASK);
	val |= (WM8994_DAC1L_TO_MIXOUTL);  	
	wm8994_write(codec, WM8994_OUTPUT_MIXER_1, val);

	// Enable DAC1R to HPOUT1R path
	val = wm8994_read(codec, WM8994_OUTPUT_MIXER_2);
	val &= ~(WM8994_DAC1R_TO_HPOUT1R_MASK|WM8994_DAC1R_TO_MIXOUTR_MASK);
	val |= (WM8994_DAC1R_TO_MIXOUTR);
	wm8994_write(codec, WM8994_OUTPUT_MIXER_2, val);

	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_3);
	val &= ~(WM8994_MIXOUTLVOL_ENA_MASK|WM8994_MIXOUTRVOL_ENA_MASK|WM8994_MIXOUTL_ENA_MASK|WM8994_MIXOUTR_ENA_MASK);
	val |= (WM8994_MIXOUTL_ENA|WM8994_MIXOUTR_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, 0x0030);

	val = wm8994_read(codec, WM8994_DC_SERVO_1);
	val &= ~(0x0303);
	val = (0x0303);
	wm8994_write(codec, WM8994_DC_SERVO_1, val);

	msleep(160); // 160ms delay

	TestReturn1 = wm8994_read(codec, WM8994_DC_SERVO_4);

	TestLow=(signed char)(TestReturn1 & 0xff);
	TestHigh=(signed char)((TestReturn1>>8)&0xff);
	TestLow1 = ((signed short)(TestLow-5))&0x00ff;
	TestHigh1 = (((signed short)(TestHigh-5))<<8)&0xff00;
	TestReturn2 = (TestLow1|TestHigh1);
	wm8994_write(codec, WM8994_DC_SERVO_4, TestReturn2);

	val = wm8994_read(codec, WM8994_DC_SERVO_1);
	val &= ~(0x000F);
	val = (0x000F);
	wm8994_write(codec, WM8994_DC_SERVO_1, val);

	msleep(20);

	// Intermediate HP settings
	val = wm8994_read(codec, WM8994_ANALOGUE_HP_1);
	val &= ~(WM8994_HPOUT1R_DLY_MASK|WM8994_HPOUT1R_OUTP_MASK|WM8994_HPOUT1R_RMV_SHORT_MASK|
		WM8994_HPOUT1L_DLY_MASK|WM8994_HPOUT1L_OUTP_MASK|WM8994_HPOUT1L_RMV_SHORT_MASK);
	val = (WM8994_HPOUT1L_RMV_SHORT|WM8994_HPOUT1L_OUTP|WM8994_HPOUT1L_DLY|WM8994_HPOUT1R_RMV_SHORT|
		WM8994_HPOUT1R_OUTP|WM8994_HPOUT1R_DLY);
	wm8994_write(codec, WM8994_ANALOGUE_HP_1, val);

	//Unmute DAC1 left
	val = wm8994_read(codec, WM8994_DAC1_LEFT_VOLUME);
	val &= ~(WM8994_DAC1L_MUTE_MASK|WM8994_DAC1L_VOL_MASK);
	val |= (TUNING_DAC1L_VOL);
	wm8994_write(codec, WM8994_DAC1_LEFT_VOLUME, val);

	//Unmute and volume ctrl RightDAC
	val = wm8994_read(codec, WM8994_DAC1_RIGHT_VOLUME);
	val &= ~(WM8994_DAC1R_MUTE_MASK|WM8994_DAC1R_VOL_MASK);
	val |= (TUNING_DAC1R_VOL); //0 db volume
	wm8994_write(codec, WM8994_DAC1_RIGHT_VOLUME, val);

	// Unmute the AF1DAC1	
	val = wm8994_read(codec, WM8994_AIF1_DAC1_FILTERS_1);
	val &= ~(WM8994_AIF1DAC1_MUTE_MASK|WM8994_AIF1DAC1_MONO_MASK);
	val |= (WM8994_AIF1DAC1_UNMUTE);
	wm8994_write(codec, WM8994_AIF1_DAC1_FILTERS_1, val);

	printk("[WM8994] wm8994_set_voipcall_headphone - rec main mic\n");

	// 300H // Mixing left channel output to right channel // val: 0x0010
	val = wm8994_read(codec, WM8994_AIF1_CONTROL_1);
	val &= ~(WM8994_AIF1ADCL_SRC_MASK|WM8994_AIF1ADCR_SRC_MASK);
	wm8994_write(codec, WM8994_AIF1_CONTROL_1, val);

	wm8994_write(codec, WM8994_ANTIPOP_2, 0x68); // Main mic volume issue fix

	// 01H // VMID_SEL_NORMAL, BIAS_ENA, MICB1_ENA
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1);
	val &= ~(WM8994_VMID_SEL_MASK|WM8994_BIAS_ENA_MASK);
	val |= (WM8994_VMID_SEL_NORMAL|WM8994_BIAS_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

	// 700H // GPIO1 // GP1_DIR IN, GP1_PD EN, GP1_DB DE-BOUNCE, GP1_FN = LOGIC LVL 0
	wm8994_write(codec, WM8994_GPIO_1, 0xA101);

	// 02H // MIXINL_ENA, IN1L_ENA
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_2);
	val &= ~(WM8994_MIXINL_ENA_MASK|WM8994_IN1L_ENA_MASK);
	val |= (WM8994_MIXINL_ENA|WM8994_IN1L_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_2, val);

	if(!wm8994->testmode_config_flag)
	{	
		// 18H // IN1L PGA // IN1L UNMUTE, SET VOL
		val = wm8994_read(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME);
		val &= ~(WM8994_IN1L_MUTE_MASK|WM8994_IN1L_VOL_MASK);
		val |= (WM8994_IN1L_VU|0x18);
		wm8994_write(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME, val);

		// 29H // MIXINL PGA // IN2L_TO_MIXINL MUTE, IN1L_TO_MIXINL UNMUTE, 0dB
		val = wm8994_read(codec, WM8994_INPUT_MIXER_3);
		val &= ~(WM8994_IN1L_TO_MIXINL_MASK|WM8994_IN1L_MIXINL_VOL_MASK|WM8994_MIXOUTL_MIXINL_VOL_MASK);
		//val |= (WM8994_IN1L_TO_MIXINL|WM8994_IN1L_MIXINL_VOL); // Boost On(+30dB)
		val |= (WM8994_IN1L_TO_MIXINL);	// 0db
		wm8994_write(codec, WM8994_INPUT_MIXER_3, val);
	}
#if 0
	if(wm8994->recognition_active == REC_ON)
	{
		// Voice Search Tuning
		// 400H // AIF1 ADC1 Left Volume
		val = wm8994_read(codec, WM8994_AIF1_ADC1_LEFT_VOLUME);
		val &= ~(WM8994_AIF1ADC1L_VOL_MASK);
		val |= (WM8994_AIF1ADC1_VU|TUNING_RECOGNITION_MAIN_AIF1ADCL_VOL); // ADC Digital Gain
		wm8994_write(codec, WM8994_AIF1_ADC1_LEFT_VOLUME, val);
	}
#endif
	// 28H // INPUT MIXER // IN1LP/N_TO_IN1L PGA
	val = wm8994_read(codec, WM8994_INPUT_MIXER_2);	
	val &= (WM8994_IN1LP_TO_IN1L_MASK|WM8994_IN1LN_TO_IN1L_MASK);
	val |= (WM8994_IN1LP_TO_IN1L|WM8994_IN1LN_TO_IN1L);
	wm8994_write(codec, WM8994_INPUT_MIXER_2, val);
#if 0
	if(wm8994->recognition_active == REC_ON)
	{
		// 410H // AIF1 ADC1 Filters // AIF1 ADC1 hi-path filter on
		val = wm8994_read(codec, WM8994_AIF1_ADC1_FILTERS);
		val &= ~(WM8994_AIF1ADC1L_HPF|WM8994_AIF1ADC1R_HPF);
		val |= (WM8994_AIF1ADC1L_HPF|WM8994_AIF1ADC1R_HPF); // hi-path filter on (L/R)
		wm8994_write(codec, WM8994_AIF1_ADC1_FILTERS, 0x3000);
	}
#endif
	// 04H // AIF1ADC1L_ENA, ADCL_ENA 
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_4);
	val &= ~(WM8994_AIF1ADC1L_ENA_MASK|WM8994_ADCL_ENA_MASK);
	val |= (WM8994_AIF1ADC1L_ENA|WM8994_ADCL_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_4, val);

	// 606H // ADC1L_TO_AIF1ADC1L (TIMESLOT 0) ASSIGN
	val = wm8994_read(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING);
	val &= ~(WM8994_ADC1L_TO_AIF1ADC1L_MASK);
	val |= (WM8994_ADC1L_TO_AIF1ADC1L);
	wm8994_write(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING, val);
    
    printk(SND_KERN_DEBUG "[WM8994] wm8994_set_voipcall_headphone()\n");
}

void wm8994_set_voipcall_speaker(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = codec->private_data;

	u16 val;

    close_output_path_all(codec);
	DEBUG_LOG("");
    
	// 01H // Disable end point for preventing pop up noise.
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1);
	val &= ~(WM8994_SPKOUTL_ENA_MASK);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, 0x0003);

	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_3);
	val &= ~(WM8994_SPKLVOL_ENA_MASK);
	val |= (WM8994_SPKLVOL_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_3, val);

	val = wm8994_read(codec, WM8994_SPKMIXL_ATTENUATION);
	val &= ~(WM8994_SPKMIXL_VOL_MASK);
	val |= (TUNING_SPKMIXL_ATTEN);
	wm8994_write(codec, WM8994_SPKMIXL_ATTENUATION, val);

	val = wm8994_read(codec, WM8994_SPKMIXR_ATTENUATION);
	val &= ~(WM8994_SPKMIXR_VOL_MASK);
	//val |= (TUNING_SPKMIXR_ATTEN);
	wm8994_write(codec, WM8994_SPKMIXR_ATTENUATION, val);

	if(!wm8994->testmode_config_flag)
	{
		val = wm8994_read(codec, WM8994_SPEAKER_VOLUME_LEFT);
		val &= ~(WM8994_SPKOUTL_MUTE_N_MASK|WM8994_SPKOUTL_VOL_MASK);
		val |= (WM8994_SPKOUT_VU|WM8994_SPKOUTL_MUTE_N|0x3E);
		wm8994_write(codec, WM8994_SPEAKER_VOLUME_LEFT, val);

		val = wm8994_read(codec, WM8994_SPEAKER_VOLUME_RIGHT);
		val &= ~(WM8994_SPKOUTR_MUTE_N_MASK|WM8994_SPKOUTR_VOL_MASK);
		//val |= (WM8994_SPKOUT_VU|WM8994_SPKOUTR_MUTE_N|TUNING_MP3_SPKL_VOL);
		wm8994_write(codec, WM8994_SPEAKER_VOLUME_RIGHT, val);

		val = wm8994_read(codec, WM8994_CLASSD);
		val &= ~(WM8994_SPKOUTL_BOOST_MASK);
		val |= (0x07<<WM8994_SPKOUTL_BOOST_SHIFT);
		wm8994_write(codec, WM8994_CLASSD, val);
	}

	// 610H // DAC1 Left VOL, Unmute
	val = wm8994_read(codec, WM8994_DAC1_LEFT_VOLUME);
	val &= ~(WM8994_DAC1L_MUTE_MASK|WM8994_DAC1L_VOL_MASK);
	val |= (WM8994_DAC1_VU|TUNING_DAC1L_VOL); 
	wm8994_write(codec, WM8994_DAC1_LEFT_VOLUME, val);

	// 611H // DAC1 Right VOL, Unmute
	val = wm8994_read(codec, WM8994_DAC1_RIGHT_VOLUME); 
	val &= ~(WM8994_DAC1R_MUTE_MASK|WM8994_DAC1R_VOL_MASK);
	val |= (WM8994_DAC1_VU|TUNING_DAC1R_VOL);
	wm8994_write(codec, WM8994_DAC1_RIGHT_VOLUME, val);

	if(!wm8994->testmode_config_flag)
	{
		val = wm8994_read(codec, WM8994_AIF1_CONTROL_2); 
		val &= ~(WM8994_AIF1DAC_BOOST_MASK);
		val |= (0x01<<10); // 00 0dB 01 +6dB 02 +12dB 03 +18dB
		wm8994_write(codec, WM8994_AIF1_CONTROL_2, val);
	}

	val = wm8994_read(codec, WM8994_SPKOUT_MIXERS);
	val &= ~(WM8994_SPKMIXL_TO_SPKOUTL_MASK|WM8994_SPKMIXL_TO_SPKOUTR_MASK|WM8994_SPKMIXR_TO_SPKOUTR_MASK);
	val |= (WM8994_SPKMIXL_TO_SPKOUTL);
	wm8994_write(codec, WM8994_SPKOUT_MIXERS, val);

	// 36H // Unmute the DAC path
	val = wm8994_read(codec, WM8994_SPEAKER_MIXER);
	val &= ~(WM8994_DAC1L_TO_SPKMIXL_MASK);
	val |= (WM8994_DAC1L_TO_SPKMIXL);
	wm8994_write(codec, WM8994_SPEAKER_MIXER, val);

	// 05H // AIF1DAC1L/R_ENA, DAC1L_ENA
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_5);
	val &= ~(WM8994_AIF1DAC1L_ENA_MASK|WM8994_AIF1DAC1R_ENA_MASK|WM8994_DAC1L_ENA_MASK);
	val |= (WM8994_AIF1DAC1L_ENA|WM8994_AIF1DAC1R_ENA|WM8994_DAC1L_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_5, val);

	// 420H // AIF1DAC1 Unmute, Mono
	val = wm8994_read(codec, WM8994_AIF1_DAC1_FILTERS_1);
	val &= ~(WM8994_AIF1DAC1_MUTE_MASK|WM8994_AIF1DAC1_MONO_MASK);
	val |= (WM8994_AIF1DAC1_UNMUTE|WM8994_AIF1DAC1_MONO);
	wm8994_write(codec, WM8994_AIF1_DAC1_FILTERS_1, val);

	// 601H // AIF1DAC1L_TO_DAC1L // Enable timeslot0 to left dac
	val = wm8994_read(codec, WM8994_DAC1_LEFT_MIXER_ROUTING);
	val &= ~(WM8994_AIF1DAC1L_TO_DAC1L_MASK);
	val |= (WM8994_AIF1DAC1L_TO_DAC1L);
	wm8994_write(codec, WM8994_DAC1_LEFT_MIXER_ROUTING, val);
	
	// 01H // SPKOUTL_ENA
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1);
	val &= ~(WM8994_SPKOUTL_ENA_MASK|WM8994_VMID_SEL_MASK|WM8994_BIAS_ENA_MASK);
	val |= (WM8994_SPKOUTL_ENA|WM8994_VMID_SEL_NORMAL|WM8994_BIAS_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);
#if 1
	printk("[WM8994] wm8994_set_voipcall_speaker - rec main mic\n");

	// 300H // Mixing left channel output to right channel // val: 0x0010
	val = wm8994_read(codec, WM8994_AIF1_CONTROL_1);
	val &= ~(WM8994_AIF1ADCL_SRC_MASK|WM8994_AIF1ADCR_SRC_MASK);
	wm8994_write(codec, WM8994_AIF1_CONTROL_1, val);

	wm8994_write(codec, WM8994_ANTIPOP_2, 0x68); // Main mic volume issue fix

	// 01H // VMID_SEL_NORMAL, BIAS_ENA, MICB1_ENA
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_1);
	val &= ~(WM8994_VMID_SEL_MASK|WM8994_BIAS_ENA_MASK);
	val |= (WM8994_VMID_SEL_NORMAL|WM8994_BIAS_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, val);

	// 700H // GPIO1 // GP1_DIR IN, GP1_PD EN, GP1_DB DE-BOUNCE, GP1_FN = LOGIC LVL 0
	wm8994_write(codec, WM8994_GPIO_1, 0xA101);

	// 02H // MIXINL_ENA, IN1L_ENA
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_2);
	val &= ~(WM8994_MIXINL_ENA_MASK|WM8994_IN1L_ENA_MASK);
	val |= (WM8994_MIXINL_ENA|WM8994_IN1L_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_2, val);

	if(!wm8994->testmode_config_flag)
	{	
		// 18H // IN1L PGA // IN1L UNMUTE, SET VOL
		val = wm8994_read(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME);
		val &= ~(WM8994_IN1L_MUTE_MASK|WM8994_IN1L_VOL_MASK);
		val |= (WM8994_IN1L_VU|0x1F);
		wm8994_write(codec, WM8994_LEFT_LINE_INPUT_1_2_VOLUME, val);

		// 29H // MIXINL PGA // IN2L_TO_MIXINL MUTE, IN1L_TO_MIXINL UNMUTE, 0dB
		val = wm8994_read(codec, WM8994_INPUT_MIXER_3);
		val &= ~(WM8994_IN1L_TO_MIXINL_MASK|WM8994_IN1L_MIXINL_VOL_MASK|WM8994_MIXOUTL_MIXINL_VOL_MASK);
		//val |= (WM8994_IN1L_TO_MIXINL|WM8994_IN1L_MIXINL_VOL); // Boost On(+30dB)
		val |= (WM8994_IN1L_TO_MIXINL);
		wm8994_write(codec, WM8994_INPUT_MIXER_3, val);
	}
#if 0
	if(wm8994->recognition_active == REC_ON)
	{
		// Voice Search Tuning
		// 400H // AIF1 ADC1 Left Volume
		val = wm8994_read(codec, WM8994_AIF1_ADC1_LEFT_VOLUME);
		val &= ~(WM8994_AIF1ADC1L_VOL_MASK);
		val |= (WM8994_AIF1ADC1_VU|TUNING_RECOGNITION_MAIN_AIF1ADCL_VOL); // ADC Digital Gain
		wm8994_write(codec, WM8994_AIF1_ADC1_LEFT_VOLUME, val);
	}
#endif
	// 28H // INPUT MIXER // IN1LP/N_TO_IN1L PGA
	val = wm8994_read(codec, WM8994_INPUT_MIXER_2);	
	val &= (WM8994_IN1LP_TO_IN1L_MASK|WM8994_IN1LN_TO_IN1L_MASK);
	val |= (WM8994_IN1LP_TO_IN1L|WM8994_IN1LN_TO_IN1L);
	wm8994_write(codec, WM8994_INPUT_MIXER_2, val);
#if 0
	if(wm8994->recognition_active == REC_ON)
	{
		// 410H // AIF1 ADC1 Filters // AIF1 ADC1 hi-path filter on
		val = wm8994_read(codec, WM8994_AIF1_ADC1_FILTERS);
		val &= ~(WM8994_AIF1ADC1L_HPF|WM8994_AIF1ADC1R_HPF);
		val |= (WM8994_AIF1ADC1L_HPF|WM8994_AIF1ADC1R_HPF); // hi-path filter on (L/R)
		wm8994_write(codec, WM8994_AIF1_ADC1_FILTERS, 0x3000);
	}
#endif
	// 04H // AIF1ADC1L_ENA, ADCL_ENA 
	val = wm8994_read(codec, WM8994_POWER_MANAGEMENT_4);
	val &= ~(WM8994_AIF1ADC1L_ENA_MASK|WM8994_ADCL_ENA_MASK);
	val |= (WM8994_AIF1ADC1L_ENA|WM8994_ADCL_ENA);
	wm8994_write(codec, WM8994_POWER_MANAGEMENT_4, val);

	// 606H // ADC1L_TO_AIF1ADC1L (TIMESLOT 0) ASSIGN
	val = wm8994_read(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING);
	val &= ~(WM8994_ADC1L_TO_AIF1ADC1L_MASK);
	val |= (WM8994_ADC1L_TO_AIF1ADC1L);
	wm8994_write(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING, val);
#endif    
    printk(SND_KERN_DEBUG "[WM8994] wm8994_set_voipcall_speaker()\n");
}

void wm8994_set_voipcall_bluetooth(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = codec->private_data;

	u16 val;

    close_output_path_all(codec);
	DEBUG_LOG("");

    /* GPIO Configuration */
	wm8994_write(codec, WM8994_GPIO_1, 0xA101);
    wm8994_write(codec, WM8994_GPIO_2, 0xA101);
    wm8994_write(codec, WM8994_GPIO_3, 0xA101);
    wm8994_write(codec, WM8994_GPIO_4, 0xA101);
    wm8994_write(codec, WM8994_GPIO_5, 0xA101);
	wm8994_write(codec, WM8994_GPIO_6, 0xA101);
    wm8994_write(codec, WM8994_GPIO_7, 0xA101);
	wm8994_write(codec, WM8994_GPIO_8, 0xA101);
	wm8994_write(codec, WM8994_GPIO_9, 0xA101);
	wm8994_write(codec, WM8994_GPIO_10, 0xA101);
	wm8994_write(codec, WM8994_GPIO_11, 0xA101);

	/* FLL2	Setting */
	wm8994_write(codec, WM8994_FLL2_CONTROL_2, 0x2F00); // FLL1 Ctrl2, FLL1 Setting
	wm8994_write(codec, WM8994_FLL2_CONTROL_3, 0x3126); // FLL1 Ctrl3, K Value
	wm8994_write(codec, WM8994_FLL2_CONTROL_4, 0x0100); // FLL1 Ctrl4, N Value
	wm8994_write(codec, WM8994_FLL2_CONTROL_5, 0x0C88); // FLL1 Ctrl5
	wm8994_write(codec, WM8994_FLL2_CONTROL_1, (WM8994_FLL2_FRACN_ENA|WM8994_FLL2_ENA));

	wm8994_write(codec, WM8994_AIF2_CLOCKING_1, 0x0018); // AIF2 Clock Source = FLL2

	/* Clocking - 8KHz */
    wm8994_write(codec, WM8994_AIF2_RATE, 0x0003);

	/* AIF2 Interface - PCM Stereo mode */
	wm8994_write(codec, WM8994_AIF2_CONTROL_1, 0x4118); // DSP Mode, BCLK invert, LRCLK normal
    wm8994_write(codec, WM8994_AIF2_BCLK, 0x40);
    wm8994_write(codec, WM8994_AIF2_CONTROL_2, 0x4000); // Left & Right DAC receives left interface data
    wm8994_write(codec, WM8994_AIF2_MASTER_SLAVE, 0x7000);

    /* Analog Output Power Management */
    wm8994_write(codec, WM8994_POWER_MANAGEMENT_1, 0x0003);
    wm8994_write(codec, WM8994_POWER_MANAGEMENT_2, 0x6000);

	/* Digital Path Enables and Unmutes */
    wm8994_write(codec, WM8994_POWER_MANAGEMENT_4, 0x2202); // AIF2ADCL_ENA, AIF1ADC1L_ENA, ADCL_ENA
    wm8994_write(codec, WM8994_POWER_MANAGEMENT_5, 0x220A); // AIF2DACL_ENA, AIF1DAC1L_ENA, DAC2L_ENA, DAC1L_ENA

	/* Clocking */
	val = wm8994_read(codec, WM8994_CLOCKING_1);
	val |= (WM8994_DSP_FS2CLK_ENA);
	wm8994_write(codec, WM8994_CLOCKING_1, val);

    wm8994_write(codec, WM8994_GPIO_3, 0x0100); // 702h // GPIO3|BCLK2
    wm8994_write(codec, WM8994_GPIO_5, 0x0100); // 704h // GPIO5|DACDAT2
    wm8994_write(codec, WM8994_GPIO_8, 0x8100); // GPIO8|DACDAT3
    wm8994_write(codec, WM8994_GPIO_9, 0x0100); // GPIO9|ADCDAT3
    wm8994_write(codec, WM8994_GPIO_10, 0x0100); // GPIO10|LRCLK3
    wm8994_write(codec, WM8994_GPIO_11, 0x0100); // GPIO11|BCLK3

	//for BT DTMF Play
	//Rx Path: AIF2ADCDAT2 select
    //CP(CALL) Path: Analog
	//AP(DTMF) Path: DACDAT1 select
	//Tx Path: GPIO8/DACDAT3 select
    wm8994_write(codec, WM8994_POWER_MANAGEMENT_6, 0x000A);

    wm8994_write(codec, 0x604, WM8994_AIF1DAC1L_TO_DAC2L); // WM8994_DAC2_LEFT_MIXER_ROUTING
    wm8994_write(codec, 0x612, 0x01C0); // DAC2L/R_VOL, 0dB, update

	wm8994_write(codec, WM8994_OVERSAMPLING, 0x0000);
    wm8994_write(codec, WM8994_AIF2_CLOCKING_1, 0x0019); // AIF2 Clock Source = FLL2

	wm8994_write(codec, WM8994_AIF1_DAC1_FILTERS_1, 0x0000);  
	wm8994_write(codec, WM8994_AIF2_DAC_FILTERS_1, 0x0080);

    // 300H // Mixing left channel output to right channel // val: 0x0010
	val = wm8994_read(codec, WM8994_AIF1_CONTROL_1);
	val &= ~(WM8994_AIF1ADCL_SRC_MASK|WM8994_AIF1ADCR_SRC_MASK);
	wm8994_write(codec, WM8994_AIF1_CONTROL_1, val);

	// 606H // ADC1L_TO_AIF1ADC1L (TIMESLOT 0) ASSIGN
	val = wm8994_read(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING);
	val &= ~(WM8994_ADC1L_TO_AIF1ADC1L_MASK);
	val |= (WM8994_AIF2DACL_TO_AIF1ADC1L);
	wm8994_write(codec, WM8994_AIF1_ADC1_LEFT_MIXER_ROUTING, val);
    
    printk(SND_KERN_DEBUG "[WM8994] wm8994_set_voipcall_bluetooth()\n");
}


