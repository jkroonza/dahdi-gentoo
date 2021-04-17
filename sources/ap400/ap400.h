/*
 * AP4XX  T1/E1 PCI Driver
 *
 * Written by Ronaldo Valiati <aligera@aligera.com.br>
 *
 * Based on previous works, designs, and archetectures conceived and
 * written by Jim Dixon <jim@lambdatel.com> and Mark Spencer <markster@digium.com>.
 *
 * Copyright (C) 2001 Jim Dixon / Zapata Telephony.
 * Copyright (C) 2001-2005, Digium, Inc.
 *
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/ioctl.h>


#define AP4_GET_ALARMS  _IOW (DAHDI_CODE, 60, int)
#define AP4_GET_SLIPS	_IOW (DAHDI_CODE, 61, int)

#define AP4XX_CARD_ID		0x41434532		// "ACE2"
#define APE4XX_CARD_ID		0x41504534		// "APE4"

#define AP_CAS_BASE		0x0080
#define AP_DATA_BASE		0x0100

#define AP_CARD_TYPE_REG	0x0001
#define AP_T1E1_CONFIG_REG	0x0003
#define AP_E1_CONFIG_REG	0x0004
#define AP_E1_STATUS_REG	0x0005
#define AP_LEDS_REG		0x0006
#define AP_CLKSRC_REG		0x0007
#define AP_HWCONFIG_REG		0x0008
#define AP_INT_CONTROL_REG	0x0009
#define AP_CNT_IRQ_REG		0x000B
#define AP_CNT_CV_REG		0x000C
#define AP_CNT_CRC_REG		0x000D
#define AP_CLEAR_IRQ_REG	0x000E
#define AP_CNT_SLIP_REG		0x000F

#define AP_HWID_MASK		0x00F0

#define AP_CLKSRC_MASK		0x07

#define AP_LIU1_LINECODE	0x0080
#define AP_LIU2_LINECODE	0x0100
#define AP_LIU_RESET_BIT	0x0200

#define AP_E1_AIS_STATUS	0x01
#define AP_E1_BFAE_STATUS	0x02
#define AP_E1_MFAE_STATUS	0x04
#define AP_E1_SYNC_STATUS	0x08
#define AP_E1_CAS_STATUS	0x10
#define AP_E1_LOS_STATUS	0x20
#define AP_E1_RAI_STATUS	0x40

#define AP_E1_RAI_CONFIG	0x01
#define AP_E1_LOOP_CONFIG	0x10
#define AP_E1_CASEN_CONFIG	0x20
#define AP_E1_PCM30_CONFIG	0x40
#define AP_E1_CRCEN_CONFIG	0x80

#define AP_INT_CTL_ENABLE	0x01
#define AP_INT_CTL_ACTIVE	0x02

#define AP_HWID_1E1_RJ		0x01
#define AP_HWID_2E1_RJ		0x00
#define AP_HWID_4E1_RJ		0x02
#define AP_HWID_T1		0x04

#define AP4_T1_NE1_SEL		0x04
#define AP4_T1_ESF_NSF		0x02
#define AP4_T1_CAS_ENABLE	0x01

#define AP4_T1_FRAME_SYNC	0x01


typedef enum {
	AP_PULS_E1_75 = 0,
	AP_PULS_E1_120,
	AP_PULS_DSX1_0FT,
	AP_PULS_DSX1_133FT,
	AP_PULS_DSX1_266FT,
	AP_PULS_DSX1_399FT,
	AP_PULS_DSX1_533FT,
	AP_PULS_J1_110,
	AP_PULS_DS1_0DB,
	AP_PULS_DS1_M075DB,
	AP_PULS_DS1_M150DB,
	AP_PULS_DS1_M225DB
} liu_mode;

