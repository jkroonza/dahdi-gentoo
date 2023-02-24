/*
 * Voicetronix OpenPCI Interface Driver for Zapata Telephony interface
 *
 * Written by Mark Spencer <markster@linux-support.net>
 *            Matthew Fredrickson <creslin@linux-support.net>
 *            Ben Kramer <ben@voicetronix.com.au>
 *            Ron Lee <ron@voicetronix.com.au>
 *
 * Copyright (C) 2001, Linux Support Services, Inc.
 * Copyright (C) 2005 - 2007, Voicetronix
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

/* Conditional debug options */
#define VERBOSE_TIMING 0

/* Driver constants */
#define DRIVER_DESCRIPTION  "Voicetronix OpenPCI DAHDI driver"
#define DRIVER_AUTHOR	"Mark Spencer <markster@digium.com> "\
			"Voicetronix <support@voicetronix.com.au>"

#define NAME      "wcopenpci"
#define MAX_PORTS 8	    /* Maximum number of ports on each carrier */
#define MAX_CARDS 8	    /* Maximum number of carriers per host */

#define DEFAULT_COUNTRY  "AUSTRALIA"


#include <linux/init.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/sched.h>

#include <dahdi/kernel.h>
#include <dahdi/version.h>
#include "proslic.h"
#include <dahdi/wctdm_user.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>

#include "fxo_modes.h"

#ifndef IRQF_DISABLED
#ifdef SA_INTERRUPT
#define IRQF_DISABLED		SA_INTERRUPT
#else
#define IRQF_DISABLED		0x0
#endif
#endif

static struct ps_country_reg {
	const char *country;
	unsigned short value;
} ps_country_regs[] = {
	{"ARGENTINA",  0x8},
	{"AUSTRALIA",  0xD},
	{"AUSTRIA",    0xD},
	{"BAHRAIN",    0xC},
	{"BELGIUM",    0xC},
	{"BRAZIL",     0x8},
	{"BULGARIA",   0xD},
	{"CANADA",     0x8},
	{"CHILE",      0x8},
	{"CHINA",      0xC},
	{"COLOMBIA",   0x8},
	{"CROATIA",    0xC},
	{"CYPRUS",     0xC},
	{"CZECH",      0xC},
	{"DENMARK",    0xC},
	{"ECUADOR",    0x8},
	{"EGYPT",      0x8},
	{"ELSALVADOR", 0x8},
	{"FINLAND",    0xC},
	{"FRANCE",     0xC},
	{"GERMANY",    0xD},
	{"GREECE",     0xC},
	{"GUAM",       0x8},
	{"HONGKONG",   0x8},
	{"HUNGARY",    0x8},
	{"ICELAND",    0xC},
	{"INDIA",      0xF},
	{"INDONESIA",  0x8},
	{"IRELAND",    0xC},
	{"ISRAEL",     0xC},
	{"ITALY",      0xC},
	{"JAPAN",      0x8},
	{"JORDAN",     0x8},
	{"KAZAKHSTAN", 0x8},
	{"KUWAIT",     0x8},
	{"LATVIA",     0xC},
	{"LEBANON",    0xC},
	{"LUXEMBOURG", 0xC},
	{"MACAO",      0x8},
	{"MALAYSIA",   0x8},
	{"MALTA",      0xC},
	{"MEXICO",     0x8},
	{"MOROCCO",    0xC},
	{"NETHERLANDS", 0xC},
	{"NEWZEALAND", 0xF},
	{"NIGERIA",    0xC},
	{"NORWAY",     0xC},
	{"OMAN",       0x8},
	{"PAKISTAN",   0x8},
	{"PERU",       0x8},
	{"PHILIPPINES", 0x8},
	{"POLAND",     0x8},
	{"PORTUGAL",   0xC},
	{"ROMANIA",    0x8},
	{"RUSSIA",     0x8},
	{"SAUDIARABIA", 0x8},
	{"SINGAPORE",  0x8},
	{"SLOVAKIA",   0xE},
	{"SLOVENIA",   0xE},
	{"SOUTHAFRICA", 0xE},
	{"SOUTHKOREA", 0x8},
	{"SPAIN",      0xC},
	{"SWEDEN",     0xC},
	{"SWITZERLAND", 0xC},
	{"SYRIA",      0x8},
	{"TAIWAN",     0x8},
	{"THAILAND",   0x8},
	{"UAE",        0x8},
	{"UK",         0xC},
	{"USA",        0x8},
	{"YEMEN",      0x8}
};

#define INOUT 2

/* Allocate enough memory for two zt chunks, receive and transmit.  Each
 * sample uses 32 bits.  Allocate an extra set just for control too */
#define VT_PCIDMA_BLOCKSIZE (DAHDI_MAX_CHUNKSIZE * INOUT * MAX_PORTS * 2 * 2)
#define VT_PCIDMA_MIDDLE    (DAHDI_MAX_CHUNKSIZE * MAX_PORTS - 4)
#define VT_PCIDMA_END       (DAHDI_MAX_CHUNKSIZE * MAX_PORTS * 2 - 4)

#define ID_DATA_MAXSIZE         30

#define NUM_CAL_REGS 12
#define NUM_FXO_REGS 60

#define TREG(addr)      (wc->ioaddr + addr)

#define TJ_CNTL         TREG(0x00)
#define TJ_OPER         TREG(0x01)
#define TJ_AUXC         TREG(0x02)
#define TJ_AUXD         TREG(0x03)
#define TJ_MASK0        TREG(0x04)
#define TJ_MASK1        TREG(0x05)
#define TJ_INTSTAT      TREG(0x06)
#define TJ_AUXR         TREG(0x07)

#define TJ_DMAWS        TREG(0x08)
#define TJ_DMAWI        TREG(0x0c)
#define TJ_DMAWE        TREG(0x10)
#define TJ_DMAWC        TREG(0x14)
#define TJ_DMARS        TREG(0x18)
#define TJ_DMARI        TREG(0x1c)
#define TJ_DMARE        TREG(0x20)
#define TJ_DMARC        TREG(0x24)

#define TJ_AUXINTPOL    TREG(0x2A)

#define TJ_AUXFUNC      TREG(0x2b)
#define TJ_SFDELAY      TREG(0x2c)
#define TJ_SERCTL       TREG(0x2d)
#define TJ_SFLC         TREG(0x2e)
#define TJ_FSCDELAY     TREG(0x2f)

#define TJ_REGBASE      TREG(0xc0)

#define PIB(addr)       (TJ_REGBASE + addr * 4)

#define HTXF_READY      (inb(PIB(0)) & 0x10)
#define HRXF_READY      (inb(PIB(0)) & 0x20)


#define VT_PORT_EMPTY	0
#define VT_PORT_VDAA	1   /* Voice DAA - FXO */
#define VT_PORT_PROSLIC	2   /* ProSLIC - FXS */

#define VBAT 0xC7

#define HKMODE_FWDACT   1
#define HKMODE_FWDONACT	2
#define HKMODE_RINGING	4

#define HOOK_ONHOOK     0
#define HOOK_OFFHOOK    1

#define	DSP_CODEC_RING		12	/* RING rising edge detected	*/
#define	DSP_CODEC_HKOFF		22	/* station port off hook        */
#define	DSP_CODEC_HKON		23	/* station port on hook         */
#define	DSP_RING_OFF		24	/* RING falling edge detected	*/
#define DSP_DROP		25

#define	DSP_CODEC_FLASH		26	/* station port hook flash      */

#define DSP_LOOP_OFFHOOK	38	/* Loop Off hook from OpenPCI   */
#define DSP_LOOP_ONHOOK		39	/* Loop On hook from OpenPCI    */
#define DSP_LOOP_POLARITY	40	/* Loop Polarity from OpenPCI   */
#define DSP_LOOP_NOBATT		41

#define DSP_PROSLIC_SANITY	50	/* Sanity alert from a ProSLIC port */
#define DSP_PROSLIC_PWR_ALARM	51	/* Power Alarm from a ProSLIC port  */
#define DSP_VDAA_ISO_FRAME_E	52	/* ISO-cap frame sync lost on VDAA port*/

#if VERBOSE_TIMING
 #define REPORT_WAIT(n,x)						    \
	 cardinfo(card->cardnum, #n " wait at %d, " #x " = %d", __LINE__, x )
#else
 #define REPORT_WAIT(n,x)
#endif

#define BUSY_WAIT(countvar,cond,delay,iter,failret)			    \
	countvar = 0;							    \
	while(cond){							    \
	    udelay(delay);						    \
	    if (++countvar > iter){					    \
		cardcrit(wc->boardnum, "busy wait FAILED at %d", __LINE__); \
		return failret;						    \
	    }								    \
	}								    \
	REPORT_WAIT(busy,i)

#define LOCKED_WAIT(countvar,cond,delay,iter,failret)			    \
	countvar = 0;							    \
	while(cond){							    \
	    udelay(delay);						    \
	    if (++countvar > iter){					    \
		dbginfo(wc->boardnum,"busy wait failed at %d",__LINE__);    \
		spin_unlock_irqrestore(&wc->lock, flags);		    \
		return failret;						    \
	    }								    \
	}								    \
	REPORT_WAIT(locked,i)

#define HTXF_WAIT()                 BUSY_WAIT(i,HTXF_READY,5,500,RET_FAIL)
#define HRXF_WAIT()                 BUSY_WAIT(i,!HRXF_READY,5,70000,RET_FAIL)
#define HTXF_WAIT_RET(failret)      BUSY_WAIT(i,HTXF_READY,5,500,failret)
#define HRXF_WAIT_RET(failret)      BUSY_WAIT(i,!HRXF_READY,5,1000,failret)

#define HTXF_WAIT_LOCKED()	    LOCKED_WAIT(i,HTXF_READY,5,500,RET_FAIL)
#define HRXF_WAIT_LOCKED()	    LOCKED_WAIT(i,!HRXF_READY,5,1000,RET_FAIL)
#define HTXF_WAIT_LOCKED_RET(failret) LOCKED_WAIT(i,HTXF_READY,5,500,failret)
#define HRXF_WAIT_LOCKED_RET(failret) LOCKED_WAIT(i,!HRXF_READY,5,1000,failret)


static struct openpci {
	struct pci_dev *dev;
	char *variety;
	int boardnum;
	int portcount;
	int porttype[MAX_PORTS];

        int firmware;
        char serial[ID_DATA_MAXSIZE];

	spinlock_t lock;

	//XXX Replace these with proper try_module_get locking in the dahdi driver.
	//int usecount;	//XXX
	//int dead;	//XXX
	union {
		struct {
			int offhook;
		} fxo;
		struct {
			int ohttimer;
			int idletxhookstate;  /* IDLE changing hook state */
			int lasttxhook;
		} fxs;
	} mod[MAX_PORTS];

	unsigned long		ioaddr;
	dma_addr_t		readdma;
	dma_addr_t		writedma;
	volatile unsigned int  *writechunk;  /* Double-word aligned write memory */
	volatile unsigned int  *readchunk;   /* Double-word aligned read memory */

	struct dahdi_chan _chans[MAX_PORTS];
	struct dahdi_chan *chans[MAX_PORTS];
	struct dahdi_device *ddev;
	struct dahdi_span span;
} *cards[MAX_CARDS];

/* You must hold this lock anytime you access or modify the cards[] array. */
static DEFINE_MUTEX(cards_mutex);

static unsigned char fxo_port_lookup[8] = { 0x0, 0x8, 0x4, 0xc, 0x10, 0x18, 0x14, 0x1c};
static unsigned char fxs_port_lookup[8] = { 0x0, 0x1, 0x2, 0x3, 0x10, 0x11, 0x12, 0x13};
static char wcopenpci[] = "Voicetronix OpenPCI";

static char *country = DEFAULT_COUNTRY;
static int reversepolarity;  /* = 0 */
static int debug;            /* = 0 */

module_param(country, charp, 0444);
module_param(debug, int, 0600);
module_param(reversepolarity, int, 0600);
MODULE_PARM_DESC(country, "Set the default country name");
MODULE_PARM_DESC(debug, "Enable verbose logging");

/* #define DEBUG_LOOP_VOLTAGE 1 */
#ifdef DEBUG_LOOP_VOLTAGE
 /* This param is a 32 bit bitfield where bit 1 << cardnum * 8 << portnum
  * will enable voltage monitoring on that port (fxo only presently)
  */
 static int voltmeter;        /* = 0 */
 module_param(voltmeter, int, 0600);
 MODULE_PARM_DESC(voltmeter, "Enable loop voltage metering");
#endif


/* boolean return values */
#define RET_OK   1
#define RET_FAIL 0

/* Convenience macros for logging */
#define info(format,...) printk(KERN_INFO NAME ": " format "\n" , ## __VA_ARGS__)
#define warn(format,...) printk(KERN_WARNING NAME ": " format "\n" , ## __VA_ARGS__)
#define crit(format,...) printk(KERN_CRIT NAME ": " format "\n" , ## __VA_ARGS__)
#define cardinfo(cardnum,format,...) info("[%02d] " format, cardnum , ## __VA_ARGS__)
#define cardwarn(cardnum,format,...) warn("[%02d] " format, cardnum , ## __VA_ARGS__)
#define cardcrit(cardnum,format,...) crit("[%02d] " format, cardnum , ## __VA_ARGS__)
#define dbginfo(cardnum,format,...) if (debug) info("[%02d] " format, cardnum , ## __VA_ARGS__)


static inline const char *porttype(struct openpci *wc, int port)
{
	switch( wc->porttype[port] ) {
		case VT_PORT_VDAA:    return "VDAA";
		case VT_PORT_PROSLIC: return "ProSLIC";
		case VT_PORT_EMPTY:   return "empty port";
		default:              return "unknown type";
	}
}


static int __read_reg_fxo(struct openpci *wc, int port, unsigned char reg,
		unsigned char *value)
{
	unsigned char portadr = fxo_port_lookup[port];
	int i;

	if (HRXF_READY) *value = inb(PIB(1));

	outb(0x11, PIB(1));    HTXF_WAIT();
	outb(0x2, PIB(1));     HTXF_WAIT();
	outb(portadr, PIB(1)); HTXF_WAIT();
	outb(reg, PIB(1));     HTXF_WAIT();
	HRXF_WAIT(); *value = inb(PIB(1));

	return RET_OK;
}

static int read_reg_fxo(struct openpci *wc, int port, unsigned char reg,
		unsigned char *value)
{
	unsigned long flags;

	spin_lock_irqsave(&wc->lock, flags);
	if (__read_reg_fxo(wc, port, reg, value)) {
		spin_unlock_irqrestore(&wc->lock, flags);
		return RET_OK;
	}
	spin_unlock_irqrestore(&wc->lock, flags);
	cardcrit(wc->boardnum, "FXO port %d, reg %d, read failed!", port, reg);
	return RET_FAIL;
}

static int __read_reg_fxs(struct openpci *wc, int port, unsigned char reg,
		unsigned char *value)
{
	unsigned char portadr = fxs_port_lookup[port];
	int i;

	if (HRXF_READY) *value = inb(PIB(1));

	outb(0x13, PIB(1));    HTXF_WAIT();
	outb(0x2, PIB(1));     HTXF_WAIT();
	outb(portadr, PIB(1)); HTXF_WAIT();
	outb(reg, PIB(1));     HTXF_WAIT();
	HRXF_WAIT(); *value = inb(PIB(1));

	return RET_OK;
}

static int read_reg_fxs(struct openpci *wc, int port, unsigned char reg,
		unsigned char *value)
{
	unsigned long flags;

	spin_lock_irqsave(&wc->lock, flags);
	if ( __read_reg_fxs(wc, port, reg, value) ) {
		spin_unlock_irqrestore(&wc->lock, flags);
		return RET_OK;
	}
	spin_unlock_irqrestore(&wc->lock, flags);
	cardcrit(wc->boardnum, "FXS port %d, reg %d, read failed!", port, reg);
	return RET_FAIL;
}

static int __write_reg_fxo(struct openpci *wc, int port, unsigned char reg,
		unsigned char value)
{
	unsigned char portadr = fxo_port_lookup[port];
	int i;

        outb(0x10, PIB(1) );   HTXF_WAIT();
        outb(0x3, PIB(1));     HTXF_WAIT();
        outb(portadr, PIB(1)); HTXF_WAIT();
        outb(reg, PIB(1));     HTXF_WAIT();
        outb(value, PIB(1));   HTXF_WAIT();

	return RET_OK;
}

static int write_reg_fxo(struct openpci *wc, int port, unsigned char reg,
		unsigned char value)
{
	unsigned long flags;

	spin_lock_irqsave(&wc->lock, flags);
	if (__write_reg_fxo(wc, port, reg, value)) {
		spin_unlock_irqrestore(&wc->lock, flags);
		return RET_OK;
	}
	spin_unlock_irqrestore(&wc->lock, flags);
	cardcrit(wc->boardnum, "FXO port %d, reg %d, write(%d) failed!",
			port, reg, value);
	return RET_FAIL;
}

static int __write_reg_fxs(struct openpci *wc, int port, unsigned char reg,
		unsigned char value)
{
	unsigned char portadr = fxs_port_lookup[port];
	int i;

        outb(0x12, PIB(1) );   HTXF_WAIT();
        outb(0x3, PIB(1));     HTXF_WAIT();
        outb(portadr, PIB(1)); HTXF_WAIT();
        outb(reg, PIB(1));     HTXF_WAIT();
        outb(value, PIB(1));   HTXF_WAIT();

	return RET_OK;
}

static int write_reg_fxs(struct openpci *wc, int port, unsigned char reg,
		unsigned char value)
{
	unsigned long flags;

	spin_lock_irqsave(&wc->lock, flags);
	if (__write_reg_fxs(wc, port, reg, value)) {
		spin_unlock_irqrestore(&wc->lock, flags);
		return RET_OK;
	}
	spin_unlock_irqrestore(&wc->lock, flags);
	cardcrit(wc->boardnum, "FXS port %d, reg %d, write(%d) failed!", port, reg, value);
	return RET_FAIL;
}

static int __wait_indreg_fxs(struct openpci *wc, int port)
{
	unsigned char value;
	int count = 100;

	while (--count) {
		if ( __read_reg_fxs(wc, port, I_STATUS, &value)) {
			if (value == 0)
				return RET_OK;
		} else {
			cardcrit(wc->boardnum,
				 "failed to read port %d PS_IND_ADDR_ST, retrying...",
				 port);
		}
		udelay(5);
	}
	cardcrit(wc->boardnum, "Failed to wait for indirect reg write to port %d", port);
	return RET_FAIL;
}

static int write_indreg_fxs(struct openpci *wc, int port, unsigned char reg,
		unsigned short value)
{
	unsigned long flags;

	spin_lock_irqsave(&wc->lock, flags);
	if (__wait_indreg_fxs(wc, port)
	 && __write_reg_fxs(wc, port, IDA_LO, value & 0xff)
	 && __write_reg_fxs(wc, port, IDA_HI, (value & 0xff00)>>8)
	 && __write_reg_fxs(wc, port, IAA, reg)
	 && __wait_indreg_fxs(wc, port))
	{
		spin_unlock_irqrestore(&wc->lock, flags);
		return RET_OK;
	}
	spin_unlock_irqrestore(&wc->lock, flags);
	cardcrit(wc->boardnum, "FXS indreg %d write failed on port %d",
			reg, port);
	return RET_FAIL;
}

static int read_indreg_fxs(struct openpci *wc, int port, unsigned char reg, unsigned short *value)
{
	unsigned long flags;
	unsigned char lo, hi;

	spin_lock_irqsave(&wc->lock, flags);
	if (__wait_indreg_fxs(wc, port)
	 && __write_reg_fxs(wc, port, IAA, reg)
	 && __wait_indreg_fxs(wc, port)
	 && __read_reg_fxs(wc, port, IDA_LO, &lo)
	 && __read_reg_fxs(wc, port, IDA_HI, &hi) )
	{
		*value = lo | hi << 8;
		spin_unlock_irqrestore(&wc->lock, flags);
		return RET_OK;
	}
	spin_unlock_irqrestore(&wc->lock, flags);
	return RET_FAIL;
}

static void start_dma(struct openpci *wc)
{
	outb(0x0f, TJ_CNTL);
	set_current_state(TASK_INTERRUPTIBLE);
	schedule_timeout(1);
	outb(0x01, TJ_CNTL);
	outb(0x01, TJ_OPER);
}

static void restart_dma(struct openpci *wc)
{
	/* Reset Master and TDM */
	outb(0x01, TJ_CNTL);
	outb(0x01, TJ_OPER);
}

/* You must hold the card spinlock to call this function */
static int __ping_arm(struct openpci *wc)
{
	int i;
	int pong = 0;

	while (pong != 0x02) {
		outb(0x02, PIB(1)); HTXF_WAIT();
		HRXF_WAIT(); pong = inb(PIB(1));
		dbginfo(wc->boardnum, "ping_arm returned %x", pong);
	}
	while (pong == 0x02) {
		// Poke no-ops into the arm while it is still returning data,
		// if 500 usec elapses with no further response from it then
		// the message queue is should be completely cleared.
		outb(0x00, PIB(1)); HTXF_WAIT();
		i = 100;
		while (!HRXF_READY && --i)
			udelay(5);
		if (i == 0)
			break;
		pong = inb(PIB(1));
		dbginfo(wc->boardnum, "ping_arm returned %x.", pong);
	}
	return RET_OK;
}

static void arm_event(struct openpci *wc, char *msg)
{
	int port = msg[0];

	switch (msg[1]) {
		case DSP_LOOP_OFFHOOK:
			dahdi_hooksig(wc->chans[port], DAHDI_RXSIG_OFFHOOK);
			dbginfo(wc->boardnum, "Port %d Loop OffHook", port);
			break;

		case DSP_LOOP_ONHOOK:
			dahdi_hooksig(wc->chans[port], DAHDI_RXSIG_ONHOOK);
			dbginfo(wc->boardnum, "Port %d Loop OnHook", port);
			break;

		case DSP_LOOP_POLARITY:
			dahdi_qevent_lock(wc->chans[port], DAHDI_EVENT_POLARITY);
			dbginfo(wc->boardnum, "Port %d Loop Polarity", port);
			break;

		case DSP_CODEC_RING:
			dahdi_hooksig(wc->chans[port], DAHDI_RXSIG_RING);
			dbginfo(wc->boardnum, "Port %d Ring On", port);
			break;

		case DSP_RING_OFF:
			dahdi_hooksig(wc->chans[port], DAHDI_RXSIG_OFFHOOK);
			dbginfo(wc->boardnum, "Port %d Ring Off", port);
			break;

		case DSP_CODEC_HKOFF:
			dahdi_hooksig(wc->chans[port], DAHDI_RXSIG_OFFHOOK);
			dbginfo(wc->boardnum, "Port %d Station OffHook", port);
			if (reversepolarity)
				wc->mod[port].fxs.idletxhookstate = 5;
			else
				wc->mod[port].fxs.idletxhookstate = 1;
			break;

		case DSP_CODEC_HKON:
			dahdi_hooksig(wc->chans[port], DAHDI_RXSIG_ONHOOK);
			dbginfo(wc->boardnum, "Port %d Station OnHook", port);
			if (reversepolarity)
				wc->mod[port].fxs.idletxhookstate = 6;
			else
				wc->mod[port].fxs.idletxhookstate = 2;
			break;

		case DSP_CODEC_FLASH:
			dahdi_qevent_lock(wc->chans[port],
					DAHDI_EVENT_WINKFLASH);
			dbginfo(wc->boardnum, "Port %d Station Flash", port);
			break;

		case DSP_DROP:
		case DSP_LOOP_NOBATT:
			break;

			//XXX What to do to recover from these?
		case DSP_PROSLIC_SANITY:
			dbginfo(wc->boardnum, "Port %d ProSlic has gone insane!", port);
			break;

		case DSP_PROSLIC_PWR_ALARM:
		{
			char errbuf[32] = " Unknown", *p = errbuf;
			int i = 49;

			msg[2] >>= 2;
			for (; i < 55; ++i, msg[2] >>= 1 )
			    if (msg[2] & 1) {
				    *(++p) = 'Q';
				    *(++p) = i;
				    *(++p) = ',';
			    }
			if (p != errbuf)
				*p = '\0';
			cardcrit(wc->boardnum,"%d: ProSlic power ALARM:%s",
					msg[0], errbuf);
			//write_reg_fxs(wc, port, 64, wc->mod[port].fxs.lasttxhook );
			return;
		}

		case DSP_VDAA_ISO_FRAME_E:
			dbginfo(wc->boardnum, "Port %d VDAA has lost ISO-Cap frame lock", port);
			break;

		default:
			cardwarn(wc->boardnum, "Unknown message from Arm[%d] for port %d",
						msg[1], port);
			break;
	}
}

/* You must hold the card spinlock to call this function */
static inline int __read_arm_byte( struct openpci *wc, unsigned char *msg)
{
	int i;

	HRXF_WAIT(); *msg = inb(PIB(1));
	return RET_OK;
}

static inline int read_arm_msg( struct openpci *wc, unsigned char *msg)
{
	unsigned long flags;
	int i, d, count;
	int ret = RET_OK;

	spin_lock_irqsave(&wc->lock, flags);
	outb(0x08, PIB(1)); HTXF_WAIT_LOCKED();
	//XXX Do we need to clear the interrupt flag even if this fails?
	HRXF_WAIT_LOCKED(); count = inb(PIB(1));
	if (count == 0) {
		ret = RET_FAIL;
	} else if ( count < 3 || count > 4 ) {
		cardcrit(wc->boardnum, "BOGUS arm message size %d, flushing queue", count);
		// NB: This may take a while (up to 500usec or more) to complete
		//     and we are in the isr at present when this is called, so
		//     we may miss an interrupt or two while this is done in the
		//     bottom half, but we are already in trouble, so...
		d = debug; debug = 5; __ping_arm( wc ); debug = d;
		ret = RET_FAIL;
	} else while (--count ) {
		if (! __read_arm_byte(wc, msg)) {
			cardcrit(wc->boardnum,
				 "Failed to read arm message %d more bytes expected",
				 count);
			ret = RET_FAIL;
			break;
		}
		++msg;
	}
	outb(0x09, PIB(1)); HTXF_WAIT_LOCKED();
	spin_unlock_irqrestore(&wc->lock, flags);
	return ret;
}

static void openpci_arm_work( void *cardptr )
{
	struct openpci *wc = (struct openpci*)cardptr;
	unsigned char armmsg[4];

	if (read_arm_msg(wc, armmsg))
		arm_event(wc, armmsg);
}


static inline void openpci_write(struct openpci *wc, unsigned char flags)
{
	int x,y;
	volatile unsigned int *writechunk;

	if (flags & 0x01)
		writechunk = wc->writechunk;
	else if (flags & 0x02)
		writechunk = wc->writechunk + DAHDI_CHUNKSIZE*2;
	else {
		cardcrit(wc->boardnum, "bad write interrupt flags %x, at %x",
					flags, inb(TJ_DMAWC) );
		return;
	}
	/* get data */
	dahdi_transmit(&wc->span);
	for (y = 0, x = 0; x < DAHDI_CHUNKSIZE; ++x) {
		/* Send a sample, as a 32-bit word */
#ifdef __BIG_ENDIAN
#warning No big endian support (yet)
#else
		/* transmit second 4 ports */
		writechunk[y] = 0;
		if (wc->porttype[4])
			writechunk[y] |= (wc->chans[4]->writechunk[x] << 24);
		else
			writechunk[y] |= (0x01 << 24);
		if (wc->porttype[5])
			writechunk[y] |= (wc->chans[5]->writechunk[x] << 16);
		if (wc->porttype[6])
			writechunk[y] |= (wc->chans[6]->writechunk[x] << 8);
		if (wc->porttype[7])
			writechunk[y] |= (wc->chans[7]->writechunk[x]);
		++y;

		/* transmit first 4 ports */
		writechunk[y] = 0x01000000;
		/* Make sure first port doesnt equal 0x00 */
		if (wc->porttype[0]) {
			if (wc->chans[0]->writechunk[x] == 0)
				writechunk[y] |= (0x01 << 24);
			else
				writechunk[y] |= (wc->chans[0]->writechunk[x] << 24);
		}
		//else writechunk[y] |= (0x00 << 24);
		if (wc->porttype[1])
			writechunk[y] |= (wc->chans[1]->writechunk[x] << 16);
		if (wc->porttype[2])
			writechunk[y] |= (wc->chans[2]->writechunk[x] << 8);
		if (wc->porttype[3])
			writechunk[y] |= (wc->chans[3]->writechunk[x]);
		++y;
#endif
	}
}

static inline void openpci_read(struct openpci *wc, unsigned char flags)
{
	int x,y;
	volatile unsigned int *readchunk;

	if (flags & 0x08)
		readchunk = wc->readchunk + DAHDI_CHUNKSIZE*2;
	else if (flags & 0x04)
		readchunk = wc->readchunk;
	else {
		cardcrit(wc->boardnum, "bad read interrupt flags %x, at %x",
					flags, inb(TJ_DMARC));
		return;
	}

	for (y = 0,x = 0; x < DAHDI_CHUNKSIZE; ++x) {
#ifdef __BIG_ENDIAN
#warning No big endian support (yet)
#else
		/* Receive first 4 ports */

		if (wc->porttype[0])
			wc->chans[0]->readchunk[x] = (readchunk[y] >> 24) & 0xff;
		if (wc->porttype[1])
			wc->chans[1]->readchunk[x] = (readchunk[y] >> 16) & 0xff;
		if (wc->porttype[2])
			wc->chans[2]->readchunk[x] = (readchunk[y] >> 8) & 0xff;
		if (wc->porttype[3])
			wc->chans[3]->readchunk[x] = (readchunk[y]) & 0xff;
		++y;
		/* Receive second 4 ports */
		if (wc->porttype[4])
			wc->chans[4]->readchunk[x] = (readchunk[y] >> 24) & 0xff;
		if (wc->porttype[5])
			wc->chans[5]->readchunk[x] = (readchunk[y] >> 16) & 0xff;
		if (wc->porttype[6])
			wc->chans[6]->readchunk[x] = (readchunk[y] >> 8) & 0xff;
		if (wc->porttype[7])
			wc->chans[7]->readchunk[x] = (readchunk[y]) & 0xff;
		++y;
#endif
	}
	/* XXX We're wasting 8 taps.  We should get closer :( */
	for (x = 0; x < MAX_PORTS; x++) {
		if (wc->porttype[x])
			dahdi_ec_chunk(wc->chans[x], wc->chans[x]->readchunk, wc->chans[x]->writechunk);
	}
	dahdi_receive(&wc->span);
}

static irqreturn_t openpci_isr(int irq, void *dev_id)
{
	struct openpci *wc = dev_id;
	unsigned long flags;
	unsigned char status;

	spin_lock_irqsave(&wc->lock, flags);
	status = inb(TJ_INTSTAT);
	outb(status, TJ_INTSTAT);

	if (!status) {
		if (inb(TJ_AUXR) & 0x02) {
			spin_unlock_irqrestore(&wc->lock, flags);
			return IRQ_NONE;
		}
		spin_unlock_irqrestore(&wc->lock, flags);
		openpci_arm_work(wc);
		return IRQ_HANDLED;
	}
	if (status & 0x10) {
		/* PCI Master abort */
		cardcrit(wc->boardnum, "PCI Master Abort.");
		/* Stop DMA, wait for watchdog */
		outb(0x00, TJ_OPER);
		spin_unlock_irqrestore(&wc->lock, flags);
		return IRQ_HANDLED;
	}
	spin_unlock_irqrestore(&wc->lock, flags);

	if (status & 0x20) {
		/* PCI Target abort */
		cardcrit(wc->boardnum, "PCI Target Abort.");
		return IRQ_HANDLED;
	}
	if (status & 0x03) {
		openpci_write(wc, status);
	}
	if (status & 0x0c) {
	    #ifdef DEBUG_LOOP_VOLTAGE
	    //{{{
		static int counter[MAX_CARDS];
		int card = wc->boardnum;
		int port = ++counter[card] & 0x07;
		int ignore = counter[card] & 0xf0;

		if (!ignore && (voltmeter & ((1 << (card * 8)) << port))) {
			unsigned char lv;
			if (wc->porttype[port] == VT_PORT_VDAA &&
					read_reg_fxo(wc, port, 29, &lv))
				cardinfo(wc->boardnum, "Port %d loop voltage %d",
							port, lv < 128 ? lv :
							lv - 256);
		}
	    //}}}
	    #endif
		openpci_read(wc, status);
	}

	return IRQ_HANDLED;
}

static int openpci_ioctl(struct dahdi_chan *chan, unsigned int cmd,
		unsigned long data)
{
	struct wctdm_stats stats;
	struct wctdm_regs regs;
	struct wctdm_regop regop;
	struct wctdm_echo_coefs echoregs;
	struct openpci *wc = chan->pvt;
	int port = chan->chanpos - 1;
	int x;

	switch (cmd) {
	case DAHDI_ONHOOKTRANSFER:
		if (wc->porttype[port] != VT_PORT_PROSLIC)
			return -EINVAL;
		if (get_user(x, (int __user *)data))
			return -EFAULT;
		wc->mod[port].fxs.ohttimer = x << 3;
		if (reversepolarity)
			wc->mod[port].fxs.idletxhookstate = 0x6;	/* OHT mode when idle */
		else
			wc->mod[port].fxs.idletxhookstate = 0x2;
		switch (wc->mod[port].fxs.lasttxhook) {
		    case 0x1:
		    case 0x5:
			if (reversepolarity)
				wc->mod[port].fxs.lasttxhook = 0x6;
			else
				wc->mod[port].fxs.lasttxhook = 0x2;
			if (!write_reg_fxs(wc, port, 64,
						wc->mod[port].fxs.lasttxhook))
				return -EIO;
		}
		break;
	case DAHDI_SETPOLARITY:
		if (get_user(x, (int __user *)data))
			return -EFAULT;
		if (wc->porttype[port] != VT_PORT_PROSLIC)
			return -EINVAL;
		/* Can't change polarity while ringing or when open */
		if ((wc->mod[port].fxs.lasttxhook == 0x04) ||
		    (wc->mod[port].fxs.lasttxhook == 0x00))
			return -EINVAL;

		if ((x && !reversepolarity) || (!x && reversepolarity))
			wc->mod[port].fxs.lasttxhook |= 0x04;
		else
			wc->mod[port].fxs.lasttxhook &= ~0x04;
		if (!write_reg_fxs(wc, port, 64, wc->mod[port].fxs.lasttxhook))
			return -EIO;
		break;
	case WCTDM_GET_STATS:
		if (wc->porttype[port] == VT_PORT_PROSLIC) {
			unsigned char	linevolt;
			if (read_reg_fxs(wc, port, 80, &linevolt))
				stats.tipvolt = linevolt * -376;
			else
				return -EIO;
			if (read_reg_fxs(wc, port, 81, &linevolt))
				stats.ringvolt = linevolt * -376;
			else
				return -EIO;
			if (read_reg_fxs(wc, port, 82, &linevolt))
				stats.batvolt = linevolt * -376;
			else
				return -EIO;
		} else if (wc->porttype[port] == VT_PORT_VDAA) {
			unsigned char	linevolt;
			if (read_reg_fxo(wc, port, 29, &linevolt))
				stats.tipvolt = stats.ringvolt = stats.batvolt = linevolt * 1000;
			else
				return -EIO;
		} else
			return -EINVAL;
		if (copy_to_user((void __user *)data, &stats, sizeof(stats)))
			return -EFAULT;
		break;
	case WCTDM_GET_REGS:
		if (wc->porttype[port] == VT_PORT_PROSLIC) {
			for (x = 0; x < NUM_INDIRECT_REGS; x++)
				if (!read_indreg_fxs(wc, port, x,
							&regs.indirect[x]))
					return -EIO;
			for (x = 0; x < NUM_REGS; x++)
				if (!read_reg_fxs(wc, port, x, &regs.direct[x]))
					return -EIO;
		} else {
			memset(&regs, 0, sizeof(regs));
			for (x = 0; x < NUM_FXO_REGS; x++) {
				if (!read_reg_fxo(wc, port, x, &regs.direct[x]))
					return -EIO;
			}
		}
		if (copy_to_user((void __user *)data, &regs, sizeof(regs)))
			return -EFAULT;
		break;
	case WCTDM_SET_REG:
		if (copy_from_user(&regop, (void __user *)data, sizeof(regop)))
			return -EFAULT;
		if (regop.indirect) {
			if (wc->porttype[port] != VT_PORT_PROSLIC)
				return -EINVAL;
			printk("Setting indirect %d to 0x%04x on %d\n",
				regop.reg, regop.val, chan->chanpos);
			if (!write_indreg_fxs(wc, port, regop.reg, regop.val))
				return -EIO;
		} else {
			regop.val &= 0xff;
			printk("Setting direct %d to %04x on %d\n",
				regop.reg, regop.val, chan->chanpos);
			if (wc->porttype[port] == VT_PORT_PROSLIC) {
				if (!write_reg_fxs(wc, port, regop.reg,
							regop.val))
					return -EIO;
			} else {
				if (!write_reg_fxo(wc, port, regop.reg,
							regop.val))
					return -EIO;
			}
		}
		break;
	case WCTDM_SET_ECHOTUNE:
		cardinfo(wc->boardnum, "Setting echo registers");
		if (copy_from_user(&echoregs, (void __user *)data, sizeof(echoregs)))
			return -EFAULT;

		if (wc->porttype[port] == VT_PORT_VDAA) {
			/* Set the ACIM and digital echo canceller registers */
			if (!write_reg_fxo(wc, port, 30, echoregs.acim)
			 || ! write_reg_fxo(wc, port, 45, echoregs.coef1)
			 || ! write_reg_fxo(wc, port, 46, echoregs.coef2)
			 || ! write_reg_fxo(wc, port, 47, echoregs.coef3)
			 || ! write_reg_fxo(wc, port, 48, echoregs.coef4)
			 || ! write_reg_fxo(wc, port, 49, echoregs.coef5)
			 || ! write_reg_fxo(wc, port, 50, echoregs.coef6)
			 || ! write_reg_fxo(wc, port, 51, echoregs.coef7)
			 || ! write_reg_fxo(wc, port, 52, echoregs.coef8))
			{
				cardcrit(wc->boardnum, "Failed to set echo registers");
				return -EIO;
			}
			break;
		} else {
			return -EINVAL;
		}
		break;
	default:
		return -ENOTTY;
	}
	return 0;
}

static int openpci_open(struct dahdi_chan *chan)
{
	struct openpci *wc = chan->pvt;
	if (!wc->porttype[chan->chanpos-1])
		return -ENODEV;

	//XXX This is WRONG and can prang in a race.  We must pass THIS_MODULE
	//    as the owner of the span that holds the pointer to this function,
	//    then bump the refcount in the dahdi code _BEFORE_ the potentially
	//    fatal call to an invalid pointer is made.
	//if (wc->dead) return -ENODEV;
	//wc->usecount++;
	try_module_get(THIS_MODULE);  //XXX

	return 0;
}

static inline struct openpci* openpci_from_span(struct dahdi_span *span) {
	return container_of(span, struct openpci, span);
}

static int openpci_watchdog(struct dahdi_span *span, int event)
{
	info("TDM: Restarting DMA");
	restart_dma(openpci_from_span(span));
	return 0;
}

static int openpci_close(struct dahdi_chan *chan)
{
	struct openpci *wc = chan->pvt;
	int port = chan->chanpos - 1;

	//XXX wc->usecount--;
	//XXX This is WRONG and can prang in a race.  We must pass THIS_MODULE
	//    as the owner of the span that holds the pointer to this function,
	//    then bump the refcount in the dahdi code _BEFORE_ the potentially
	//    fatal call to an invalid pointer is made.
	module_put(THIS_MODULE);
	if (wc->porttype[port] == VT_PORT_PROSLIC) {
		if (reversepolarity)
			wc->mod[port].fxs.idletxhookstate = 5;
		else
			wc->mod[port].fxs.idletxhookstate = 1;
	}
	/* If we're dead, release us now */
	//XXX if (!wc->usecount && wc->dead) openpci_release(wc);

	return 0;
}

static int openpci_hooksig(struct dahdi_chan *chan, enum dahdi_txsig txsig)
{
	struct openpci *wc = chan->pvt;
	int port = chan->chanpos - 1;
	int new_hk_state;

	dbginfo(wc->boardnum, "Setting %s port %d hook state %s",
		 wc->porttype[port] == VT_PORT_VDAA ? "FXO" : "FXS",
		 port,
		 txsig == 0 ? "ONHOOK" :
		 txsig == 1 ? "OFFHOOK" :
		 txsig == 2 ? "START" :
		 txsig == 3 ? "KEWL" : "UNKNOWN" );

	switch(wc->porttype[port]) {
	    case VT_PORT_VDAA:
		switch(txsig) {
		    case DAHDI_TXSIG_START:
		    case DAHDI_TXSIG_OFFHOOK:
			if (write_reg_fxo(wc, port, 5, 0x9)
					&& write_reg_fxo(wc, port, 0x20, 0x0))
				wc->mod[port].fxo.offhook = 1;
			else
				cardcrit(wc->boardnum,
						"Failed set fxo off-hook");
			break;

		    case DAHDI_TXSIG_ONHOOK:
			if (write_reg_fxo(wc, port, 5, 0x8)
			 && write_reg_fxo(wc, port, 0x20, 0x3))
				wc->mod[port].fxo.offhook = 0;
			else
				cardcrit(wc->boardnum, "Failed set fxo on-hook");
			break;

		    default:
			cardcrit(wc->boardnum,
				 "Can't set FXO port %d tx state to %d",
				 port, txsig);
		}
		break;

	    case VT_PORT_PROSLIC:
		new_hk_state = wc->mod[port].fxs.lasttxhook;
		switch(txsig) {
		    case DAHDI_TXSIG_ONHOOK:
			switch(chan->sig) {
			case DAHDI_SIG_EM:
			case DAHDI_SIG_FXOKS:
			case DAHDI_SIG_FXOLS:
				new_hk_state = wc->mod[port].fxs.idletxhookstate;
				break;
			case DAHDI_SIG_FXOGS:
				new_hk_state = 3;
				break;
			}
			break;

		    case DAHDI_TXSIG_OFFHOOK:
			switch(chan->sig) {
			case DAHDI_SIG_EM:
				new_hk_state = 5;
				break;
			default:
				new_hk_state = wc->mod[port].fxs.idletxhookstate;
				break;
			}
			break;

		    case DAHDI_TXSIG_START:
			new_hk_state = 4;
			break;

		    case DAHDI_TXSIG_KEWL:
			new_hk_state = 0;
			break;

		    default:
			cardinfo(wc->boardnum,
				 "Can't set FXS port %d tx state to %d",
				 port, txsig);
		}
		dbginfo(wc->boardnum, "%s port %d hook state old %d, new %d",
			 wc->porttype[port] == VT_PORT_VDAA ? "FXO" : "FXS",
			 port, wc->mod[port].fxs.lasttxhook, new_hk_state );

		if (new_hk_state != wc->mod[port].fxs.lasttxhook) {
			if (write_reg_fxs(wc, port, 64, new_hk_state))
				wc->mod[port].fxs.lasttxhook = new_hk_state;
			else
				cardcrit(wc->boardnum,
					 "Failed to set port %d fxs hookstate from %d to %d",
					 port, wc->mod[port].fxs.lasttxhook,
					 new_hk_state);
		}
		break;

	    default:
		cardcrit(wc->boardnum,
			 "Unknown module type %d in openpci_hooksig",
			 wc->porttype[port] );
	}
	return 0;
}

static const struct dahdi_span_ops openpci_span_ops = {
	.owner = THIS_MODULE,
	.hooksig = openpci_hooksig,
	.open = openpci_open,
	.close = openpci_close,
	.ioctl = openpci_ioctl,
	.watchdog = openpci_watchdog
};

static int span_initialize(struct openpci *wc)
{
	int x;

	wc->ddev = dahdi_create_device();
	//XXX Set a THIS_MODULE as the owner of the span...
	/* Zapata stuff */
	sprintf(wc->span.name, "WCTDM/%d", wc->boardnum);
	sprintf(wc->span.desc, "%s Board %d", wc->variety, wc->boardnum + 1);
	wc->ddev->location = kasprintf(GFP_KERNEL, "PCI Bus %02d Slot %02d",
				      wc->dev->bus->number,
				      PCI_SLOT(wc->dev->devfn) + 1);
	if (!wc->ddev->location)
		return -ENOMEM;
	for (x = 0; x < MAX_PORTS; x++) {
		struct dahdi_chan *chan = &wc->_chans[x];
		wc->chans[x] = chan;
		sprintf(chan->name, "WCTDM/%d/%d", wc->boardnum, x);
		chan->sigcap = DAHDI_SIG_FXOKS | DAHDI_SIG_FXOLS
			| DAHDI_SIG_FXOGS | DAHDI_SIG_SF | DAHDI_SIG_EM
			| DAHDI_SIG_CLEAR;
		chan->sigcap |= DAHDI_SIG_FXSKS | DAHDI_SIG_FXSLS
			| DAHDI_SIG_SF | DAHDI_SIG_CLEAR;
		chan->chanpos = x+1;
		chan->pvt = wc;
	}
	wc->ddev->manufacturer = "Voicetronix";
	wc->ddev->devicetype = wc->variety;
	wc->span.deflaw = DAHDI_LAW_MULAW;
	wc->span.chans = wc->chans;
	wc->span.channels = MAX_PORTS;
	wc->span.flags = DAHDI_FLAG_RBS;
	wc->span.ops = &openpci_span_ops;

	list_add_tail(&wc->span.device_node, &wc->ddev->spans);
	if (dahdi_register_device(wc->ddev, &wc->dev->dev)) {
		cardcrit(wc->boardnum, "Unable to register device with dahdi");
		return RET_FAIL;
	}
	return RET_OK;
}

static int get_port_type(struct openpci *wc, int port)
{
	int i, type;
	unsigned long flags;

	spin_lock_irqsave(&wc->lock, flags);
	outb(0x20, PIB(1)); HTXF_WAIT_LOCKED_RET(VT_PORT_EMPTY);
	outb(port, PIB(1)); HTXF_WAIT_LOCKED_RET(VT_PORT_EMPTY);
	HRXF_WAIT_LOCKED_RET(VT_PORT_EMPTY); type = inb(PIB(1));
	spin_unlock_irqrestore(&wc->lock, flags);

	return type;
}

static int check_ports(struct openpci *wc)
{
	int i = 0;

	wc->portcount = 0;
	for (; i < MAX_PORTS; ++i ) {
		wc->porttype[i] = get_port_type(wc, i);
		dbginfo(wc->boardnum,"%d: %s", i, porttype(wc,i));

		switch( wc->porttype[i] ) {
		    case VT_PORT_PROSLIC:
			/* By default, don't send on hook */
			if (reversepolarity)
				wc->mod[i].fxs.idletxhookstate = 5;
			else
				wc->mod[i].fxs.idletxhookstate = 1;
			/* FALLTHROUGH */
		    case VT_PORT_VDAA:
			++wc->portcount;
		}
	}
	/* we 'succeed' if any ports were discovered. */
	return wc->portcount ? RET_OK : RET_FAIL;
}

static int configure_vdaa_country(struct openpci *wc, int port, char *name)
{
	unsigned char value;
	int i;

	for (i = 0; i < sizeof(fxo_modes)/sizeof(struct fxo_mode); ++i) {
		if (!strcmp(fxo_modes[i].name, name)) {
			dbginfo(wc->boardnum, "%d: Setting country to %s",
					port, name);
			goto part2;
		}
	}
	i = 3;
	cardinfo(wc->boardnum, "Using default country %s", fxo_modes[i].name);

part2:
	value  = (fxo_modes[i].ohs << 6);
	value |= (fxo_modes[i].rz << 1);
	value |= (fxo_modes[i].rt << 0);
	if (!write_reg_fxo(wc, port, 16, value)) goto hell;

	/* DC Termination Control - Register 26 */
	value  = (fxo_modes[i].dcv << 6);
	value |= (fxo_modes[i].mini << 4);
	value |= (fxo_modes[i].ilim << 1);
	if (!write_reg_fxo(wc, port, 26, value)) goto hell;

	/* AC Termination Control - Register 30 */
	value = (fxo_modes[i].acim << 0);
	if (!write_reg_fxo(wc, port, 30, value)) goto hell;

	/* DAA Control 5 - Register 31 */
	msleep(1);
	if (!read_reg_fxo(wc, port, 31, &value)) goto hell;

	value = (value & 0xf7) | (fxo_modes[i].ohs2 << 3);
	value = value | 0x02;
	if (! write_reg_fxo(wc, port, 31, value)) goto hell;

	return RET_OK;

hell:
	cardcrit(wc->boardnum, "port %d failed configure vdaa country", port);
	return RET_FAIL;
}

/* Do not call this from an interrupt context, it may sleep. */
static void configure_vdaa_port(struct openpci *wc, int port)
{
	/* Set Country - default to Australia */
	if (configure_vdaa_country(wc, port, country))
		++wc->portcount;
	else {
		cardcrit(wc->boardnum, "FAILED to configure vdaa port %d.  Disabled.", port);
		wc->porttype[port] = VT_PORT_EMPTY;
	}
}

static int configure_proslic_country(struct openpci *wc, int port,
		const char *name)
{
	int i;

	for (i = 0; i < sizeof(ps_country_regs)/sizeof(struct ps_country_reg);
			++i) {
		if (!strcmp(ps_country_regs[i].country, name)) {
			dbginfo(wc->boardnum, "%d: Setting country to %s",
					port, name);
			goto part2;
		}
	}
	return -EINVAL;

part2:

	if (!write_reg_fxs(wc, port, 10, ps_country_regs[i].value)) {
		cardcrit(wc->boardnum,"%d: failed to write PS_IMPEDANCE", port);
		return -EIO;
	}
	return 0;
}

/* Do not call this from an interrupt context, it may sleep. */
static void configure_proslic_port(struct openpci *wc, int port)
{
	/* Set Country - default to Australia */
	switch (configure_proslic_country(wc, port, country)) {
		case 0:
			break;

		case -EINVAL:
			cardwarn(wc->boardnum, "%d: Country '%s' unknown, using default",
					port, country);
			if (configure_proslic_country(wc, port,
						DEFAULT_COUNTRY) == 0 )
				goto hell;

		default:
			goto hell;
	}

	++wc->portcount;
	return;

hell:
	cardcrit(wc->boardnum, "FAILED to configure proslic port %d.  Disabled.",
			port);
	wc->porttype[port] = VT_PORT_EMPTY;
}

/* Do not call this from an interrupt context, it may (indirectly) sleep. */
static int configure_ports(struct openpci *wc)
{
	unsigned long flags;
	int i;

	wc->portcount = 0;
	for (i = 0; i < MAX_PORTS; ++i) {
		switch (wc->porttype[i]) {
		    case VT_PORT_VDAA:    configure_vdaa_port(wc,i);    break;
		    case VT_PORT_PROSLIC: configure_proslic_port(wc,i); break;
		}
	}

	spin_lock_irqsave(&wc->lock, flags);
	outb(0x2c, PIB(1)); HTXF_WAIT_LOCKED();
	outb(0xff, PIB(1)); HTXF_WAIT_LOCKED();
	spin_unlock_irqrestore(&wc->lock, flags);

	/* otherwise we 'succeed' if any ports were configured successfully. */
	return wc->portcount ? RET_OK : RET_FAIL;
}

static int __get_arm_id(struct openpci *wc, int field, char *value)
{
	int i;
	int x = 0;
	int count = 0;

	outb(0x01, PIB(1));  HTXF_WAIT();
	outb(field, PIB(1)); HTXF_WAIT();
	HRXF_WAIT(); count = inb(PIB(1));
	if (count > ID_DATA_MAXSIZE) {
		cardcrit(wc->boardnum, "Too many bytes of id(%d) data %d/%d",
					 field, count, ID_DATA_MAXSIZE);
		return RET_FAIL;
	}
	//cardinfo(wc->boardnum, "get_arm_id(%d): byte count %d",field,count);
	for (; x < count; ++x) {
		HRXF_WAIT(); *value = inb(PIB(1));
		//cardinfo(wc->boardnum, "get_arm_id(%d): byte %d => 0x%02x",field,x,tmp);
		++value;
	}
	return RET_OK;
}

static void enable_interrupts(struct openpci *wc)
{
	outb(0x3f, TJ_MASK0);
	outb(0x02, TJ_MASK1);
}

static void disable_interrupts(struct openpci *wc)
{
	outb(0x00, TJ_MASK0);
	outb(0x00, TJ_MASK1);
}

/* Do not call this from an interrupt context, it may sleep. */
static int check_arm(struct openpci *wc)
{
	char model[ID_DATA_MAXSIZE + 1] = { 0 };
	char date[ID_DATA_MAXSIZE + 1]  = { 0 };
	unsigned long flags;
	int i = 0;
	int tmp = 0;

	spin_lock_irqsave(&wc->lock, flags);
	while ((tmp != 0x88) && (++i < 100)) {
		outb(0x88, PIB(0));
		msleep(1);
		tmp = inb(PIB(1));
	}
	if (i >= 1000)
		goto limbo;
	dbginfo(wc->boardnum, "Arm responded on attempt %d", i);

	/* Flush out the queue if we sent several pings before a response. */
	if (i > 1)
		__ping_arm(wc);

	if (!__get_arm_id(wc, 0, model))
		goto hell;
	sscanf(model, "OpenPCI8.%02d", &(wc->firmware));
	cardinfo(wc->boardnum, "  model: %s", model);

	if (!__get_arm_id(wc, 1, date))
		goto hell;
	cardinfo(wc->boardnum, "  date: %s", date);

	if (!__get_arm_id(wc, 2, wc->serial))
		goto hell;
	cardinfo(wc->boardnum, "  serial: %s", wc->serial);

	spin_unlock_irqrestore(&wc->lock, flags);
	return RET_OK;

hell:
	spin_unlock_irqrestore(&wc->lock, flags);
	cardwarn(wc->boardnum, "Found ARM processor, dumb firmware.");
	return RET_OK;

limbo:
	spin_unlock_irqrestore(&wc->lock, flags);
	return RET_FAIL;
}

static int arm_monitor(struct openpci *wc, int on)
{
	int i;
	outb(on ? 0x06 : 0x07, PIB(1)); HTXF_WAIT();
	return RET_OK;
}

static int __devinit openpci_probe_board(struct pci_dev *pdev,
		const struct pci_device_id *ent)
{
	struct openpci *wc;
	int boardnum = 0;
	int failret = -ENOMEM;
	int tmp = 0;
	int i;
	unsigned long flags;

	if (ent->driver_data != (kernel_ulong_t)&wcopenpci) {
		info("Probe of non-OpenPCI card, ignoring.");
		return -EINVAL;
	}
	wc = kzalloc(sizeof(struct openpci), GFP_KERNEL);
	if (!wc)
		return -ENOMEM;

	mutex_lock(&cards_mutex);
	for (; boardnum < MAX_CARDS && cards[boardnum]; ++boardnum)
		;
	if (boardnum >= MAX_CARDS) {
		crit("Too many OpenPCI cards(%d), max is %d.",
				boardnum, MAX_CARDS);
		mutex_unlock(&cards_mutex);
		goto hell;
	}
	cards[boardnum] = wc;
	mutex_unlock(&cards_mutex);

	spin_lock_init(&wc->lock);
	pci_set_drvdata(pdev, wc);

	wc->boardnum = boardnum;
	wc->dev      = pdev;
	wc->variety  = wcopenpci;

	cardinfo(boardnum, "Initialising card");
	if (pci_enable_device(pdev)) {
		failret = -EIO;
		goto hell_2;
	}
	wc->ioaddr = pci_resource_start(pdev, 0);
	if (!request_region(wc->ioaddr, 0xff, NAME)) {
		cardcrit(boardnum, "Failed to lock IO region, another driver already using it");
		failret = -EBUSY;
		goto hell_2;
	}

	spin_lock_irqsave(&wc->lock, flags);
	outb(0xff, TJ_AUXD);		/* Set up TJ to access the ARM */
	outb(0x78, TJ_AUXC);		/* Set up for Jtag */
	outb(0x00, TJ_CNTL);		/* pull ERST low */
	spin_unlock_irqrestore(&wc->lock, flags);
	msleep(1);			/* Wait a bit */

	dbginfo(boardnum, "Starting ARM");
	spin_lock_irqsave(&wc->lock, flags);
	outb(0x01, TJ_CNTL);            /* pull ERST high again */
	spin_unlock_irqrestore(&wc->lock, flags);
	msleep(100);                    /* Give it all a chance to boot */

	if (!check_arm(wc)) {
		cardcrit(boardnum, "Couldnt find ARM processor");
		failret = -EIO;
		goto hell_3;
	}
	if (wc->firmware < 11) {
		cardcrit(boardnum,
			 "Firmware version %d not supported by this driver",
			 wc->firmware);
		cardcrit(boardnum, " contact Voicetronix to have it updated");
		failret = -ENODEV;
		goto hell_3;
	}
	if (!check_ports(wc)) {
		cardcrit(boardnum, "Couldnt find ports!");
		failret = -EIO;
		goto hell_3;
	}

	wc->writechunk = dma_alloc_coherent(&pdev->dev, VT_PCIDMA_BLOCKSIZE,
			&wc->writedma, GFP_ATOMIC);
	if (!wc->writechunk) {
		cardcrit(boardnum, "Couldnt get DMA memory.");
		goto hell_3;
	}
	wc->readchunk = wc->writechunk + DAHDI_MAX_CHUNKSIZE *
		(MAX_PORTS * 2 / sizeof(int));
	wc->readdma = wc->writedma + DAHDI_MAX_CHUNKSIZE * (MAX_PORTS*2);

	memset((void *)wc->writechunk, 0, VT_PCIDMA_BLOCKSIZE);

	spin_lock_irqsave(&wc->lock, flags);
	outb(0xc1, TJ_SERCTL);
	outb(0x0, TJ_FSCDELAY);

	outl(wc->writedma,                    TJ_DMAWS);
	outl(wc->writedma + VT_PCIDMA_MIDDLE, TJ_DMAWI);
	outl(wc->writedma + VT_PCIDMA_END,    TJ_DMAWE);
	outl(wc->readdma,                     TJ_DMARS);
	outl(wc->readdma + VT_PCIDMA_MIDDLE,  TJ_DMARI);
	outl(wc->readdma + VT_PCIDMA_END,     TJ_DMARE);

	/* Clear interrupts */
	outb(0xff, TJ_INTSTAT);
	spin_unlock_irqrestore(&wc->lock, flags);

	if (!arm_monitor(wc, 1)) {
		cardcrit(boardnum, "failed to start arm monitoring");
		failret = -EIO;
		goto hell_4;
	}
	msleep(1000);

	i = 0;
	while (tmp != 0x88 && ++i < 1000) {
		outb(0x88, PIB(0));
		msleep(250);
		tmp = inb(PIB(1));
	}
	if (i >= 1000) {
		cardcrit(boardnum, "FAILED to initialise board");
		goto hell_4;
	}

	if (!check_ports(wc)) {
		cardcrit(boardnum, "FAILED to initialise ports");
		failret = -EIO;
		goto hell_4;
	}
	if (!configure_ports(wc)) {
		cardcrit(boardnum, "Failed to configure ports.");
		failret = -EIO;
		goto hell_4;
	}
	cardinfo(wc->boardnum, "have %d configured ports", wc->portcount);

	if (!span_initialize(wc)) {
		cardcrit(boardnum, "Failed to register with dahdi driver");
		failret = -EFAULT;
		goto hell_4;
	}

	/* Finalize signalling  */
	for (i = 0; i < MAX_PORTS; ++i) {
		if (wc->porttype[i] == VT_PORT_VDAA)
			wc->chans[i]->sigcap = DAHDI_SIG_FXSKS | DAHDI_SIG_FXSLS
				| DAHDI_SIG_CLEAR | DAHDI_SIG_SF;
		else if (wc->porttype[i] == VT_PORT_PROSLIC)
			wc->chans[i]->sigcap = DAHDI_SIG_FXOKS | DAHDI_SIG_FXOLS
				| DAHDI_SIG_FXOGS | DAHDI_SIG_SF
				| DAHDI_SIG_CLEAR | DAHDI_SIG_EM;
		else if (wc->porttype[i])
			cardcrit(wc->boardnum, "Port %d has unknown type (%d)",
					i, wc->porttype[i]);
	}

	/* Enable bus mastering */
	pci_set_master(pdev);

	if (request_irq(pdev->irq, openpci_isr, IRQF_SHARED, NAME, wc)) {
		cardcrit(boardnum, "Cant get IRQ!");
		failret = -EIO;
		goto hell_5;
	}
	cardinfo(boardnum, "Got IRQ %d", pdev->irq);

	enable_interrupts(wc);
	start_dma(wc);

	cardinfo(boardnum, "Initialised card.");
	return 0;

hell_5:
	dahdi_unregister_device(wc->ddev);
hell_4:
	if (wc->writechunk)
		dma_free_coherent(&pdev->dev, VT_PCIDMA_BLOCKSIZE,
				    (void *)wc->writechunk, wc->writedma);
hell_3:
	outb(0x00, TJ_CNTL);
	release_region(wc->ioaddr, 0xff);
hell_2:
	cards[boardnum] = NULL;
hell:
	kfree(wc->ddev->location);
	dahdi_free_device(wc->ddev);
	kfree(wc);
	return failret;
}

static void __devexit openpci_remove_board(struct pci_dev *pdev)
{
	struct openpci *wc = pci_get_drvdata(pdev);

	if (!wc)
		return;

	arm_monitor(wc, 0);

	/* Stop DMA */
	outb(0x00, TJ_OPER);
	disable_interrupts(wc);

	//XXX Replace this usecount business...
	//    and do this BEFORE we invalidate everything above...
	//    check that we wont try to write to it in the meantime.
	/* Release span, possibly delayed */
	//XXX if (!wc->usecount) openpci_release(wc); else wc->dead = 1;

	dahdi_unregister_device(wc->ddev);
	outb(0x00, TJ_CNTL);

	dma_free_coherent(&pdev->dev, VT_PCIDMA_BLOCKSIZE,
			(void *)wc->writechunk, wc->writedma);
	free_irq(pdev->irq, wc);

	release_region(wc->ioaddr, 0xff);

	mutex_lock(&cards_mutex);
	cards[wc->boardnum] = NULL;
	mutex_unlock(&cards_mutex);

	kfree(wc->ddev->location);
	dahdi_free_device(wc->ddev);
	kfree(wc);
	cardinfo(wc->boardnum, "Removed OpenPCI card.");
}

static DEFINE_PCI_DEVICE_TABLE(openpci_pci_tbl) = {
	{ 0xe159, 0x0001, PCI_ANY_ID, PCI_ANY_ID, 0, 0, (kernel_ulong_t) &wcopenpci },
	{ 0 }
};

MODULE_DEVICE_TABLE(pci, openpci_pci_tbl);

static struct pci_driver openpci_driver = {
	.name = NAME,
	.probe = openpci_probe_board,
	.remove = __devexit_p(openpci_remove_board),
	.id_table = openpci_pci_tbl,
};

static int __init openpci_init(void)
{
#ifdef __BIG_ENDIAN
	warn("No big endian support (yet)");
	return -ENODEV;
#endif

	if (pci_register_driver(&openpci_driver))
		return -ENODEV;

	info("Module loaded %s", debug ? "with debug enabled" : "");
	return 0;
}

static void __exit openpci_cleanup(void)
{
	pci_unregister_driver(&openpci_driver);
	info("Module exit");
}

module_init(openpci_init);
module_exit(openpci_cleanup);

MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DAHDI_VERSION);
MODULE_LICENSE("GPL");
