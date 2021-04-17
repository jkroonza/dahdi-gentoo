/*
 * AP4XX PCI Card Driver
 *
 * Written by Ronaldo Valiati <aligera@aligera.com.br>
 *
 * Based on previous works, designs, and architectures conceived and
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

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <dahdi/kernel.h>
#include <linux/moduleparam.h>

#include "ap400.h"

//#define AP400_DEBUG
#ifdef AP400_DEBUG
#define PDEBUG(fmt, args...) { \
	printk(KERN_DEBUG "AP400 (%d): ",__LINE__); \
	printk(fmt "\n", ## args); \
}
#else
#define PDEBUG(fmt, args...)
#endif

/*
 * Tasklets provide better system interactive response at the cost of the
 * possibility of losing a frame of data at very infrequent intervals.  If
 * you are more concerned with the performance of your machine, enable the
 * tasklets.  If you are strict about absolutely no drops, then do not enable
 * tasklets.
 */

/* #define ENABLE_TASKLETS */


/* Work queues are a way to better distribute load on SMP systems */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20))
/*
 * Work queues can significantly improve performance and scalability
 * on multi-processor machines, but requires bypassing some kernel
 * API's, so it's not guaranteed to be compatible with all kernels.
 */
/* #define ENABLE_WORKQUEUES */
#endif

/* Enable HDLC support by hardware */
#ifdef AP400_HDLC
#include "ap400_hdlc/ap400_hdlc.c"
#endif

//#define APEC_SUPPORT
#ifdef APEC_SUPPORT
#include "apec.h"
#endif

/* Workarounds */
#ifndef IRQF_SHARED
#define IRQF_SHARED		SA_SHIRQ
#endif
#ifndef IRQF_DISABLED
#ifdef SA_INTERRUPT
#define IRQF_DISABLED		SA_INTERRUPT
#else
#define IRQF_DISABLED		0x0
#endif
#endif
#ifndef __iomem
#define __iomem
#endif

/* Enable prefetching may help performance */
#define ENABLE_PREFETCH

/* Define to get more attention-grabbing but slightly more I/O using
   alarm status */
#define FANCY_ALARM

#define DEBUG_MAIN 		(1 << 0)
#define DEBUG_DTMF 		(1 << 1)
#define DEBUG_REGS 		(1 << 2)
#define DEBUG_TSI  		(1 << 3)
#define DEBUG_ECHOCAN 		(1 << 4)
#define DEBUG_RBS 		(1 << 5)
#define DEBUG_FRAMER		(1 << 6)

static int clock_source = -1;
static int tdm_loop = 0;
static int apec_enable = 1;
module_param(tdm_loop, int, 0600);
module_param(apec_enable, int, 0600);

#ifdef ENABLE_WORKQUEUES
#include <linux/cpumask.h>

/* XXX UGLY!!!! XXX  We have to access the direct structures of the workqueue which
  are only defined within workqueue.c because they don't give us a routine to allow us
  to nail a work to a particular thread of the CPU.  Nailing to threads gives us substantially
  higher scalability in multi-CPU environments though! */

/*
 * The per-CPU workqueue (if single thread, we always use cpu 0's).
 *
 * The sequence counters are for flush_scheduled_work().  It wants to wait
 * until until all currently-scheduled works are completed, but it doesn't
 * want to be livelocked by new, incoming ones.  So it waits until
 * remove_sequence is >= the insert_sequence which pertained when
 * flush_scheduled_work() was called.
 */

struct cpu_workqueue_struct {

	spinlock_t lock;

	long remove_sequence;	/* Least-recently added (next to run) */
	long insert_sequence;	/* Next to add */

	struct list_head worklist;
	wait_queue_head_t more_work;
	wait_queue_head_t work_done;

	struct workqueue_struct *wq;
	task_t *thread;

	int run_depth;		/* Detect run_workqueue() recursion depth */
} ____cacheline_aligned;

/*
 * The externally visible workqueue abstraction is an array of
 * per-CPU workqueues:
 */
struct workqueue_struct {
	struct cpu_workqueue_struct cpu_wq[NR_CPUS];
	const char *name;
	struct list_head list; 	/* Empty if single thread */
};

/* Preempt must be disabled. */
static void __ap4_queue_work(struct cpu_workqueue_struct *cwq,
			 struct work_struct *work)
{
	unsigned long flags;

	spin_lock_irqsave(&cwq->lock, flags);
	work->wq_data = cwq;
	list_add_tail(&work->entry, &cwq->worklist);
	cwq->insert_sequence++;
	wake_up(&cwq->more_work);
	spin_unlock_irqrestore(&cwq->lock, flags);
}

/*
 * Queue work on a workqueue. Return non-zero if it was successfully
 * added.
 *
 * We queue the work to the CPU it was submitted, but there is no
 * guarantee that it will be processed by that CPU.
 */
static inline int ap4_queue_work(struct workqueue_struct *wq, struct work_struct *work, int cpu)
{
	int ret = 0;

	if (!test_and_set_bit(0, &work->pending)) {
		BUG_ON(!list_empty(&work->entry));
		__ap4_queue_work(wq->cpu_wq + cpu, work);
		ret = 1;
	}
	return ret;
}

#endif

static int debug=0;
static int timingcable;
static int highestorder;
static int t1e1override = -1;
static int j1mode = 0;
static int loopback = 0;
static int alarmdebounce = 0;

/* Enabling bursting can more efficiently utilize PCI bus bandwidth, but
   can also cause PCI bus starvation, especially in combination with other
   aggressive cards.  Please note that burst mode has no effect on CPU
   utilization / max number of calls / etc. */
static int noburst = 1;
static int debugslips = 0;
static int polling = 0;

#ifdef FANCY_ALARM
static int altab[] = {
0, 0, 0, 1, 2, 3, 4, 6, 8, 9, 11, 13, 16, 18, 20, 22, 24, 25, 27, 28, 29, 30, 31, 31, 32, 31, 31, 30, 29, 28, 27, 25, 23, 22, 20, 18, 16, 13, 11, 9, 8, 6, 4, 3, 2, 1, 0, 0,
};
#endif

#define FLAG_STARTED (1 << 0)
#define FLAG_NMF (1 << 1)
#define FLAG_SENDINGYELLOW (1 << 2)

#define	TYPE_T1	1		/* is a T1 card */
#define	TYPE_E1	2		/* is an E1 card */
#define TYPE_J1 3		/* is a running J1 */

struct devtype {
	char *desc;
	unsigned int flags;
};

static struct devtype ap401  = { "Aligera AP401", 0 };
static struct devtype ap402  = { "Aligera AP402", 0 };
static struct devtype ap404  = { "Aligera AP404", 0 };
static struct devtype ape401  = { "Aligera APE401", 0 };
static struct devtype ape402  = { "Aligera APE402", 0 };
static struct devtype ape404  = { "Aligera APE404", 0 };

struct ap4;

struct ap4_span {
	struct ap4 *owner;
	unsigned int *writechunk;					/* Double-word aligned write memory */
	unsigned int *readchunk;					/* Double-word aligned read memory */
	int spantype;		/* card type, T1 or E1 or J1 */
	int sync;
	int psync;
	int alarmtimer;
	int redalarms;
	int notclear;
	int alarmcount;
	int spanflags;
	int syncpos;
	int e1check;			/* E1 check */
	int reload_cas;
	unsigned char casbuf[15];
	unsigned int slipcount;
	struct dahdi_span span;
	unsigned char txsigs[16];	/* Transmit sigs */
	int loopupcnt;
	int loopdowncnt;
	unsigned char ec_chunk1[31][DAHDI_CHUNKSIZE]; /* first EC chunk buffer */
	unsigned char ec_chunk2[31][DAHDI_CHUNKSIZE]; /* second EC chunk buffer */
	int irqmisses;
#ifdef ENABLE_WORKQUEUES
	struct work_struct swork;
#endif
	struct dahdi_chan *chans[32];		/* Individual channels */
};

struct ap4_regs {
	volatile u32 card_id;		// 00h R0
	volatile u16 fpga_ver;		// 04h R1
	volatile u16 span_num;		// 06h R1
	u32 __unused;			// 08h R2
	volatile u32 liu_config;	// 0Ch R3
	volatile u32 e1_config;		// 10h R4
	volatile u32 e1_status;		// 14h R5
	volatile u32 leds;		// 18h R6
	volatile u32 clock_source;	// 1Ch R7
	u32 __unused3[8];		// 20h - 3Ch R8 - R15
	volatile u32 echo_ctrl;		// 40h R16
	volatile u32 echo_data;		// 44h R17
	volatile u32 t1_status;		// 48h R18
	volatile u32 t1_config;		// 4Ch R19
};

struct ap4 {
	/* This structure exists one per card */
	struct pci_dev *dev;		/* Pointer to PCI device */
	struct dahdi_device *ddev;
	struct ap4_regs *hw_regs;
	unsigned int intcount;
	int flag_1st_irq;
	int num;			/* Which card we are */
	int fpgaver;		/* version of FPGA */
	int hwid;			/* hardware ID */
	int globalconfig;	/* Whether global setup has been done */
	int syncsrc;			/* active sync source */
	struct ap4_span *tspans[4];	/* Individual spans */
	int numspans;			/* Number of spans on the card */
	int blinktimer[4];
#ifdef FANCY_ALARM
	int alarmpos[4];
#endif
	int irq;			/* IRQ used by device */
	int order;			/* Order */
	int flags;			/* Device flags */
	int ledreg;				/* LED Register */
	int e1recover;			/* E1 recovery timer */
	unsigned long memaddr;		/* Base address of card */
	unsigned long memlen;
	volatile unsigned int *membase;	/* Base address of card */
	int spansstarted;		/* number of spans started */
	/* spinlock_t lock; */		/* lock context */
	spinlock_t reglock;		/* lock register access */
	volatile unsigned int *writechunk;	/* Double-word aligned write memory */
	volatile unsigned int *readchunk;	/* Double-word aligned read memory */
#ifdef ENABLE_WORKQUEUES
	atomic_t worklist;
	struct workqueue_struct *workq;
#else
#ifdef ENABLE_TASKLETS
	int taskletrun;
	int taskletsched;
	int taskletpending;
	int taskletexec;
	int txerrors;
	struct tasklet_struct ap4_tlet;
#endif
#endif
	unsigned int passno;	/* number of interrupt passes */
	struct devtype *dt;
	char *variety;
	int last0;		/* for detecting double-missed IRQ */
	int checktiming;	/* Set >0 to cause the timing source to be checked */
#ifdef AP400_HDLC
	struct card_s *hdlc_card;
#endif
#ifdef APEC_SUPPORT
	int apec_enable;
	struct apec_s *apec;
#endif
};


static void __set_clear(struct ap4 *wc, int span);
static int ap4_startup(struct file *file, struct dahdi_span *span);
static int ap4_shutdown(struct dahdi_span *span);
static int ap4_rbsbits(struct dahdi_chan *chan, int bits);
static int ap4_maint(struct dahdi_span *span, int cmd);
static int ap4_ioctl(struct dahdi_chan *chan, unsigned int cmd, unsigned long data);
static void __ap4_set_timing_source(struct ap4 *wc, int unit);
static void __ap4_check_alarms(struct ap4 *wc, int span);
static void __ap4_check_sigbits(struct ap4 *wc, int span);


#define AP_ACTIVATE	(1 << 12)

#define AP_OFF	(0)
#define AP_ON	(1)

#define MAX_AP4_CARDS 64

#ifdef ENABLE_TASKLETS
static void ap4_tasklet(unsigned long data);
#endif

static struct ap4 *cards[MAX_AP4_CARDS];

//#define ap_debugk(fmt,args...) printk("ap400 -> %s: "fmt, __PRETTY_FUNCTION__, ##args)
#define ap_debugk(fmt,args...)

//#define TIMER_DEBUG	1

#ifdef TIMER_DEBUG
struct timer_list ap4xx_opt_timer;
unsigned int delay = 1000;
module_param(delay, uint, S_IRUGO);
#endif

#define PCI_DEVICE_ID_AP4XX		0x1004

static inline void __ap4_set_led(struct ap4 *wc, int span, int color)
{
	wc->ledreg &= ~(AP_ON << span);
	wc->ledreg |= (color << span);
	*(wc->membase+AP_LEDS_REG) &= ~0x0000000F;
	*(wc->membase+AP_LEDS_REG) |= ((wc->ledreg)&0x0F);
}

static inline void ap4_activate(struct ap4 *wc)
{
	wc->ledreg |= AP_ACTIVATE;
}

static void __set_clear(struct ap4 *wc, int span)
{
	int i,j;
	int oldnotclear;
	unsigned short val=0;
	struct ap4_span *ts = wc->tspans[span];

	oldnotclear = ts->notclear;
	if (ts->spantype == TYPE_T1) {
		for (i=0;i<24;i++) {
			j = (i/8);
			if (ts->span.chans[i]->flags & DAHDI_FLAG_CLEAR) {
				val |= 1 << (7 - (i % 8));
				ts->notclear &= ~(1 << i);
			} else
				ts->notclear |= (1 << i);
			if ((i % 8)==7) {
				val = 0;
			}
		}
	} else {
		for (i=0;i<31;i++) {
			if (ts->span.chans[i]->flags & DAHDI_FLAG_CLEAR)
				ts->notclear &= ~(1 << i);
			else
				ts->notclear |= (1 << i);
		}
	}
}

#ifdef APEC_SUPPORT

#define APEC_CTRL_RESET		0x80000000
#define APEC_CTRL_DDR_NCKE	0x40000000
#define APEC_CTRL_EC_DISABLE	0x20000000
#define APEC_CTRL_DAS		0x00080000
#define APEC_CTRL_RD		0x00040000
#define APEC_CTRL_REQ		0x00020000
#define APEC_CTRL_READY		0x00010000

#define APEC_ACCESS_TIMEOUT	1000

static inline u16 oct_raw_read (struct ap4_regs *regs, unsigned short addr)
{
	unsigned short data;
	// Poll ready bit
	while ((regs->echo_ctrl & APEC_CTRL_READY) == 0);
	// Write control bits and address
	regs->echo_ctrl = APEC_CTRL_RD | APEC_CTRL_REQ | (addr & 0xFFFF);
	while ((regs->echo_ctrl & APEC_CTRL_READY) == 0);
	data = regs->echo_data & 0xFFFF;
	//PDEBUG("Raw Read 0x%04hX @ 0x%08X", data, addr);
	return data;
}

static inline void oct_raw_write (struct ap4_regs *regs, unsigned short addr,
							unsigned short data)
{
	// Poll ready bit
	while ((regs->echo_ctrl & APEC_CTRL_READY) == 0);
	// Write data, then control bits and address
	regs->echo_data = data & 0xFFFF;
	regs->echo_ctrl = APEC_CTRL_REQ | (addr & 0xFFFF);
	// Poll ready bit
	while ((regs->echo_ctrl & APEC_CTRL_READY) == 0);
	//PDEBUG("Raw Write 0x%04hX @ 0x%08X", data, addr);
	//oct_raw_read(regs, addr);
}

static inline int oct_ext_wait (struct ap4_regs *regs)
{
	int i = APEC_ACCESS_TIMEOUT;
	while ((oct_raw_read(regs, 0x0) & 0x100) && (i-- > 0));
	if (i == -1) {
		printk(KERN_WARNING "Wait access_req timeout\n");
		return -1;
	}
	return 0;
}

static inline u16 oct_ind_read (struct ap4_regs *regs, unsigned int addr)
{
	// Poll access_req bit
	if (oct_ext_wait(regs))
		return 0;
	// Write extended indirect registers
	oct_raw_write(regs, 0x8, (addr >> 20) & 0x1FFF);
	oct_raw_write(regs, 0xA, (addr >> 4) & 0xFFFF);
	oct_raw_write(regs, 0x0, ((addr & 0xE) << 8) | 0x101);
	// Poll access_req bit
	if (oct_ext_wait(regs))
		return 0;
	// Return data
	return oct_raw_read(regs, 0x4);
}

static inline void oct_ind_write (struct ap4_regs *regs, unsigned int addr,
							unsigned short data)
{
	// Poll access_req bit
	if (oct_ext_wait(regs))
		return;
	oct_raw_write(regs, 0x8, (addr >> 20) & 0x1FFF);
	oct_raw_write(regs, 0xA, (addr >> 4) & 0xFFFF);
	oct_raw_write(regs, 0x4, data);
	oct_raw_write(regs, 0x0, ((addr & 0xE) << 8) | 0x3101);
	// Poll access_req bit
	if (oct_ext_wait(regs))
		return;
}

static inline u16 oct_dir_read (struct ap4_regs *regs, unsigned int addr)
{
	// Poll access_req bit
	if (oct_ext_wait(regs))
		return 0;
	// Write extended direct registers
	oct_raw_write(regs, 0x8, (addr >> 20) & 0x1FFF);
	oct_raw_write(regs, 0xA, (addr >> 4) & 0xFFFF);
	oct_raw_write(regs, 0x0, 0x1);
	regs->echo_ctrl = APEC_CTRL_DAS | APEC_CTRL_RD | APEC_CTRL_REQ | (addr & 0xFFFF);
	while ((regs->echo_ctrl & APEC_CTRL_READY) == 0);
	// Return data
	return regs->echo_data;
}

static inline void oct_dir_write (struct ap4_regs *regs, unsigned int addr,
							unsigned short data)
{
	// Poll access_req bit
	if (oct_ext_wait(regs))
		return;
	// Write extended direct registers
	oct_raw_write(regs, 0x8, (addr >> 20) & 0x1FFF);
	oct_raw_write(regs, 0xA, (addr >> 4) & 0xFFFF);
	oct_raw_write(regs, 0x0, 0x3001);
	regs->echo_data = data & 0xFFFF;
	regs->echo_ctrl = APEC_CTRL_DAS | APEC_CTRL_REQ | (addr & 0xFFFF);
	while ((regs->echo_ctrl & APEC_CTRL_READY) == 0);
}


unsigned int oct_read (void *card, unsigned int addr)
{
	struct ap4 *wc = card;
	int flags;
	unsigned short data;
	spin_lock_irqsave(&wc->reglock, flags);
	data = oct_ind_read(wc->hw_regs, addr);
	spin_unlock_irqrestore(&wc->reglock, flags);
	PDEBUG("Read 0x%04hX @ 0x%08X", data, addr);
	return data;
}

void oct_write (void *card, unsigned int addr, unsigned int data)
{
	struct ap4 *wc = card;
	int flags;
	spin_lock_irqsave(&wc->reglock, flags);
	oct_ind_write(wc->hw_regs, addr, data);
	spin_unlock_irqrestore(&wc->reglock, flags);
	PDEBUG("Write 0x%04hX @ 0x%08X", data, addr);
}

static int ap4_apec_init(struct ap4 *wc)
{
	int laws[4];
	int i;
	unsigned int apec_capacity;
	struct firmware embedded_firmware;
	const struct firmware *firmware = &embedded_firmware;
#if !defined(HOTPLUG_FIRMWARE)
	extern void _binary_OCT6104E_64D_ima_size;
	extern u8 _binary_OCT6104E_64D_ima_start[];
	extern void _binary_OCT6104E_128D_ima_size;
	extern u8 _binary_OCT6104E_128D_ima_start[];
#else
	static const char oct64_firmware[] = "OCT6104E-64D.ima";
	static const char oct128_firmware[] = "OCT6104E-128D.ima";
#endif

	// Enable DDR and Reset Octasic
	wc->hw_regs->echo_ctrl |= APEC_CTRL_RESET;
	wc->hw_regs->echo_ctrl |= APEC_CTRL_DDR_NCKE;
	udelay(500);
	wc->hw_regs->echo_ctrl &= APEC_CTRL_RESET;
	wc->hw_regs->echo_ctrl &= APEC_CTRL_DDR_NCKE;
	wc->hw_regs->echo_ctrl &= APEC_CTRL_EC_DISABLE;

	/* Setup alaw vs ulaw rules */
	for (i = 0; i < wc->numspans; i++) {
		if (wc->tspans[i]->span.channels > 24)
			laws[i] = 1;	// E1: alaw
		else
			laws[i] = 0;	// T1: ulaw
	}

	switch ((apec_capacity = apec_capacity_get(wc))) {
	case 64:
#if defined(HOTPLUG_FIRMWARE)
		if ((request_firmware(&firmware, oct64_firmware, &wc->dev->dev) != 0) ||
		    !firmware) {
			printk("%s: firmware %s not available from userspace\n",
					wc->variety, oct64_firmware);
			return -1;
		}
#else
		embedded_firmware.data = _binary_OCT6104E_64D_ima_start;
		/* Yes... this is weird. objcopy gives us a symbol containing
		   the size of the firmware, not a pointer to a variable containing
		   the size. The only way we can get the value of the symbol
		   is to take its address, so we define it as a pointer and
		   then cast that value to the proper type.
		*/
		embedded_firmware.size = (size_t) &_binary_OCT6104E_64D_ima_size;
#endif
		break;
	case 128:
#if defined(HOTPLUG_FIRMWARE)
		if ((request_firmware(&firmware, oct128_firmware, &wc->dev->dev) != 0) ||
		    !firmware) {
			printk("%s: firmware %s not available from userspace\n",
					wc->variety, oct128_firmware);
			return -1;
		}
#else
		embedded_firmware.data = _binary_OCT6104E_128D_ima_start;
		/* Yes... this is weird. objcopy gives us a symbol containing
		   the size of the firmware, not a pointer to a variable containing
		   the size. The only way we can get the value of the symbol
		   is to take its address, so we define it as a pointer and
		   then cast that value to the proper type.
		*/
		embedded_firmware.size = (size_t) &_binary_OCT6104E_128D_ima_size;
#endif
		break;
	default:
		printk(KERN_INFO "Unsupported channel capacity found on"
				"echo cancellation module (%d).\n", apec_capacity);
		return -1;
	}

	if (!(wc->apec = apec_init(wc, laws, wc->numspans, firmware))) {
		printk(KERN_WARNING "APEC: Failed to initialize\n");
		if (firmware != &embedded_firmware)
			release_firmware(firmware);
		return -1;
	}

	if (firmware != &embedded_firmware)
		release_firmware(firmware);

	printk(KERN_INFO "APEC: Present and operational servicing %d span(s)\n", wc->numspans);
	return 0;
}

void ap4_apec_release(struct ap4 *wc)
{
	// Disabel DDR and reset Octasic
	wc->hw_regs->echo_ctrl |= APEC_CTRL_RESET;
	wc->hw_regs->echo_ctrl |= APEC_CTRL_DDR_NCKE;
	wc->hw_regs->echo_ctrl |= APEC_CTRL_EC_DISABLE;
	if (wc->apec)
		apec_release(wc->apec);
}


static int ap4_echocan(struct dahdi_chan *chan, int eclen)
{
	struct ap4 *wc = chan->pvt;
	int channel;

	if (!wc->apec)
		return -ENODEV;
	if (debug)
		printk(KERN_DEBUG "AP400: ap4_echocan @ Span %d Channel %d Length: %d\n",
				chan->span->offset, chan->chanpos, eclen);
	channel = (chan->chanpos << 2) | chan->span->offset;
	apec_setec(wc->apec, channel, eclen);
	return 0;
}

#endif // APEC_SUPPORT


static int ap4_ioctl(struct dahdi_chan *chan, unsigned int cmd, unsigned long data)
{
	struct ap4 *wc = chan->pvt;
	int span = 0;
	int alarms = 0;
	unsigned char c, e1_cfg;

	switch(cmd) {
		case AP4_GET_ALARMS:
			if (copy_from_user(&span, (int *)data, sizeof(int)))
				return -EFAULT;
			// span starts in zero
			span--;
		if (wc->tspans[span]->spantype == TYPE_E1) {
		        /* le status e configuracao do E1 */
		        c = ((*(wc->membase+AP_E1_STATUS_REG))>>(8*span));
		        e1_cfg = ((*(wc->membase+AP_E1_CONFIG_REG))>>(8*span));
			if( c & AP_E1_LOS_STATUS) {
				alarms = 0x01;
			} else if( c & AP_E1_AIS_STATUS) {
				alarms = 0x02;
			} else if(!(c & AP_E1_BFAE_STATUS)) {
				alarms = 0x04;
				if (c & AP_E1_RAI_STATUS)
					alarms |= 0x08;
				// Erro de MFA: 00 - MFA desabilitado, 01 - erro de MFA, 10 - MFA OK
				if ( (c & AP_E1_MFAE_STATUS) && (e1_cfg & AP_E1_CRCEN_CONFIG) )
					alarms |= 0x10;
				else if ( (!(c & AP_E1_MFAE_STATUS)) && (e1_cfg & AP_E1_CRCEN_CONFIG) )
					alarms |= 0x20;
				// Erro de CAS: 00 - desabilitado, 01 - erro de CAS, 10 - CAS OK
				if ( (!(c & AP_E1_CAS_STATUS)) && (e1_cfg & AP_E1_PCM30_CONFIG))
					alarms |= 0x40;
				else if ( (c & AP_E1_CAS_STATUS) && (e1_cfg & AP_E1_PCM30_CONFIG))
					alarms |= 0x80;
			}
		} else {
			/* le status e configuracao do E1 */
		        c = ((*(wc->membase+AP_E1_STATUS_REG))>>(8*span));
		        if( c & AP_E1_LOS_STATUS)
				alarms = 0x01;
			else {
			        c = wc->hw_regs->t1_status >> (8*span);
			        if (!(c & AP4_T1_FRAME_SYNC))
			        	alarms = 0x04;
		        }
		}
			if(debug) printk("AP4_GET_ALARMS: span = %d, alarms = 0x%02x\n", span+1, alarms);
			if (copy_to_user((int *)data, &alarms, sizeof(int)))
				return -EFAULT;
			break;

		case AP4_GET_SLIPS:
			if (copy_from_user(&span, (int *)data, sizeof(int)))
				return -EFAULT;
			// span starts in zero
			span--;
			if((span < wc->numspans) && (span >=0))
				alarms = wc->tspans[span]->slipcount;
			if(debug) printk("AP4_GET_SLIPS: span = %d, slips = 0x%02x\n", span+1, alarms);
			if (copy_to_user((int *)data, &alarms, sizeof(int)))
				return -EFAULT;
			break;

		default:
			PDEBUG("%s: Unknown IOCTL CODE!", wc->variety);
			return -ENOTTY;
	}
	return 0;
}

static inline struct ap4_span* ap4_span_from_span(struct dahdi_span *span) {
	return container_of(span, struct ap4_span, span);
}

static int ap4_maint(struct dahdi_span *span, int cmd)
{
	struct ap4_span *ts = ap4_span_from_span(span);
	struct ap4 *wc = ts->owner;


	if (ts->spantype == TYPE_E1) {
		switch(cmd) {
		case DAHDI_MAINT_NONE:
			printk("XXX Turn off local and remote loops E1 XXX\n");
			*(wc->membase+AP_E1_CONFIG_REG) &= ~(AP_E1_LOOP_CONFIG<<((span->spanno-1)*8));
			break;
		case DAHDI_MAINT_LOCALLOOP:
			printk("XXX Turn on local loopback E1 XXX\n");
			break;
		case DAHDI_MAINT_REMOTELOOP:
			printk("XXX Turn on remote loopback E1 XXX\n");
			break;
		case DAHDI_MAINT_LOOPUP:
			printk("XXX Turn on local loopback on E1 #%d instead of send loopup code XXX\n", span->spanno);
			*(wc->membase+AP_E1_CONFIG_REG) |= (AP_E1_LOOP_CONFIG<<((span->spanno-1)*8));
			break;
		case DAHDI_MAINT_LOOPDOWN:
			printk("XXX Turn on local loopback on E1 #%d instead of send loopdown code XXX\n", span->spanno);
			*(wc->membase+AP_E1_CONFIG_REG) |= (AP_E1_LOOP_CONFIG<<((span->spanno-1)*8));
			break;
		default:
			printk("%s: Unknown E1 maint command: %d\n", wc->variety, cmd);
			break;
		}
	} else {
		switch(cmd) {
	    case DAHDI_MAINT_NONE:
			printk("XXX Turn off local and remote loops T1 XXX\n");
			break;
	    case DAHDI_MAINT_LOCALLOOP:
			printk("XXX Turn on local loop and no remote loop XXX\n");
			break;
	    case DAHDI_MAINT_REMOTELOOP:
			printk("XXX Turn on remote loopup XXX\n");
			break;
	    case DAHDI_MAINT_LOOPUP:
			break;
	    case DAHDI_MAINT_LOOPDOWN:
			break;
	    default:
			printk("%s: Unknown T1 maint command: %d\n", wc->variety, cmd);
			break;
	   }
    }
	return 0;
}

static int ap4_rbsbits(struct dahdi_chan *chan, int bits)
{
	u_char m,c;
	int k,n,b;
	struct ap4 *wc = chan->pvt;
	struct ap4_span *ts = wc->tspans[chan->span->offset];
	unsigned long flags;
	volatile unsigned int *writecas = (wc->membase+AP_CAS_BASE);
	unsigned int allspansbits;

	//ap_debugk("chan->channo = %d, int bits = 0x%08x\n", chan->channo, bits);
	if(debug & DEBUG_RBS) printk("Setting bits to %d on channel %s\n", bits, chan->name);
	spin_lock_irqsave(&wc->reglock, flags);
	k = chan->span->offset;
	if (ts->spantype == TYPE_E1) { /* do it E1 way */
		if (chan->chanpos == 16) {
			spin_unlock_irqrestore(&wc->reglock, flags);
			return 0;
		}
		n = chan->chanpos - 1;
		if (chan->chanpos > 15) n--;
		b = (n % 15);
		c = ts->txsigs[b];
		m = (n / 15) << 2; /* nibble selector */
		c &= (0xf << m); /* keep the other nibble */
		c |= (bits & 0xf) << (4 - m); /* put our new nibble here */
		ts->txsigs[b] = c;
		/* monta a word de 32 bits com informacao de todos os spans */
		allspansbits =  wc->tspans[0]->txsigs[b];
		if (wc->numspans > 1) {
			allspansbits |=	(wc->tspans[1]->txsigs[b] << 8);
		}
		if (wc->numspans == 4) {
			allspansbits |=	(wc->tspans[2]->txsigs[b] << 16) |
							(wc->tspans[3]->txsigs[b] << 24);
		}
		/* output them to the chip */
		writecas[b] = allspansbits;
		ap_debugk("escrito 0x%08x para ser transmitido pelo CAS (b = %d)\n", allspansbits, b);
#if 0
	} else if (ts->span.lineconfig & DAHDI_CONFIG_D4) {
		n = chan->chanpos - 1;
		b = (n/4);
		c = ts->txsigs[b];
		m = ((3 - (n % 4)) << 1); /* nibble selector */
		c &= ~(0x3 << m); /* keep the other nibble */
		c |= ((bits >> 2) & 0x3) << m; /* put our new nibble here */
		ts->txsigs[b] = c;
		  /* output them to the chip */
		//__ap4_out( ... );
	} else if (ts->span.lineconfig & DAHDI_CONFIG_ESF) {
#endif
	} else {
		n = chan->chanpos - 1;
		b = (n/2);
		c = ts->txsigs[b];
		m = ((n % 2) << 2); /* nibble selector */
		c &= (0xf << m); /* keep the other nibble */
		c |= (bits & 0xf) << (4 - m); /* put our new nibble here */
		ts->txsigs[b] = c;
		  /* output them to the chip */
		/* monta a word de 32 bits com informacao de todos os spans */
		allspansbits =  wc->tspans[0]->txsigs[b];
		if (wc->numspans > 1) {
			allspansbits |=	(wc->tspans[1]->txsigs[b] << 8);
		}
		if (wc->numspans == 4) {
			allspansbits |=	(wc->tspans[2]->txsigs[b] << 16) |
							(wc->tspans[3]->txsigs[b] << 24);
		}
		/* output them to the chip */
		writecas[b] = allspansbits;
		ap_debugk("escrito 0x%08x para ser transmitido pelo CAS (b = %d)\n", allspansbits, b);
	}
	spin_unlock_irqrestore(&wc->reglock, flags);
	if (debug & DEBUG_RBS)
		printk("Finished setting RBS bits\n");
	return 0;
}

static int ap4_shutdown(struct dahdi_span *span)
{
	int tspan;
	int wasrunning;
	unsigned long flags;
	struct ap4_span *ts = ap4_span_from_span(span);
	struct ap4 *wc = ts->owner;

	tspan = span->offset + 1;
	if (tspan < 0) {
		printk("%s: '%d' isn't us?\n", wc->variety, span->spanno);
		return -1;
	}

	spin_lock_irqsave(&wc->reglock, flags);
	wasrunning = span->flags & DAHDI_FLAG_RUNNING;

	span->flags &= ~DAHDI_FLAG_RUNNING;
	if (wasrunning)
		wc->spansstarted--;
	__ap4_set_led(wc, span->offset, AP_OFF);
	if (((wc->numspans == 4) &&
	    (!(wc->tspans[0]->span.flags & DAHDI_FLAG_RUNNING)) &&
	    (!(wc->tspans[1]->span.flags & DAHDI_FLAG_RUNNING)) &&
	    (!(wc->tspans[2]->span.flags & DAHDI_FLAG_RUNNING)) &&
	    (!(wc->tspans[3]->span.flags & DAHDI_FLAG_RUNNING)))
	    			||
	    ((wc->numspans == 2) &&
	    (!(wc->tspans[0]->span.flags & DAHDI_FLAG_RUNNING)) &&
	    (!(wc->tspans[1]->span.flags & DAHDI_FLAG_RUNNING)))
	    			||
	    ((wc->numspans == 1) &&
	    (!(wc->tspans[0]->span.flags & DAHDI_FLAG_RUNNING)))) {
		/* No longer in use, disable interrupts */
		printk("%s: Disabling interrupts since there are no active spans\n",
				wc->variety);
	} else wc->checktiming = 1;
	spin_unlock_irqrestore(&wc->reglock, flags);
	if (debug & DEBUG_MAIN)
		printk("Span %d (%s) shutdown\n", span->spanno, span->name);
	return 0;
}

static int ap4_spanconfig(struct file *file, struct dahdi_span *span,
		struct dahdi_lineconfig *lc)
{
	int i;
	struct ap4_span *ts = ap4_span_from_span(span);
	struct ap4 *wc = ts->owner;
	unsigned int val;

	printk("About to enter spanconfig!\n");
	if (debug & DEBUG_MAIN)
		printk("%s: Configuring span %d\n", wc->variety, span->spanno);
	/* XXX We assume lineconfig is okay and shouldn't XXX */
	span->lineconfig = lc->lineconfig;
	span->txlevel = lc->lbo;
	span->rxlevel = 0;
	if (lc->sync < 0)
		lc->sync = 0;
	if (lc->sync > 4)
		lc->sync = 0;

	/* remove this span number from the current sync sources, if there */
	for(i = 0; i < wc->numspans; i++) {
		if (wc->tspans[i]->sync == span->spanno) {
			wc->tspans[i]->sync = 0;
			wc->tspans[i]->psync = 0;
		}
	}
	wc->tspans[span->offset]->syncpos = lc->sync;
	/* if a sync src, put it in proper place */
	if (lc->sync) {
		wc->tspans[lc->sync - 1]->sync = span->spanno;
		wc->tspans[lc->sync - 1]->psync = span->offset + 1;
	}
	wc->checktiming = 1;
	/* If we're already running, then go ahead and apply the changes */
	if (span->flags & DAHDI_FLAG_RUNNING)
		return ap4_startup(file, span);

	// Limpa contadores de slips, crc e bpv
	val = (*(wc->membase + AP_CNT_SLIP_REG));
	val = (*(wc->membase + AP_CNT_CRC_REG));
	val = (*(wc->membase + AP_CNT_CV_REG));

	ap_debugk("habilitando interrupcao!\n");
	// Nao considera as primeiras interrupcoes na soma das IRQs perdidas
	wc->flag_1st_irq = 16;
	// Enable interrupt
	*(wc->membase + AP_INT_CONTROL_REG) |= AP_INT_CTL_ENABLE;
	// Limpa interrupcao da FPGA para forcar borda de subida na proxima
	val = *(wc->membase + AP_CLEAR_IRQ_REG);

	printk("Done with spanconfig!\n");
	return 0;
}

static int ap4_chanconfig(struct file *file, struct dahdi_chan *chan,
		int sigtype)
{
	int alreadyrunning;
	unsigned long flags;
	struct ap4 *wc = chan->pvt;

	alreadyrunning = wc->tspans[chan->span->offset]->span.flags & DAHDI_FLAG_RUNNING;
	if (debug & DEBUG_MAIN) {
		if (alreadyrunning)
			printk("%s: Reconfigured channel %d (%s) sigtype %d\n",
					wc->variety, chan->channo, chan->name, sigtype);
		else
			printk("%s: Configured channel %d (%s) sigtype %d\n",
					wc->variety, chan->channo, chan->name, sigtype);
	}
	spin_lock_irqsave(&wc->reglock, flags);
	if (alreadyrunning)
		__set_clear(wc, chan->span->offset);
	spin_unlock_irqrestore(&wc->reglock, flags);
	return 0;
}

static int ap4_open(struct dahdi_chan *chan)
{
	try_module_get(THIS_MODULE);
	return 0;
}

static int ap4_close(struct dahdi_chan *chan)
{
	module_put(THIS_MODULE);
	return 0;
}

static const struct dahdi_span_ops ap4_span_ops = {
	.owner = THIS_MODULE,
	.spanconfig = ap4_spanconfig,
	.chanconfig = ap4_chanconfig,
	.startup = ap4_startup,
	.shutdown = ap4_shutdown,
	.rbsbits = ap4_rbsbits,
	.maint = ap4_maint,
	.open = ap4_open,
	.close  = ap4_close,
#ifdef APEC_SUPPORT
	.echocan = ap4_echocan,
#endif
	.ioctl = ap4_ioctl
};

static void init_spans(struct ap4 *wc)
{
	int x,y;
	struct ap4_span *ts;

	for (x=0;x<wc->numspans;x++) {
		ts = wc->tspans[x];
		sprintf(ts->span.name, "AP4%d%d/%d/%d", 0, wc->numspans, wc->num, x + 1);
		snprintf(ts->span.desc, sizeof(ts->span.desc) - 1, "AP4%d%d Card %d Span %d", 0, wc->numspans, wc->num+1, x+1);
		ts->span.ops = &ap4_span_ops;
		if (ts->spantype == TYPE_E1) {
			ts->span.channels = 31;
			ts->span.spantype = SPANTYPE_DIGITAL_E1;
			ts->span.linecompat = DAHDI_CONFIG_HDB3 | DAHDI_CONFIG_CCS | DAHDI_CONFIG_CRC4;
			ts->span.deflaw = DAHDI_LAW_ALAW;
		} else {
			ts->span.channels = 24;
			ts->span.spantype = SPANTYPE_DIGITAL_T1;
			ts->span.linecompat = DAHDI_CONFIG_AMI | DAHDI_CONFIG_B8ZS | DAHDI_CONFIG_D4 | DAHDI_CONFIG_ESF;
			ts->span.deflaw = DAHDI_LAW_MULAW;
		}
		ts->span.chans = ts->chans;
		ts->span.flags = DAHDI_FLAG_RBS;
		ts->owner = wc;
		ts->span.offset = x;
		ts->writechunk = (void *)(wc->writechunk + x * 32 * 2);
		ts->readchunk = (void *)(wc->readchunk + x * 32 * 2);
		for (y=0;y<wc->tspans[x]->span.channels;y++) {
			struct dahdi_chan *mychans = ts->chans[y];
			sprintf(mychans->name, "AP4%d%d/%d/%d/%d", 0, wc->numspans, wc->num, x + 1, y + 1);
			mychans->sigcap = DAHDI_SIG_EM | DAHDI_SIG_CLEAR | DAHDI_SIG_FXSLS | DAHDI_SIG_FXSGS | DAHDI_SIG_FXSKS |
									 DAHDI_SIG_FXOLS | DAHDI_SIG_FXOGS | DAHDI_SIG_FXOKS | DAHDI_SIG_CAS | DAHDI_SIG_EM_E1 | DAHDI_SIG_DACS_RBS;
			mychans->pvt = wc;
			mychans->chanpos = y + 1;
		}
	}
	printk("%s: Spans initialized\n", wc->variety);
}



static void __ap4_set_timing_source(struct ap4 *wc, int unit)
{
	unsigned int timing;
	int x;

	if (unit != wc->syncsrc) {
		if ((unit > -1) && (unit < 4)) {
			/* define fonte de clock para interface escolhida */
			timing = *(wc->membase+AP_CLKSRC_REG);
			timing &= ~AP_CLKSRC_MASK;
			timing |= unit+1;
			*(wc->membase+AP_CLKSRC_REG) = timing;
		} else {
			/* define clock para interno */
			timing = *(wc->membase+AP_CLKSRC_REG);
			timing &= ~AP_CLKSRC_MASK;
			*(wc->membase+AP_CLKSRC_REG) = timing;
		}
		wc->syncsrc = unit;
		if ((unit < 0) || (unit > 3))
			unit = 0;
		else
			unit++;
		for (x=0;x<wc->numspans;x++)
			wc->tspans[x]->span.syncsrc = unit;
	} else {
		if (debug & DEBUG_MAIN)
			printk("%s: Timing source already set to %d\n",
					wc->variety, unit);
	}
	printk("%s: Timing source set to %d (clksrc_reg = 0x%08x)\n",
			wc->variety, unit, *(wc->membase+AP_CLKSRC_REG));
}

static void __ap4_set_timing_source_auto(struct ap4 *wc)
{
	int x;

	wc->checktiming = 0;
	for (x=0;x<wc->numspans;x++) {
		if (wc->tspans[x]->sync) {
			if ((wc->tspans[wc->tspans[x]->psync - 1]->span.flags & DAHDI_FLAG_RUNNING) &&
				!(wc->tspans[wc->tspans[x]->psync - 1]->span.alarms & (DAHDI_ALARM_RED | DAHDI_ALARM_BLUE) )) {
					/* Valid timing source */
					__ap4_set_timing_source(wc, wc->tspans[x]->psync - 1);
					return;
			}
		}
	}
	__ap4_set_timing_source(wc, 4);
}

static void __ap4_configure_t1(struct ap4 *wc, int unit, int lineconfig, int txlevel)
{
	char *framing, *line;
	unsigned int config = 0;
	unsigned int param = 0;
	unsigned int linecode = 0;

	wc->tspans[unit]->spantype = TYPE_T1;
	wc->tspans[unit]->span.channels = 24;
	wc->tspans[unit]->span.deflaw = DAHDI_LAW_MULAW;

	/* Configure line code */
	if (unit < 2)
		linecode = AP_LIU1_LINECODE;
	else
		linecode = AP_LIU2_LINECODE;
	if (lineconfig & DAHDI_CONFIG_AMI) {
		*(wc->membase+AP_LEDS_REG) |= linecode;
		line = "AMI";
	} else {
		*(wc->membase+AP_LEDS_REG) &= ~linecode;
		line = "B8ZS";
	}

	/* loopback test*/
	//wc->hw_regs->e1_config |= (AP_E1_LOOP_CONFIG  << (8 * unit));
	//printk("E1 config = 0x%08x\n", wc->hw_regs->e1_config);

	/* Configure T1 */
	config = wc->hw_regs->liu_config;
	config &= ~(0x000000ff << (8 * unit));
	config |= (AP_PULS_DSX1_0FT << (8 * unit));
	wc->hw_regs->liu_config = config;

	param = AP4_T1_NE1_SEL | AP4_T1_CAS_ENABLE;
	if (lineconfig & DAHDI_CONFIG_D4) {
		framing = "D4";
	} else {
		framing = "ESF";
		param |= AP4_T1_ESF_NSF;
	}
	config = wc->hw_regs->t1_config;
	config &= ~(0x000000ff << (8 * unit));
	config |= (param << (8 * unit));
	wc->hw_regs->t1_config = config;

	printk("T1 Status: 0x%08x\tT1 Config: 0x%08x\tPARAM: 0x%08x\n",
			wc->hw_regs->t1_status, wc->hw_regs->t1_config, param);

	if (!polling) {
		__ap4_check_alarms(wc, unit);
		__ap4_check_sigbits(wc, unit);
	}
	printk("%s: Span %d configured for %s/%s\n", wc->variety, unit + 1, framing, line);
}

static void __ap4_configure_e1(struct ap4 *wc, int unit, int lineconfig)
{
	char *crc4 = "";
	char *framing, *line;
	unsigned int e1s_cfg, config = 0;
	unsigned int linecode = 0;

	wc->tspans[unit]->spantype = TYPE_E1;
	wc->tspans[unit]->span.channels = 31;
	wc->tspans[unit]->span.deflaw = DAHDI_LAW_ALAW;

	if (loopback) {
	}

	if (lineconfig & DAHDI_CONFIG_CRC4) {
		crc4 = "/CRC4";
		config |= AP_E1_CRCEN_CONFIG;
	}

	if(unit < 2)
		linecode = AP_LIU1_LINECODE;
	else
		linecode = AP_LIU2_LINECODE;
	/* Configure line interface */
	if (lineconfig & DAHDI_CONFIG_AMI) {
		*(wc->membase+AP_LEDS_REG) |= linecode;
		line = "AMI";
	} else {
		*(wc->membase+AP_LEDS_REG) &= ~linecode;
		line = "HDB3";
	}

	if (lineconfig & DAHDI_CONFIG_CCS) {
		framing = "CCS";
	} else {
		framing = "CAS";
		config |= (AP_E1_CASEN_CONFIG | AP_E1_PCM30_CONFIG);
	}

	e1s_cfg = *(wc->membase+AP_E1_CONFIG_REG);
	e1s_cfg &= ~(0x000000ff<<(8*unit));
	e1s_cfg |= (config<<(8*unit));
	*(wc->membase+AP_E1_CONFIG_REG) = e1s_cfg;

	/* Disable T1 framer */
	config = wc->hw_regs->t1_config;
	config &= ~(0x000000ff << (8 * unit));
	wc->hw_regs->t1_config = config;

	/* Configure LIU Signalling */
	e1s_cfg = *(wc->membase+AP_T1E1_CONFIG_REG);
	e1s_cfg &= ~(0x000000ff<<(8*unit));
	e1s_cfg |= (AP_PULS_E1_120<<(8*unit));
	*(wc->membase+AP_T1E1_CONFIG_REG) = e1s_cfg;

	if (!polling) {
		__ap4_check_alarms(wc, unit);
		__ap4_check_sigbits(wc, unit);
	}
	printk("%s: Span %d configured for %s/%s%s\n",
 			wc->variety, unit + 1, framing, line, crc4);
}

static int ap4_startup(struct file *file, struct dahdi_span *span)
{
	int i;
	int tspan;
	unsigned long flags;
	int alreadyrunning;
	struct ap4_span *ts = ap4_span_from_span(span);
	struct ap4 *wc = ts->owner;

	printk("About to enter startup!\n");
	tspan = span->offset + 1;
	if (tspan < 0) {
		printk("%s: Span '%d' isn't us?\n", wc->variety, span->spanno);
		return -1;
	}

	spin_lock_irqsave(&wc->reglock, flags);

	alreadyrunning = span->flags & DAHDI_FLAG_RUNNING;

	/* initialize the start value for the entire chunk of last ec buffer */
	for(i = 0; i < span->channels; i++)
	{
		memset(ts->ec_chunk1[i],
			DAHDI_LIN2X(0, span->chans[i]),DAHDI_CHUNKSIZE);
		memset(ts->ec_chunk2[i],
			DAHDI_LIN2X(0, span->chans[i]),DAHDI_CHUNKSIZE);
	}

	/* Force re-evaluation fo timing source */
//	if (timingcable)
		wc->syncsrc = -1;

	if ((span->lineconfig & DAHDI_CONFIG_D4) || (span->lineconfig & DAHDI_CONFIG_ESF)) {
		/* is a T1 card */
		__ap4_configure_t1(wc, span->offset, span->lineconfig, span->txlevel);
	} else { /* is a E1 card */
		__ap4_configure_e1(wc, span->offset, span->lineconfig);
	}

	/* Note clear channel status */
	wc->tspans[span->offset]->notclear = 0;
	__set_clear(wc, span->offset);

	if (!alreadyrunning) {
		span->flags |= DAHDI_FLAG_RUNNING;
		wc->spansstarted++;
		/* enable interrupts */

		if (!polling) {
			__ap4_check_alarms(wc, span->offset);
			__ap4_check_sigbits(wc, span->offset);
		}
	}
	spin_unlock_irqrestore(&wc->reglock, flags);

	if (wc->tspans[0]->sync == span->spanno) printk("SPAN %d: Primary Sync Source\n",span->spanno);
	if (wc->numspans > 1) {
		if (wc->tspans[1]->sync == span->spanno) printk("SPAN %d: Secondary Sync Source\n",span->spanno);
	}
	if (wc->numspans == 4) {
		if (wc->tspans[2]->sync == span->spanno) printk("SPAN %d: Tertiary Sync Source\n",span->spanno);
		if (wc->tspans[3]->sync == span->spanno) printk("SPAN %d: Quaternary Sync Source\n",span->spanno);
	}

#ifdef APEC_SUPPORT
	if (!apec_enable || !wc->apec_enable)
		wc->hw_regs->echo_ctrl = 0xe0000000;
	else if (!alreadyrunning && !wc->apec)
			if (ap4_apec_init(wc))
				ap4_apec_release(wc);
#else
	wc->hw_regs->echo_ctrl = 0xe0000000;
#endif

	printk("Completed startup!\n");
	return 0;
}


static void ap4_receiveprep(struct ap4 *wc)
{
	volatile unsigned int *readchunk;
	unsigned int buffer[32];
	unsigned char *byte = (unsigned char *) buffer;
	int i, j, k;

	readchunk = (wc->membase + (AP_DATA_BASE));
	for (i = 0; i < DAHDI_CHUNKSIZE; i++) {
		/* Prefetch Card data */
		for (j = 0; j < 32; ++j) {
			buffer[j] = readchunk[j];
		}
		for (j = 0; j < wc->numspans; j++) {
			/* Set first timeslot for first channel */
			if (wc->tspans[j]->spantype == TYPE_E1) {
				for (k = 0; k < 31; ++k) {
					/* Skip first timeslot from E1 */
					wc->tspans[j]->span.chans[k]->readchunk[i] =
							byte[4*(k+1)+j];
				}
			}
			else {
				for (k = 0; k < 24; ++k) {
					wc->tspans[j]->span.chans[k]->readchunk[i] =
							byte[4*k+j];
				}
			}
		}
		readchunk += 32;
	}

	for (i = 0; i < wc->numspans; i++) {
		if (wc->tspans[i]->span.flags & DAHDI_FLAG_RUNNING) {
			for (j = 0; j < wc->tspans[i]->span.channels; j++) {
				/* Echo cancel double buffered data */
				dahdi_ec_chunk(wc->tspans[i]->span.chans[j],
				    wc->tspans[i]->span.chans[j]->readchunk,
					wc->tspans[i]->ec_chunk2[j]);
				memcpy(wc->tspans[i]->ec_chunk2[j],wc->tspans[i]->ec_chunk1[j],
					DAHDI_CHUNKSIZE);
				memcpy(wc->tspans[i]->ec_chunk1[j],
					wc->tspans[i]->span.chans[j]->writechunk,
						DAHDI_CHUNKSIZE);
			}
			dahdi_receive(&wc->tspans[i]->span);
		}
	}
}

#if (DAHDI_CHUNKSIZE != 8)
#error Sorry, AP400 driver does not support chunksize != 8
#endif

#ifdef ENABLE_WORKQUEUES
static void workq_handlespan(void *data)
{
	struct ap4_span *ts = data;
	struct ap4 *wc = ts->owner;

//	__receive_span(ts);
//	__transmit_span(ts);
	atomic_dec(&wc->worklist);
	atomic_read(&wc->worklist);

}
#endif

static void ap4_transmitprep(struct ap4 *wc)
{
	volatile unsigned int *writechunk;
	int x,y,z;
	unsigned int tmp;

	for (y=0;y<wc->numspans;y++) {
		if (wc->tspans[y]->span.flags & DAHDI_FLAG_RUNNING)
			dahdi_transmit(&wc->tspans[y]->span);
	}

	writechunk = (wc->membase+(AP_DATA_BASE));
	for (x=0;x<DAHDI_CHUNKSIZE;x++) {
		// Once per chunk
		for (z=0;z<32;z++) {
			// All channels
			tmp = 0;
			for (y = 0; y < wc->numspans; ++y) {
				if (wc->tspans[y]->spantype == TYPE_T1 && z < 24)
					tmp |= (wc->tspans[y]->span.chans[z]->writechunk[x]
					                           << (8*y));
				else /* Span Type is E1 */
					if (z > 0) /* Skip first timeslot */
						tmp |= (wc->tspans[y]->span.chans[z-1]->writechunk[x]
									<< (8*y));
			}
			writechunk[z] = tmp;
		}
		// Advance pointer by 4 TDM frame lengths
		writechunk += 32;
	}

}

static void ap4_tdm_loop(struct ap4 *wc)
{
	volatile unsigned int *buf_ptr;
	int x,z;
	unsigned int tmp;

	buf_ptr = (wc->membase+AP_DATA_BASE);

	for (x=0;x<DAHDI_CHUNKSIZE;x++) {
		// Once per chunk
		for (z=0;z<32;z++) {
			tmp = buf_ptr[z];
			buf_ptr[z] = tmp;
		}
		buf_ptr += 32;
	}
}

static void __ap4_check_sigbits(struct ap4 *wc, int span)
{
	int a,i,rxs;
	struct ap4_span *ts = wc->tspans[span];
	volatile unsigned int *readcas = (wc->membase+AP_CAS_BASE);

//	if (debug & DEBUG_RBS)
//		printk("Checking sigbits on span %d\n", span + 1);

	if (!(ts->span.flags & DAHDI_FLAG_RUNNING))
		return;
	// se span estiver com alarme RED ou BLUE...
	if( (ts->span.alarms & DAHDI_ALARM_RED) || (ts->span.alarms & DAHDI_ALARM_BLUE) ) {
		ts->reload_cas = 4;
	} else if(ts->reload_cas > 0) {
		// da mais um tempo para framer recuperar e enviar bits de CAS validos
		ts->reload_cas--;
	}

	if (ts->spantype == TYPE_E1) {
		for (i = 0; i < 15; i++) {

			// Se estamos em alarme ou recuperando de um entao mascara os bits para "1101" (bloqueado)
			if(ts->reload_cas) {
				a = 0xdd;
			} else {
				a = (int) ts->casbuf[i];
			}
			ts->casbuf[i] = (unsigned char) (readcas[i] >> (8*span))&0xff;

			/* Get high channel in low bits */
			rxs = (a & 0xf);
			if (!(ts->span.chans[i+16]->sig & DAHDI_SIG_CLEAR)) {
				if (ts->span.chans[i+16]->rxsig != rxs) {
					ap_debugk("CAS no canal %d mudou de 0x%02x para 0x%02x\n", i+16, ts->span.chans[i+16]->rxsig, rxs);
					dahdi_rbsbits(ts->span.chans[i+16], rxs);
				}
			}
			rxs = (a >> 4) & 0xf;
			if (!(ts->span.chans[i]->sig & DAHDI_SIG_CLEAR)) {
				if (ts->span.chans[i]->rxsig != rxs) {
					ap_debugk("CAS no canal %d mudou de 0x%02x para 0x%02x\n", i, ts->span.chans[i]->rxsig, rxs);
					dahdi_rbsbits(ts->span.chans[i], rxs);
				}
			}
		}
	} else if (ts->span.lineconfig & DAHDI_CONFIG_D4) {
		for (i = 0; i < 12; i++) {
			a = (unsigned char) (readcas[i] >> (8*span)) & 0xcc;
			rxs = a & 0xc;
			//rxs = (a & 0xc) >> 2;
			if (!(ts->span.chans[2*i]->sig & DAHDI_SIG_CLEAR)) {
				if (ts->span.chans[2*i]->rxsig != rxs)
					dahdi_rbsbits(ts->span.chans[2*i], rxs);
			}
			rxs = (a >> 4) & 0xc;
			//rxs = ((a >> 4) & 0xc) >> 2;
			if (!(ts->span.chans[2*i+1]->sig & DAHDI_SIG_CLEAR)) {
				if (ts->span.chans[2*i+1]->rxsig != rxs)
					dahdi_rbsbits(ts->span.chans[2*i+1], rxs);
			}
		}
	} else { // ESF
		for (i = 0; i < 12; i++) {
			a = (unsigned char) (readcas[i] >> (8*span)) & 0xff;
			rxs = (a & 0xf);
			if (!(ts->span.chans[2*i+1]->sig & DAHDI_SIG_CLEAR)) {
				/* XXX Not really reset on every trans! XXX */
				if (ts->span.chans[2*i+1]->rxsig != rxs) {
					dahdi_rbsbits(ts->span.chans[2*i+1], rxs);
				}
			}
			rxs = (a >> 4) & 0xf;
			if (!(ts->span.chans[2*i]->sig & DAHDI_SIG_CLEAR)) {
				/* XXX Not really reset on every trans! XXX */
				if (ts->span.chans[2*i]->rxsig != rxs) {
					dahdi_rbsbits(ts->span.chans[2*i], rxs);
				}
			}
		}
	}
}

static void __ap4_check_alarms(struct ap4 *wc, int span)
{
	unsigned char c;
	int alarms;
	int x,j;
	struct ap4_span *ts = wc->tspans[span];
	unsigned int e1_cfg;

	if (!(ts->span.flags & DAHDI_FLAG_RUNNING))
		return;

	/* Assume no alarms */
	alarms = DAHDI_ALARM_NONE;

	/* And consider only carrier alarms */
	ts->span.alarms &= (DAHDI_ALARM_RED | DAHDI_ALARM_BLUE | DAHDI_ALARM_NOTOPEN);

	if (ts->span.lineconfig & DAHDI_CONFIG_NOTOPEN) {
		for (x=0,j=0;x < ts->span.channels;x++)
			if ((ts->span.chans[x]->flags & DAHDI_FLAG_OPEN)
#ifdef CONFIG_DAHDI_NET
					||
			    (ts->span.chans[x]->flags & DAHDI_FLAG_NETDEV)
#endif
			    )
				j++;
		if (!j)
			alarms |= DAHDI_ALARM_NOTOPEN;
	}

/* le status e configuracao do E1 */
	if (wc->tspans[span]->spantype == TYPE_E1) {
		c = ((*(wc->membase+AP_E1_STATUS_REG))>>(8*span));
		e1_cfg = ((*(wc->membase+AP_E1_CONFIG_REG))>>(8*span));

		if ((c & AP_E1_LOS_STATUS)||(c & AP_E1_BFAE_STATUS)||(c & AP_E1_AIS_STATUS)) {
			if (ts->alarmcount >= alarmdebounce)
				alarms |= DAHDI_ALARM_RED;
			else
				ts->alarmcount++;
		} else
			ts->alarmcount = 0;

		if ( c & AP_E1_MFAE_STATUS )
			alarms |= DAHDI_ALARM_BLUE;

		if ( (!(c & AP_E1_CAS_STATUS)) && (e1_cfg & AP_E1_PCM30_CONFIG))
			alarms |= DAHDI_ALARM_BLUE;
	} else {
		c = ((*(wc->membase+AP_E1_STATUS_REG))>>(8*span));
		if (c & AP_E1_LOS_STATUS) {
			if (ts->alarmcount >= alarmdebounce)
				alarms |= DAHDI_ALARM_RED;
			else
				ts->alarmcount++;
		} else
			ts->alarmcount = 0;
		c = wc->hw_regs->t1_status >> (8 * span);
		if (!(c & AP4_T1_FRAME_SYNC))
			alarms |= DAHDI_ALARM_RED;
	}

	if (((!ts->span.alarms) && alarms) ||
	    (ts->span.alarms && (!alarms)))
		wc->checktiming = 1;

	/* Keep track of recovering */
	if ((!alarms) && ts->span.alarms)
		ts->alarmtimer = DAHDI_ALARMSETTLE_TIME;
	if (ts->alarmtimer)
		alarms |= DAHDI_ALARM_RECOVER;


	// If receiving alarms, go into Yellow alarm state
	if (alarms && !(ts->spanflags & FLAG_SENDINGYELLOW)) {
		printk("Setting yellow alarm on span %d\n", span + 1);
		e1_cfg = *(wc->membase+AP_E1_CONFIG_REG);
		e1_cfg |= (AP_E1_RAI_CONFIG<<(8*span));
		*(wc->membase+AP_E1_CONFIG_REG) = e1_cfg;
		ts->spanflags |= FLAG_SENDINGYELLOW;
	} else if ((!alarms) && (ts->spanflags & FLAG_SENDINGYELLOW)) {
		printk("Clearing yellow alarm on span %d\n", span + 1);
		e1_cfg = *(wc->membase+AP_E1_CONFIG_REG);
		e1_cfg &= ~(AP_E1_RAI_CONFIG<<(8*span));
		*(wc->membase+AP_E1_CONFIG_REG) = e1_cfg;
		ts->spanflags &= ~FLAG_SENDINGYELLOW;
	}

	// Re-check the timing source when we enter/leave alarm, not withstanding yellow alarm
	if (c & AP_E1_RAI_STATUS)
		alarms |= DAHDI_ALARM_YELLOW;

	if (ts->span.mainttimer || ts->span.maintstat)
		alarms |= DAHDI_ALARM_LOOPBACK;

	ts->span.alarms = alarms;
	dahdi_alarm_notify(&ts->span);
}

static void __ap4_do_counters(struct ap4 *wc)
{
	int span;

	for (span=0;span<wc->numspans;span++) {
		struct ap4_span *ts = wc->tspans[span];
		int docheck=0;
		if (ts->loopupcnt || ts->loopdowncnt)
			docheck++;
		if (ts->alarmtimer) {
			if (!--ts->alarmtimer) {
				docheck++;
				ts->span.alarms &= ~(DAHDI_ALARM_RECOVER);
			}
		}
		if (docheck) {
			if (!polling)
				__ap4_check_alarms(wc, span);
			dahdi_alarm_notify(&ts->span);
		}
	}
}

static inline void __handle_leds(struct ap4 *wc)
{
	int x, span_status;
	#define MAX_BLINKTIMER	0x14

	for (x=0;x<wc->numspans;x++) {
		struct ap4_span *ts = wc->tspans[x];
		/* le status do E1 (para avaliar LOS) */
		span_status = ((*(wc->membase+AP_E1_STATUS_REG))>>(8*x));
		if (ts->span.flags & DAHDI_FLAG_RUNNING) {
			if(span_status&AP_E1_LOS_STATUS) {
				if (wc->blinktimer[x] >= (altab[wc->alarmpos[x]] /*>> 1*/)) {
					__ap4_set_led(wc, x, AP_ON);
				}
				if (wc->blinktimer[x] >= (MAX_BLINKTIMER-1)) {
					__ap4_set_led(wc, x, AP_OFF);
				}
				wc->blinktimer[x] += 1;
			} else if (ts->span.alarms & (DAHDI_ALARM_RED | DAHDI_ALARM_BLUE)) {
				if (wc->blinktimer[x] >= (altab[wc->alarmpos[x]] /*>> 1*/)) {
					__ap4_set_led(wc, x, AP_ON);
				}
				if (wc->blinktimer[x] >= (MAX_BLINKTIMER-2)) {
					__ap4_set_led(wc, x, AP_OFF);
				}
				wc->blinktimer[x] += 3;
			} /*else if (ts->span.alarms & DAHDI_ALARM_YELLOW) {
				// Yellow Alarm
				__ap4_set_led(wc, x, AP_ON);
			} else if (ts->span.mainttimer || ts->span.maintstat) {

				if (wc->blinktimer == (altab[wc->alarmpos] >> 1)) {
					__ap4_set_led(wc, x, AP_GREEN);
				}
				if (wc->blinktimer == 0xf) {
					__ap4_set_led(wc, x, AP_OFF);
				}

			} */else {
				/* No Alarm */
				__ap4_set_led(wc, x, AP_ON);
			}
		}	else
				__ap4_set_led(wc, x, AP_OFF);

		if (wc->blinktimer[x] > MAX_BLINKTIMER) {
			wc->blinktimer[x] = 0;
			wc->alarmpos[x]++;
			if (wc->alarmpos[x] >= (sizeof(altab) / sizeof(altab[0])))
				wc->alarmpos[x] = 0;
		}

	}
}


static irqreturn_t ap4_interrupt(int irq, void *dev_id)
{
	struct ap4 *wc = dev_id;
	unsigned long flags;
	int x;
	static unsigned int val, cfg;
	unsigned int cnt_irq_misses;
	static unsigned int cnt_tmp;
	int ret = 0;

	/* retorna se interrupcao nao foi habilitada ou nao esta ativa */
	cfg = *(wc->membase + AP_INT_CONTROL_REG);
	if((cfg & AP_INT_CTL_ENABLE) == 0 || (cfg & AP_INT_CTL_ACTIVE) == 0) {
		ret = 0;
		goto out;
	}
	/* se chegamos aqui eh porque a interrupcao esta habilitada
	 * e esta ativa, ou seja, foi gerada pelo nosso cartao.
	 * Agora damos o ack da interrupcao */
	val = *(wc->membase + AP_CLEAR_IRQ_REG);

	/* conta interrupcoes perdidas */
	if (wc->flag_1st_irq > 0) {
		// nao considera as primeiras passagens pela rotina
		cnt_irq_misses = (*(wc->membase+AP_CNT_IRQ_REG));
		// so considera int. para o cartao
		if(cnt_irq_misses) {
			wc->flag_1st_irq--;
			*(wc->membase+AP_CNT_IRQ_REG)=0;
		}
		// zera erro de CRC
		cnt_tmp = (*(wc->membase + AP_CNT_CRC_REG));
	} else {
		// neste registro da FPGA temos o numero de interrupcoes que aconteceram
		// desde o ultimo reset do contador de interrupcoes. O normal eh ler 1.
		cnt_irq_misses = (*(wc->membase+AP_CNT_IRQ_REG));
		// Se for zero significa que a interrupcao nao foi gerada pelo nosso cartao
		if(cnt_irq_misses == 0) {
			if(debug) printk("Interrupcao gerada mas nao pela FPGA?!\n");
			ret = 0;
			goto out;
		}
		// reseta o contador
		*(wc->membase+AP_CNT_IRQ_REG)=0;
		for(x=0;x<(wc->numspans);x++)
			wc->ddev->irqmisses += (cnt_irq_misses-1);
	}

	if (!wc->spansstarted) {
		/* Not prepped yet! */
		ret = 0;
		goto out;
	}

	wc->intcount++;

#ifdef ENABLE_WORKQUEUES
	int cpus = num_online_cpus();
	atomic_set(&wc->worklist, wc->numspans);
	if (wc->tspans[0]->span.flags & DAHDI_FLAG_RUNNING)
		ap4_queue_work(wc->workq, &wc->tspans[0]->swork, 0);
	else
		atomic_dec(&wc->worklist);
	if (wc->numspans > 1) {
		if (wc->tspans[1]->span.flags & DAHDI_FLAG_RUNNING)
			ap4_queue_work(wc->workq, &wc->tspans[1]->swork, 1 % cpus);
		else
			atomic_dec(&wc->worklist);
	}
	if (wc->numspans == 4) {
		if (wc->tspans[2]->span.flags & DAHDI_FLAG_RUNNING)
			ap4_queue_work(wc->workq, &wc->tspans[2]->swork, 2 % cpus);
		else
			atomic_dec(&wc->worklist);
		if (wc->tspans[3]->span.flags & DAHDI_FLAG_RUNNING)
			ap4_queue_work(wc->workq, &wc->tspans[3]->swork, 3 % cpus);
		else
			atomic_dec(&wc->worklist);
	}
#else
	if (tdm_loop == 1)
		ap4_tdm_loop(wc);
	else {
		ap4_receiveprep(wc);
		ap4_transmitprep(wc);
	}
#endif

	// Estatisticas a cada 128ms
	if(!(wc->intcount&0x7f)){
		clock_source = wc->hw_regs->clock_source;
		cnt_tmp = (*(wc->membase + AP_CNT_CV_REG));
		for(x=0;x<(wc->numspans);x++)
			wc->tspans[x]->span.count.bpv += (cnt_tmp>>(8*x))&0xff;
		cnt_tmp = (*(wc->membase + AP_CNT_CRC_REG));
		for(x=0;x<(wc->numspans);x++)
			wc->tspans[x]->span.count.crc4 += (cnt_tmp>>(8*x))&0xff;
		cnt_tmp = (*(wc->membase + AP_CNT_SLIP_REG));
		for(x=0;x<(wc->numspans);x++) {
			if (((cnt_tmp>>(8*x))&0xff) && (!(wc->tspans[x]->span.alarms & DAHDI_ALARM_RED)) ){
				wc->tspans[x]->slipcount++;
				if(debug) printk("Slip detected on span %d: slipcount = %d\n", x+1, wc->tspans[x]->slipcount);
			}
		}
	}

	spin_lock_irqsave(&wc->reglock, flags);

	__handle_leds(wc);

	__ap4_do_counters(wc);

	//x = wc->intcount & 15;
	x = wc->intcount & 7;
	switch(x) {
	case 0:
	case 1:
	case 2:
	case 3:
		__ap4_check_alarms(wc, x);
		break;
	case 4:
	case 5:
	case 6:
	case 7:
		__ap4_check_sigbits(wc, x - 4);
		break;
	}

	if (wc->checktiming > 0)
		__ap4_set_timing_source_auto(wc);
	spin_unlock_irqrestore(&wc->reglock, flags);
	/* IRQ was treated */
	ret = 1;
out:
#ifdef AP400_HDLC
	/* Call AP400_HDLC_CARD IRQ handler before leave */
	ret |= ap400_intr_handler(irq, wc->hdlc_card);
#endif

	return IRQ_RETVAL(ret);
}


static int __devinit ap4_launch(struct ap4 *wc)
{
	int x;
	unsigned long flags;

	if (wc->tspans[0]->span.flags & DAHDI_FLAG_REGISTERED)
		return 0;
	printk("%s: Launching card: %d\n", wc->variety, wc->order);

	/* Setup serial parameters and system interface */
	for (x=0;x<4;x++) {
		//ap4_serial_setup(wc, x);
		wc->globalconfig = 1;
	}

	for (x=0; x<wc->numspans; x++)
		list_add_tail(&wc->tspans[x]->span.device_node,
				&wc->ddev->spans);
	if (dahdi_register_device(wc->ddev, &wc->dev->dev)) {
		printk(KERN_ERR "Unable to register span %s\n", wc->tspans[0]->span.name);
		return -1; /* FIXME: proper error handling */
	}
	wc->checktiming = 1;
	spin_lock_irqsave(&wc->reglock, flags);
//	__ap4_set_timing_source(wc,4);
	spin_unlock_irqrestore(&wc->reglock, flags);
#ifdef ENABLE_TASKLETS
	tasklet_init(&wc->ap4_tlet, ap4_tasklet, (unsigned long)wc);
#endif
	return 0;
}


static int ap4xx_liu_reset(struct ap4 *wc)
{
 	unsigned int jiffies_hold = jiffies;
	*(wc->membase+AP_LEDS_REG) |= AP_LIU_RESET_BIT;
	while(jiffies<=(jiffies_hold+2));
	*(wc->membase+AP_LEDS_REG) &= ~AP_LIU_RESET_BIT;
	return 0;
}


static int ap4xx_bus_test(struct ap4 *wc)
{
	int tst_result = 0;
	unsigned int val;

	*(wc->membase+AP_E1_CONFIG_REG) = 0xAAAAAAAA;
	*wc->membase = 0; // flush
	val = *(wc->membase+AP_E1_CONFIG_REG);
	if(val != 0xAAAAAAAA) {
		printk("Escrito 0xAAAAAAAA, lido 0x%08X!\n", val);
		tst_result++;
	}
	*(wc->membase+AP_E1_CONFIG_REG) = 0x55555555;
	*wc->membase = 0; // flush
	val = *(wc->membase+AP_E1_CONFIG_REG);
	if(val != 0x55555555) {
		printk("Escrito 0x55555555, lido 0x%08X!\n", val);
		tst_result++;
	}
	*(wc->membase+AP_E1_CONFIG_REG) = 0xFFFFFFFF;
	*wc->membase = 0; // flush
	val = *(wc->membase+AP_E1_CONFIG_REG);
	if(val != 0xFFFFFFFF) {
		printk("Escrito 0xFFFFFFFF, lido 0x%08X!\n", val);
		tst_result++;
	}
	*(wc->membase+AP_E1_CONFIG_REG) = 0x00000000;
	*wc->membase = 0xFFFFFFFF; // flush
	val = *(wc->membase+AP_E1_CONFIG_REG);
	if(val != 0x00000000) {
		printk("Escrito 0x00000000, lido 0x%08X!\n", val);
		tst_result++;
	}
	return tst_result;
}

#ifdef TIMER_DEBUG
void ap4xx_opt_timeout(unsigned long arg)
{
	struct pci_dev *dev = (struct pci_dev *)arg;
	struct ap4 *wc = pci_get_drvdata(dev);

//	ap_debugk("wc->tspans[0]->span.chans[1].readchunk[1] = 0x%02x\n", wc->tspans[0]->span.chans[0].readchunk[1]);
//	ap_debugk("e1s_cfg = 0x%08x\n", *(wc->membase+AP_E1_CONFIG_REG));
//	ap_debugk("e1_status = 0x%08x\n", *(wc->membase + AP_E1_STATUS_REG));
//	ap_debugk("clk_cfg = 0x%08x\n", *(wc->membase+0x07));
//	ap_debugk("e1_data = 0x%08x\n", *(wc->membase + (AP_DATA_BASE + 1)));
//	ap_debugk("cas_data = 0x%08x\n", *(wc->membase + AP_CAS_BASE));

	// dispara timer novamente
	init_timer(&ap4xx_opt_timer);
	ap4xx_opt_timer.function = ap4xx_opt_timeout;
	ap4xx_opt_timer.data = arg;
	ap4xx_opt_timer.expires = jiffies + (delay/4);
	add_timer(&ap4xx_opt_timer);

}
#endif

static inline int ap4_card_detect (struct ap4 *wc) {
	int i;
	if ((wc->hw_regs->card_id != AP4XX_CARD_ID) &&
			(wc->hw_regs->card_id != APE4XX_CARD_ID)) {
		printk("AP400: Unknown card ID(0x%08X)! Aborting...\n", wc->hw_regs->card_id);
		return -EPERM;
	}
	// Test bus integrity
	for (i=0; i < 1000; i++) {
		if (ap4xx_bus_test(wc)) {
			printk("AP400: Bus integrity test failed! Aborting...\n");
			return -EIO;
		}
	}
	printk("AP400: Bus integrity OK!\n");

	wc->fpgaver = wc->hw_regs->fpga_ver;
	wc->numspans = wc->hw_regs->span_num;
	wc->hwid = ((*(wc->membase+AP_HWCONFIG_REG))&AP_HWID_MASK)>>4;

	if ((wc->hwid == AP_HWID_1E1_RJ && wc->numspans != 1) ||
			(wc->hwid == AP_HWID_2E1_RJ && wc->numspans != 2) ||
			(wc->hwid == AP_HWID_4E1_RJ && wc->numspans != 4)) {
		printk("AP400: Incompatible Hardware ID(0x%02x)! Aborting...\n", wc->hwid);
		return -EIO;
	}

	if (wc->hw_regs->card_id == AP4XX_CARD_ID)
		switch (wc->numspans) {
		case 1:
			wc->dt = (struct devtype *) &ap401;
			break;
		case 2:
			wc->dt = (struct devtype *) &ap402;
			break;
		case 4:
			wc->dt = (struct devtype *) &ap404;
			break;
		default:
			printk("AP400: Unsupported spans number(%d)! Aborting...\n",
					wc->numspans);
			return -EPERM;
		}
	else
		switch (wc->numspans) {
		case 1:
			wc->dt = (struct devtype *) &ape401;
			break;
		case 2:
			wc->dt = (struct devtype *) &ape402;
			break;
		case 4:
			wc->dt = (struct devtype *) &ape404;
			break;
		default:
			printk("APE400: Unsupported spans number(%d)! Aborting...\n",
					wc->numspans);
			return -EPERM;
	}

	wc->variety = wc->dt->desc;
	printk("Found a %s (firmware version %d.%d) at base address %08lx, remapped to %p\n",
			wc->variety, wc->fpgaver >> 8, wc->fpgaver & 0xFF,
			wc->memaddr, wc->membase);

	return 0;
}

static void __devexit ap4_remove_one(struct pci_dev *pdev);

static int __devinit ap4_init_one(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	int res;
	struct ap4 *wc;
	int x,f;
	int basesize;
	static int initd_ifaces=0;
	// Initialize pointer struct
	if(!initd_ifaces){
		memset((void *)cards,0,(sizeof(struct ap4 *))*MAX_AP4_CARDS);
		initd_ifaces=1;
	}

	if ((res = pci_enable_device(pdev)) != 0) {
		goto out;
	}
	// Allocate card struct
	wc = kmalloc(sizeof(struct ap4), GFP_KERNEL);
	if (wc == NULL) {
		res = -ENOMEM;
		goto out;
	}

	memset(wc, 0x0, sizeof(struct ap4));
	spin_lock_init(&wc->reglock);

	basesize = DAHDI_MAX_CHUNKSIZE * 32 * 2 * 4;

	// Request PCI regions
	if ((res = pci_request_regions(pdev, "ap400")) != 0) {
		printk("AP400: Unable to request regions!\n");
		goto out;
	}

	// Remap PCI address
	wc->memaddr = pci_resource_start(pdev, 2);
	wc->memlen = pci_resource_len(pdev, 2);
	wc->membase = ioremap(wc->memaddr, wc->memlen);
	if(wc->membase == NULL) {
		printk("AP400: ioremap failed!\n");
		res = -EIO;
		goto out;
	}
	wc->hw_regs = (struct ap4_regs *) wc->membase;

	// Detect Card model
	if ((res = ap4_card_detect(wc)) != 0)
		goto out;

	ap4xx_liu_reset(wc);

	// This rids of the Double missed interrupt message after loading
	wc->last0 = 1;

	wc->dev = pdev;

	// 32 channels, Double-buffer, Read/Write, 4 spans
	wc->writechunk = kmalloc(basesize * 2, GFP_KERNEL);
	if (!wc->writechunk) {
		printk("%s: Unable to allocate memory!\n", wc->variety);
		res = -ENOMEM;
		goto out;
	}

	// Read is after the whole write piece (in words)
	wc->readchunk = wc->writechunk + basesize / 4;


	// Initialize Write/Buffers to all blank data
	memset((void *) wc->writechunk, 0x00, basesize);
	memset((void *) wc->readchunk, 0xff, basesize);

	/* Keep track of which device we are */
	pci_set_drvdata(pdev, wc);

	/* inicializa contador de interrupcao */
	wc->intcount = 0;

	for(x = 0; x < MAX_AP4_CARDS; x++) {
		if (!cards[x]) break;
	}

	if (x >= MAX_AP4_CARDS) {
		printk("No cards[] slot available!!\n");
		res = -ENOMEM;
		goto out;
	}

	wc->num = x;
	cards[x] = wc;

	/* Allocate pieces we need here, consider 31 channels for E1*/
	for (x=0;x<4;x++) {
		wc->tspans[x] = kmalloc(sizeof(struct ap4_span), GFP_KERNEL);
		if (wc->tspans[x]) {
			memset(wc->tspans[x], 0, sizeof(struct ap4_span));
			wc->tspans[x]->spantype = TYPE_E1;
		} else {
			res = -ENOMEM;
			goto out;
		}
		for (f = 0; f < 31; f++) {
			if (!(wc->tspans[x]->chans[f] = kmalloc(sizeof(*wc->tspans[x]->chans[f]), GFP_KERNEL))) {
				res = -ENOMEM;
				goto out;
			}
			memset(wc->tspans[x]->chans[f], 0, sizeof(*wc->tspans[x]->chans[f]));
		}
#ifdef ENABLE_WORKQUEUES
		INIT_WORK(&wc->tspans[x]->swork, workq_handlespan, wc->tspans[x]);
#endif
		wc->tspans[x]->spanflags |= wc->dt->flags;
	}

	if (request_irq(pdev->irq, ap4_interrupt, IRQF_DISABLED | IRQF_SHARED, "ap400", wc))
	{
		printk("%s: Unable to request IRQ %d\n", wc->variety, pdev->irq);
		res = -EIO;
		goto out;
	}

	wc->ddev = dahdi_create_device();
	wc->ddev->location = kasprintf(GFP_KERNEL, "PCI Bus %02d Slot %02d",
			wc->dev->bus->number, PCI_SLOT(wc->dev->devfn) + 1);
	if (!wc->ddev->location)
		return -ENOMEM; /* FIXME: proper error handling */
	wc->ddev->manufacturer = "Aligera";
	wc->ddev->devicetype = wc->variety;
	wc->ddev->irqmisses = 0;
	init_spans(wc);

	/* Launch cards as appropriate */
	x = 0;
	for(;;) {
		/* Find a card to activate */
		f = 0;
		for (x=0;cards[x];x++) {
			if (cards[x]->order <= highestorder) {
				ap4_launch(cards[x]);
				if (cards[x]->order == highestorder)
					f = 1;
			}
		}
		/* If we found at least one, increment the highest order and search again, otherwise stop */
		if (f)
			highestorder++;
		else
			break;
	}

#ifdef APEC_SUPPORT
	if (wc->fpgaver >= 0x0400)
		wc->apec_enable = 1;
#endif

#ifdef TIMER_DEBUG
	// dispara timer de debug
	init_timer(&ap4xx_opt_timer);
	ap4xx_opt_timer.function = ap4xx_opt_timeout;
	ap4xx_opt_timer.data = (unsigned long) pdev;
	ap4xx_opt_timer.expires = jiffies + 100;
	add_timer(&ap4xx_opt_timer);
#endif

	/* Initialize HDLC_CARD */
#ifdef AP400_HDLC
	u8 __iomem *base_addr[3];
	unsigned int bar_size[3];
	int i;
	base_addr[2] = (void *) wc->membase;
	bar_size[2] = wc->memlen;
	for (i = 0; i < 2; i++) {
		bar_size[i] = (u32) pci_resource_len(pdev, i);
		base_addr[i] = ioremap(pci_resource_start(pdev, i),
								bar_size[i]);
		if (base_addr[i] == NULL) {
			printk(KERN_ERR "Memory map failed\n");
			res = -ENODEV;
			goto out;
		}
	}
	ap400_card_init(&wc->hdlc_card, base_addr, bar_size);
	ap400_intr_enable(wc->hdlc_card);
#endif

	res = 0;
out:
	if (res != 0) {
		ap4_remove_one(pdev);
	}
	return res;
}

static void __devexit ap4_remove_one(struct pci_dev *pdev)
{
	struct ap4 *wc = pci_get_drvdata(pdev);
	int x;

	if (wc) {
		ap_debugk("desabilita interrupcao!\n");
		// desabilita interrupcao
		*(wc->membase + AP_INT_CONTROL_REG) &= ~AP_INT_CTL_ENABLE;

#ifdef APEC_SUPPORT
		// Stop echo cancellation module
		ap4_apec_release(wc);
#endif
		/* Unregister spans */
		dahdi_unregister_device(wc->ddev);
		kfree(wc->ddev->location);
		dahdi_free_device(wc->ddev);
#ifdef ENABLE_WORKQUEUES
		if (wc->workq) {
			flush_workqueue(wc->workq);
			destroy_workqueue(wc->workq);
		}
#endif

#ifdef TIMER_DEBUG
		del_timer(&ap4xx_opt_timer);
#endif

		wc->hw_regs = NULL;
		if(wc->membase)
			iounmap((void *)wc->membase);

		/* Immediately free resources */
		kfree((void *) wc->writechunk);

#ifdef AP400_HDLC
		/* Remove HDLC Card */
		ap400_card_remove(wc->hdlc_card);
		if (wc->hdlc_card->cfg_base_addr)
			iounmap(wc->hdlc_card->cfg_base_addr);
		if (wc->hdlc_card->buf_base_addr)
			iounmap(wc->hdlc_card->buf_base_addr);
		kfree(wc->hdlc_card);
#endif
		free_irq(pdev->irq, wc);

		cards[wc->num] = NULL;
		for (x=0;x<wc->numspans;x++) {
			if (wc->tspans[x])
				kfree(wc->tspans[x]);
		}
		kfree(wc);
	}
	pci_release_regions(pdev);
	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
	printk(KERN_INFO "AP400 driver removed\n");
}


static struct pci_device_id ap4_pci_tbl[] __devinitdata =
{
	{ PCI_DEVICE(PCI_VENDOR_ID_XILINX, PCI_DEVICE_ID_AP4XX), },
	{ 0, }
};


static struct pci_driver ap4_driver = {
	.name = 	"Unified ap4xx driver",
	.probe = 	ap4_init_one,
#ifdef LINUX26
	.remove =	__devexit_p(ap4_remove_one),
#else
	.remove =	ap4_remove_one,
#endif
	.id_table = ap4_pci_tbl,
};

static int __init ap4_init(void)
{
	int res;
	printk("Unified AP4XX PCI Card Driver\n");
	res = pci_register_driver(&ap4_driver);
	if (res) {
		return -ENODEV;
	}
	return 0;
}

static void __exit ap4_cleanup(void)
{
	printk("Unified AP4XX PCI Card Driver Cleanup\n");
	pci_unregister_driver(&ap4_driver);
}


MODULE_AUTHOR("Aligera (aligera@aligera.com.br)");
MODULE_DESCRIPTION("Unified AP4XX PCI Card Driver");
#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif
module_param(debug, int, 0600);
module_param(loopback, int, 0600);
module_param(noburst, int, 0600);
module_param(debugslips, int, 0600);
module_param(polling, int, 0600);
module_param(timingcable, int, 0600);
module_param(t1e1override, int, 0600);
module_param(alarmdebounce, int, 0600);
module_param(j1mode, int, 0600);

MODULE_DEVICE_TABLE(pci, ap4_pci_tbl);

module_init(ap4_init);
module_exit(ap4_cleanup);
