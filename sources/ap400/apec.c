/*
 * AP400 Echo Cancelation Hardware support
 *
 * Written by Wagner Gegler <aligera@aligera.com.br>
 *
 * Based on previous work written by Mark Spencer <markster@digium.com>
 *
 * Copyright (C) 2005-2006 Digium, Inc.
 *
 * Mark Spencer <markster@digium.com>
 *
 * All Rights Reserved
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

#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/string.h>
#include <linux/time.h>
#include <linux/version.h>
#include <linux/delay.h>

#include "apec.h"
#include "oct6100api/oct6100_api.h"

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
#include <linux/config.h>
#else
#include <linux/autoconf.h>
#endif

/* API for Octasic access */
UINT32 Oct6100UserGetTime(tPOCT6100_GET_TIME f_pTime)
{
	/* Why couldn't they just take a timeval like everyone else? */
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,0,0)
	struct timeval tv;
#else
	struct timespec64 tv;
#endif
	unsigned long long total_usecs;
	unsigned int mask = ~0;

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 0, 0)
	do_gettimeofday(&tv);
	total_usecs = (((unsigned long long)(tv.tv_sec)) * 1000000) +
				  (((unsigned long long)(tv.tv_usec)));
#else
	ktime_get_real_ts64(&tv);
	total_usecs = (((unsigned long long)(tv.tv_sec)) * 1000000) +
				  (((unsigned long long)(tv.tv_nsec))) / 1000;
#endif
	f_pTime->aulWallTimeUs[0] = (total_usecs & mask);
	f_pTime->aulWallTimeUs[1] = (total_usecs >> 32);
	return cOCT6100_ERR_OK;
}

UINT32 Oct6100UserMemSet(PVOID f_pAddress, UINT32 f_ulPattern, UINT32 f_ulLength)
{
	memset(f_pAddress, f_ulPattern, f_ulLength);
	return cOCT6100_ERR_OK;
}

UINT32 Oct6100UserMemCopy(PVOID f_pDestination, const void *f_pSource, UINT32 f_ulLength)
{
	memcpy(f_pDestination, f_pSource, f_ulLength);
	return cOCT6100_ERR_OK;
}

UINT32 Oct6100UserCreateSerializeObject(tPOCT6100_CREATE_SERIALIZE_OBJECT f_pCreate)
{
	return cOCT6100_ERR_OK;
}

UINT32 Oct6100UserDestroySerializeObject(tPOCT6100_DESTROY_SERIALIZE_OBJECT f_pDestroy)
{
#ifdef OCTASIC_DEBUG
	printk("I should never be called! (destroy serialize object)\n");
#endif
	return cOCT6100_ERR_OK;
}

UINT32 Oct6100UserSeizeSerializeObject(tPOCT6100_SEIZE_SERIALIZE_OBJECT f_pSeize)
{
	/* Not needed */
	return cOCT6100_ERR_OK;
}

UINT32 Oct6100UserReleaseSerializeObject(tPOCT6100_RELEASE_SERIALIZE_OBJECT f_pRelease)
{
	/* Not needed */
	return cOCT6100_ERR_OK;
}

UINT32 Oct6100UserDriverWriteApi(tPOCT6100_WRITE_PARAMS f_pWriteParams)
{
	oct_write(f_pWriteParams->pProcessContext, f_pWriteParams->ulWriteAddress, f_pWriteParams->usWriteData);
	return cOCT6100_ERR_OK;
}

UINT32 Oct6100UserDriverWriteSmearApi(tPOCT6100_WRITE_SMEAR_PARAMS f_pSmearParams)
{
	unsigned int x;
	for (x=0;x<f_pSmearParams->ulWriteLength;x++) {
		oct_write(f_pSmearParams->pProcessContext, f_pSmearParams->ulWriteAddress + (x << 1), f_pSmearParams->usWriteData);
	}
	return cOCT6100_ERR_OK;
}

UINT32 Oct6100UserDriverWriteBurstApi(tPOCT6100_WRITE_BURST_PARAMS f_pBurstParams)
{
	unsigned int x;
	for (x=0;x<f_pBurstParams->ulWriteLength;x++) {
		oct_write(f_pBurstParams->pProcessContext, f_pBurstParams->ulWriteAddress + (x << 1), f_pBurstParams->pusWriteData[x]);
	}
	return cOCT6100_ERR_OK;
}

UINT32 Oct6100UserDriverReadApi(tPOCT6100_READ_PARAMS f_pReadParams)
{
	*(f_pReadParams->pusReadData) = oct_read(f_pReadParams->pProcessContext, f_pReadParams->ulReadAddress);
	return cOCT6100_ERR_OK;
}

UINT32 Oct6100UserDriverReadBurstApi(tPOCT6100_READ_BURST_PARAMS f_pBurstParams)
{
	unsigned int x;
	for (x=0;x<f_pBurstParams->ulReadLength;x++) {
		f_pBurstParams->pusReadData[x] = oct_read(f_pBurstParams->pProcessContext, f_pBurstParams->ulReadAddress + (x << 1));
	}
	return cOCT6100_ERR_OK;
}

#if 0
#define cOCT6100_ECHO_OP_MODE_DIGITAL cOCT6100_ECHO_OP_MODE_HT_FREEZE
#else
#define cOCT6100_ECHO_OP_MODE_DIGITAL cOCT6100_ECHO_OP_MODE_POWER_DOWN
#endif

struct apec_s {
	tPOCT6100_INSTANCE_API pApiInstance;
	UINT32 aulEchoChanHndl[128];
	int chanflags[128];
	int ecmode[128];
	int numchans;
};

#define FLAG_DTMF	 (1 << 0)
#define FLAG_MUTE	 (1 << 1)
#define FLAG_ECHO	 (1 << 2)

static void apec_setecmode(struct apec_s *apec, int channel, int mode)
{
	tOCT6100_CHANNEL_MODIFY *modify;
	UINT32 ulResult;

	if (apec->ecmode[channel] == mode)
		return;
	modify = kmalloc(sizeof(tOCT6100_CHANNEL_MODIFY), GFP_ATOMIC);
	if (!modify) {
		printk("APEC: Unable to allocate memory for setec!\n");
		return;
	}
	Oct6100ChannelModifyDef(modify);
	modify->ulEchoOperationMode = mode;
	modify->ulChannelHndl = apec->aulEchoChanHndl[channel];
	ulResult = Oct6100ChannelModify(apec->pApiInstance, modify);
	if (ulResult != GENERIC_OK) {
		printk("Failed to apply echo can changes on channel %d!\n", channel);
	} else {
#ifdef OCTASIC_DEBUG
		printk("Echo can on channel %d set to %d\n", channel, mode);
#endif
		apec->ecmode[channel] = mode;
	}
	kfree(modify);
}

void apec_setec(struct apec_s *apec, int channel, int eclen)
{
	if (eclen) {
		apec->chanflags[channel] |= FLAG_ECHO;
		apec_setecmode(apec, channel, cOCT6100_ECHO_OP_MODE_HT_RESET);
		apec_setecmode(apec, channel, cOCT6100_ECHO_OP_MODE_NORMAL);
	} else {
		apec->chanflags[channel] &= ~FLAG_ECHO;
		if (apec->chanflags[channel] & (FLAG_DTMF | FLAG_MUTE)) {
			apec_setecmode(apec, channel, cOCT6100_ECHO_OP_MODE_HT_RESET);
			apec_setecmode(apec, channel, cOCT6100_ECHO_OP_MODE_HT_FREEZE);
		} else
			apec_setecmode(apec, channel, cOCT6100_ECHO_OP_MODE_DIGITAL);
	}
	printk("APEC: Setting EC on channel %d to %d\n", channel, eclen);
}

int apec_checkirq(struct apec_s *apec)
{
	tOCT6100_INTERRUPT_FLAGS InterruptFlags;

	Oct6100InterruptServiceRoutineDef(&InterruptFlags);
	Oct6100InterruptServiceRoutine(apec->pApiInstance, &InterruptFlags);

	return InterruptFlags.fToneEventsPending ? 1 : 0;
}

unsigned int apec_capacity_get(void *wc)
{
	UINT32 ulResult;

	tOCT6100_API_GET_CAPACITY_PINS CapacityPins;

	Oct6100ApiGetCapacityPinsDef(&CapacityPins);
	CapacityPins.pProcessContext = wc;
	CapacityPins.ulMemoryType = cOCT6100_MEM_TYPE_DDR;
	CapacityPins.fEnableMemClkOut = TRUE;
	CapacityPins.ulMemClkFreq = cOCT6100_MCLK_FREQ_133_MHZ;

	ulResult = Oct6100ApiGetCapacityPins(&CapacityPins);
	if (ulResult != cOCT6100_ERR_OK) {
		printk("Failed to get chip capacity, code %08x!\n", ulResult);
		return 0;
	}
	return CapacityPins.ulCapacityValue;
}

struct apec_s *apec_init(void *wc, int *isalaw, int numspans, const struct firmware *firmware)
{
	tOCT6100_CHIP_OPEN *ChipOpen;
	tOCT6100_GET_INSTANCE_SIZE InstanceSize;
	tOCT6100_CHANNEL_OPEN *ChannelOpen;
	UINT32 ulResult;
	struct apec_s *apec;
	int x, law;
#ifdef CONFIG_4KSTACKS
	unsigned long flags;
#endif

	if (!(apec = kmalloc(sizeof(struct apec_s), GFP_KERNEL)))
		return NULL;

	memset(apec, 0, sizeof(struct apec_s));

	if (!(ChipOpen = kmalloc(sizeof(tOCT6100_CHIP_OPEN), GFP_KERNEL))) {
		kfree(apec);
		return NULL;
	}

	memset(ChipOpen, 0, sizeof(tOCT6100_CHIP_OPEN));

	if (!(ChannelOpen = kmalloc(sizeof(tOCT6100_CHANNEL_OPEN), GFP_KERNEL))) {
		kfree(apec);
		kfree(ChipOpen);
		return NULL;
	}

	memset(ChannelOpen, 0, sizeof(tOCT6100_CHANNEL_OPEN));

	for (x=0;x<128;x++)
		apec->ecmode[x] = -1;

	apec->numchans = numspans * 32;
	printk("APEC: echo cancellation for %d channels\n", apec->numchans);

	Oct6100ChipOpenDef(ChipOpen);

	/* Setup Chip Open Parameters */
	ChipOpen->ulUpclkFreq = cOCT6100_UPCLK_FREQ_33_33_MHZ;
	Oct6100GetInstanceSizeDef(&InstanceSize);

	ChipOpen->pProcessContext = wc;

	ChipOpen->pbyImageFile = firmware->data;
	ChipOpen->ulImageSize = firmware->size;

	ChipOpen->fEnableMemClkOut = TRUE;
	ChipOpen->ulMemClkFreq = cOCT6100_MCLK_FREQ_133_MHZ;
	ChipOpen->ulMaxChannels = apec->numchans;
	ChipOpen->ulMemoryType = cOCT6100_MEM_TYPE_DDR;
	ChipOpen->ulMemoryChipSize = cOCT6100_MEMORY_CHIP_SIZE_32MB;
	ChipOpen->ulNumMemoryChips = 1;
	ChipOpen->ulMaxTdmStreams = 4;
	ChipOpen->aulTdmStreamFreqs[0] = cOCT6100_TDM_STREAM_FREQ_8MHZ;
	ChipOpen->ulTdmSampling = cOCT6100_TDM_SAMPLE_AT_FALLING_EDGE;
#if 0
	ChipOpen->fEnableAcousticEcho = TRUE;
#endif

	ulResult = Oct6100GetInstanceSize(ChipOpen, &InstanceSize);
	if (ulResult != cOCT6100_ERR_OK) {
		printk("Failed to get instance size, code %08x!\n", ulResult);
		kfree(apec);
		return NULL;
	}


	apec->pApiInstance = vmalloc(InstanceSize.ulApiInstanceSize);
	if (!apec->pApiInstance) {
		printk("Out of memory (can't allocate %d bytes)!\n", InstanceSize.ulApiInstanceSize);
		kfree(apec);
		kfree(ChipOpen);
		kfree(ChannelOpen);
		return NULL;
	}

	/* I don't know what to curse more in this comment, the problems caused by
	 * the 4K kernel stack limit change or the octasic API for being so darn
	 * stack unfriendly.  Stupid, stupid, stupid.  So we disable IRQs so we
	 * don't run the risk of overflowing the stack while we initialize the
	 * octasic. */
#ifdef CONFIG_4KSTACKS
	local_irq_save(flags);
#endif
	ulResult = Oct6100ChipOpen(apec->pApiInstance, ChipOpen);
	if (ulResult != cOCT6100_ERR_OK) {
		printk("Failed to open chip, code %08x!\n", ulResult);
#ifdef CONFIG_4KSTACKS
		local_irq_restore(flags);
#endif
		kfree(apec);
		kfree(ChipOpen);
		kfree(ChannelOpen);
		return NULL;
	}
	for (x=0; x < 128; x++) {
		/* execute this loop always on 4 span cards but
		*  on 2 span cards only execute for the channels related to our spans */
		if ((x & 0x3) < numspans) {
			/* span timeslots are interleaved 12341234...
		 	*  therefore, the lower 2 bits tell us which span this
			*  timeslot/channel
		 	*/
			if (isalaw[x & 0x03])
				law = cOCT6100_PCM_A_LAW;
			else
				law = cOCT6100_PCM_U_LAW;
			Oct6100ChannelOpenDef(ChannelOpen);
			ChannelOpen->pulChannelHndl = &apec->aulEchoChanHndl[x];
			ChannelOpen->ulUserChanId = x;
			ChannelOpen->TdmConfig.ulRinPcmLaw = law;
			ChannelOpen->TdmConfig.ulRinStream = 0;
			ChannelOpen->TdmConfig.ulRinTimeslot = x;
			ChannelOpen->TdmConfig.ulSinPcmLaw = law;
			ChannelOpen->TdmConfig.ulSinStream = 1;
			ChannelOpen->TdmConfig.ulSinTimeslot = x;
			ChannelOpen->TdmConfig.ulSoutPcmLaw = law;
			ChannelOpen->TdmConfig.ulSoutStream = 2;
			ChannelOpen->TdmConfig.ulSoutTimeslot = x;
			ChannelOpen->TdmConfig.ulRoutPcmLaw = law;
			ChannelOpen->TdmConfig.ulRoutStream = 3;
			ChannelOpen->TdmConfig.ulRoutTimeslot = x;
			ChannelOpen->VqeConfig.fEnableNlp = TRUE;
			ChannelOpen->VqeConfig.fRinDcOffsetRemoval = TRUE;
			ChannelOpen->VqeConfig.fSinDcOffsetRemoval = TRUE;

			ChannelOpen->fEnableToneDisabler = TRUE;
			ChannelOpen->ulEchoOperationMode = cOCT6100_ECHO_OP_MODE_DIGITAL;

			ulResult = Oct6100ChannelOpen(apec->pApiInstance, ChannelOpen);
			if (ulResult != GENERIC_OK) {
				printk("Failed to open channel %d!\n", x);
			}
		}
	}

#ifdef CONFIG_4KSTACKS
	local_irq_restore(flags);
#endif
	kfree(ChipOpen);
	kfree(ChannelOpen);
	return apec;
}

void apec_release(struct apec_s *apec)
{
	UINT32 ulResult;
	tOCT6100_CHIP_CLOSE ChipClose;

	Oct6100ChipCloseDef(&ChipClose);
	ulResult = Oct6100ChipClose(apec->pApiInstance, &ChipClose);
	if (ulResult != cOCT6100_ERR_OK) {
		printk("Failed to close chip, code %08x!\n", ulResult);
	}
	vfree(apec->pApiInstance);
	kfree(apec);
	printk(KERN_INFO "APEC: Releasing...\n");
}
