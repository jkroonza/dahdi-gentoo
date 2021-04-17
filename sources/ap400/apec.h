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

#ifndef _APEC_H_
#define _APEC_H_

#include <linux/firmware.h>

struct apec_s;

/* From AP400 */
unsigned int oct_read(void *card, unsigned int addr);
void oct_write(void *card, unsigned int addr, unsigned int data);

/* From APEC */
struct apec_s *apec_init(void *wc, int *isalaw, int numspans, const struct firmware *firmware);
unsigned int apec_capacity_get(void *wc);
void apec_setec(struct apec_s *instance, int channel, int eclen);
int apec_checkirq(struct apec_s *apec);
void apec_release(struct apec_s *instance);

#endif /*_APEC_H_*/
