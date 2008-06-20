/*
 * rfm12.h - GPIO interface driver for RFM12 low-cost FSK transmitter
 *
 * Copyright (C) 2008 Stefan Siegl <stesie@brokenpipe.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#ifndef RFM12_H
#define RFM12_H

#include <linux/skbuff.h>

#define MODULE_NAME "h3600_rfm12"

/* Replace DEBUG by printk(a) to enable debug output. */
#if 0
#define DEBUG(a...)   printk(KERN_INFO MODULE_NAME ": " a)
#else
#define DEBUG(a...)   do { } while(0)
#endif
#define ERROR(a...)   printk(KERN_ERR  MODULE_NAME ": " a)

#define STATS ((struct net_device_stats *) rfm12_dev.priv)

#define RxBW400		1
#define RxBW340		2
#define RxBW270		3
#define RxBW200		4
#define RxBW134		5
#define RxBW67		6

#define TxBW15		0
#define TxBW30		1
#define TxBW45		2
#define TxBW60		3
#define TxBW75		4
#define TxBW90		5
#define TxBW105		6
#define TxBW120		7

#define LNA_0		0
#define LNA_6		1
#define LNA_14		2
#define LNA_20		3

#define RSSI_103	0
#define RSSI_97		1
#define RSSI_91		2
#define RSSI_85		3
#define RSSI_79		4
#define RSSI_73		5
#define RSSI_67		6
#define	RSSI_61		7

#define PWRdB_0		0
#define PWRdB_3		1
#define PWRdB_6		2
#define PWRdB_9		3
#define PWRdB_12	4
#define PWRdB_15	5
#define PWRdB_18	6
#define PWRdB_21	7

#define RFM12TxBDW(kfrq)	((u8) (kfrq / 15) - 1)
#define RFM12FREQ(freq)	        ((freq - 430.0) / 0.0025)	



#endif	/* RFM12_H */
