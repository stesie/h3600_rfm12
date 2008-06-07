/*
 * h3600_rfm12.c - RFM12 FSK-transmitter kernel driver for iPAQ h3600
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

#include <linux/module.h>
#include <linux/kmod.h>
#include <linux/kernel.h>

#include <asm/arch-sa1100/h3600_hal.h>
#include "h3600_rfm12.h"

/*
 * RFM12 Chip low-level functions
 */
#define chip_trans(a)  h3600_rfm12(a)

static enum {
	CHIP_IDLE,
	CHIP_RX,
	CHIP_RX_DATA,

	CHIP_TX,
	CHIP_TX_PREAMBLE_1,
	CHIP_TX_PREAMBLE_2,
	CHIP_TX_PREFIX_1,
	CHIP_TX_PREFIX_2,
	CHIP_TX_SIZE,
	CHIP_TX_DATA,
	CHIP_TX_DATAEND,
	CHIP_TX_SUFFIX_1,
	CHIP_TX_SUFFIX_2,
	CHIP_TX_END,
} chip_status;			/* Locked by chip_lock. */

static struct sk_buff *rx_packet, *tx_packet;
static int packet_len;
static int packet_index;


static void
chip_setbandwidth(u8 bandwidth, u8 gain, u8 drssi)
{
	chip_trans (0x9400 | ((bandwidth & 7) << 5)
		    | ((gain & 3) << 3) | (drssi & 7));
}


static void
chip_setfreq(u16 freq)
{	
	if (freq < 96)		/* 430,2400MHz */
		freq = 96;

	else if (freq > 3903)	/* 439,7575MHz */
		freq = 3903;

	chip_trans (0xA000 | freq);
}


static void
chip_setbaud(u16 baud)
{
	if (baud < 663)
		return;

	/* Baudrate = 344827,58621 / (R + 1) / (1 + CS * 7) */
	if (baud < 5400)
		chip_trans (0xC680 | ((43104 / baud) - 1));
	else
		chip_trans (0xC600 | ((344828UL / baud) - 1));
}


static void
chip_setpower(u8 power, u8 mod)
{	
	chip_trans (0x9800 | (power & 7) | ((mod & 15) << 4));
}


static void
chip_init (void)
{
	chip_trans (0xC0E0);	/* AVR CLK: 10MHz */
	chip_trans (0x80D7);	/* Enable FIFO */
	chip_trans (0xC2AB);	/* Data Filter: internal */
	chip_trans (0xCA81);	/* Set FIFO mode */
	chip_trans (0xE000);	/* disable wakeuptimer */
	chip_trans (0xC800);	/* disable low duty cycle */
	chip_trans (0xC4F7);	/* autotuning: -10kHz...+7,5kHz */
	chip_trans (0x0000);

	chip_setfreq (RFM12FREQ (433.92));
	chip_setbandwidth (5, 1, 4);
	chip_setbaud (8620);
	chip_setpower (0, 2);
}


static void
chip_rxstart (void)
{
	chip_trans (0x82C8);	/* RX on */
	chip_trans (0xCA81);	/* set FIFO mode */
	chip_trans (0xCA83);	/* enable FIFO */

	chip_status = CHIP_RX;
}


/*
 * Module related functions, glueing everything together.
 */
int
init_module (void)
{
	printk (KERN_INFO MODULE_NAME ": init_module called.\n");

	chip_init ();
	chip_rxstart ();

	/* initiate dummy read */
	h3600_rfm12 (0x0000);

	return 0;
}


void
cleanup_module (void)
{
	printk (KERN_INFO MODULE_NAME ": cleanup_module called.\n");
}


MODULE_DESCRIPTION ("RFM12 FSK-transmitter driver for iPAQ h3600");
MODULE_AUTHOR ("Stefan Siegl");
MODULE_LICENSE ("GPL");
