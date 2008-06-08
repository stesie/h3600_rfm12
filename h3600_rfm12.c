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
#include <linux/netdevice.h>
#include <linux/if_arp.h>

#include <asm/arch-sa1100/h3600_hal.h>
#include "h3600_rfm12.h"


/* Some forward declarations. */
static void chip_handler (unsigned char byte);

static void rfm12_net_wake_queue (void);
static void rfm12_net_rx (struct sk_buff *skb);

struct net_device rfm12_dev;



/*
 * RFM12 Chip low-level functions
 */
#define chip_trans(a)  h3600_rfm12(a)

static enum {
	CHIP_IDLE,
	CHIP_RX,
	CHIP_RX_DATA,
	CHIP_RX_FINISH,

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
} chip_status;

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

/* Special variant of chip_trans, that tests the return value of `chip_trans',
   unless successful requeues the bh for execution and immediately returns. */
#define chip_trans_bh(a)					\
	do {							\
	        int _r = chip_trans(a);				\
		if(_r) {					\
			queue_task (&chip_task, &tq_timer);	\
			return;					\
		}						\
	} while(0)

#define chip_rxstart() (chip_status = CHIP_RX, chip_trans (0xfe00))
#define chip_rxstop()  (chip_trans (0x8208))
#define chip_txstart() (chip_status = CHIP_TX, chip_bh (NULL))


static void chip_bh (void *foo);
static struct tq_struct chip_task = {
        routine:        chip_bh,
};


static void
chip_bh (void *foo)
{
	(void) foo;
	/* This function is executed in softirq context, therefore chip_trans
	   will not be able to sleep until other communication with the AVR
	   has completed.

	   Furthermore it's running in (hard) interrupt context since the
	   interrupt handler directly calls this functions to make immediate
	   use of the TX routines. */

	switch (chip_status) {
	case CHIP_RX_FINISH:
		if (rx_packet) {
			rfm12_net_rx (rx_packet);
			rx_packet = NULL;

			DEBUG ("%s: packet done.\n", __FUNCTION__);
		}

		/* fall through */
	case CHIP_IDLE:

		if (tx_packet)
			/* send queued packet now. */
			chip_txstart ();

		else {
			/* Restart RX. */
			if (chip_rxstart ())
				/* Failed, try again. XXX */
				queue_task (&chip_task, &tq_timer);
		}
		break;

	case CHIP_TX:
		STATS->tx_packets ++;

	case CHIP_TX_PREAMBLE_1:
	case CHIP_TX_PREAMBLE_2:
		chip_trans_bh (0xF1AA);
		chip_status ++;
		break;

	case CHIP_TX_PREFIX_1:
		chip_trans_bh (0xF02D);
		chip_status ++;
		break;

	case CHIP_TX_PREFIX_2:
		chip_trans_bh (0xF0D4);
		chip_status ++;
		break;

	case CHIP_TX_SIZE:
		chip_trans_bh (0xF000 | (tx_packet->len & 0xFF));
		chip_status ++;
		break;

	case CHIP_TX_DATA:
		STATS->tx_bytes ++;
		chip_trans_bh (0xF000 | (tx_packet->data[0] & 0xFF));
		skb_pull (tx_packet, 1);
		if (!tx_packet->len)
			chip_status = CHIP_TX_SUFFIX_1;
		break;

	case CHIP_TX_SUFFIX_1:
	case CHIP_TX_SUFFIX_2:
		chip_trans_bh (0xF0AA);
		chip_status ++;
		break;

	case CHIP_TX_END:
		chip_trans_bh (0x8208); /* TX off */
		DEBUG (MODULE_NAME ": TX: complete.\n");

		dev_kfree_skb_irq (tx_packet);
		tx_packet = NULL;

		rfm12_net_wake_queue ();

		/* Reenable RX next time around. */
		chip_status = CHIP_IDLE;
		queue_task (&chip_task, &tq_timer);
		
		break;

	default:
		ERROR ("%s: unexpected chip_status=%d\n",
		       __FUNCTION__, chip_status);
	}
}


static void
chip_handler (unsigned char byte)
{
	switch (chip_status) {
	case CHIP_RX:
		packet_len = byte;
		DEBUG ("RX: len=%d\n", packet_len);

		if (packet_len)
			rx_packet = alloc_skb (packet_len, GFP_ATOMIC);
		else
			rx_packet = NULL;

		if (rx_packet) {
			packet_index = 0;
			chip_status = CHIP_RX_DATA;
		} else {
			chip_status = CHIP_IDLE;

			/* run bottom half to initiate restart */
			queue_task (&chip_task, &tq_timer);
		}
		break;

	case CHIP_RX_DATA:
		skb_put (rx_packet, 1)[0] = byte;

		if (++ packet_index == packet_len) {
			DEBUG ("RX: complete.\n");

			/* run bottom half */
			chip_status = CHIP_RX_FINISH; 
			queue_task (&chip_task, &tq_timer);
		}

		break;

	case CHIP_RX_FINISH:
		DEBUG ("%s: RX_FINISH set, ignoring.\n", __FUNCTION__);
		break;		/* hmm, packet already complete */

	case CHIP_TX:
	case CHIP_TX_PREAMBLE_1:
	case CHIP_TX_PREAMBLE_2:
	case CHIP_TX_PREFIX_1:
	case CHIP_TX_PREFIX_2:
	case CHIP_TX_SIZE:
	case CHIP_TX_DATA:
	case CHIP_TX_DATAEND:
	case CHIP_TX_SUFFIX_1:
	case CHIP_TX_SUFFIX_2:
	case CHIP_TX_END:
		chip_bh (NULL);	/* Implemented in _bh, use these. */
		break;

	default:
		ERROR ("%s: unexpected chip_status=%d\n",
		       __FUNCTION__, chip_status);
	}
}


static int
chip_queue_tx (struct sk_buff *skb)
{
	int ret = 0;

	if (chip_status <= CHIP_RX) {
		/* Chip is inactive, we can transmit immediately. */
		tx_packet = skb;
		chip_txstart ();
	}

	else if (tx_packet == NULL)
		/* Chip is active (receiving), queue packet. */
		tx_packet = skb;

	else {
		ret = 1;	/* error */
		//STATS->tx_dropped ++;
	}

	return ret;
}



static struct h3600_driver_ops g_driver_ops = {
        rfm12:          chip_handler,
};




/*
 * Status call timer
 */

static struct timer_list rfm12_timer;

static void
rfm12_status_timer (void)
{
	chip_trans (0x0000);

	/* Update timer to expire in 15 seconds. */
	del_timer (&rfm12_timer);
	rfm12_timer.expires = jiffies + 15 * HZ;
	add_timer (&rfm12_timer);
}




/*
 * Network device guts
 */

#ifndef ARPHRD_NONE
/* ARPHRD_NONE not available, let's use void type instead. XXX */
#define ARPHRD_NONE  ARPHRD_VOID
#endif

static int
rfm12_net_open (struct net_device *dev)
{
	netif_start_queue(dev);
        return 0;
}


static int
rfm12_net_close(struct net_device *dev)
{
        netif_stop_queue(dev);
        return 0;
}


static int
rfm12_net_xmit (struct sk_buff *skb, struct net_device *dev)
{
	DEBUG ("TX: len=%d.\n", skb->len);

	netif_stop_queue (dev);

	if (chip_queue_tx (skb)) {
		ERROR ("BUG. TX ring full when queue awake!\n");
		return 1;
	}

	return 0;
}


static struct net_device_stats *
rfm12_net_stats (struct net_device *dev)
{
	return (struct net_device_stats *) dev->priv;
}


static int
rfm12_net_init (struct net_device *dev)
{
	DEBUG ("%s.\n", __FUNCTION__);

	dev->hard_header_len = 0;
	dev->addr_len = 0;
	dev->mtu = 254;

	dev->type = ARPHRD_NONE;
	dev->flags = IFF_NOARP | IFF_MULTICAST | IFF_BROADCAST;
	dev->tx_queue_len = 10;

	dev->priv = kmalloc (sizeof (struct net_device_stats), GFP_KERNEL);
	if (! dev->priv)
		return -ENOMEM;
	memset (dev->priv, 0, sizeof (struct net_device_stats));

	return 0;
}


static int
rfm12_net_register (void)
{
	int result = 0;

	strcpy (rfm12_dev.name, "rfm12");
	if ((result = register_netdev (&rfm12_dev)))
		printk (MODULE_NAME ": register_netdev failed: %d\n", result);

	return result;	
}


static void
rfm12_net_unregister (void)
{
	unregister_netdev (&rfm12_dev);
}


static void
rfm12_net_rx (struct sk_buff *skb)
{
	skb->dev = &rfm12_dev;
	skb->mac.raw = skb->data;
	skb->protocol = __constant_htons(ETH_P_IP);

	netif_rx (skb);

	rfm12_dev.last_rx = jiffies;
}


static void
rfm12_net_wake_queue (void)
{
	netif_wake_queue (&rfm12_dev);
}


struct net_device rfm12_dev = { 
        init:            rfm12_net_init,
	open:            rfm12_net_open,
	stop:            rfm12_net_close,
	hard_start_xmit: rfm12_net_xmit,
	get_stats:       rfm12_net_stats,
};



/*
 * Module related functions, glueing everything together.
 */
int
init_module (void)
{
	printk (KERN_INFO MODULE_NAME ": init_module called.\n");

	if (rfm12_net_register ())
		return 1;

	chip_init ();
	chip_rxstart ();

	/* initiate dummy read */
	chip_trans (0x0000);

	h3600_hal_register_driver (&g_driver_ops);

	init_timer (&rfm12_timer);
	rfm12_timer.expires = jiffies + HZ; /* timeout after one second. */
	rfm12_timer.function = (void *) rfm12_status_timer;
	add_timer (&rfm12_timer);

	return 0;
}


void
cleanup_module (void)
{
	printk (KERN_INFO MODULE_NAME ": cleanup_module called.\n");

	/* Remove the status timer. */
	del_timer (&rfm12_timer);

	/* Remove the net device. */
	rfm12_net_unregister ();

	/* Disable interrupt. */
	h3600_hal_unregister_driver (&g_driver_ops);

	/* Completely shutdown the rfm12 module */
	chip_trans (0x8200);
}


MODULE_DESCRIPTION ("RFM12 FSK-transmitter driver for iPAQ h3600");
MODULE_AUTHOR ("Stefan Siegl");
MODULE_LICENSE ("GPL");
