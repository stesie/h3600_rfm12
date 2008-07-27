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

static int rfm12_baud = 8620;
MODULE_PARM (rfm12_baud, "i");
MODULE_PARM_DESC (rfm12_baud, "transmitter baudrate (default 8620)");

/* Some forward declarations. */
static void chip_handler (int byte);

static void rfm12_net_wake_queue (void);
static void rfm12_net_rx (struct sk_buff *skb);

static struct net_device rfm12_dev;

/* Timestamp (jiffies) when last chip_status-change happened,
   used to calculate chip restart. */
static unsigned long timer_last_status_change;

/* Timestamp (jiffies again) when the last rfm12 status
   query took place. */
static unsigned long timer_last_status_query;


/*
 * RFM12 Chip low-level functions
 */
#define chip_trans_raw(buf,len) h3600_rfm12(buf,len)

static inline int
chip_trans(unsigned short a)
{
	unsigned char buf[2];
	
	buf[0] = ((a) & 0xFF00) >> 8;
	buf[1] = ((a) & 0x00FF);

	return chip_trans_raw (buf, 2);
}


static enum chip_status_t {
	CHIP_REINIT,

	CHIP_IDLE,
	CHIP_RX,
	CHIP_RX_WAIT_LEN_2,
	CHIP_RX_DATA,

	CHIP_TX,
	CHIP_TX_PREAMBLE_1,
	CHIP_TX_PREAMBLE_2,
	CHIP_TX_PREFIX_1,
	CHIP_TX_PREFIX_2,
	CHIP_TX_SIZE_HI,
	CHIP_TX_SIZE_LO,
	CHIP_TX_DATA,
	CHIP_TX_DATAEND,
	CHIP_TX_SUFFIX_1,
	CHIP_TX_SUFFIX_2,
	CHIP_TX_TRIGGER,
	CHIP_TX_WAIT,
} chip_status;

#define update_chip_status(a) (timer_last_status_change = jiffies,	\
			       chip_status = (a))


static struct sk_buff *rx_packet, *tx_packet;
static int packet_len;
static int packet_index;
static int buffer_position;

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
	chip_setfreq (RFM12FREQ (433.92));
	chip_setbandwidth (5, 1, 4);
	chip_setbaud (rfm12_baud);
	chip_setpower (0, 2);
}

#define chip_rxstart() (update_chip_status(CHIP_RX), chip_trans (0xf100))
#define chip_rxstop()  (chip_trans (0x8208))
#define chip_txstart()					\
	do {						\
		update_chip_status(CHIP_TX);		\
		STATS->tx_packets ++;			\
		queue_task (&chip_task, &tq_timer);	\
	} while(0)


static void chip_bh (void *foo);
static struct tq_struct chip_task = {
        routine:        chip_bh,
};


/* Special variants of chip_trans, that tests the return value of `chip_trans',
   unless successful requeues the bh for execution, restores the context
   and immediately returns. */
#define __bh(a)								\
	do {								\
	        int _r = a;						\
		if(_r) {						\
			DEBUG ("%s: __bh fail in line %d.\n",		\
			       __FUNCTION__, __LINE__);			\
			queue_task (&chip_task, &tq_timer);		\
			chip_status = restore_status;			\
			buffer_position = restore_buffer_position;	\
			if (restore_data) {				\
				tx_packet->len = restore_len;		\
				tx_packet->data = restore_data;		\
			}						\
			return;						\
		}							\
	} while(0)

#define chip_trans_bh(a)       __bh(chip_trans(a))
#define chip_trans_raw_bh(a,b) __bh(chip_trans_raw(a,b))
#define chip_rxstart_bh()      __bh(chip_rxstart())
#define chip_rxstop_bh()       __bh(chip_rxstop())


static unsigned char
chip_generate_tx_data (void)
{
	switch (chip_status) {
	case CHIP_TX:
	case CHIP_TX_PREAMBLE_1:
	case CHIP_TX_PREAMBLE_2:
		chip_status ++;
		return 0xAA;

	case CHIP_TX_PREFIX_1:
		chip_status ++;
		return 0x2D;

	case CHIP_TX_PREFIX_2:
		chip_status ++;
		return 0xD4;

	case CHIP_TX_SIZE_HI:
		chip_status ++;
		return (tx_packet->len >> 8) & 0xFF; /* Always zero actually. */

	case CHIP_TX_SIZE_LO:
		chip_status ++;
		return tx_packet->len & 0xFF;

	case CHIP_TX_DATA:
		STATS->tx_bytes ++;
		int r = (tx_packet->data[0] & 0xFF);
		skb_pull (tx_packet, 1);
		if (!tx_packet->len)
			chip_status = CHIP_TX_SUFFIX_1;
		return r;

	case CHIP_TX_SUFFIX_1:
	case CHIP_TX_SUFFIX_2:
		chip_status ++;
		return 0xAA;

	default:
		ERROR ("%s: eeeek, cannot handle status %d.\n",
		       __FUNCTION__, chip_status);
		return 0;
	}
}


static void
chip_bh (void *foo)
{
	(void) foo;

	/* track-back values, in case __bh fails. */
	enum chip_status_t restore_status = chip_status;
	unsigned int restore_len = tx_packet ? tx_packet->len : 0;
	unsigned char *restore_data = tx_packet ? tx_packet->data : NULL;
	unsigned int restore_buffer_position = buffer_position;

	/* This function is executed in softirq context, therefore chip_trans
	   will not be able to sleep until other communication with the AVR
	   has completed.  */

	switch (chip_status) {
	case CHIP_REINIT:
		chip_trans_bh (0x0100); /* request re-init, triggers interrupt */
		update_chip_status (CHIP_IDLE);
		break;

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
		DEBUG ("%s: preparing TX.\n", __FUNCTION__);
		buffer_position = 3;

	case CHIP_TX_PREAMBLE_1:
	case CHIP_TX_PREAMBLE_2:
	case CHIP_TX_PREFIX_1:
	case CHIP_TX_PREFIX_2:
	case CHIP_TX_SIZE_HI:
	case CHIP_TX_SIZE_LO:
	case CHIP_TX_DATA:
	case CHIP_TX_DATAEND:
	case CHIP_TX_SUFFIX_1:
	case CHIP_TX_SUFFIX_2: {
		unsigned char buf[15];
		unsigned short len = 1;

		/* send TX-triggering command only on first shot,
		   so we'll be able to make use of the underrun recognition. */
		buf[0] = 0xF0 | buffer_position;

		while (chip_status < CHIP_TX_TRIGGER && len < 15) {
			buf[len] = chip_generate_tx_data ();
			len ++;
		}

		/* try to commit data, will return if fail */
		chip_trans_raw_bh (buf, len);

		/* fill next buffer, next time around */
		buffer_position ++;

		/* if (chip_status == CHIP_TX_TRIGGER)
		   DEBUG (MODULE_NAME ": TX: buffer fill complete.\n"); */

		timer_last_status_change = jiffies;
		break;
	}

	case CHIP_TX_TRIGGER: 
		DEBUG ("%s: triggering TX.\n", __FUNCTION__);
		chip_trans_bh (0xf200 | (buffer_position - 1));

		update_chip_status(CHIP_TX_WAIT);
		break;

	default:
		ERROR ("%s: unexpected chip_status=%d\n",
		       __FUNCTION__, chip_status);
	}
}


static void
chip_handler (int byte)
{
	if (byte < 0 && (chip_status == CHIP_RX || chip_status == CHIP_RX_DATA)) {
		/* Got notification while in RX mode, the AVR should have
		   reinitialized the RFM12 module meanwhile, therefore we just
		   have to either trigger a TX or restart RX. 
		   This happens quite often (every time the touchpad is pressed. */
		DEBUG ("Gnah! Got notification during RX, reinitializing.\n");

		update_chip_status (CHIP_REINIT);
		queue_task (&chip_task, &tq_timer);
		return;
	}

	switch (chip_status) {
	case CHIP_IDLE:
		queue_task (&chip_task, &tq_timer);
		break;
		
	case CHIP_RX:
		packet_len = byte << 8;
		update_chip_status (CHIP_RX_WAIT_LEN_2);
		break;

	case CHIP_RX_WAIT_LEN_2:
		packet_len |= byte;
		DEBUG ("RX: len=%d\n", packet_len);

		if (packet_len > 0 && packet_len <= 1500)
			rx_packet = alloc_skb (packet_len, GFP_ATOMIC);
		else
			rx_packet = NULL;

		if (rx_packet) {
			packet_index = 0;
			update_chip_status(CHIP_RX_DATA);
		} else {
			update_chip_status(CHIP_IDLE);

			/* run bottom half to initiate restart */
			queue_task (&chip_task, &tq_timer);
		}
		break;

	case CHIP_RX_DATA:
		skb_put (rx_packet, 1)[0] = byte;

		/* make sure not to time out. */
		timer_last_status_change = jiffies;

		if (++ packet_index == packet_len) {
			DEBUG ("RX: complete.\n");

			rfm12_net_rx (rx_packet);
			rx_packet = NULL;

			if (tx_packet)
				chip_txstart ();

			else
				update_chip_status(CHIP_RX);
		}

		break;

	case CHIP_TX:
	case CHIP_TX_PREAMBLE_1:
	case CHIP_TX_PREAMBLE_2:
	case CHIP_TX_PREFIX_1:
	case CHIP_TX_PREFIX_2:
	case CHIP_TX_SIZE_HI:
	case CHIP_TX_SIZE_LO:
	case CHIP_TX_DATA:
	case CHIP_TX_DATAEND:
	case CHIP_TX_SUFFIX_1:
	case CHIP_TX_SUFFIX_2:
	case CHIP_TX_TRIGGER:
		/* Implemented in _bh, use these. */
		// chip_bh (NULL);
		queue_task (&chip_task, &tq_timer);
		break;

	case CHIP_TX_WAIT:
		DEBUG ("%s: TX complete (0x%02x).\n", __FUNCTION__, byte);
		/* RX mode has been re-enabled by the AVR meanwhile. */

		update_chip_status(CHIP_RX);

		dev_kfree_skb_irq (tx_packet);
		tx_packet = NULL;

		rfm12_net_wake_queue ();
		break;


	default:
		ERROR ("%s: unexpected chip_status=%d, byte=0x%02x\n",
		       __FUNCTION__, chip_status, byte);
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
		STATS->tx_dropped ++;
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
	/* Time out transfer after 200ms. */
	if (chip_status != CHIP_RX
	    && timer_last_status_change < jiffies - HZ / 5) {
		ERROR ("transfer timed out, status=%d\n", chip_status);

		if (chip_status >= CHIP_TX) {
			if (tx_packet)
				dev_kfree_skb_irq (tx_packet);
			tx_packet = NULL;

			rfm12_net_wake_queue ();
		}

		update_chip_status(CHIP_REINIT);

		/* leave the work to the bottom half. */
		queue_task (&chip_task, &tq_timer);
	}

	/* Do a status query every 15 seconds. */
	if (chip_status <= CHIP_RX
	    && timer_last_status_query < jiffies - HZ * 15) {
		DEBUG ("%s: status req, chip_status = %d.\n",
		       __FUNCTION__, chip_status);

		chip_trans (0x0000); /* may fail. */
		timer_last_status_query = jiffies;
	}

	/* Update timer to expire in 0.2 seconds again. */
	del_timer (&rfm12_timer);
	rfm12_timer.expires = jiffies + HZ / 5;
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

	if (skb->len > 173) {
		ERROR ("cowardly refusing to send packets longer "
		       "than 173 bytes.\n");
		return 1;
	}

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
	/* DEBUG ("%s.\n", __FUNCTION__); */

	dev->hard_header_len = 0;
	dev->addr_len = 0;
	dev->mtu = 173;

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

	switch ((skb->data[0] & 0xF0) >> 4) {
	case 4:
		skb->protocol = __constant_htons(ETH_P_IP);
		break;
	case 6:
		skb->protocol = __constant_htons(ETH_P_IPV6);
		break;
	default:
		ERROR ("failed to detect protocol: 0x%02x%02x\n",
		       skb->data[0], skb->data[1]);
		dev_kfree_skb_irq (skb);
		return;
	}

	netif_rx (skb);

	rfm12_dev.last_rx = jiffies;
}


static void
rfm12_net_wake_queue (void)
{
	netif_wake_queue (&rfm12_dev);
}


static struct net_device rfm12_dev = { 
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
	DEBUG ("%s called.\n", __FUNCTION__);

	if (rfm12_net_register ())
		return 1;

	if (rfm12_baud < 663) {
		rfm12_baud = 663;
		ERROR ("using minimal baudrate of 663.");
	}

	if (rfm12_baud > 28800) {
		rfm12_baud = 28800;
		ERROR ("using maximal baudrate of 28800.");
	}

	chip_init ();
	chip_rxstart ();

	/* initiate dummy read */
	chip_trans (0x0000);

	h3600_hal_register_driver (&g_driver_ops);

	init_timer (&rfm12_timer);
	rfm12_timer.expires = jiffies + HZ / 5;
	rfm12_timer.function = (void *) rfm12_status_timer;
	add_timer (&rfm12_timer);

	return 0;
}


void
cleanup_module (void)
{
	DEBUG ("%s called.\n", __FUNCTION__);

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
