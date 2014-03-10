/*
 * forcedeth: Ethernet driver for NVIDIA nForce media access controllers.
 *
 * Note: This driver is a cleanroom reimplementation based on reverse
 *      engineered documentation written by Carl-Daniel Hailfinger
 *      and Andrew de Quincey.
 *
 * NVIDIA, nForce and other NVIDIA marks are trademarks or registered
 * trademarks of NVIDIA Corporation in the United States and other
 * countries.
 *
 * Copyright (C) 2003,4,5 Manfred Spraul
 * Copyright (C) 2004 Andrew de Quincey (wol support)
 * Copyright (C) 2004 Carl-Daniel Hailfinger (invalid MAC handling, insane
											  *		IRQ rate fixes, bigendian fixes, cleanups, verification)
 * Copyright (c) 2004,2005,2006,2007,2008,2009 NVIDIA Corporation
 * Copyright (c) 2008-2009 Nathan Hjelm (MacOS X support)
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Known bugs:
 * We suspect that on some hardware no TX done interrupts are generated.
 * This means recovery from netif_stop_queue only happens if the hw timer
 * interrupt fires (100 times/second, configurable with NVREG_POLL_DEFAULT)
 * and the timer is active in the IRQMask, or if a rx packet arrives by chance.
 * If your hardware reliably generates tx done interrupts, then you can remove
 * DEV_NEED_TIMERIRQ from the driver_data flags.
 * DEV_NEED_TIMERIRQ will not harm you on sane hardware, only generating a few
 * superfluous timer interrupts from the nic.
 */

#if !defined(DEFINES_H)
#define DEFINES_H

/* device PCI ids */
#define PCI_VENDOR_ID_NVIDIA                    0x10de

#define PCI_DEVICE_ID_NVIDIA_NVENET_1           0x01c3
#define PCI_DEVICE_ID_NVIDIA_NVENET_2           0x0066
#define PCI_DEVICE_ID_NVIDIA_NVENET_3           0x00d6
#define PCI_DEVICE_ID_NVIDIA_NVENET_4           0x0086
#define PCI_DEVICE_ID_NVIDIA_NVENET_5           0x008c
#define PCI_DEVICE_ID_NVIDIA_NVENET_6           0x00e6
#define PCI_DEVICE_ID_NVIDIA_NVENET_7           0x00df
#define PCI_DEVICE_ID_NVIDIA_NVENET_8           0x0056
#define PCI_DEVICE_ID_NVIDIA_NVENET_9           0x0057
#define PCI_DEVICE_ID_NVIDIA_NVENET_10          0x0037
#define PCI_DEVICE_ID_NVIDIA_NVENET_11          0x0038
#define PCI_DEVICE_ID_NVIDIA_NVENET_12          0x0268
#define PCI_DEVICE_ID_NVIDIA_NVENET_13          0x0269
#define PCI_DEVICE_ID_NVIDIA_NVENET_14          0x0372
#define PCI_DEVICE_ID_NVIDIA_NVENET_15          0x0373
#define PCI_DEVICE_ID_NVIDIA_NVENET_16          0x03E5
#define PCI_DEVICE_ID_NVIDIA_NVENET_17          0x03E6
#define PCI_DEVICE_ID_NVIDIA_NVENET_18          0x03EE
#define PCI_DEVICE_ID_NVIDIA_NVENET_19          0x03EF
#define PCI_DEVICE_ID_NVIDIA_NVENET_20          0x0450
#define PCI_DEVICE_ID_NVIDIA_NVENET_21          0x0451
#define PCI_DEVICE_ID_NVIDIA_NVENET_22          0x0452
#define PCI_DEVICE_ID_NVIDIA_NVENET_23          0x0453
#define PCI_DEVICE_ID_NVIDIA_NVENET_24          0x054C
#define PCI_DEVICE_ID_NVIDIA_NVENET_25          0x054D
#define PCI_DEVICE_ID_NVIDIA_NVENET_26          0x054E
#define PCI_DEVICE_ID_NVIDIA_NVENET_27          0x054F
#define PCI_DEVICE_ID_NVIDIA_NVENET_28          0x07DC
#define PCI_DEVICE_ID_NVIDIA_NVENET_29          0x07DD
#define PCI_DEVICE_ID_NVIDIA_NVENET_30          0x07DE
#define PCI_DEVICE_ID_NVIDIA_NVENET_31          0x07DF
#define PCI_DEVICE_ID_NVIDIA_NVENET_32          0x0760
#define PCI_DEVICE_ID_NVIDIA_NVENET_33          0x0761
#define PCI_DEVICE_ID_NVIDIA_NVENET_34          0x0762
#define PCI_DEVICE_ID_NVIDIA_NVENET_35          0x0763
#define PCI_DEVICE_ID_NVIDIA_NVENET_36          0x0AB0
#define PCI_DEVICE_ID_NVIDIA_NVENET_37          0x0AB1
#define PCI_DEVICE_ID_NVIDIA_NVENET_38          0x0AB2
#define PCI_DEVICE_ID_NVIDIA_NVENET_39          0x0AB3
#define PCI_DEVICE_ID_NVIDIA_NVENET_40          0x0D7D

#define TX_WORK_PER_LOOP  64
#define RX_WORK_PER_LOOP  64

/* device flags */
#define DEV_NEED_TIMERIRQ          0x0000001  /* set the timer irq flag in the irq mask */
#define DEV_NEED_LINKTIMER         0x0000002  /* poll link settings. Relies on the timer irq */
#define DEV_HAS_LARGEDESC          0x0000004  /* device supports jumbo frames and needs packet format 2 */
#define DEV_HAS_HIGH_DMA           0x0000008  /* device supports 64bit dma */
#define DEV_HAS_CHECKSUM           0x0000010  /* device supports tx and rx checksum offloads */
#define DEV_HAS_VLAN               0x0000020  /* device supports vlan tagging and striping */
#define DEV_HAS_MSI                0x0000040  /* device supports MSI */
#define DEV_HAS_MSI_X              0x0000080  /* device supports MSI-X */
#define DEV_HAS_POWER_CNTRL        0x0000100  /* device supports power savings */
#define DEV_HAS_STATISTICS_V1      0x0000200  /* device supports hw statistics version 1 */
#define DEV_HAS_STATISTICS_V2      0x0000600  /* device supports hw statistics version 2 */
#define DEV_HAS_STATISTICS_V3      0x0000e00  /* device supports hw statistics version 3 */
#define DEV_HAS_TEST_EXTENDED      0x0001000  /* device supports extended diagnostic test */
#define DEV_HAS_MGMT_UNIT          0x0002000  /* device supports management unit */
#define DEV_HAS_CORRECT_MACADDR    0x0004000  /* device supports correct mac address order */
#define DEV_HAS_COLLISION_FIX      0x0008000  /* device supports tx collision fix */
#define DEV_HAS_PAUSEFRAME_TX_V1   0x0010000  /* device supports tx pause frames version 1 */
#define DEV_HAS_PAUSEFRAME_TX_V2   0x0020000  /* device supports tx pause frames version 2 */
#define DEV_HAS_PAUSEFRAME_TX_V3   0x0040000  /* device supports tx pause frames version 3 */
#define DEV_NEED_TX_LIMIT          0x0080000  /* device needs to limit tx */
#define DEV_NEED_TX_LIMIT2         0x0180000  /* device needs to limit tx, expect for some revs */
#define DEV_HAS_GEAR_MODE          0x0200000  /* device supports gear mode */
#define DEV_NEED_PHY_INIT_FIX      0x0400000  /* device needs specific phy workaround */
#define DEV_NEED_LOW_POWER_FIX     0x0800000  /* device needs special power up workaround */
#define DEV_NEED_MSI_FIX           0x1000000  /* device needs msi workaround */

#define FLAG_MASK_V1 0xffff0000
#define FLAG_MASK_V2 0xffffc000
#define LEN_MASK_V1 (0xffffffff ^ FLAG_MASK_V1)
#define LEN_MASK_V2 (0xffffffff ^ FLAG_MASK_V2)

#define NV_TX_LASTPACKET	(1<<16)
#define NV_TX_RETRYERROR	(1<<19)
#define NV_TX_RETRYCOUNT_MASK	(0xF<<20)
#define NV_TX_FORCED_INTERRUPT	(1<<24)
#define NV_TX_DEFERRED		(1<<26)
#define NV_TX_CARRIERLOST	(1<<27)
#define NV_TX_LATECOLLISION	(1<<28)
#define NV_TX_UNDERFLOW		(1<<29)
#define NV_TX_ERROR		(1<<30)
#define NV_TX_VALID		(1<<31)

#define NV_TX2_LASTPACKET	(1<<29)
#define NV_TX2_RETRYERROR	(1<<18)
#define NV_TX2_RETRYCOUNT_MASK	(0xF<<19)
#define NV_TX2_FORCED_INTERRUPT	(1<<30)
#define NV_TX2_DEFERRED		(1<<25)
#define NV_TX2_CARRIERLOST	(1<<26)
#define NV_TX2_LATECOLLISION	(1<<27)
#define NV_TX2_UNDERFLOW	(1<<28)
/* error and valid are the same for both */
#define NV_TX2_ERROR		(1<<30)
#define NV_TX2_VALID		(1<<31)
#define NV_TX2_TSO		(1<<28)
#define NV_TX2_TSO_SHIFT	14
#define NV_TX2_TSO_MAX_SHIFT	14
#define NV_TX2_TSO_MAX_SIZE	(1<<NV_TX2_TSO_MAX_SHIFT)
#define NV_TX2_CHECKSUM_L3	(1<<27)
#define NV_TX2_CHECKSUM_L4	(1<<26)

#define NV_TX3_VLAN_TAG_PRESENT (1<<18)

#define NV_RX_DESCRIPTORVALID	(1<<16)
#define NV_RX_MISSEDFRAME	(1<<17)
#define NV_RX_SUBTRACT1		(1<<18)
#define NV_RX_ERROR1		(1<<23)
#define NV_RX_ERROR2		(1<<24)
#define NV_RX_ERROR3		(1<<25)
#define NV_RX_ERROR4		(1<<26)
#define NV_RX_CRCERR		(1<<27)
#define NV_RX_OVERFLOW		(1<<28)
#define NV_RX_FRAMINGERR	(1<<29)
#define NV_RX_ERROR		(1<<30)
#define NV_RX_ERROR_MASK        (NV_RX_ERROR1|NV_RX_ERROR2|NV_RX_ERROR3|NV_RX_ERROR4|NV_RX_CRCERR|NV_RX_OVERFLOW|NV_RX_FRAMINGERR)
#define NV_RX_AVAIL		(1<<31)

#define NV_RX2_CHECKSUMMASK	(0x1C000000)
#define NV_RX2_CHECKSUM_IP	(0x10000000)
#define NV_RX2_CHECKSUM_IP_TCP	(0x14000000)
#define NV_RX2_CHECKSUM_IP_UDP	(0x18000000)
#define NV_RX2_DESCRIPTORVALID	(1<<29)
#define NV_RX2_SUBTRACT1	(1<<25)
#define NV_RX2_ERROR1		(1<<18)
#define NV_RX2_ERROR2		(1<<19)
#define NV_RX2_ERROR3		(1<<20)
#define NV_RX2_ERROR4		(1<<21)
#define NV_RX2_CRCERR		(1<<22)
#define NV_RX2_OVERFLOW		(1<<23)
#define NV_RX2_FRAMINGERR	(1<<24)
/* error and avail are the same for both */
#define NV_RX2_ERROR		(1<<30)
#define NV_RX2_ERROR_MASK       (NV_RX2_ERROR1|NV_RX2_ERROR2|NV_RX2_ERROR3|NV_RX2_ERROR4|NV_RX2_CRCERR|NV_RX2_OVERFLOW|NV_RX2_FRAMINGERR)
#define NV_RX2_AVAIL		(1<<31)

#define NV_RX3_VLAN_TAG_PRESENT (1<<16)
#define NV_RX3_VLAN_TAG_MASK	(0x0000FFFF)

/* Miscelaneous hardware related defines: */
#define NV_PCI_REGSZ_VER1      	0x270
#define NV_PCI_REGSZ_VER2      	0x2d4
#define NV_PCI_REGSZ_VER3      	0x604
#define NV_PCI_REGSZ_MAX        0x604

/* various timeout delays: all in usec */
#define NV_TXRX_RESET_DELAY	4
#define NV_TXSTOP_DELAY1	10
#define NV_TXSTOP_DELAY1MAX	500000
#define NV_TXSTOP_DELAY2	100
#define NV_RXSTOP_DELAY1	10
#define NV_RXSTOP_DELAY1MAX	500000
#define NV_RXSTOP_DELAY2	100
#define NV_SETUP5_DELAY		5
#define NV_SETUP5_DELAYMAX	50000
#define NV_POWERUP_DELAY	5
#define NV_POWERUP_DELAYMAX	5000
#define NV_MIIBUSY_DELAY	50
#define NV_MIIPHY_DELAY	10
#define NV_MIIPHY_DELAYMAX	10000
#define NV_MAC_RESET_DELAY	64

#define NV_WAKEUPPATTERNS	5
#define NV_WAKEUPMASKENTRIES	4

/* General driver defaults */
#define NV_WATCHDOG_TIMEO	(5*HZ)

#define RX_RING_DEFAULT		128
#define TX_RING_DEFAULT		256
#define RX_RING_MIN		128
#define TX_RING_MIN		64
#define RING_MAX_DESC_VER_1	1024
#define RING_MAX_DESC_VER_2_3	16384

/* 
* If your nic mysteriously hangs then try to reduce the limits
 * to 1/0: It might be required to set NV_TX_LASTPACKET in the
 * last valid ring entry. But this would be impossible to
 * implement - probably a disassembly error.
 */
#define TX_LIMIT_STOP   255
#define TX_LIMIT_START  254

/* rx/tx mac addr + type + vlan + align + slack*/
#define NV_RX_HEADERS		(64)
/* even more slack. */
#define NV_RX_ALLOC_PAD		(64)

/* maximum mtu size */
#define NV_PKTLIMIT_1	1500	/* hard limit not known */
#define NV_PKTLIMIT_2	9100	/* Actual limit according to NVidia: 9202 */

#define HZ             1000        /* HZ is the number of milliseconds in a second */
#define OOM_REFILL	   (1+HZ/20)   /* wait about 1/20th a second before trying to allocate memory again */
#define POLL_WAIT	     (1+HZ/100)  /* no polling needed on darwin (interupt handlers do not run in interrupt context) */
#define LINK_TIMEOUT	 (3*HZ)      /* check the link status every 3 seconds */
#define STATS_INTERVAL (10*HZ)     /* update stats every 10 seconds */

/* PHY defines */
#define PHY_OUI_MARVELL 0x5043
#define PHY_OUI_CICADA  0x03f1
#define PHYID1_OUI_MASK 0x03ff
#define PHYID1_OUI_SHFT 6
#define PHYID2_OUI_MASK 0xfc00
#define PHYID2_OUI_SHFT 10
#define PHY_INIT1       0x0f000
#define PHY_INIT2       0x0e00
#define PHY_INIT3       0x01000
#define PHY_INIT4       0x0200
#define PHY_INIT5       0x0004
#define PHY_INIT6       0x02000
#define PHY_GIGABIT     0x0100

#define PHY_TIMEOUT     0x1
#define PHY_ERROR       0x2

#define PHY_100 0x1
#define PHY_1000        0x2
#define PHY_HALF        0x100

/*
 * desc_ver values:
 * The nic supports three different descriptor types:
 * - DESC_VER_1: Original
 * - DESC_VER_2: support for jumbo frames.
 * - DESC_VER_3: 40-bit addressing.
 */
enum {
  DESC_VER_1 = 1,
  DESC_VER_2 = 2,
  DESC_VER_3 = 3
};

/* PHY defines */
#define PHY_OUI_MARVELL		0x5043
#define PHY_OUI_CICADA		0x03f1
#define PHY_OUI_VITESSE		0x01c1
#define PHY_OUI_REALTEK		0x0732
#define PHY_OUI_REALTEK2	0x0020
#define PHYID1_OUI_MASK	0x03ff
#define PHYID1_OUI_SHFT	6
#define PHYID2_OUI_MASK	0xfc00
#define PHYID2_OUI_SHFT	10
#define PHYID2_MODEL_MASK		0x03f0
#define PHY_MODEL_REALTEK_8211		0x0110
#define PHY_REV_MASK			0x0001
#define PHY_REV_REALTEK_8211B		0x0000
#define PHY_REV_REALTEK_8211C		0x0001
#define PHY_MODEL_REALTEK_8201		0x0200
#define PHY_MODEL_MARVELL_E3016		0x0220
#define PHY_MODEL_MARVELL_E1011		0xb0
#define PHY_MARVELL_E3016_INITMASK	0x0300
#define PHY_CICADA_INIT1	0x0f000
#define PHY_CICADA_INIT2	0x0e00
#define PHY_CICADA_INIT3	0x01000
#define PHY_CICADA_INIT4	0x0200
#define PHY_CICADA_INIT5	0x0004
#define PHY_CICADA_INIT6	0x02000
#define PHY_VITESSE_INIT_REG1	0x1f
#define PHY_VITESSE_INIT_REG2	0x10
#define PHY_VITESSE_INIT_REG3	0x11
#define PHY_VITESSE_INIT_REG4	0x12
#define PHY_VITESSE_INIT_MSK1	0xc
#define PHY_VITESSE_INIT_MSK2	0x0180
#define PHY_VITESSE_INIT1	0x52b5
#define PHY_VITESSE_INIT2	0xaf8a
#define PHY_VITESSE_INIT3	0x8
#define PHY_VITESSE_INIT4	0x8f8a
#define PHY_VITESSE_INIT5	0xaf86
#define PHY_VITESSE_INIT6	0x8f86
#define PHY_VITESSE_INIT7	0xaf82
#define PHY_VITESSE_INIT8	0x0100
#define PHY_VITESSE_INIT9	0x8f82
#define PHY_VITESSE_INIT10	0x0
#define PHY_REALTEK_INIT_REG1	0x1f
#define PHY_REALTEK_INIT_REG2	0x19
#define PHY_REALTEK_INIT_REG3	0x13
#define PHY_REALTEK_INIT_REG4	0x14
#define PHY_REALTEK_INIT_REG5	0x18
#define PHY_REALTEK_INIT_REG6	0x11
#define PHY_REALTEK_INIT_REG7   0x01
#define PHY_REALTEK_INIT1	0x0000
#define PHY_REALTEK_INIT2	0x8e00
#define PHY_REALTEK_INIT3	0x0001
#define PHY_REALTEK_INIT4	0xad17
#define PHY_REALTEK_INIT5	0xfb54
#define PHY_REALTEK_INIT6	0xf5c7
#define PHY_REALTEK_INIT7	0x1000
#define PHY_REALTEK_INIT8	0x0003
#define PHY_REALTEK_INIT9       0x0008
#define PHY_REALTEK_INIT10      0x0005
#define PHY_REALTEK_INIT11      0x0200
#define PHY_REALTEK_INIT_MSK1	0x0003

#define PHY_GIGABIT	0x0100

#define PHY_TIMEOUT	0x1
#define PHY_ERROR	0x2

#define PHY_100	0x1
#define PHY_1000	0x2
#define PHY_HALF	0x100

#define NV_PAUSEFRAME_RX_CAPABLE 0x0001
#define NV_PAUSEFRAME_TX_CAPABLE 0x0002
#define NV_PAUSEFRAME_RX_ENABLE  0x0004
#define NV_PAUSEFRAME_TX_ENABLE  0x0008
#define NV_PAUSEFRAME_RX_REQ     0x0010
#define NV_PAUSEFRAME_TX_REQ     0x0020
#define NV_PAUSEFRAME_AUTONEG    0x0040

/* MSI/MSI-X defines */
#define NV_MSI_X_MAX_VECTORS  16
#define NV_MSI_X_VECTORS_MASK 0x000f
#define NV_MSI_CAPABLE        0x0010
#define NV_MSI_X_CAPABLE      0x0020
#define NV_MSI_ENABLED        0x0040
#define NV_MSI_X_ENABLED      0x0080

#define NV_RESTART_TX         0x1
#define NV_RESTART_RX         0x2

#define NV_TX_LIMIT_COUNT     16

#define NV_DYNAMIC_THRESHOLD        4
#define NV_DYNAMIC_MAX_QUIET_COUNT  2048

#define MII_SREVISION	0x16
#define MII_RESV1		0x17
#define MII_NCONFIG		0x1c
#define MII_BMCR        0x00
#define MII_BMSR		0x01
#define MII_PHYSID1		0x02
#define MII_PHYSID2		0x03
#define MII_ADVERTISE	0x04
#define BMCR_ANRESTART	0x0200
#define BMCR_ANENABLE	0x1000
#define BMCR_RESET		0x8000
#define BMCR_PDOWN      0x0800

// NEED THIS
#define BMSR_LSTATUS	0x0004
#define BMSR_ANEGCAPABLE   0x0008
#define BMSR_ANEGCOMPLETE   0x0020
#define BMSR_100FULL2	0x0200  /* Can do 100BASE-T2 HDX */
#define BMSR_100HALF2	0x0400  /* Can do 100BASE-T2 FDX */
#define BMSR_10HALF		0x0800  /* Can do 10mbps, half-duplex  */
#define BMSR_10FULL		0x1000  /* Can do 10mbps, full-duplex  */
#define BMSR_100HALF	0x2000  /* Can do 100mbps, half-duplex */
#define BMSR_100FULL	0x4000  /* Can do 100mbps, full-duplex */
#define BMSR_100BASE4	0x8000  /* Can do 100mbps, 4k packets  */
#define LPA_100HALF		0x0080
#define LPA_100FULL		0x0100
#define	LPA_10HALF		0x0020
#define LPA_10FULL		0x0040
#define MII_LPA			0x05

#define BMCR_SPEED100      0x2000  /* Select 100Mbps              */
#define BMCR_SPEED1000	   0x0040	/* MSB of Speed (1000)         */
#define BMCR_FULLDPLX      0x0100  /* Full duplex                 */

#define MII_1000BT_CR   0x09
#define MII_1000BT_SR   0x0a
#define ADVERTISE_10HALF	  0x0020
#define ADVERTISE_10FULL	  0x0040
#define	ADVERTISE_100HALF	  0x0080
#define ADVERTISE_100FULL	  0x0100
#define ADVERTISE_1000FULL	0x0200
#define ADVERTISE_100BASE4  0x0200
#define ADVERTISE_1000HALF	0x0100
#define LPA_1000FULL    0x0800
#define LPA_1000HALF    0x0400

#define MII_READ              (-1)

#define EXPANSION_NWAY 0x0001
#define MII_EXPANSION  0x06
#define MII_CTRL1000   0x09
#define MII_STAT1000   0x0a

#define ADVERTISE_PAUSE_CAP   0x0400
#define ADVERTISE_PAUSE_ASYM  0x0800
#define LPA_PAUSE_CAP         0x0400  /* Can pause                   */
#define LPA_PAUSE_ASYM        0x0800  /* Can pause asymetrically     */

#define Mv_LED_Control 16
#define Mv_Page_Address 22
#define Mv_LED_FORCE_OFF 0x88
#define Mv_LED_DUAL_MODE3 0x40

/* MSI */
#define NV_MSI_VECTOR_ALL   0x0
enum {
  NV_MSI_VECTOR_RX    = 0,
  NV_MSI_VECTOR_TX    = 1,
  NV_MSI_VECTOR_OTHER = 2
};

enum {
  NV_SETUP_RX_RING = 1,
  NV_SETUP_TX_RING = 2
};

#define NV_MSI_PRIV_OFFSET 0x68
#define NV_MSI_PRIV_VALUE  0xffffffff

/* Gear Backoff Seeds */
#define BACKOFF_SEEDSET_ROWS	8
#define BACKOFF_SEEDSET_LFSRS	15

#endif
