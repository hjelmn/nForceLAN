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
 *		                IRQ rate fixes, bigendian fixes, cleanups, verification)
 * Copyright (c) 2004,2005,2006,2007,2008,2009 NVIDIA Corporation
 * Copyright (C) 2006 Yiduo Wang (original Darwin port)
 * Copyright (c) 2008-2009 Nathan Hjelm (Darwin port)
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

#if !defined(FORCEDETH_HW_H)
#define FORCEDETH_HW_H

/*
 * Hardware access:
 */

enum {
	NvRegIrqStatus = 0x000,
#define NVREG_IRQSTAT_MIIEVENT    0x0040
#define NVREG_IRQSTAT_MASK        0x83ff
	NvRegIrqMask = 0x004,
#define NVREG_IRQ_RX_ERROR        0x0001
#define NVREG_IRQ_RX              0x0002
#define NVREG_IRQ_RX_NOBUF        0x0004
#define NVREG_IRQ_TX_ERR          0x0008
#define NVREG_IRQ_TX_OK           0x0010
#define NVREG_IRQ_TIMER           0x0020
#define NVREG_IRQ_LINK            0x0040
#define NVREG_IRQ_RX_FORCED       0x0080
#define NVREG_IRQ_TX_FORCED       0x0100
#define NVREG_IRQ_RECOVER_ERROR   0x8200
#define NVREG_IRQMASK_THROUGHPUT  0x00df
#define NVREG_IRQMASK_CPU         0x0060
#define NVREG_IRQ_TX_ALL          (NVREG_IRQ_TX_ERR|NVREG_IRQ_TX_OK|NVREG_IRQ_TX_FORCED)
#define NVREG_IRQ_RX_ALL          (NVREG_IRQ_RX_ERROR|NVREG_IRQ_RX|NVREG_IRQ_RX_NOBUF|NVREG_IRQ_RX_FORCED)
#define NVREG_IRQ_OTHER           (NVREG_IRQ_TIMER|NVREG_IRQ_LINK|NVREG_IRQ_RECOVER_ERROR)
	NvRegUnknownSetupReg6 = 0x008,
#define NVREG_UNKSETUP6_VAL		3

/*
 * NVREG_POLL_DEFAULT is the interval length of the timer source on the nic
 * NVREG_POLL_DEFAULT=97 would result in an interval length of 1 ms
 */
	NvRegPollingInterval = 0x00c,
#define NVREG_POLL_DEFAULT_THROUGHPUT 65535 /* backup tx cleanup if loop max reached */
#define NVREG_POLL_DEFAULT_CPU	         13
	NvRegMSIMap0 = 0x020,
	NvRegMSIMap1 = 0x024,

	NvRegMSIIrqMask = 0x030,
/* older devices can use only 8 vectors */
#define NVREG_MSI_VECTOR_0_ENABLED  0x0001
#define NVREG_MSI_VECTOR_1_ENABLED  0x0002
#define NVREG_MSI_VECTOR_2_ENABLED  0x0004
#define NVREG_MSI_VECTOR_3_ENABLED  0x0008
#define NVREG_MSI_VECTOR_4_ENABLED  0x0010
#define NVREG_MSI_VECTOR_5_ENABLED  0x0020
#define NVREG_MSI_VECTOR_6_ENABLED  0x0040
#define NVREG_MSI_VECTOR_7_ENABLED  0x0080
#define NVREG_MSI_VECTOR_8_ENABLED  0x0100

/* MCP79 can use any of 16 MSI vectors */
#define NVREG_MSI_VECTOR_9_ENABLED  0x0200
#define NVREG_MSI_VECTOR_10_ENABLED 0x0400
#define NVREG_MSI_VECTOR_11_ENABLED 0x0800
#define NVREG_MSI_VECTOR_12_ENABLED 0x1000
#define NVREG_MSI_VECTOR_13_ENABLED 0x2000
#define NVREG_MSI_VECTOR_14_ENABLED 0x4000
#define NVREG_MSI_VECTOR_15_ENABLED 0x8000
/* you get the idea */

	NvRegMisc1 = 0x080,
#define NVREG_MISC1_PAUSE_TX	0x01
#define NVREG_MISC1_HD		0x02
#define NVREG_MISC1_FORCE	0x3b0f3c

	NvRegMacReset = 0x34,
#define NVREG_MAC_RESET_ASSERT	0x0F3
	NvRegTransmitterControl = 0x084,
#define NVREG_XMITCTL_START	0x01
#define NVREG_XMITCTL_MGMT_ST	0x40000000
#define NVREG_XMITCTL_SYNC_MASK		0x000f0000
#define NVREG_XMITCTL_SYNC_NOT_READY	0x0
#define NVREG_XMITCTL_SYNC_PHY_INIT	0x00040000
#define NVREG_XMITCTL_MGMT_SEMA_MASK	0x00000f00
#define NVREG_XMITCTL_MGMT_SEMA_FREE	0x0
#define NVREG_XMITCTL_HOST_SEMA_MASK	0x0000f000
#define NVREG_XMITCTL_HOST_SEMA_ACQ	0x0000f000
#define NVREG_XMITCTL_HOST_LOADED	0x00004000
#define NVREG_XMITCTL_TX_PATH_EN	0x01000000
#define NVREG_XMITCTL_DATA_START        0x00100000
#define NVREG_XMITCTL_DATA_READY        0x00010000
#define NVREG_XMITCTL_DATA_ERROR        0x00020000
	NvRegTransmitterStatus = 0x088,
#define NVREG_XMITSTAT_BUSY	0x01

	NvRegPacketFilterFlags = 0x8c,
#define NVREG_PFF_PAUSE_RX	0x08
#define NVREG_PFF_ALWAYS	0x7F0000
#define NVREG_PFF_PROMISC	0x80
#define NVREG_PFF_MYADDR	0x20
#define NVREG_PFF_LOOPBACK	0x10

	NvRegOffloadConfig = 0x90,
#define NVREG_OFFLOAD_HOMEPHY	0x601
#define NVREG_OFFLOAD_NORMAL	RX_NIC_BUFSIZE
	NvRegReceiverControl = 0x094,
#define NVREG_RCVCTL_START	0x01
#define NVREG_RCVCTL_RX_PATH_EN	0x01000000
	NvRegReceiverStatus = 0x98,
#define NVREG_RCVSTAT_BUSY	0x01

	NvRegSlotTime = 0x9c,
#define NVREG_SLOTTIME_LEGBF_ENABLED	0x80000000
#define NVREG_SLOTTIME_10_100_FULL	0x00007f00
#define NVREG_SLOTTIME_1000_FULL 	0x0003ff00
#define NVREG_SLOTTIME_HALF		0x0000ff00
#define NVREG_SLOTTIME_DEFAULT	 	0x00007f00
#define NVREG_SLOTTIME_MASK		0x000000ff

	NvRegTxDeferral = 0xA0,
#define NVREG_TX_DEFERRAL_DEFAULT		0x15050f
#define NVREG_TX_DEFERRAL_RGMII_10_100		0x16070f
#define NVREG_TX_DEFERRAL_RGMII_1000		0x14050f
#define NVREG_TX_DEFERRAL_RGMII_STRETCH_10	0x16190f
#define NVREG_TX_DEFERRAL_RGMII_STRETCH_100	0x16300f
#define NVREG_TX_DEFERRAL_MII_STRETCH		0x152000
	NvRegRxDeferral = 0xA4,
#define NVREG_RX_DEFERRAL_DEFAULT	0x16
	NvRegMacAddrA = 0xA8,
	NvRegMacAddrB = 0xAC,
	NvRegMulticastAddrA = 0xB0,
#define NVREG_MCASTADDRA_FORCE	0x01
	NvRegMulticastAddrB = 0xB4,
	NvRegMulticastMaskA = 0xB8,
#define NVREG_MCASTMASKA_NONE		0xffffffff
	NvRegMulticastMaskB = 0xBC,
#define NVREG_MCASTMASKB_NONE		0xffff

	NvRegPhyInterface = 0xC0,
#define PHY_RGMII		0x10000000
	NvRegBackOffControl = 0xC4,
#define NVREG_BKOFFCTRL_DEFAULT			0x70000000
#define NVREG_BKOFFCTRL_SEED_MASK		0x000003ff
#define NVREG_BKOFFCTRL_SELECT			24
#define NVREG_BKOFFCTRL_GEAR			12

	NvRegTxRingPhysAddr = 0x100,
	NvRegRxRingPhysAddr = 0x104,
	NvRegRingSizes = 0x108,
#define NVREG_RINGSZ_TXSHIFT 0
#define NVREG_RINGSZ_RXSHIFT 16
	NvRegTransmitPoll = 0x10c,
#define NVREG_TRANSMITPOLL_MAC_ADDR_REV	0x00008000
	NvRegLinkSpeed = 0x110,
#define NVREG_LINKSPEED_FORCE 0x10000
#define NVREG_LINKSPEED_10	1000
#define NVREG_LINKSPEED_100	100
#define NVREG_LINKSPEED_1000	50
#define NVREG_LINKSPEED_MASK	(0xFFF)
	NvRegUnknownSetupReg5 = 0x130,
#define NVREG_UNKSETUP5_BIT31	(1<<31)
	NvRegTxWatermark = 0x13c,
#define NVREG_TX_WM_DESC1_DEFAULT	0x0200010
#define NVREG_TX_WM_DESC2_3_DEFAULT	0x1e08000
#define NVREG_TX_WM_DESC2_3_1000	0xfe08000
	NvRegTxRxControl = 0x144,
#define NVREG_TXRXCTL_KICK	0x0001
#define NVREG_TXRXCTL_BIT1	0x0002
#define NVREG_TXRXCTL_BIT2	0x0004
#define NVREG_TXRXCTL_IDLE	0x0008
#define NVREG_TXRXCTL_RESET	0x0010
#define NVREG_TXRXCTL_RXCHECK	0x0400
#define NVREG_TXRXCTL_DESC_1	0
#define NVREG_TXRXCTL_DESC_2	0x002100
#define NVREG_TXRXCTL_DESC_3	0xc02200
#define NVREG_TXRXCTL_VLANSTRIP 0x00040
#define NVREG_TXRXCTL_VLANINS	0x00080
	NvRegTxRingPhysAddrHigh = 0x148,
	NvRegRxRingPhysAddrHigh = 0x14C,
	NvRegTxPauseFrame = 0x170,
#define NVREG_TX_PAUSEFRAME_DISABLE	0x0fff0080
#define NVREG_TX_PAUSEFRAME_ENABLE_V1	0x01800010
#define NVREG_TX_PAUSEFRAME_ENABLE_V2	0x056003f0
#define NVREG_TX_PAUSEFRAME_ENABLE_V3	0x09f00880
	NvRegTxPauseFrameLimit = 0x174,
#define NVREG_TX_PAUSEFRAMELIMIT_ENABLE 0x00010000
	NvRegMIIStatus = 0x180,
#define NVREG_MIISTAT_ERROR		0x0001
#define NVREG_MIISTAT_LINKCHANGE	0x0008
#define NVREG_MIISTAT_MASK_RW		0x0007
#define NVREG_MIISTAT_MASK_ALL		0x000f
	NvRegMIIMask = 0x184,
#define NVREG_MII_LINKCHANGE		0x0008

	NvRegAdapterControl = 0x188,
#define NVREG_ADAPTCTL_START	0x02
#define NVREG_ADAPTCTL_LINKUP	0x04
#define NVREG_ADAPTCTL_PHYVALID	0x40000
#define NVREG_ADAPTCTL_RUNNING	0x100000
#define NVREG_ADAPTCTL_PHYSHIFT	24
	NvRegMIISpeed = 0x18c,
#define NVREG_MIISPEED_BIT8	(1<<8)
#define NVREG_MIIDELAY	5
	NvRegMIIControl = 0x190,
#define NVREG_MIICTL_INUSE	0x08000
#define NVREG_MIICTL_WRITE	0x00400
#define NVREG_MIICTL_ADDRSHIFT	5
	NvRegMIIData = 0x194,
	NvRegTxUnicast = 0x1a0,
	NvRegTxMulticast = 0x1a4,
	NvRegTxBroadcast = 0x1a8,
	NvRegWakeUpFlags = 0x200,
#define NVREG_WAKEUPFLAGS_VAL		0x7770
#define NVREG_WAKEUPFLAGS_BUSYSHIFT	24
#define NVREG_WAKEUPFLAGS_ENABLESHIFT	16
#define NVREG_WAKEUPFLAGS_D3SHIFT	12
#define NVREG_WAKEUPFLAGS_D2SHIFT	8
#define NVREG_WAKEUPFLAGS_D1SHIFT	4
#define NVREG_WAKEUPFLAGS_D0SHIFT	0
#define NVREG_WAKEUPFLAGS_ACCEPT_MAGPAT		0x01
#define NVREG_WAKEUPFLAGS_ACCEPT_WAKEUPPAT	0x02
#define NVREG_WAKEUPFLAGS_ACCEPT_LINKCHANGE	0x04
#define NVREG_WAKEUPFLAGS_ENABLE	0x1111

        NvRegMgmtUnitGetVersion = 0x204,
#define NVREG_MGMTUNITGETVERSION        0x01
        NvRegMgmtUnitVersion = 0x208,
#define NVREG_MGMTUNITVERSION           0x08
	NvRegPowerCap = 0x268,
#define NVREG_POWERCAP_D3SUPP	(1<<30)
#define NVREG_POWERCAP_D2SUPP	(1<<26)
#define NVREG_POWERCAP_D1SUPP	(1<<25)
	NvRegPowerState = 0x26c,
#define NVREG_POWERSTATE_POWEREDUP	0x8000
#define NVREG_POWERSTATE_VALID		0x0100
#define NVREG_POWERSTATE_MASK		0x0003
#define NVREG_POWERSTATE_D0		0x0000
#define NVREG_POWERSTATE_D1		0x0001
#define NVREG_POWERSTATE_D2		0x0002
#define NVREG_POWERSTATE_D3		0x0003
        NvRegMgmtUnitControl = 0x278,
#define NVREG_MGMTUNITCONTROL_INUSE     0x20000
	NvRegTxCnt = 0x280,
	NvRegTxZeroReXmt = 0x284,
	NvRegTxOneReXmt = 0x288,
	NvRegTxManyReXmt = 0x28c,
	NvRegTxLateCol = 0x290,
	NvRegTxUnderflow = 0x294,
	NvRegTxLossCarrier = 0x298,
	NvRegTxExcessDef = 0x29c,
	NvRegTxRetryErr = 0x2a0,
	NvRegRxFrameErr = 0x2a4,
	NvRegRxExtraByte = 0x2a8,
	NvRegRxLateCol = 0x2ac,
	NvRegRxRunt = 0x2b0,
	NvRegRxFrameTooLong = 0x2b4,
	NvRegRxOverflow = 0x2b8,
	NvRegRxFCSErr = 0x2bc,
	NvRegRxFrameAlignErr = 0x2c0,
	NvRegRxLenErr = 0x2c4,
	NvRegRxUnicast = 0x2c8,
	NvRegRxMulticast = 0x2cc,
	NvRegRxBroadcast = 0x2d0,
	NvRegTxDef = 0x2d4,
	NvRegTxFrame = 0x2d8,
	NvRegRxCnt = 0x2dc,
	NvRegTxPause = 0x2e0,
	NvRegRxPause = 0x2e4,
	NvRegRxDropFrame = 0x2e8,
	NvRegVlanControl = 0x300,
#define NVREG_VLANCONTROL_ENABLE	0x2000
	NvRegMSIXMap0 = 0x3e0,
	NvRegMSIXMap1 = 0x3e4,
	NvRegMSIXIrqStatus = 0x3f0,

	NvRegPowerState2 = 0x600,
#define NVREG_POWERSTATE2_POWERUP_MASK		0x0F15
#define NVREG_POWERSTATE2_POWERUP_REV_A3	0x0001
#define NVREG_POWERSTATE2_PHY_RESET             0x0004
#define NVREG_POWERSTATE2_GATE_CLOCKS           0x0F00
};

#endif
