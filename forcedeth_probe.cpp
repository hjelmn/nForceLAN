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

#include "forcedeth.h"

/* Define my superclass */
#define super IOEthernetController

struct pci_device_id {
	const char *device_string;
	UInt16 vendor_id, device_id;
	UInt32 driver_data;
};

static const struct pci_device_id pci_tbl[] = {
	{	/* nForce Ethernet Controller */
		"nForce1 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_1,
		DEV_NEED_TIMERIRQ | DEV_NEED_LINKTIMER,
	},
	{	/* nForce2 Ethernet Controller */
		"nForce2 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_2,
		DEV_NEED_TIMERIRQ|DEV_NEED_LINKTIMER,
	},
	{	/* nForce3 Ethernet Controller */
		"nForce3 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_3,
		DEV_NEED_TIMERIRQ|DEV_NEED_LINKTIMER,
	},
	{	/* nForce3 Ethernet Controller */
		"nForce3 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_4,
		DEV_NEED_TIMERIRQ|DEV_NEED_LINKTIMER|DEV_HAS_LARGEDESC|DEV_HAS_CHECKSUM,
	},
	{	/* nForce3 Ethernet Controller */
		"nForce3 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_5,
		DEV_NEED_TIMERIRQ|DEV_NEED_LINKTIMER|DEV_HAS_LARGEDESC|DEV_HAS_CHECKSUM,
	},
	{	/* nForce3 Ethernet Controller */
		"nForce3 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_6,
		DEV_NEED_TIMERIRQ|DEV_NEED_LINKTIMER|DEV_HAS_LARGEDESC|DEV_HAS_CHECKSUM,
	},
	{	/* nForce3 Ethernet Controller */
		"nForce3 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_7,
		DEV_NEED_TIMERIRQ|DEV_NEED_LINKTIMER|DEV_HAS_LARGEDESC|DEV_HAS_CHECKSUM,
	},
	{	/* CK804 Ethernet Controller */
		"CK804 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_8,
		DEV_NEED_LINKTIMER|DEV_HAS_LARGEDESC|DEV_HAS_CHECKSUM|DEV_HAS_HIGH_DMA|DEV_HAS_STATISTICS_V1|DEV_NEED_TX_LIMIT,
	},
	{	/* CK804 Ethernet Controller */
		"CK804 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_9,
		DEV_NEED_LINKTIMER|DEV_HAS_LARGEDESC|DEV_HAS_CHECKSUM|DEV_HAS_HIGH_DMA|DEV_HAS_STATISTICS_V1|DEV_NEED_TX_LIMIT,
	},
	{	/* MCP04 Ethernet Controller */
		"MCP04 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_10,
		DEV_NEED_LINKTIMER|DEV_HAS_LARGEDESC|DEV_HAS_CHECKSUM|DEV_HAS_HIGH_DMA|DEV_HAS_STATISTICS_V1|DEV_NEED_TX_LIMIT,
	},
	{	/* MCP04 Ethernet Controller */
		"MCP04 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_11,
		DEV_NEED_LINKTIMER|DEV_HAS_LARGEDESC|DEV_HAS_CHECKSUM|DEV_HAS_HIGH_DMA|DEV_HAS_STATISTICS_V1|DEV_NEED_TX_LIMIT,
	},
	{	/* MCP51 Ethernet Controller */
		"MCP51 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_12,
		DEV_NEED_LINKTIMER|DEV_HAS_HIGH_DMA|DEV_HAS_POWER_CNTRL|DEV_HAS_STATISTICS_V1|DEV_NEED_LOW_POWER_FIX,
	},
	{	/* MCP51 Ethernet Controller */
		"MCP51 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_13,
		DEV_NEED_LINKTIMER|DEV_HAS_HIGH_DMA|DEV_HAS_POWER_CNTRL|DEV_HAS_STATISTICS_V1|DEV_NEED_LOW_POWER_FIX,
	},
	{	/* MCP55 Ethernet Controller */
		"MCP55 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_14,
		DEV_NEED_LINKTIMER|DEV_HAS_LARGEDESC|DEV_HAS_CHECKSUM|DEV_HAS_HIGH_DMA|DEV_HAS_VLAN|DEV_HAS_MSI|DEV_HAS_MSI_X|DEV_HAS_POWER_CNTRL|DEV_HAS_PAUSEFRAME_TX_V1|DEV_HAS_STATISTICS_V2|DEV_HAS_TEST_EXTENDED|DEV_HAS_MGMT_UNIT|DEV_NEED_TX_LIMIT|DEV_NEED_MSI_FIX,
	},
	{	/* MCP55 Ethernet Controller */
		"MCP55 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_15,
		DEV_NEED_LINKTIMER|DEV_HAS_LARGEDESC|DEV_HAS_CHECKSUM|DEV_HAS_HIGH_DMA|DEV_HAS_VLAN|DEV_HAS_MSI|DEV_HAS_MSI_X|DEV_HAS_POWER_CNTRL|DEV_HAS_PAUSEFRAME_TX_V1|DEV_HAS_STATISTICS_V2|DEV_HAS_TEST_EXTENDED|DEV_HAS_MGMT_UNIT|DEV_NEED_TX_LIMIT|DEV_NEED_MSI_FIX,
	},
	{	/* MCP61 Ethernet Controller */
		"MCP61 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_16,
		DEV_NEED_LINKTIMER|DEV_HAS_HIGH_DMA|DEV_HAS_POWER_CNTRL|DEV_HAS_MSI|DEV_HAS_PAUSEFRAME_TX_V1|DEV_HAS_STATISTICS_V2|DEV_HAS_TEST_EXTENDED|DEV_HAS_MGMT_UNIT|DEV_HAS_CORRECT_MACADDR|DEV_NEED_MSI_FIX,
	},
	{	/* MCP61 Ethernet Controller */
		"MCP61 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_17,
		DEV_NEED_LINKTIMER|DEV_HAS_HIGH_DMA|DEV_HAS_POWER_CNTRL|DEV_HAS_MSI|DEV_HAS_PAUSEFRAME_TX_V1|DEV_HAS_STATISTICS_V2|DEV_HAS_TEST_EXTENDED|DEV_HAS_MGMT_UNIT|DEV_HAS_CORRECT_MACADDR|DEV_NEED_MSI_FIX,
	},
	{	/* MCP61 Ethernet Controller */
		"MCP61 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_18,
		DEV_NEED_LINKTIMER|DEV_HAS_HIGH_DMA|DEV_HAS_POWER_CNTRL|DEV_HAS_MSI|DEV_HAS_PAUSEFRAME_TX_V1|DEV_HAS_STATISTICS_V2|DEV_HAS_TEST_EXTENDED|DEV_HAS_MGMT_UNIT|DEV_HAS_CORRECT_MACADDR|DEV_NEED_MSI_FIX,
	},
	{	/* MCP61 Ethernet Controller */
		"MCP61 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_19,
		DEV_NEED_LINKTIMER|DEV_HAS_HIGH_DMA|DEV_HAS_POWER_CNTRL|DEV_HAS_MSI|DEV_HAS_PAUSEFRAME_TX_V1|DEV_HAS_STATISTICS_V2|DEV_HAS_TEST_EXTENDED|DEV_HAS_MGMT_UNIT|DEV_HAS_CORRECT_MACADDR|DEV_NEED_MSI_FIX,
	},
	{	/* MCP65 Ethernet Controller */
		"MCP65 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_20,
		DEV_NEED_LINKTIMER|DEV_HAS_LARGEDESC|DEV_HAS_HIGH_DMA|DEV_HAS_POWER_CNTRL|DEV_HAS_MSI|DEV_HAS_PAUSEFRAME_TX_V1|DEV_HAS_STATISTICS_V2|DEV_HAS_TEST_EXTENDED|DEV_HAS_MGMT_UNIT|DEV_HAS_CORRECT_MACADDR|DEV_NEED_TX_LIMIT|DEV_HAS_GEAR_MODE|DEV_NEED_MSI_FIX,
	},
	{	/* MCP65 Ethernet Controller */
		"MCP65 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_21,
		DEV_NEED_LINKTIMER|DEV_HAS_LARGEDESC|DEV_HAS_HIGH_DMA|DEV_HAS_POWER_CNTRL|DEV_HAS_MSI|DEV_HAS_PAUSEFRAME_TX_V1|DEV_HAS_STATISTICS_V2|DEV_HAS_TEST_EXTENDED|DEV_HAS_MGMT_UNIT|DEV_HAS_CORRECT_MACADDR|DEV_NEED_TX_LIMIT|DEV_HAS_GEAR_MODE|DEV_NEED_MSI_FIX,
	},
	{	/* MCP65 Ethernet Controller */
		"MCP65 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_22,
		DEV_NEED_LINKTIMER|DEV_HAS_LARGEDESC|DEV_HAS_HIGH_DMA|DEV_HAS_POWER_CNTRL|DEV_HAS_MSI|DEV_HAS_PAUSEFRAME_TX_V1|DEV_HAS_STATISTICS_V2|DEV_HAS_TEST_EXTENDED|DEV_HAS_MGMT_UNIT|DEV_HAS_CORRECT_MACADDR|DEV_NEED_TX_LIMIT|DEV_HAS_GEAR_MODE|DEV_NEED_MSI_FIX,
	},
	{	/* MCP65 Ethernet Controller */
		"MCP65 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_23,
		DEV_NEED_LINKTIMER|DEV_HAS_LARGEDESC|DEV_HAS_HIGH_DMA|DEV_HAS_POWER_CNTRL|DEV_HAS_MSI|DEV_HAS_PAUSEFRAME_TX_V1|DEV_HAS_STATISTICS_V2|DEV_HAS_TEST_EXTENDED|DEV_HAS_MGMT_UNIT|DEV_HAS_CORRECT_MACADDR|DEV_NEED_TX_LIMIT|DEV_HAS_GEAR_MODE|DEV_NEED_MSI_FIX,
	},
	{	/* MCP67 Ethernet Controller */
		"MCP67 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_24,
		DEV_NEED_LINKTIMER|DEV_HAS_HIGH_DMA|DEV_HAS_POWER_CNTRL|DEV_HAS_MSI|DEV_HAS_PAUSEFRAME_TX_V1|DEV_HAS_STATISTICS_V2|DEV_HAS_TEST_EXTENDED|DEV_HAS_MGMT_UNIT|DEV_HAS_CORRECT_MACADDR|DEV_HAS_GEAR_MODE|DEV_NEED_MSI_FIX,
	},
	{	/* MCP67 Ethernet Controller */
		"MCP67 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_25,
		DEV_NEED_LINKTIMER|DEV_HAS_HIGH_DMA|DEV_HAS_POWER_CNTRL|DEV_HAS_MSI|DEV_HAS_PAUSEFRAME_TX_V1|DEV_HAS_STATISTICS_V2|DEV_HAS_TEST_EXTENDED|DEV_HAS_MGMT_UNIT|DEV_HAS_CORRECT_MACADDR|DEV_HAS_GEAR_MODE|DEV_NEED_MSI_FIX,
	},
	{	/* MCP67 Ethernet Controller */
		"MCP67 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_26,
		DEV_NEED_LINKTIMER|DEV_HAS_HIGH_DMA|DEV_HAS_POWER_CNTRL|DEV_HAS_MSI|DEV_HAS_PAUSEFRAME_TX_V1|DEV_HAS_STATISTICS_V2|DEV_HAS_TEST_EXTENDED|DEV_HAS_MGMT_UNIT|DEV_HAS_CORRECT_MACADDR|DEV_HAS_GEAR_MODE|DEV_NEED_MSI_FIX,
	},
	{	/* MCP67 Ethernet Controller */
		"MCP67 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_27,
		DEV_NEED_LINKTIMER|DEV_HAS_HIGH_DMA|DEV_HAS_POWER_CNTRL|DEV_HAS_MSI|DEV_HAS_PAUSEFRAME_TX_V1|DEV_HAS_STATISTICS_V2|DEV_HAS_TEST_EXTENDED|DEV_HAS_MGMT_UNIT|DEV_HAS_CORRECT_MACADDR|DEV_HAS_GEAR_MODE|DEV_NEED_MSI_FIX,
	},
	{	/* MCP73 Ethernet Controller */
		"MCP73 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_28,
		DEV_NEED_LINKTIMER|DEV_HAS_HIGH_DMA|DEV_HAS_POWER_CNTRL|DEV_HAS_MSI|DEV_HAS_PAUSEFRAME_TX_V1|DEV_HAS_STATISTICS_V2|DEV_HAS_TEST_EXTENDED|DEV_HAS_MGMT_UNIT|DEV_HAS_CORRECT_MACADDR|DEV_HAS_COLLISION_FIX|DEV_HAS_GEAR_MODE|DEV_NEED_MSI_FIX,
	},
	{	/* MCP73 Ethernet Controller */
		"MCP73 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_29,
		DEV_NEED_LINKTIMER|DEV_HAS_HIGH_DMA|DEV_HAS_POWER_CNTRL|DEV_HAS_MSI|DEV_HAS_PAUSEFRAME_TX_V1|DEV_HAS_STATISTICS_V2|DEV_HAS_TEST_EXTENDED|DEV_HAS_MGMT_UNIT|DEV_HAS_CORRECT_MACADDR|DEV_HAS_COLLISION_FIX|DEV_HAS_GEAR_MODE|DEV_NEED_MSI_FIX,
	},
	{	/* MCP73 Ethernet Controller */
		"MCP74 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_30,
		DEV_NEED_LINKTIMER|DEV_HAS_HIGH_DMA|DEV_HAS_POWER_CNTRL|DEV_HAS_MSI|DEV_HAS_PAUSEFRAME_TX_V1|DEV_HAS_STATISTICS_V2|DEV_HAS_TEST_EXTENDED|DEV_HAS_MGMT_UNIT|DEV_HAS_CORRECT_MACADDR|DEV_HAS_COLLISION_FIX|DEV_HAS_GEAR_MODE|DEV_NEED_MSI_FIX,
	},
	{	/* MCP73 Ethernet Controller */
		"MCP73 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_31,
		DEV_NEED_LINKTIMER|DEV_HAS_HIGH_DMA|DEV_HAS_POWER_CNTRL|DEV_HAS_MSI|DEV_HAS_PAUSEFRAME_TX_V1|DEV_HAS_STATISTICS_V2|DEV_HAS_TEST_EXTENDED|DEV_HAS_MGMT_UNIT|DEV_HAS_CORRECT_MACADDR|DEV_HAS_COLLISION_FIX|DEV_HAS_GEAR_MODE|DEV_NEED_MSI_FIX,
	},
	{	/* MCP77 Ethernet Controller */
		"MCP77 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_32,
		DEV_NEED_LINKTIMER|DEV_HAS_CHECKSUM|DEV_HAS_HIGH_DMA|DEV_HAS_MSI|DEV_HAS_POWER_CNTRL|DEV_HAS_PAUSEFRAME_TX_V2|DEV_HAS_STATISTICS_V3|DEV_HAS_TEST_EXTENDED|DEV_HAS_MGMT_UNIT|DEV_HAS_CORRECT_MACADDR|DEV_HAS_COLLISION_FIX|DEV_NEED_TX_LIMIT2|DEV_HAS_GEAR_MODE|DEV_NEED_PHY_INIT_FIX|DEV_NEED_MSI_FIX,
	},
	{	/* MCP77 Ethernet Controller */
		"MCP77 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_33,
		DEV_NEED_LINKTIMER|DEV_HAS_CHECKSUM|DEV_HAS_HIGH_DMA|DEV_HAS_MSI|DEV_HAS_POWER_CNTRL|DEV_HAS_PAUSEFRAME_TX_V2|DEV_HAS_STATISTICS_V3|DEV_HAS_TEST_EXTENDED|DEV_HAS_MGMT_UNIT|DEV_HAS_CORRECT_MACADDR|DEV_HAS_COLLISION_FIX|DEV_NEED_TX_LIMIT2|DEV_HAS_GEAR_MODE|DEV_NEED_PHY_INIT_FIX|DEV_NEED_MSI_FIX,
	},
	{	/* MCP77 Ethernet Controller */
		"MCP77 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_34,
		DEV_NEED_LINKTIMER|DEV_HAS_CHECKSUM|DEV_HAS_HIGH_DMA|DEV_HAS_MSI|DEV_HAS_POWER_CNTRL|DEV_HAS_PAUSEFRAME_TX_V2|DEV_HAS_STATISTICS_V3|DEV_HAS_TEST_EXTENDED|DEV_HAS_MGMT_UNIT|DEV_HAS_CORRECT_MACADDR|DEV_HAS_COLLISION_FIX|DEV_NEED_TX_LIMIT2|DEV_HAS_GEAR_MODE|DEV_NEED_PHY_INIT_FIX|DEV_NEED_MSI_FIX,
	},
	{	/* MCP77 Ethernet Controller */
		"MCP77 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_35,
		DEV_NEED_LINKTIMER|DEV_HAS_CHECKSUM|DEV_HAS_HIGH_DMA|DEV_HAS_MSI|DEV_HAS_POWER_CNTRL|DEV_HAS_PAUSEFRAME_TX_V2|DEV_HAS_STATISTICS_V3|DEV_HAS_TEST_EXTENDED|DEV_HAS_MGMT_UNIT|DEV_HAS_CORRECT_MACADDR|DEV_HAS_COLLISION_FIX|DEV_NEED_TX_LIMIT2|DEV_HAS_GEAR_MODE|DEV_NEED_PHY_INIT_FIX|DEV_NEED_MSI_FIX,
	},
	{	/* MCP79 Ethernet Controller */
		"MCP79 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_36,
		DEV_NEED_LINKTIMER|DEV_HAS_LARGEDESC|DEV_HAS_CHECKSUM|DEV_HAS_HIGH_DMA|DEV_HAS_MSI|DEV_HAS_POWER_CNTRL|DEV_HAS_PAUSEFRAME_TX_V3|DEV_HAS_STATISTICS_V3|DEV_HAS_TEST_EXTENDED|DEV_HAS_CORRECT_MACADDR|DEV_HAS_COLLISION_FIX|DEV_NEED_TX_LIMIT2|DEV_HAS_GEAR_MODE|DEV_NEED_PHY_INIT_FIX|DEV_NEED_MSI_FIX,
	},
	{	/* MCP79 Ethernet Controller */
		"MCP79 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_37,
		DEV_NEED_LINKTIMER|DEV_HAS_LARGEDESC|DEV_HAS_CHECKSUM|DEV_HAS_HIGH_DMA|DEV_HAS_MSI|DEV_HAS_POWER_CNTRL|DEV_HAS_PAUSEFRAME_TX_V3|DEV_HAS_STATISTICS_V3|DEV_HAS_TEST_EXTENDED|DEV_HAS_CORRECT_MACADDR|DEV_HAS_COLLISION_FIX|DEV_NEED_TX_LIMIT2|DEV_HAS_GEAR_MODE|DEV_NEED_PHY_INIT_FIX|DEV_NEED_MSI_FIX,
	},
	{	/* MCP79 Ethernet Controller */
		"MCP79 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_38,
		DEV_NEED_LINKTIMER|DEV_HAS_LARGEDESC|DEV_HAS_CHECKSUM|DEV_HAS_HIGH_DMA|DEV_HAS_MSI|DEV_HAS_POWER_CNTRL|DEV_HAS_PAUSEFRAME_TX_V3|DEV_HAS_STATISTICS_V3|DEV_HAS_TEST_EXTENDED|DEV_HAS_CORRECT_MACADDR|DEV_HAS_COLLISION_FIX|DEV_NEED_TX_LIMIT2|DEV_HAS_GEAR_MODE|DEV_NEED_PHY_INIT_FIX|DEV_NEED_MSI_FIX,
	},
	{	/* MCP79 Ethernet Controller */
		"MCP79 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_39,
		DEV_NEED_LINKTIMER|DEV_HAS_LARGEDESC|DEV_HAS_CHECKSUM|DEV_HAS_HIGH_DMA|DEV_HAS_MSI|DEV_HAS_POWER_CNTRL|DEV_HAS_PAUSEFRAME_TX_V3|DEV_HAS_STATISTICS_V3|DEV_HAS_TEST_EXTENDED|DEV_HAS_CORRECT_MACADDR|DEV_HAS_COLLISION_FIX|DEV_NEED_TX_LIMIT2|DEV_HAS_GEAR_MODE|DEV_NEED_PHY_INIT_FIX|DEV_NEED_MSI_FIX,
	},
	{       /* MCP89 Ethernet Controller */
		"MCP89 Ethernet Controller",
		PCI_VENDOR_ID_NVIDIA,
		PCI_DEVICE_ID_NVIDIA_NVENET_40,
		DEV_NEED_LINKTIMER|DEV_HAS_LARGEDESC|DEV_HAS_CHECKSUM|DEV_HAS_HIGH_DMA|DEV_HAS_MSI|DEV_HAS_POWER_CNTRL|DEV_HAS_PAUSEFRAME_TX_V3|DEV_HAS_STATISTICS_V3|DEV_HAS_TEST_EXTENDED|DEV_HAS_CORRECT_MACADDR|DEV_HAS_COLLISION_FIX|DEV_HAS_GEAR_MODE|DEV_NEED_PHY_INIT_FIX,
	},
	{0,},
};

void nForceLAN::deviceSetup (void) {
  UInt32 maxRingLen;

	if (_deviceFlags & (DEV_HAS_VLAN | DEV_HAS_MSI_X | DEV_HAS_POWER_CNTRL | DEV_HAS_STATISTICS_V2))
		_registerSize = NV_PCI_REGSZ_VER3;
	else if (_deviceFlags & DEV_HAS_STATISTICS_V1)
		_registerSize = NV_PCI_REGSZ_VER2;
	else
		_registerSize = NV_PCI_REGSZ_VER1;

	/* handle different descriptor versions */
	if (_deviceFlags & DEV_HAS_HIGH_DMA) {
		/* packet format 3: supports 40-bit addressing */
		_descVers        = DESC_VER_3;
		_useExtendedRing = true;
		_txrxCtlBits     = NVREG_TXRXCTL_DESC_3;

#if defined(__LP64__)
		_physMemMask = 0x000000FFFFFFFFFFULL;
#else
		_physMemMask = 0x00000000FFFFFFFFULL;
#endif
	} else {
		_useExtendedRing = false;
		_physMemMask = 0x00000000FFFFFFFFULL;

		if (_deviceFlags & DEV_HAS_LARGEDESC) {
			/* packet format 2: supports jumbo frames */
			_descVers = DESC_VER_2;
			_txrxCtlBits = NVREG_TXRXCTL_DESC_2;
		} else {
			/* original packet format */
			_descVers = DESC_VER_1;
			_txrxCtlBits = NVREG_TXRXCTL_DESC_1;
		}
	}

	NVLOG_DEBUG(0, "Physical memory mask = 0x%llx\n", _physMemMask);
	
	if (_descVers == DESC_VER_1) {
		_txFlags = NV_TX_VALID;
		_rxAvailableFlag = NV_RX_AVAIL;
		maxRingLen = RING_MAX_DESC_VER_1;
	} else {
		_txFlags = NV_TX2_VALID;
    _rxAvailableFlag = NV_RX2_AVAIL;
		maxRingLen = RING_MAX_DESC_VER_2_3;
	}

  /* verify ring lengths */
  if (_rxRingLen > maxRingLen)
	  _rxRingLen = maxRingLen;
  if (_txRingLen > maxRingLen)
	  _txRingLen = maxRingLen;

  _packetLimit = NV_PKTLIMIT_1;
  if (_deviceFlags & (DEV_HAS_LARGEDESC | DEV_HAS_HIGH_DMA))
	  _packetLimit = NV_PKTLIMIT_2;

  if (_deviceFlags & DEV_HAS_CHECKSUM) {
	  _hasHWCRC = true;
	  _txrxCtlBits |= NVREG_TXRXCTL_RXCHECK;
  }

  _vlanCtlBits = 0;
  if (_deviceFlags & DEV_HAS_VLAN)
	  _vlanCtlBits = NVREG_VLANCONTROL_ENABLE;

  _msiFlags = 0;
  if (_deviceFlags & DEV_HAS_MSI)
	  _msiFlags |= NV_MSI_CAPABLE;

  if (_deviceFlags & DEV_HAS_MSI_X)
	  _msiFlags |= NV_MSI_X_CAPABLE;

  if (_optimizationMode == NV_OPTIMIZATION_MODE_CPU) {
	  _irqMask = NVREG_IRQMASK_CPU;
	  if (_msiFlags & NV_MSI_X_CAPABLE)
		  _msiFlags |= 0x0001;
  } else if (_optimizationMode == NV_OPTIMIZATION_MODE_DYNAMIC && !(_deviceFlags & DEV_NEED_TIMERIRQ)) {
	  /* start off in throughput mode */
	  _irqMask = NVREG_IRQMASK_THROUGHPUT;
	  /* remove support for msix mode */
	  _msiFlags &= ~NV_MSI_X_CAPABLE;
  } else {
	  _optimizationMode = NV_OPTIMIZATION_MODE_THROUGHPUT;
	  _irqMask = NVREG_IRQMASK_THROUGHPUT;
	  if (_msiFlags & NV_MSI_X_CAPABLE) /* set number of vectors */
		  _msiFlags |= 0x0003;
  }

  _pauseFlags = NV_PAUSEFRAME_RX_CAPABLE | NV_PAUSEFRAME_RX_REQ | NV_PAUSEFRAME_AUTONEG;
  if ((_deviceFlags & DEV_HAS_PAUSEFRAME_TX_V1) ||
      (_deviceFlags & DEV_HAS_PAUSEFRAME_TX_V2) ||
      (_deviceFlags & DEV_HAS_PAUSEFRAME_TX_V3))
	  _pauseFlags |= NV_PAUSEFRAME_TX_CAPABLE | NV_PAUSEFRAME_TX_REQ;

  _txLimit = false;

  if (_deviceFlags & DEV_NEED_TIMERIRQ && !_disableTimerIRQ)
	  _irqMask |= NVREG_IRQ_TIMER;
	
  /* Limit the number of tx's outstanding for hw bug */
  if (_deviceFlags & DEV_NEED_TX_LIMIT) {
	  if (!(_revisionID >= 0xA2 && ((_deviceFlags & DEV_NEED_TX_LIMIT2) == DEV_NEED_TX_LIMIT2)))
		  _txLimit = true;
  }
}

bool nForceLAN::deviceDetect (void) {
	int i;
	
	for (i = 0 ; pci_tbl[i].vendor_id ; i++)
		if (pci_tbl[i].vendor_id == _vendorID && pci_tbl[i].device_id == _deviceID) {
			_deviceFlags  = pci_tbl[i].driver_data;
			_deviceString = pci_tbl[i].device_string;
			
			NVLOG (0, "found nVidia %s device %04x:%04x with flags 0x%08x\n", _deviceString, _vendorID, _deviceID, (unsigned int)_deviceFlags);
			deviceSetup ();
			
			return true;
		}
			
	return false;
}

IOService *nForceLAN::probe (IOService *provider, SInt32 *score) {
	OSData *dataVal;
	void *foo;

	NVLOG (1, "probing...\n");

	*score = 0;

	do {
		provider->retain();

		dataVal     = (OSData *)provider->getProperty ("vendor-id");
		if (!dataVal)
			break;

		foo = (void *)dataVal->getBytesNoCopy ();
		_vendorID = foo? *((UInt32 *)foo) : 0;
		
		dataVal     = (OSData *)provider->getProperty ("device-id");
		if (!dataVal)
			break;

		foo = (void *)dataVal->getBytesNoCopy ();
		_deviceID = foo ? *((UInt32 *)foo) : 0;

		dataVal     = (OSData *)provider->getProperty ("subsystem-vendor-id");
		if (!dataVal)
			break;
		
		foo = (void *)dataVal->getBytesNoCopy ();
		_subVendorID = foo ? *((UInt32 *)foo) : 0;
		
		dataVal     = (OSData *)provider->getProperty ("subsystem-id");
		if (!dataVal)
			break;

		foo = (void *)dataVal->getBytesNoCopy ();
		_subDeviceID = foo ? *((UInt32 *)foo) : 0;

		dataVal     = (OSData *)provider->getProperty ("revision-id");
		if (!dataVal)
			break;

		foo = (void *)dataVal->getBytesNoCopy ();
		_revisionID = foo ? *((UInt32 *)foo) : 0;
				
		/* check if device is supported */
		if (!deviceDetect())
			break;

		provider->setProperty ("name", "ethernet");
		provider->release ();
		
		*score = 100;
		return this;
	} while (0);
		
	provider->release ();
	
	return NULL;
}

const OSString* nForceLAN::newVendorString (void) const {
    return OSString::withCString ("nVidia");
}

const OSString* nForceLAN::newModelString (void) const {
	int i;

	if (_deviceString)
		return OSString::withCString(_deviceString);
	else {
		for (i = 0 ; pci_tbl[i].vendor_id ; i++)
			if (pci_tbl[i].vendor_id == _vendorID && pci_tbl[i].device_id == _deviceID)
				return OSString::withCString(pci_tbl[i].device_string);
	}

	return NULL;
}
