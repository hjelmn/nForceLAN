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
 * forcedeth_power.cpp:
 *   Power management subroutines for nForce LAN driver.
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

#include <IOKit/IOLib.h>
#include <IOKit/pci/IOPCIDevice.h>
#include <IOKit/pwr_mgt/IOPM.h>

#include "forcedeth.h"

#define super IOEthernetController

#define kFiveSeconds 5000000

IOReturn nForceLAN::getPacketFilters (const OSSymbol *group, UInt32 *filters) const {	
	if (group == gIOEthernetWakeOnLANFilterGroup) {
		*filters = kIOEthernetWakeOnMagicPacket;
		return kIOReturnSuccess;
	}
	
	return super::getPacketFilters( group, filters );
}

IOReturn nForceLAN::setWakeOnMagicPacket (bool active) {	
  writeRegister(NvRegWakeUpFlags, active ? NVREG_WAKEUPFLAGS_ENABLE : 0);
	
  _wolEnabled = active;
	return kIOReturnSuccess;
}

/* register power management support. it is possible that older nForce cards should return kIOUnsupported here */
IOReturn nForceLAN::registerWithPolicyMaker (IOService *policyMaker) {
	static IOPMPowerState powerStateArray [kforcedethPowerStateCount] = {
		/* kforcedethPowerStateOff will cause disable () to be called (kIOPMDeviceUsable flag unset) */
		{1, 0,                 0,            0,            0, 0, 0, 0, 0, 0, 0, 0},
		/* kforcedethPowerStateOn will cause enable () to be called (kIOPMDeviceUsable flag set) */
		{1, kIOPMDeviceUsable, kIOPMPowerOn, kIOPMPowerOn, 0, 0, 0, 0, 0, 0, 0, 0}};

	NVLOG_DEBUG (1, "registerWithPolicyMaker: entering ...\n");

	/* the network driver is powered on */
	_pmPowerState  = kforcedethPowerStateOn;
	_pmSetState    = _pmPowerState;
	_pmPolicyMaker = policyMaker;
	_pmPowerStateChanged = false;

	/* finally, register our power states */
	return _pmPolicyMaker->registerPowerDriver (this, powerStateArray, kforcedethPowerStateCount);
}

IOReturn nForceLAN::powerStateWillChangeTo (IOPMPowerFlags capabilities, unsigned long stateNumber, IOService *whatDevice) {
  /* let disable/enable know that we have changed power states */
  _pmPowerStateChanged = (_pmPowerState != stateNumber);

	return IOPMAckImplied;
}

IOReturn nForceLAN::powerStateDidChangeTo (IOPMPowerFlags capabilities, unsigned long stateNumber, IOService *whatDevice) {
	return IOPMAckImplied;
}

IOReturn nForceLAN::setPowerState (unsigned long powerStateOrdinal, IOService *policyMaker) {
	NVLOG_DEBUG (1, "setPowerState: old state: %d, new state: %lu\n", _pmPowerState, powerStateOrdinal);

	_pmPowerState = powerStateOrdinal;

	return IOPMAckImplied;
}
