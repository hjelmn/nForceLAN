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

#define super IOEthernetController
 
// REQUIRED! This macro defines the class's constructors, destructors,
// and several other methods I/O Kit requires. Do NOT use super as the
// second parameter. You must use the literal name of the superclass.
OSDefineMetaClassAndStructors(nForceLAN, IOEthernetController);

bool nForceLAN::init (OSDictionary *dict) {
  OSBoolean *boolVal;
  OSNumber *numberVal;

  bool res = super::init(dict);

  _numInterrupts = 0;
  _haveMgmtSema = _ifEnabled = _recoverError = _promiscuousMode = _multicastMode = false;

  _mtu = 1500;
  _quietCount = _mgmtVersion = _mcList = _phyOui = _phyModel = _phyRev = 0;

  _map = NULL;
	_device = NULL;
	_interface = NULL;
  _nicLinkTimer = NULL;
	_nicStatTimer = NULL;
	_nicPollTimer = NULL;
  _dictionary = NULL;
	_workLoop = NULL;
	_ethernetStats = NULL;
	_stats = NULL;
  _txMbufCursor = NULL;
	_rxRingMem = NULL;
	_txRingMem = NULL;
	_rxRing = NULL;
	_txRing = NULL;
	_rxMemDesc = NULL;
	_txMemDesc = NULL;
  _txChangeOwnerTail = NULL;
	_txChangeOwnerHead = NULL;

  /* defaults */
  _logLevel         = 0;
  _optimizationMode = NV_OPTIMIZATION_MODE_DYNAMIC;
  _disableMSI       = false;	
  _disableTimerIRQ  = false;	
  _lowPowerSpeed    = true;
  _phyCross         = NV_CROSSOVER_DETECTION_ENABLED;
  _autoneg          = true;
  _txRingLen        = 4 * TX_RING_DEFAULT;
  _rxRingLen        = 4 * RX_RING_DEFAULT;
  _phyPowerDown     = false;
  _macOverride      = false;

  /* get user options */
  numberVal = (OSNumber *)getProperty ("LogLevel");
  if (numberVal)
    _logLevel = numberVal->unsigned16BitValue();
	
  boolVal = (OSBoolean *)getProperty("DisableMSI");
  if (boolVal)
    _disableMSI = boolVal->getValue ();

  boolVal = (OSBoolean *)getProperty("DisableTimerIRQ");
  if (boolVal)
    _disableTimerIRQ = boolVal->getValue ();

  boolVal = (OSBoolean *)getProperty("LowPowerSpeed");
  if (boolVal)
    _lowPowerSpeed = boolVal->getValue ();

  boolVal = (OSBoolean *)getProperty("CrossoverDetection");
  if (boolVal)
    _phyCross = boolVal->getValue () ? NV_CROSSOVER_DETECTION_ENABLED : NV_CROSSOVER_DETECTION_DISABLED;

  boolVal = (OSBoolean *)getProperty("AutoNegotiation");
  if (boolVal)
    _autoneg = boolVal->getValue ();

  boolVal = (OSBoolean *)getProperty("PhyPowerDown");
  if (boolVal)
    _phyPowerDown = boolVal->getValue ();

  boolVal = (OSBoolean *)getProperty("MACOverride");
  if (boolVal)
    _macOverride = boolVal->getValue ();

  numberVal = (OSNumber *)getProperty ("OptimizationMode");
  if (numberVal)
    _optimizationMode = numberVal->unsigned16BitValue();

  if (_optimizationMode > 2)
    _optimizationMode = 2;
		
  numberVal = (OSNumber *)getProperty ("RXRingSize");
  if (numberVal)
    _rxRingLen = numberVal->unsigned16BitValue();
	
  if (_rxRingLen < RX_RING_MIN)
    _rxRingLen = RX_RING_MIN;
	
  numberVal = (OSNumber *)getProperty ("TXRingSize");
  if (numberVal)
    _txRingLen = numberVal->unsigned16BitValue();
	
  if (_txRingLen < TX_RING_MIN)
    _txRingLen = TX_RING_MIN;

  NVLOG (1, "log verbosity to %d\n", (int)_logLevel);
  NVLOG (1, "message signaled interrupts (MSI): %sabled (if available)\n", _disableMSI ? "dis" : "en");
  NVLOG (1, "optimization mode: %s\n", _optimizationMode == NV_OPTIMIZATION_MODE_THROUGHPUT  ? "throughput" : _optimizationMode == NV_OPTIMIZATION_MODE_CPU ? "cpu" : "dynamic");
  NVLOG (1, "timer IRQ: %sabled\n", _disableTimerIRQ ? "dis" : "en");
  
  return res;
}

bool nForceLAN::start (IOService *provider) {
  int powerstate;
  bool res;
  
  NVLOG (0, "Version " FORCEDETH_VERSION " starting\n");
  
  do {
    _numInterrupts = 0;
    _nicPollTimer = NULL;
    
    if (!(res = super::start (provider)))
      break;
    
    if ((_device = OSDynamicCast(IOPCIDevice, provider)) == 0) {
      NVLOG (0, "cannot retrieve PCI device.\n");
      break;
    }
    
    _device->retain ();
    
    if( (_device->open(this)) == 0) {
      NVLOG (0, "failed to open PCI device.\n");
      break;
    }
		
    if (_device->requestPowerDomainState (kIOPMPowerOn, (IOPowerConnection *) getParentEntry (gIOPowerPlane), IOPMLowestState) != IOPMNoErr) {
      NVLOG (0, "could not get power domain state\n");
      break;
    }
		
    (void)initPCIConfigSpace();
    
    NVLOG_DEBUG (1, "PCI system 0x%04X:0x%04X, subsystem 0x%04X:0x%04X revision 0x%02X opened.\n",
                 _vendorID, _deviceID, _subVendorID, _subDeviceID, _revisionID);
    
    if (!mapMemoryRegisters ())
      break;
    
    if (!allocateRings ()) {
      NVLOG (0, "could not allocate memory for TX/RX rings\n");
      break;
    }
    
    if (!getMACAddressFromHardware ())
      break;
		
    /* disable Wakeup on LAN */
    writeRegister(NvRegWakeUpFlags, 0);
    _wolEnabled = false;
    
    /* handle device power state */
    if (_deviceFlags & DEV_HAS_POWER_CNTRL)  {
      NVLOG_DEBUG (1, "taking PHY and NIC out of low power mode\n");
      powerstate = readRegister(NvRegPowerState2);
      powerstate &= ~NVREG_POWERSTATE2_POWERUP_MASK;
      
      if ((_deviceFlags & DEV_NEED_LOW_POWER_FIX)  && _revisionID >= 0xA3)
        powerstate |= NVREG_POWERSTATE2_POWERUP_REV_A3;
      
      writeRegister(NvRegPowerState2, (UInt32) powerstate);
    }
    
    if (!getAndInitializePhy ())
      break;
    
    if (!setupEventSources ())
      break;
		
    /* close the device. it will be opened on demand later */
    _device->close (this);
		
    /* Set some defaults */
    _linkspeed = NVREG_LINKSPEED_FORCE | NVREG_LINKSPEED_10;
    _duplex = false;
    
    attachInterface(&_interface);
    
    return true;
  } while (false);

	/* close the device if it is open. it will be released later */
  if (_device && _device->isOpen ())
    _device->close (this);

  /* stop the superclass if it has been started */
  if (res)
    super::stop(provider);
  
  return false;
}

IOReturn nForceLAN::enable(IONetworkInterface *interface) {
  UInt32 i, low;
	
  if (_ifEnabled || _inEnable)
    return kIOReturnSuccess;
  
  _inEnable = true;
	
  NVLOG_DEBUG (1, "enabling NIC (Power State Change = %s)...\n", _pmPowerStateChanged ? "True" : "False");
  
  if (!_device || (!_device->isOpen () && (_device->open(this)) == 0)) {
    NVLOG (0, "failed to open PCI device.\n");
    return kIOReturnError;
  }
  
  _nicTx = 0;
  _curRx = 0;
  _linkspeed = 0;
  
  setBufSize();
	
  do {
    /* power up phy */
    miiRW (_physicalAddress, MII_BMCR, miiRW(_physicalAddress, MII_BMCR, MII_READ) & ~BMCR_PDOWN);

    txrxGate (false);

    // 1) erase previous misconfiguration
    if (_deviceFlags & DEV_HAS_POWER_CNTRL)
      MACReset ();
		
    // 4.1-1: stop adapter: ignored, 4.3 seems to be overkill
    _promiscuousMode = false;
    _multicastMode = false;
    writeRegister(NvRegMulticastAddrA, NVREG_MCASTADDRA_FORCE);
    writeRegister(NvRegMulticastAddrB, 0);
    writeRegister(NvRegMulticastMaskA, NVREG_MCASTMASKA_NONE);
    writeRegister(NvRegMulticastMaskB, NVREG_MCASTMASKB_NONE);
    writeRegister(NvRegPacketFilterFlags, 0);
		
    writeRegister(NvRegTransmitterControl, 0);
    writeRegister(NvRegReceiverControl, 0);
		
    writeRegister(NvRegAdapterControl, 0);
		
    if (_pauseFlags & NV_PAUSEFRAME_TX_CAPABLE)
      writeRegister (NvRegTxPauseFrame, NVREG_TX_PAUSEFRAME_DISABLE);
    
    // 2) initialize descriptor rings
		
    if (!initRing())
      break;

    writeRegister (NvRegLinkSpeed, 0);
    writeRegister (NvRegTransmitPoll, readRegister (NvRegTransmitPoll) & NVREG_TRANSMITPOLL_MAC_ADDR_REV);
    txrxReset ();
    writeRegister (NvRegUnknownSetupReg6, 0);
    _inShutdown = false;
		
    // 3) set mac address
    copyMacToHW();
		
    // 4) give hw rings
    setupHWRings (NV_SETUP_TX_RING | NV_SETUP_RX_RING);
    writeRegister(NvRegRingSizes, ((_rxRingLen - 1) << NVREG_RINGSZ_RXSHIFT) + ((_txRingLen - 1) << NVREG_RINGSZ_TXSHIFT));
		
    // 5) continue setup
    writeRegister(NvRegLinkSpeed, _linkspeed);
    if (_descVers == DESC_VER_1)
      writeRegister (NvRegTxWatermark, NVREG_TX_WM_DESC1_DEFAULT);
    else
      writeRegister (NvRegTxWatermark, NVREG_TX_WM_DESC2_3_DEFAULT);
    writeRegister (NvRegTxRxControl, _txrxCtlBits);
    writeRegister (NvRegVlanControl, _vlanCtlBits);
    pciPush ();
    writeRegister (NvRegTxRxControl, NVREG_TXRXCTL_BIT1 | _txrxCtlBits);
    regDelay (NvRegUnknownSetupReg5, NVREG_UNKSETUP5_BIT31, NVREG_UNKSETUP5_BIT31, NV_SETUP5_DELAY, NV_SETUP5_DELAYMAX,
              "SetupReg5, Bit 31 remained off");
    writeRegister (NvRegMIIMask, 0);
    writeRegister (NvRegIrqStatus, NVREG_IRQSTAT_MASK);
    writeRegister (NvRegMIIStatus, NVREG_MIISTAT_MASK_ALL);
		
    // 6) continue setup
    writeRegister(NvRegMisc1, NVREG_MISC1_FORCE | NVREG_MISC1_HD);
    writeRegister(NvRegTransmitterStatus, readRegister(NvRegTransmitterStatus));
    writeRegister(NvRegPacketFilterFlags, NVREG_PFF_ALWAYS);
    writeRegister(NvRegOffloadConfig, _rxBufferSize);
		
    writeRegister(NvRegReceiverStatus, readRegister(NvRegReceiverStatus));
		
    low = random() & NVREG_SLOTTIME_MASK;
    if (_descVers == DESC_VER_1) {
      writeRegister (NvRegSlotTime, low|NVREG_SLOTTIME_DEFAULT);
    } else {
      if (!(_deviceFlags & DEV_HAS_GEAR_MODE)) {
        /* setup legacy backoff */
        writeRegister (NvRegSlotTime, NVREG_SLOTTIME_LEGBF_ENABLED|NVREG_SLOTTIME_10_100_FULL|low);
      } else {
        writeRegister (NvRegSlotTime, NVREG_SLOTTIME_10_100_FULL);
        gearBackoffReseed();
      }
    }
		
    writeRegister (NvRegTxDeferral, NVREG_TX_DEFERRAL_DEFAULT);
    writeRegister (NvRegRxDeferral, NVREG_RX_DEFERRAL_DEFAULT);
		
    /* default to throughput */
    if (_optimizationMode == NV_OPTIMIZATION_MODE_THROUGHPUT)
      writeRegister (NvRegPollingInterval, NVREG_POLL_DEFAULT_THROUGHPUT);
    else
      writeRegister (NvRegPollingInterval, NVREG_POLL_DEFAULT_CPU);
    writeRegister (NvRegUnknownSetupReg6, NVREG_UNKSETUP6_VAL);
    writeRegister (NvRegAdapterControl, (_physicalAddress << NVREG_ADAPTCTL_PHYSHIFT) | NVREG_ADAPTCTL_PHYVALID | NVREG_ADAPTCTL_RUNNING);
    writeRegister (NvRegMIISpeed, NVREG_MIISPEED_BIT8 | NVREG_MIIDELAY);
    writeRegister (NvRegMIIMask, NVREG_MII_LINKCHANGE);
    if (_wolEnabled)
      writeRegister (NvRegWakeUpFlags, NVREG_WAKEUPFLAGS_ENABLE);
		
    i = readRegister(NvRegPowerState);
    if( (i & NVREG_POWERSTATE_POWEREDUP) == 0 )
      writeRegister(NvRegPowerState, NVREG_POWERSTATE_POWEREDUP | i);
    pciPush ();
    IODelay(10);
    writeRegister(NvRegPowerState, readRegister(NvRegPowerState) | NVREG_POWERSTATE_VALID);
		
    disableHWInterrupts (_irqMask);
		
    writeRegister(NvRegMIIStatus, NVREG_MIISTAT_MASK_ALL);
    writeRegister(NvRegIrqStatus, NVREG_IRQSTAT_MASK);
    pciPush ();
		
    mapInterrupts ();
    // Enable timer
		
    // Enable interrupts;
    enableHWInterrupts(_irqMask);
		
    writeRegister(NvRegMulticastAddrA, NVREG_MCASTADDRA_FORCE);
    writeRegister(NvRegMulticastAddrB, 0);
    writeRegister(NvRegMulticastMaskA, NVREG_MCASTMASKA_NONE);
    writeRegister(NvRegMulticastMaskB, NVREG_MCASTMASKB_NONE);
    writeRegister(NvRegPacketFilterFlags, NVREG_PFF_ALWAYS | NVREG_PFF_MYADDR);
		
    UInt32 miistat;
    miistat = readRegister(NvRegMIIStatus);
    writeRegister(NvRegMIIStatus, NVREG_MIISTAT_MASK_ALL);
    
    NVLOG_DEBUG (1, "starting transmit/receive engines. MII Status: %x\n", miistat);
    _linkspeed = 0;
    (void) updateLinkSpeed();
    startRxTx();
    
#if 0
    NVLOG_DEBUG (0, "Registers after initialization:\n");
    for (int __i = 0 ; __i < _registerSize / 64 + 1 ; __i++)
      for (int __j = 0 ; __j < 16 && (__j + __i) * 4 < _registerSize ; __j ++) {
        if (!__j)
          NVLOG (0, "%04x: %08x ", __i, readRegister(__i * 64 + __j * 4));
        else
          NVLOGC (0, "%08x%s", readRegister(__i * 64 + __j * 4), (__j == 15 ) ? "\n" : " ");
      }
    NVLOGC (0, "\n");
#endif
    
    IONetworkData *outputStatsData = _outputQueue->getStatisticsData();
    if (!outputStatsData || !(_outputStats = (IOOutputQueueStats*)outputStatsData->getBuffer()))
      break;
    
    _outputQueue->setCapacity (_txRingLen);
    _outputQueue->start();
    
    _inEnable = false;
    _ifEnabled = true;
    
    /* enable the stat timer */
    if (_nicStatTimer)
      _nicStatTimer->setTimeoutMS (STATS_INTERVAL);
    
    /* enable the link timer if one has been created */
    if (_nicLinkTimer)
      _nicLinkTimer->setTimeoutMS(LINK_TIMEOUT);
    
    _workLoop->enableAllInterrupts();

    return kIOReturnSuccess;
  } while (false);
  
  drainRing();
  _inEnable = false;
  if (_device && _device->isOpen ())
    _device->close(this);
  
  return kIOReturnError;
}

IOReturn nForceLAN::disable(IONetworkInterface *interface) {
  if (_ifEnabled == false || !_device)
    return kIOReturnSuccess;
	
  NVLOG_DEBUG (1, "disabling NIC (Power State Change = %s)...\n", _pmPowerStateChanged ? "True" : "False");
  _inShutdown = 1;

  /* disable timers */
  if (_nicLinkTimer)
    _nicLinkTimer->cancelTimeout ();
  
  if (_nicStatTimer)
    _nicStatTimer->cancelTimeout ();

  setLinkStatus(kIONetworkLinkValid, 0);
  stopRxTx();
  txrxReset();
	
  disableHWInterrupts (_irqMask);
	
  _outputQueue->setCapacity(0);
  _outputQueue->flush();
  _outputQueue->stop();
  
  drainRing();
  
  if ((_wolEnabled && _pmPowerStateChanged && _pmPowerState == kforcedethPowerStateOn) || !_phyPowerDown) {
    txrxGate (false);
    writeRegister (NvRegPacketFilterFlags, NVREG_PFF_ALWAYS|NVREG_PFF_MYADDR);
    startRx();
  } else {
    /* power down phy */
    miiRW(_physicalAddress, MII_BMCR, miiRW(_physicalAddress, MII_BMCR, MII_READ) | BMCR_PDOWN);

    txrxGate (true);
  }
	
  if (!((_wolEnabled && _pmPowerStateChanged && _pmPowerState == kforcedethPowerStateOn) || !_phyPowerDown)) {
    if (_device && _device->isOpen ())
      _device->close (this);
  }

  _ifEnabled = false;
  return kIOReturnSuccess;
}


void nForceLAN::resume (void) {
	UInt16 powerStateReg;
	
  NVLOG_DEBUG (1, "resuming after sleep\n");
	
  powerStateReg = kPCIPMCSPMEStatus | kPCIPMCSPowerStateD0;
  _device->configWrite16 (_pmPCICapabilityPtr + 4, powerStateReg);
  
  /* now restore the device's state */
  _device->restoreDeviceState();

  if (_deviceFlags & DEV_NEED_MSI_FIX)
    _device->configWrite32 (NV_MSI_PRIV_OFFSET, NV_MSI_PRIV_VALUE);
  
	/* restore phy state, including autoneg */
  phyInit ();
	
  _pmPowerStateChanged = false;
}

void nForceLAN::suspend (void) {
	UInt16 powerStateReg;

  NVLOG_DEBUG (1, "suspending before sleep\n");
	
  /* save the current device state */
  _device->saveDeviceState();
  
  powerStateReg = kPCIPMCSPMEStatus | kPCIPMCSPowerStateD3;
  if (_wolEnabled)
    powerStateReg |= kPCIPMCSPMEEnable;
  
  _device->configWrite16 (_pmPCICapabilityPtr + 4, powerStateReg);
  
  _pmPowerStateChanged = false;
}

void nForceLAN::restorePHY (void) {
	UInt16 phy_reserved, mii_control;
  
	if (_phyOui == PHY_OUI_REALTEK && _phyModel == PHY_MODEL_REALTEK_8201 && _phyCross == NV_CROSSOVER_DETECTION_DISABLED) {
		miiRW(_physicalAddress, PHY_REALTEK_INIT_REG1, PHY_REALTEK_INIT3);
		phy_reserved = miiRW(_physicalAddress, PHY_REALTEK_INIT_REG2, MII_READ);
		phy_reserved &= ~PHY_REALTEK_INIT_MSK1;
		phy_reserved |= PHY_REALTEK_INIT8;
		miiRW(_physicalAddress, PHY_REALTEK_INIT_REG2, phy_reserved);
		miiRW(_physicalAddress, PHY_REALTEK_INIT_REG1, PHY_REALTEK_INIT1);
    
		/* restart auto negotiation */
		mii_control = miiRW(_physicalAddress, MII_BMCR, MII_READ) | (BMCR_ANRESTART | BMCR_ANENABLE);
		miiRW(_physicalAddress, MII_BMCR, mii_control);
	}
}

void nForceLAN::restoreMAC (void) {  
	/* special op: write back the misordered MAC address - otherwise
	 * the next nv_probe would see a wrong address.
	 */
	writeRegister(NvRegMacAddrA, _origMac[0]);
	writeRegister(NvRegMacAddrB, _origMac[1]);
	writeRegister(NvRegTransmitPoll, readRegister(NvRegTransmitPoll) & ~NVREG_TRANSMITPOLL_MAC_ADDR_REV);
}

void nForceLAN::stop (IOService *provider) {
  NVLOG_DEBUG (1, "stopping\n");
  
  restoreMAC ();
  restorePHY ();
  
  super::stop(provider);
}

void nForceLAN::free(void) {
  int i;

  NVLOG_DEBUG (2, "freeing...\n");

  if (_nicPollTimer) {
    if (_workLoop)
      _workLoop->removeEventSource(_nicPollTimer);
    _nicPollTimer->release();
    _nicPollTimer = NULL;
  }

  if (_nicStatTimer) {
    if (_workLoop)
      _workLoop->removeEventSource(_nicStatTimer);
    _nicStatTimer->release();
    _nicStatTimer = NULL;
  }

  if (_nicLinkTimer) {
    if (_workLoop)
      _workLoop->removeEventSource(_nicLinkTimer);
    _nicLinkTimer->release();
    _nicLinkTimer = NULL;
  }
  
  for (i = 0 ; i < _numInterrupts ; i++)
    if (_interrupts[i]) {
      if (_workLoop)
        _workLoop->removeEventSource(_interrupts[i]);
      _interrupts[i]->release();
      _interrupts[i] = NULL;
    }

  _numInterrupts = 0;
	
  if (_map) {
    _map->release();
    _map = NULL;
  }

  /* _interface will automatically be detached once the last reference to it has been removed */
  if (_interface) {
    _interface->release ();
    _interface = NULL;
  }

  if (_device) {
    _device->release();
    _device = NULL;
  }

  if (_workLoop) {
    _workLoop->release();
    _workLoop = NULL;
  }

  if (_dictionary)
    _dictionary->release();

  if (_txMbufCursor)
    _txMbufCursor->release ();
  
  deallocateRings ();

  super::free ();
}

UInt32 nForceLAN::outputPacket(mbuf_t buf, void *param) {
  UInt32 checksumDemanded, extra_tx_flags, txFlags, maxSegments;
  UInt32 vlan_tag;
  unsigned int startNr, nr, entries, i;
  IOPhysicalSegment vector[TX_LIMIT_STOP];
  
  /* syncronization of the transmit queue is handled by IOBasicOutputQueue so no additional synchronization is needed here */
  
  /* set checksum flags */
  extra_tx_flags = 0;
  
  if (_hasHWCRC && _descVers != DESC_VER_1) {
    getChecksumDemand (buf, kChecksumFamilyTCPIP, &checksumDemanded);
    
    if (checksumDemanded & kChecksumIP)
      extra_tx_flags |= NV_TX2_CHECKSUM_L3;
    
    if ((checksumDemanded & kChecksumTCP) || (checksumDemanded & kChecksumUDP))
      extra_tx_flags |= NV_TX2_CHECKSUM_L4;
  }
  
  maxSegments = (_nextTx + _txRingLen - _nicTx) % _txRingLen - 1;
  if (maxSegments > TX_LIMIT_STOP)
    maxSegments = TX_LIMIT_STOP;
  
  if ((entries = _txMbufCursor->getPhysicalSegmentsWithCoalesce (buf, &vector[0], maxSegments)) == 0) {
    NVLOG (1, "can't fit packet segments in the tx ring, stalling.\n");
    return kIOReturnOutputStall;
  }
  
  NVLOG_DEBUG (2, "splitting packet into %d fragments\n", entries);
  
  startNr = _nextTx;
  /* don't set TX_VALID in the first fragment bit until all segments are setup */
  txFlags = 0;
  
  for (i = 0, nr = startNr ; i < entries ; i++) {
    _txRing[nr].m_buf  = NULL;
    _txRing[nr].f_size = vector[i].length;
    _txRing[nr].p_addr = vector[i].location;
    _txRing[nr].v_addr = NULL;
    
    setupRingDescriptor (_txRing, nr, (_txRing[nr].f_size - 1) | txFlags);
    
    txFlags = _txFlags;
    
    if (likely (i + 1 < entries))
        nr += likely (nr + 1 < _txRingLen) ? 1 : -nr;
  }
  
  /* increment _nextTx now (not earlier) to avoid potential race condition with txDone */
  _nextTx = (nr + 1 < _txRingLen) ? nr + 1 : 0;
  
  if (_txLimit && _useExtendedRing) {
    /* Limit the number of outstanding tx. Setup all fragments, but
     * do not set the VALID bit on the first descriptor. Save a pointer
     * to that descriptor and also for next skb_map element.
     */
    
    if (_txPktsInProgress == NV_TX_LIMIT_COUNT) {
      NVLOG_DEBUG (1, "tx packet limit exceeded. changing valid flag until there is more room\n");
      
      /* add the packet to the list of packets sent after the limit was reached */
      if (!_txChangeOwnerHead) {
        _txChangeOwnerHead = &_txRing[startNr];
        _txChangeOwnerHead->changeOwnerNext = NULL;
        _txChangeOwnerTail = _txChangeOwnerHead;
      } else {
        _txChangeOwnerTail->changeOwnerNext = &_txRing[startNr];
        _txRing[startNr].changeOwnerNext = NULL;
        _txChangeOwnerTail = _txChangeOwnerTail->changeOwnerNext;
      }
      
      /* remove VALID bit */
      txFlags &= ~NV_TX2_VALID;
    } else
      _txPktsInProgress++;
  }
 
  _txRing[startNr].m_buf = buf;
  /* now set TX_VALID and other bits */
  if (_useExtendedRing) {
    if (getVlanTagDemand (buf, &vlan_tag) == 0)
      _txRing[startNr].ex->TxVlan = OSSwapHostToLittleInt32 ((vlan_tag & 0xffff) | NV_TX3_VLAN_TAG_PRESENT);
    
    _txRing[nr].ex->FlagLen      |= OSSwapHostToLittleInt32 (NV_TX2_LASTPACKET);
    _txRing[startNr].ex->FlagLen |= OSSwapHostToLittleInt32 (extra_tx_flags | txFlags);    
  } else {
    _txRing[nr].orig->FlagLen      |= OSSwapHostToLittleInt32 (_descVers == DESC_VER_1 ? NV_TX_LASTPACKET : NV_TX2_LASTPACKET);
    _txRing[startNr].orig->FlagLen |= OSSwapHostToLittleInt32 (extra_tx_flags | txFlags);
  }
  
  writeRegister (NvRegTxRxControl, NVREG_TXRXCTL_KICK | _txrxCtlBits);
  
  return kIOReturnOutputSuccess;
}

/* memory allocation */
void nForceLAN::deallocateRings (void) {
  deallocateDescriptor (&_rxMemDesc);
  deallocateDescriptor (&_txMemDesc);

  if (_rxRing)
    IOFree (_rxRing, sizeof (struct io_rings) * _rxRingLen);
  if (_txRing)
    IOFree (_txRing, sizeof (struct io_rings) * _txRingLen);
	
  _rxRing = NULL;
  _txRing = NULL;
}

bool nForceLAN::allocateDescriptor (struct io_rings *ring, IOBufferMemoryDescriptor **memDescPtr, IOPhysicalAddress *phyMem, UInt32 ringLen) {
  int i;

  /*
    TODO -- IOMallocContiguous may return a 64-bit address which would be incompatible with older hardware. Use masked allocate to
    avoid this potential problem.
   */
  
  if (_useExtendedRing) {
    RingDescEx *descPtr;
    
		/* 40-bit addressing */
    *memDescPtr = IOBufferMemoryDescriptor::inTaskWithPhysicalMask (kernel_task, kIOMemoryPhysicallyContiguous, sizeof(RingDescEx) * ringLen, _physMemMask);
    if (!*memDescPtr)
      return false;

    descPtr = (RingDescEx *) (*memDescPtr)->getBytesNoCopy ();
		if (!descPtr)
			return false;

    memset ((void *)descPtr, 0, sizeof(RingDescEx) * ringLen);
	
    for (i = 0 ; i < ringLen ; i++)
      ring[i].ex = &(descPtr[i]);
  } else {
    RingDesc *descPtr;
		
		/* 32-bit addressing */
    *memDescPtr = IOBufferMemoryDescriptor::inTaskWithPhysicalMask (kernel_task, kIOMemoryPhysicallyContiguous, sizeof(RingDesc) * ringLen, _physMemMask);
    if (!*memDescPtr)
      return false;

    descPtr = (RingDesc *) (*memDescPtr)->getBytesNoCopy ();
		if (!descPtr)
			return false;
		
    memset ((void *)descPtr, 0, sizeof(RingDesc) * ringLen);
		
    for (i = 0 ; i < ringLen ; i++)
      ring[i].orig = &(descPtr[i]);
  }

  *phyMem = (*memDescPtr)->getPhysicalAddress ();

  return true;
}

void nForceLAN::deallocateDescriptor (IOBufferMemoryDescriptor **ppMemDesc) {
  if (ppMemDesc && *ppMemDesc) {
    (*ppMemDesc)->release ();
    *ppMemDesc = NULL;
  }
}

bool nForceLAN::getIORing (IOBufferMemoryDescriptor **pMemDesc, IOPhysicalAddress *pAddrPtr, struct io_rings **ringPtr, UInt32 ringLen) {
  *ringPtr = (struct io_rings *) IOMalloc (sizeof (struct io_rings) * ringLen);
  if (*ringPtr == NULL)
    return false;
	
  return allocateDescriptor (*ringPtr, pMemDesc, pAddrPtr, ringLen);
}

bool nForceLAN::allocateRings (void) {
  _txMbufCursor = IOMbufNaturalMemoryCursor::withSpecification(_mtu, TX_LIMIT_STOP);

  if (_txMbufCursor == NULL)
    return false;
	
  return getIORing (&_rxMemDesc, &_rxRingMem, &_rxRing, _rxRingLen) && getIORing (&_txMemDesc, &_txRingMem, &_txRing, _txRingLen);
}

/* The mgmt unit and driver use a semaphore to access the phy during init */
int nForceLAN::managementAcquireSemaphore (void) {
  int i;
  UInt32 tx_ctrl, mgmt_sema;

  for (i = 0 ; i < 10 ; i++) {
    mgmt_sema = readRegister(NvRegTransmitterControl) & NVREG_XMITCTL_MGMT_SEMA_MASK;
    if (mgmt_sema == NVREG_XMITCTL_MGMT_SEMA_FREE)
      break;
    IOSleep(500);
  }

  if (mgmt_sema != NVREG_XMITCTL_MGMT_SEMA_FREE)
    return 0;

  for (i = 0 ; i < 2 ; i++) {
    tx_ctrl = readRegister(NvRegTransmitterControl) | NVREG_XMITCTL_HOST_SEMA_ACQ;
    writeRegister(NvRegTransmitterControl, tx_ctrl);

    /* verify that semaphore was acquired */
    tx_ctrl = readRegister(NvRegTransmitterControl);
    if (((tx_ctrl & NVREG_XMITCTL_HOST_SEMA_MASK) == NVREG_XMITCTL_HOST_SEMA_ACQ) &&
        ((tx_ctrl & NVREG_XMITCTL_MGMT_SEMA_MASK) == NVREG_XMITCTL_MGMT_SEMA_FREE)) {
      _haveMgmtSema = true;
      return 1;
    }

    IODelay(50);
  }

  return 0;
}

void nForceLAN::managementReleaseSemaphore (void) {
  if (_deviceFlags & DEV_HAS_MGMT_UNIT) {
    if (_haveMgmtSema) {
      UInt32 tx_ctrl = readRegister (NvRegTransmitterControl);
      tx_ctrl &= ~NVREG_XMITCTL_HOST_SEMA_ACQ;
      writeRegister (NvRegTransmitterControl, tx_ctrl);
    }
  }
}

bool nForceLAN::managementGetVersion (void) {
  UInt32 data_ready = readRegister (NvRegTransmitterControl);
  UInt32 data_ready2 = 0;
  int tries;
  bool ready = false;

  writeRegister (NvRegMgmtUnitGetVersion, NVREG_MGMTUNITGETVERSION);
  writeRegister (NvRegTransmitterControl, data_ready ^ NVREG_XMITCTL_DATA_START);

  for (tries = 0 ; tries < 1000 ; tries++)  {
    data_ready2 = readRegister (NvRegTransmitterControl);
    if ((data_ready & NVREG_XMITCTL_DATA_READY) != (data_ready2 & NVREG_XMITCTL_DATA_READY)) {
      ready = true;
      break;
    }

    IOSleep (1);
  }

  if (!ready || (data_ready2 & NVREG_XMITCTL_DATA_ERROR))
    return false;

  _mgmtVersion = readRegister (NvRegMgmtUnitVersion) & NVREG_MGMTUNITVERSION;

  return true;
}

bool nForceLAN::mapMemoryRegisters (void) {
  assert (_device);

  _map = _device->mapDeviceMemoryWithRegister (kIOPCIConfigBaseAddress0);
  if (!_map) {
    NVLOG (0, "cannot map device memory.\n");
    return false;
  }

  if (_map->getLength () < _registerSize) {
    NVLOG (0, "could not find appropriate register window.");
    /* _map will be released by free() */
    return false;
  }

  NVLOG_DEBUG (1, "mapped from 0x%X of length %llu.\n", _device->configRead32(kIOPCIConfigBaseAddress0), _map->getLength());

  /* readRegister and writeRegister use the virtual address of the registers */
  _baseAddress = (char *)_map->getVirtualAddress();

  return true;
}

bool nForceLAN::getMACAddressFromHardware (void) {		
  UInt32 tx_reg;
  
  do {
    unsigned int macAddrBytes[6];
    if (!_macOverride)
      break;

    OSString *macAddr = (OSString *)getProperty(((_device->getDeviceNumber() % 2) == 0) ? "OverrideMAC1" : "OverrideMAC2");
    
    if (!macAddr) {
      NVLOG (0, "MAC override turned on but no MAC was provided\n");
      _macOverride = false;
      break;
    }
    
    const char *macString = macAddr->getCStringNoCopy();
    
    if (!macString) {
      _macOverride = false;
      break;
    }
      
    NVLOG (0, "overridding MAC address for device number %d with: %s\n", _device->getDeviceNumber(), macString);
      
    int ret = sscanf (macString, "%02x:%02x:%02x:%02x:%02x:%02x", macAddrBytes, macAddrBytes + 1,
                      macAddrBytes + 2, macAddrBytes + 3, macAddrBytes + 4, macAddrBytes + 5);

    _macAddr.bytes[0] = macAddrBytes[0];
    _macAddr.bytes[1] = macAddrBytes[1];
    _macAddr.bytes[2] = macAddrBytes[2];
    _macAddr.bytes[3] = macAddrBytes[3];
    _macAddr.bytes[4] = macAddrBytes[4];
    _macAddr.bytes[5] = macAddrBytes[5];
    
    if (ret != 6) {
      NVLOG (0, "malformed mac override: %s\n", macString);
      _macOverride = false;
    }
  } while (0);

  _origMac[0] = readRegister(NvRegMacAddrA);
  _origMac[1] = readRegister(NvRegMacAddrB);
  
  /* check the workaround bit for correct mac address order */
  tx_reg = readRegister (NvRegTransmitPoll);

  if (!(_deviceFlags & DEV_HAS_CORRECT_MACADDR) && (tx_reg & NVREG_TRANSMITPOLL_MAC_ADDR_REV)) {
    unsigned char tmpBytes[6];

    tmpBytes[0] = (_origMac[0] >>  0) & 0xff;
    tmpBytes[1] = (_origMac[0] >>  8) & 0xff;
    tmpBytes[2] = (_origMac[0] >> 16) & 0xff;
    tmpBytes[3] = (_origMac[0] >> 24) & 0xff;
    tmpBytes[4] = (_origMac[1] >>  0) & 0xff;
    tmpBytes[5] = (_origMac[1] >>  8) & 0xff;
    /*
     * Set orig mac address back to the reversed version.
     * This flag will be cleared during low power transition.
     * Therefore, we should always put back the reversed address.
     */
    _origMac[0] = (tmpBytes[5] << 0) + (tmpBytes[4] << 8) + (tmpBytes[3] << 16) + (tmpBytes[2] << 24);
    _origMac[1] = (tmpBytes[1] << 0) + (tmpBytes[0] << 8);
  }
  
  if (!_macOverride) {
    if ((_deviceFlags & DEV_HAS_CORRECT_MACADDR) || (tx_reg & NVREG_TRANSMITPOLL_MAC_ADDR_REV)) {
      /* mac address is already in correct order */
      _macAddr.bytes[0] = (_origMac[0] >> 0) & 0xff;
      _macAddr.bytes[1] = (_origMac[0] >> 8) & 0xff;
      _macAddr.bytes[2] = (_origMac[0] >> 16) & 0xff;
      _macAddr.bytes[3] = (_origMac[0] >> 24) & 0xff;
      _macAddr.bytes[4] = (_origMac[1] >> 0) & 0xff;
      _macAddr.bytes[5] = (_origMac[1] >> 8) & 0xff;
    } else {
      _macAddr.bytes[0] = (_origMac[1] >> 8) & 0xff;
      _macAddr.bytes[1] = (_origMac[1] >> 0) & 0xff;
      _macAddr.bytes[2] = (_origMac[0] >> 24) & 0xff;
      _macAddr.bytes[3] = (_origMac[0] >> 16) & 0xff;
      _macAddr.bytes[4] = (_origMac[0] >> 8) & 0xff;
      _macAddr.bytes[5] = (_origMac[0] >> 0) & 0xff;
    }
    
    if ((_origMac[0] == 0 && _macAddr.bytes[0] == 0 && _macAddr.bytes[1] == 0) ||
        (_origMac[0] == 0xffffffff && _macAddr.bytes[0] == 0xff && _macAddr.bytes[1] == 0xff) ||
        (_macAddr.bytes[0] == 0x04 && _macAddr.bytes[1] == 0x4b && _macAddr.bytes[2] == 0x80)) {
      OSString *acpiPath;
      UInt32 deviceSignature;
      
      NVLOG (1, "MAC address %02X:%02X:%02X:%02X:%02X:%02X given by device is not valid. Generating a pseudo-random MAC....\n",
             _macAddr.bytes[0], _macAddr.bytes[1], _macAddr.bytes[2], _macAddr.bytes[3], _macAddr.bytes[4], _macAddr.bytes[5]);
      
      _macAddr.bytes[0] = 0x00;
      _macAddr.bytes[1] = 0x00;
      _macAddr.bytes[3] = 0x6c;
      
      /* MAC address needs to be consistent on each boot so use data from the IORegistry to generate the pseudo-random MAC. */
      acpiPath = (OSString *)_device->getProperty ("acpi-path");
      if (acpiPath) {
        NVLOG_DEBUG (0, "acpi-path: %s, length = %lu\n", acpiPath->getCStringNoCopy (), strlen (acpiPath->getCStringNoCopy ()));
      
        deviceSignature = crc32 (crc32 (0, NULL, 0), acpiPath->getCStringNoCopy (), strlen (acpiPath->getCStringNoCopy ()));
        deviceSignature += _vendorID + _deviceID;
      
        NVLOG_DEBUG (0, "device signature: %08x\n", deviceSignature);
      
        /* Use the first and last byte in the MAC */
        _macAddr.bytes[4] = deviceSignature & 0xff;
        _macAddr.bytes[5] = deviceSignature >> 24;
      }
    }
  }

  NVLOG_DEBUG (0, "found nForce LAN with MAC: %02X:%02X:%02X:%02X:%02X:%02X.\n",
               _macAddr.bytes[0], _macAddr.bytes[1], _macAddr.bytes[2], _macAddr.bytes[3],
               _macAddr.bytes[4], _macAddr.bytes[5]);	

  return true;
}

bool nForceLAN::getAndInitializePhy (void) {
  bool physicalInitialized;
  UInt32 phystate, phystate_orig;
  int i;

  /* clear phy state and temporarily halt phy interrupts */
  writeRegister(NvRegMIIMask, 0);
  phystate = readRegister(NvRegAdapterControl);
  if (phystate & NVREG_ADAPTCTL_RUNNING) {
    phystate_orig = 1;
    phystate &= ~NVREG_ADAPTCTL_RUNNING;
    writeRegister(NvRegAdapterControl, phystate);
  }
  writeRegister (NvRegMIIStatus, NVREG_MIISTAT_MASK_ALL);

  physicalInitialized = false;
  if (_deviceFlags & DEV_HAS_MGMT_UNIT) {
    /* management unit running on the mac? */
    if ((readRegister(NvRegTransmitterControl) & NVREG_XMITCTL_MGMT_ST) &&
	(readRegister(NvRegTransmitterControl) & NVREG_XMITCTL_SYNC_PHY_INIT) &&
	managementAcquireSemaphore() && managementGetVersion()) {
      _macInUse = (_mgmtVersion > 0) ? readRegister (NvRegMgmtUnitControl) & NVREG_MGMTUNITCONTROL_INUSE : 1;

      NVLOG_DEBUG (1, "mgmt unit is running. mac in use %x.\n", _macInUse);

      if (_macInUse && ((readRegister (NvRegTransmitterControl) & NVREG_XMITCTL_SYNC_MASK) == NVREG_XMITCTL_SYNC_PHY_INIT)) {
          /* phy is inited by mgmt unit */
          physicalInitialized = true;
          NVLOG_DEBUG (1, "phy already initialized by mgmt unit.\n");
      } /* else: we need to init the phy */
    }
  }
 
  _physicalAddress = 0;
  _phyOui = 0;

  for (i = 1 ; _physicalAddress == 0 && i <= 32 ; i++) {
    int id1, id2;
    int nPhyAddr = i & 0x1F;

    id1 = miiRW(nPhyAddr, MII_PHYSID1, MII_READ);
    id2 = miiRW(nPhyAddr, MII_PHYSID2, MII_READ);;

    if (id1 < 0 || id1 == 0xffff || id2 < 0 || id2 == 0xffff)
      continue;

    _phyModel = id2 & PHYID2_MODEL_MASK;

    id1 = (id1 & PHYID1_OUI_MASK) << PHYID1_OUI_SHFT;
    id2 = (id2 & PHYID2_OUI_MASK) >> PHYID2_OUI_SHFT;

    NVLOG_DEBUG (1, "found PHY 0x%04x:0x%04x at address %d.\n", id1, id2, nPhyAddr);
    _physicalAddress = nPhyAddr;
    _phyOui = id1 | id2;

    /* Realtek hardcoded phy id1 to all zero's on certain phys */
    if (_phyOui == PHY_OUI_REALTEK2)
      _phyOui = PHY_OUI_REALTEK;
    /* Setup phy revision for Realtek */
    if (_phyOui == PHY_OUI_REALTEK && _phyModel == PHY_MODEL_REALTEK_8211)
      _phyRev = miiRW(_physicalAddress, MII_RESV1, MII_READ) & PHY_REV_MASK;
  }

  do {
    if (_phyOui == 0 && _physicalAddress == 0) {
      NVLOG (0, "could not find a valid PHY.\n");
      break;
    }

#if defined(DUMP_REGISTERS)
    int j, m;
    
    NVLOG (0, "Oui = %x, Model = %x, Revision = %x\n", _phyOui, _phyModel, _phyRev);
    NVLOG (0, "Dumping contents of lower mii registers before initialization\n");
    
    for (j = 0 ; j < 4 ; j++)
      for (m = 0 ; m < 8 ; m++)
        if (!m)
          NVLOG (0, "%04x: %08x ", j * 8, miiRW(_physicalAddress, j*8 + m, MII_READ));
        else
          NVLOGC (0, "%08x%s", miiRW(_physicalAddress, j*8 + m, MII_READ), (m == 7) ? "\n" : " ");
#endif

    if (!physicalInitialized) {
      if (phyInit() != 0)
        break;
    } else
      /* see if it is a gigabit phy */
      _gigabit = miiRW (_physicalAddress, MII_BMSR, MII_READ) & PHY_GIGABIT;

    return true;
  } while (0);
  
  if (phystate_orig)
    writeRegister (NvRegAdapterControl, phystate | NVREG_ADAPTCTL_RUNNING);

  return false;
}

void nForceLAN::enableHWInterrupts (UInt32 mask) {
  writeRegister (NvRegIrqMask, mask);
  
  NVLOG (0, "enabling interrupts with mask: 0x%08x and msiEnable: 0x%08x\n", (unsigned int)mask, (unsigned int)_msiEnable);

  if (_msiFlags & NV_MSI_ENABLED)
    writeRegister (NvRegMSIIrqMask, _msiEnable);
	
  pciPush ();
}

void nForceLAN::disableHWInterrupts (UInt32 mask) {
  if (_msiFlags & NV_MSI_X_ENABLED)
    writeRegister (NvRegIrqMask, mask);
  else {
    if (_msiFlags & NV_MSI_ENABLED)
      writeRegister (NvRegMSIIrqMask, 0);
    writeRegister (NvRegIrqMask, 0);
  }
	
  pciPush ();
}

IOReturn nForceLAN::selectMedium (const IONetworkMedium * medium) {
  IOMediumType type;

  NVLOG_DEBUG (1, "selectMedium (%s)\n", medium->getName()->getCStringNoCopy());
	
  if (!OSDynamicCast (IONetworkMedium, medium))
    /* use auto by default */
    medium = IONetworkMedium::getMediumWithType (_dictionary, kIOMediumEthernetAuto);

  if (!medium)
    return kIOReturnError;

  type = medium->getType ();
  _autoneg = false;

  if (type & kIOMediumOptionFlowControl) {
    NVLOG_DEBUG (1, "turning on flow control\n");
    updatePause (NV_PAUSEFRAME_RX_ENABLE | NV_PAUSEFRAME_TX_ENABLE);
  } else
    updatePause (0);
  
  type &= ~kIOMediumOptionFlowControl;
  type &= ~kIOMediumOptionLoopback;
  
  switch (type) {
    case kIOMediumEthernet1000BaseT | kIOMediumOptionFullDuplex:
      _fixedMode = LPA_1000FULL;
      break;
    case kIOMediumEthernet100BaseT2 | kIOMediumOptionFullDuplex:
    case kIOMediumEthernet100BaseTX | kIOMediumOptionFullDuplex:
      _fixedMode = LPA_100FULL;
      break;
    case kIOMediumEthernet100BaseT2 | kIOMediumOptionHalfDuplex:
    case kIOMediumEthernet100BaseTX | kIOMediumOptionHalfDuplex:
    case kIOMediumEthernet100BaseT4:
      _fixedMode = LPA_100HALF;
      break;
    case kIOMediumEthernet10BaseT | kIOMediumOptionHalfDuplex:
      _fixedMode = LPA_10HALF;
      break;
    case kIOMediumEthernet10BaseT | kIOMediumOptionFullDuplex:
      _fixedMode = LPA_10FULL;
      break;
    default:
      _fixedMode = 0;
      _autoneg = true;
  }
  
  NVLOG_DEBUG (1, "fixedMode = %d autoneg = %d\n", _fixedMode, _autoneg);
	
  setCurrentMedium (medium);
  updateLinkSpeed();
	
  return kIOReturnSuccess;
}

bool nForceLAN::configureInterface (IONetworkInterface *interface) {
  IONetworkData *data;

  if (super::configureInterface(interface) == false)
    return false;

  data = interface->getNetworkData(kIONetworkStatsKey);
  if (!data || !(_stats = (IONetworkStats*)data->getBuffer()))
    return false;
	
  data = interface->getNetworkData (kIOEthernetStatsKey);
  if (!data  || !(_ethernetStats = (IOEthernetStats *) data->getBuffer()))
    return false;
	
  return true;
}

void nForceLAN::setLowSpeed (void) {
  int adv = 0;
  int lpa = 0;
  int adv_lpa, bmcr, tries = 0;
  int mii_status;
  UInt32 control_1000;
	
  if (!_autoneg || ((_linkspeed & 0xFFF) != NVREG_LINKSPEED_1000))
    return;
	
  NVLOG_DEBUG (1, "setting low speed mode to conserve power\n");
	
  adv = miiRW (_physicalAddress, MII_ADVERTISE, MII_READ);
  lpa = miiRW (_physicalAddress, MII_LPA, MII_READ);
  control_1000 = miiRW (_physicalAddress, MII_CTRL1000, MII_READ);
	
  adv_lpa = lpa & adv;
	
  if ((adv_lpa & LPA_10FULL) || (adv_lpa & LPA_10HALF)) {
    adv &= ~(ADVERTISE_100BASE4 | ADVERTISE_100FULL | ADVERTISE_100HALF);
    control_1000 &= ~(ADVERTISE_1000FULL|ADVERTISE_1000HALF);
    NVLOG_DEBUG (1, "set low speed to 10mbs\n");
  } else if ((adv_lpa & LPA_100FULL) || (adv_lpa & LPA_100HALF))
    control_1000 &= ~(ADVERTISE_1000FULL|ADVERTISE_1000HALF);
  else
    return;
	
  /* set new advertisements */
  miiRW (_physicalAddress, MII_ADVERTISE, adv);
  miiRW (_physicalAddress, MII_CTRL1000, control_1000);
	
  bmcr = miiRW (_physicalAddress, MII_BMCR, MII_READ) | BMCR_ANENABLE;
  if (_phyModel == PHY_MODEL_MARVELL_E3016) {
    /* reset the phy in order for settings to stick,
     * and cause autoneg to start */
    if (phyReset (bmcr)) {
      NVLOG (1, "phy reset failed\n");
      return;
    }
  } else
    miiRW (_physicalAddress, MII_BMCR, bmcr | BMCR_ANRESTART);
	
  miiRW (_physicalAddress, MII_BMSR, MII_READ);
  mii_status = miiRW(_physicalAddress, MII_BMSR, MII_READ);
  while (!(mii_status & BMSR_ANEGCOMPLETE)) {
    IOSleep (100);
    mii_status = miiRW (_physicalAddress, MII_BMSR, MII_READ);
    if (tries++ > 50)
      break;
  }
	
  updateLinkSpeed ();
}

UInt32 nForceLAN::getFeatures (void) const {
  return kIONetworkFeatureHardwareVlan | 0x8/* | kIONetworkFeatureMultiPages */;
}

IOReturn nForceLAN::getHardwareAddress(IOEthernetAddress * addrP) {
  if (_map) {
    *addrP = _macAddr;
    return kIOReturnSuccess;
  }

  return kIOReturnError;
}

IOReturn nForceLAN::setHardwareAddress(const IOEthernetAddress * addrP) {
  bool enabled = _ifEnabled;

  if (enabled)
    disable (_interface);

  _macAddr = *addrP;;

	/* enable will copy the MAC addr to the hardware */
  if (enabled)
    enable (_interface);

  return kIOReturnSuccess;
}

IOOutputQueue *nForceLAN::createOutputQueue () {
  /* use a basic output queue to protect outputPacket yet allow tx/rx to occur at the same time */
  /* XXX -- is there a good reason for passing an initial size? */
  _outputQueue = IOBasicOutputQueue::withTarget (this, TX_LIMIT_START);
  return _outputQueue;
}

IOReturn nForceLAN::getChecksumSupport (UInt32 *checksumMask, UInt32 checksumFamily, bool isOutput) {
	*checksumMask = 0;

  if (checksumFamily != kChecksumFamilyTCPIP)
    NVLOG_DEBUG (0, "operating system wants information for unknown checksum family.\n");
  else if (_hasHWCRC)
    /* when checksum offloading is available it is available for both rx/tx */
    *checksumMask = kChecksumIP | kChecksumTCP | kChecksumUDP;

  return kIOReturnSuccess;
}

IOReturn nForceLAN::setMode (bool multicast, bool promiscuous) {
  UInt32 addr[2] = {0, 0};
  UInt32 mask[2];
  UInt32 pff = readRegister (NvRegPacketFilterFlags) & NVREG_PFF_PAUSE_RX;

  NVLOG_DEBUG (1, "setMode (%s, %s)\n", (multicast) ? "true" : "false", (promiscuous) ? "true" : "false");

  pff |= promiscuous ? NVREG_PFF_PROMISC : NVREG_PFF_MYADDR;
	
  if (!promiscuous && (multicast || _mcList)) {
    UInt32 alwaysOff[2];
    UInt32 alwaysOn[2];

    if (multicast) {
      alwaysOn[0]  = alwaysOn[1] = alwaysOff[0] = alwaysOff[1] = 0;
    } else {
      alwaysOn[0]  = _alwaysOn[0];
      alwaysOn[1]  = _alwaysOn[1];
      alwaysOff[0] = _alwaysOff[0];
      alwaysOff[1] = _alwaysOff[1];
    }

    addr[0] = alwaysOn[0];
    addr[1] = alwaysOn[1];
    mask[0] = alwaysOn[0] | alwaysOff[0];
    mask[1] = alwaysOn[1] | alwaysOff[1];
  } else {
    mask[0] = NVREG_MCASTMASKA_NONE;
    mask[1] = NVREG_MCASTMASKB_NONE;
  }

  addr[0] |= NVREG_MCASTADDRA_FORCE;
  pff |= NVREG_PFF_ALWAYS;

  stopRx ();
	
  writeRegister (NvRegMulticastAddrA, addr[0]);
  writeRegister (NvRegMulticastAddrB, addr[1]);
  writeRegister (NvRegMulticastMaskA, mask[0]);
  writeRegister (NvRegMulticastMaskB, mask[1]);
  writeRegister (NvRegPacketFilterFlags, pff);
		
  startRx();

  return kIOReturnSuccess;
}

IOReturn nForceLAN::setMulticastMode(bool active) {
  IOReturn ret = setMode (active, _promiscuousMode);

  if (ret == kIOReturnSuccess)
    _multicastMode = active;
	
  return ret;
}

IOReturn nForceLAN::setPromiscuousMode(bool active) {
  IOReturn ret = setMode (_multicastMode, active);

  if (ret == kIOReturnSuccess)
    _promiscuousMode = active;
	
  return ret;
}

IOReturn nForceLAN::setMaxPacketSize (UInt32 maxSize) {
  NVLOG_DEBUG (1, "setting mtu to %d\n", maxSize);
	
  if (_mtu == maxSize)
    return kIOReturnSuccess;
	
  _mtu = maxSize;
	
  if (_ifEnabled && !_inEnable) {
    /*
     * It seems that the nic preloads valid ring entries into an
     * internal buffer. The procedure for flushing everything is
     * guessed, there is probably a simpler approach.
     * Changing the MTU is a rare event, so it shouldn't matter.
     */
		
    disableHWInterrupts (_irqMask);
    /* stop engines */
    stopRxTx ();
    txrxReset ();
    /* drain rx queue */
    drainRing ();
    /* reinit driver view of the rx queue */
    setBufSize ();
		
    if (!initRing () && !_inShutdown)
      _nicPollTimer->setTimeoutMS(OOM_REFILL);
		
    writeRegister (NvRegOffloadConfig, _rxBufferSize);
    setupHWRings (NV_SETUP_RX_RING | NV_SETUP_TX_RING);
    writeRegister (NvRegRingSizes, ((_rxRingLen - 1) << NVREG_RINGSZ_RXSHIFT) + ((_txRingLen - 1) << NVREG_RINGSZ_TXSHIFT));
    pciPush ();
		
    writeRegister (NvRegTxRxControl, _txrxCtlBits | NVREG_TXRXCTL_KICK);
    pciPush ();
		
    /* restart rx engine */
    startRxTx ();
    enableHWInterrupts (_irqMask);
  } else
    /* go ahead and set the internal buffer size */
    setBufSize ();
	
  return kIOReturnSuccess;
}

IOReturn nForceLAN::getMaxPacketSize (UInt32 *maxSize) const {
  if (maxSize)
    *maxSize = _packetLimit;
	
  NVLOG_DEBUG (1, "reporting max packet size as: %d\n", _packetLimit);
	
  return kIOReturnSuccess;
}

IOReturn nForceLAN::getMinPacketSize (UInt32 *minSize) const {
  if (minSize)
    *minSize = 1500;
	
  NVLOG_DEBUG (1, "reporting min packet size as: %d\n", 1500);
	
  return kIOReturnSuccess;
}

IOReturn nForceLAN::setMulticastList(IOEthernetAddress *addrs, UInt32 count) {
  UInt32 a, b;
  int i;

  _alwaysOn[0] = _alwaysOn[1] = _alwaysOff[0] = _alwaysOff[1] = 0xffffffff;
	
  _mcList = count;
	
  for (i = 0; i < count; i++) {
    a = (addrs[i].bytes[0] << 0) + (addrs[i].bytes[1] << 8) + (addrs[i].bytes[2] << 16) + (addrs[i].bytes[3] << 24);
    b = (addrs[i].bytes[4] << 0) + (addrs[i].bytes[5] << 8);
		
    _alwaysOn[0]  &= a;
    _alwaysOff[0] &= ~a;
    _alwaysOn[1]  &= b;
    _alwaysOff[1] &= ~b;
  }
	
  setMode (_multicastMode, _promiscuousMode);
	
  return kIOReturnSuccess;
}
