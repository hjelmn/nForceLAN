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

void nForceLAN::saveLEDStats (void) {
	int i;
	
	miiRW (_physicalAddress, Mv_Page_Address, 3);
	IODelay (5);
	
	for (i = 0 ; i < 3 ; i++) {
		_ledStats[i] = miiRW (_physicalAddress, Mv_LED_Control + i, MII_READ);	
		NVLOG_DEBUG (2, "save LED reg%d: value=0x%x\n", (unsigned int)Mv_LED_Control + i, (unsigned int)_ledStats[i]);
	}
}

void nForceLAN::restoreLEDStats (void) {
	int i;
	
	miiRW (_physicalAddress, Mv_Page_Address, 3);
	IODelay (5);
	
	for (i = 0 ; i < 3 ; i++) {
		miiRW (_physicalAddress, Mv_LED_Control + i, _ledStats[i]);	
		IODelay (1);
		NVLOG_DEBUG (2, "restore LED reg%d: value=0x%x\n", (unsigned int)Mv_LED_Control + i, (unsigned int)_ledStats[i]);
	}
}

void nForceLAN::LEDOn (void) {
	LEDSetState (Mv_LED_DUAL_MODE3);
}

void nForceLAN::LEDOff (void) {
	LEDSetState (Mv_LED_FORCE_OFF);
	IODelay (1);
}

void nForceLAN::LEDSetState (UInt32 state) {
	NVLOG_DEBUG (2, "setting LED state to %d\n", (unsigned int)state);
	
	miiRW (_physicalAddress, Mv_Page_Address, 3);
	IODelay (5);
	
	miiRW (_physicalAddress, Mv_LED_Control, state);	
}

bool nForceLAN::regDelay(int offset, UInt32 mask, UInt32 target, int delay, int delayMax, const char *msg) {
	pciPush ();
	do {
		IODelay(delay);
		delayMax -= delay;
		if( delayMax < 0 ) {
			if( msg )
				NVLOG (0, "%s\n", msg);
			return true;
		}
	} while ((readRegister(offset) & mask) != target);

	return false;
}

int nForceLAN::miiRW (int addr, int miireg, int value) {
	UInt32 reg;

	writeRegister (NvRegMIIStatus, NVREG_MIISTAT_MASK_RW);
	reg = readRegister(NvRegMIIControl);

	if (reg & NVREG_MIICTL_INUSE) {
		writeRegister(NvRegMIIControl, NVREG_MIICTL_INUSE);
		IODelay(NV_MIIBUSY_DELAY);
	}

	reg = (addr << NVREG_MIICTL_ADDRSHIFT) | miireg;
	if (value != MII_READ) {
		writeRegister(NvRegMIIData, value);
		reg |= NVREG_MIICTL_WRITE;
	}

	writeRegister(NvRegMIIControl, reg);

	if (regDelay(NvRegMIIControl, NVREG_MIICTL_INUSE, 0, NV_MIIPHY_DELAY, NV_MIIPHY_DELAYMAX, NULL))
		return -1;
	else if( value != MII_READ )
		return 0;
	else if( readRegister(NvRegMIIStatus) & NVREG_MIISTAT_ERROR )
		return -1;
	else
		return readRegister(NvRegMIIData);
}

void nForceLAN::copyMacToHW () {
	UInt32 mac[2];
	
	mac[0] = (_macAddr.bytes[0] << 0) + (_macAddr.bytes[1] << 8) + (_macAddr.bytes[2] << 16) + (_macAddr.bytes[3] << 24);
	mac[1] = (_macAddr.bytes[4] << 0) + (_macAddr.bytes[5] << 8);
	writeRegister(NvRegMacAddrA, mac[0]);
	writeRegister(NvRegMacAddrB, mac[1]);
}


void nForceLAN::setBufSize () {
	if (_mtu > _packetLimit)
    /* this will never happen unless Apple ignores the result of getMaxPacketSize or we set our initial mtu too high */
		_mtu = _packetLimit;
		
	if (_mtu < kIOEthernetMaxPacketSize && kIOEthernetMaxPacketSize < _packetLimit)
    /* this also SHOULD not happen */
		_rxBufferSize = kIOEthernetMaxPacketSize + NV_RX_HEADERS;
	else
		_rxBufferSize = _mtu + NV_RX_HEADERS;
	
	NVLOG_DEBUG (1, "setBufSize/Setting with regards to mtu(%d) -- %d bytes\n", (unsigned int)_mtu, (unsigned int)_rxBufferSize);
}

void nForceLAN::updatePause (UInt32 pause_flags) {
	_pauseFlags &= ~(NV_PAUSEFRAME_TX_ENABLE | NV_PAUSEFRAME_RX_ENABLE);

	if (_pauseFlags & NV_PAUSEFRAME_RX_CAPABLE) {
		UInt32 pff = readRegister (NvRegPacketFilterFlags) & ~NVREG_PFF_PAUSE_RX;
		if (pause_flags & NV_PAUSEFRAME_RX_ENABLE) {
			writeRegister (NvRegPacketFilterFlags, pff|NVREG_PFF_PAUSE_RX);
			_pauseFlags |= NV_PAUSEFRAME_RX_ENABLE;
		} else
			writeRegister (NvRegPacketFilterFlags, pff);
	}
	if (_pauseFlags & NV_PAUSEFRAME_TX_CAPABLE) {
		UInt32 regmisc = readRegister (NvRegMisc1) & ~NVREG_MISC1_PAUSE_TX;
		if (pause_flags & NV_PAUSEFRAME_TX_ENABLE) {
			UInt32 pause_enable = NVREG_TX_PAUSEFRAME_ENABLE_V1;
			if (_deviceFlags & DEV_HAS_PAUSEFRAME_TX_V2)
				pause_enable = NVREG_TX_PAUSEFRAME_ENABLE_V2;
			if (_deviceFlags & DEV_HAS_PAUSEFRAME_TX_V3)
				pause_enable = NVREG_TX_PAUSEFRAME_ENABLE_V3;
			writeRegister (NvRegTxPauseFrame, pause_enable);
			writeRegister (NvRegMisc1, regmisc|NVREG_MISC1_PAUSE_TX);
			_pauseFlags |= NV_PAUSEFRAME_TX_ENABLE;
		} else {
			writeRegister (NvRegTxPauseFrame, NVREG_TX_PAUSEFRAME_DISABLE);
			writeRegister (NvRegMisc1, regmisc);
		}
	}
}

bool nForceLAN::updateLinkSpeed (void) {
	int miiStatus;
	int adv = 0, lpa;
	int newls;
	bool newdup;
	bool retval;
	int txrx_flags = 0;
	int speed = 0;
	IOMediumType type = kIOMediumEthernetNone;

	UInt32 controlGigabit, statusGigabit, phyReg, txreg, adv_pause, pause_flags, phy_exp, lpa_pause;
	
	miiRW(_physicalAddress, MII_BMSR, MII_READ);
	miiStatus = miiRW(_physicalAddress, MII_BMSR, MII_READ);
	
	retval = false;

  NVLOG_DEBUG (0, "updating link speed. BMSR status: %x\n", miiStatus);
  
	do {
		if (!(miiStatus & BMSR_LSTATUS)) {
			newls = NVREG_LINKSPEED_FORCE | NVREG_LINKSPEED_10;
			newdup = false;
			break;
		}

    if (_autoneg) {
      if (!(miiStatus & BMSR_ANEGCOMPLETE)) {
        newls = NVREG_LINKSPEED_FORCE | NVREG_LINKSPEED_10;
        newdup = false;
        break;
      }

      adv = miiRW(_physicalAddress, MII_ADVERTISE, MII_READ);
      lpa = miiRW(_physicalAddress, MII_LPA, MII_READ);

      if (_gigabit == PHY_GIGABIT) {
        controlGigabit = miiRW(_physicalAddress, MII_CTRL1000, MII_READ);
        statusGigabit = miiRW(_physicalAddress, MII_STAT1000, MII_READ);
        
        if ((controlGigabit & ADVERTISE_1000FULL) && (statusGigabit & LPA_1000FULL)) {
          newls = NVREG_LINKSPEED_FORCE | NVREG_LINKSPEED_1000;
          retval = true;
          newdup = true;
          break;
        }
      }
      
      lpa = lpa & adv;
    } else {
      lpa = _fixedMode;

      if (_gigabit == PHY_GIGABIT && _fixedMode & LPA_1000FULL) {
        newls = NVREG_LINKSPEED_FORCE | NVREG_LINKSPEED_1000;
        retval = true;
        newdup = true;
        break;
      }
    }
    
    retval = true;
    
		// FIXME: handle parallel detection properly
    if (lpa & LPA_100FULL) {
			newls = NVREG_LINKSPEED_FORCE | NVREG_LINKSPEED_100;
			newdup = true;
		} else if (lpa & LPA_100HALF) {
			newls = NVREG_LINKSPEED_FORCE | NVREG_LINKSPEED_100;
			newdup = false;
		} else if (lpa & LPA_10FULL) {
			newls = NVREG_LINKSPEED_FORCE | NVREG_LINKSPEED_10;
			newdup = true;
		} else if (lpa & LPA_10HALF) {
			newls = NVREG_LINKSPEED_FORCE | NVREG_LINKSPEED_10;
			newdup = false;
		} else {
			newls = NVREG_LINKSPEED_FORCE | NVREG_LINKSPEED_10;
			newdup = true;
		}
	} while (false);

	if (_duplex == newdup && _linkspeed == newls)
		return retval;

	_duplex = newdup;
	_linkspeed = newls;

	/* The transmitter and receiver must be restarted for safe update */
	if (readRegister (NvRegTransmitterControl) & NVREG_XMITCTL_START) {
		txrx_flags |= NV_RESTART_TX;
		stopTx();
	}
	if (readRegister (NvRegReceiverControl) & NVREG_RCVCTL_START) {
		txrx_flags |= NV_RESTART_RX;
		stopRx();
	}

	if (_gigabit == PHY_GIGABIT) {
		phyReg = readRegister(NvRegSlotTime) & ~(0x3FF00);
		if (((_linkspeed & 0xFFF) == NVREG_LINKSPEED_10) || ((_linkspeed & 0xFFF) == NVREG_LINKSPEED_100))
			phyReg |= NVREG_SLOTTIME_10_100_FULL;
		else if ((_linkspeed & 0xFFF) == NVREG_LINKSPEED_1000)
			phyReg |= NVREG_SLOTTIME_1000_FULL;

		writeRegister(NvRegSlotTime, phyReg);
	}

	phyReg = readRegister(NvRegPhyInterface) & ~(PHY_HALF|PHY_100|PHY_1000);

	if (!_duplex)
		phyReg |= PHY_HALF;

	if ((_linkspeed & NVREG_LINKSPEED_MASK) == NVREG_LINKSPEED_100)
		phyReg |= PHY_100;
	else if( (_linkspeed & NVREG_LINKSPEED_MASK) == NVREG_LINKSPEED_1000)
		phyReg |= PHY_1000;

	writeRegister(NvRegPhyInterface, phyReg);

	phy_exp = miiRW(_physicalAddress, MII_EXPANSION, MII_READ) & EXPANSION_NWAY; /* autoneg capable */
	if (phyReg & PHY_RGMII) {
		if ((_linkspeed & NVREG_LINKSPEED_MASK) == NVREG_LINKSPEED_1000) {
			txreg = NVREG_TX_DEFERRAL_RGMII_1000;
		} else {
			if (!phy_exp && !_duplex && (_deviceFlags & DEV_HAS_COLLISION_FIX)) {
				if ((_linkspeed & NVREG_LINKSPEED_MASK) == NVREG_LINKSPEED_10)
					txreg = NVREG_TX_DEFERRAL_RGMII_STRETCH_10;
				else
					txreg = NVREG_TX_DEFERRAL_RGMII_STRETCH_100;
			} else {
				txreg = NVREG_TX_DEFERRAL_RGMII_10_100;
			}
		}
	} else {
		if (!phy_exp && !_duplex && (_deviceFlags & DEV_HAS_COLLISION_FIX))
			txreg = NVREG_TX_DEFERRAL_MII_STRETCH;
		else
			txreg = NVREG_TX_DEFERRAL_DEFAULT;
	}
	writeRegister (NvRegTxDeferral, txreg);

	if (_descVers == DESC_VER_1) {
		txreg = NVREG_TX_WM_DESC1_DEFAULT;
	} else {
		if ((_linkspeed & NVREG_LINKSPEED_MASK) == NVREG_LINKSPEED_1000)
			txreg = NVREG_TX_WM_DESC2_3_1000;
		else
			txreg = NVREG_TX_WM_DESC2_3_DEFAULT;
	}
	writeRegister (NvRegTxWatermark, txreg);

	writeRegister(NvRegMisc1, NVREG_MISC1_FORCE | ( _duplex ? false : NVREG_MISC1_HD ) );
	pciPush ();
	writeRegister(NvRegLinkSpeed, _linkspeed);
	pciPush ();

	pause_flags = 0;
	/* setup pause frame */
	if (_duplex != 0) {
		if (_autoneg && _pauseFlags & NV_PAUSEFRAME_AUTONEG) {
			adv_pause = adv & (ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM);
			lpa_pause = lpa & (LPA_PAUSE_CAP | LPA_PAUSE_ASYM);

			switch (adv_pause) {
				case ADVERTISE_PAUSE_CAP:
					if (lpa_pause & LPA_PAUSE_CAP) {
						pause_flags |= NV_PAUSEFRAME_RX_ENABLE;
						if (_pauseFlags & NV_PAUSEFRAME_TX_REQ)
							pause_flags |= NV_PAUSEFRAME_TX_ENABLE;
					}
					break;
					case ADVERTISE_PAUSE_ASYM:
					if (lpa_pause == (LPA_PAUSE_CAP| LPA_PAUSE_ASYM))
						pause_flags |= NV_PAUSEFRAME_TX_ENABLE;
					break;
					case ADVERTISE_PAUSE_CAP| ADVERTISE_PAUSE_ASYM:
					if (lpa_pause & LPA_PAUSE_CAP) {
						pause_flags |=  NV_PAUSEFRAME_RX_ENABLE;
						if (_pauseFlags & NV_PAUSEFRAME_TX_REQ)
							pause_flags |= NV_PAUSEFRAME_TX_ENABLE;
					}
					if (lpa_pause == LPA_PAUSE_ASYM)
						pause_flags |= NV_PAUSEFRAME_RX_ENABLE;
					break;
			}
		} else
			pause_flags = _pauseFlags;
	}
	updatePause(pause_flags);

	if ((_linkspeed & NVREG_LINKSPEED_MASK) == NVREG_LINKSPEED_10) {
		type = kIOMediumEthernet10BaseT;
		speed = 10;
	}

	if ((_linkspeed & NVREG_LINKSPEED_MASK) == NVREG_LINKSPEED_100) {
		type = kIOMediumEthernet100BaseTX;
		speed = 100;
	}

	if ((_linkspeed & NVREG_LINKSPEED_MASK) == NVREG_LINKSPEED_1000) {
		type = kIOMediumEthernet1000BaseT;
		speed = 1000;
	}

	type |= (_duplex) ? kIOMediumOptionFullDuplex : kIOMediumOptionHalfDuplex;

	if (pause_flags)
		type |= kIOMediumOptionFlowControl;

	if (txrx_flags & NV_RESTART_TX)
		startTx ();
	if (txrx_flags & NV_RESTART_RX)
		startRx ();

	if (retval) {
		NVLOG(0, "Link speed now %dMbps(duplex = %s, flow control = %s), code 0x%x.\n", speed, _duplex ? "Full" : "Half", pause_flags ? "Yes" : "No", (unsigned int)_linkspeed);
		setLinkStatus(kIONetworkLinkValid | kIONetworkLinkActive, IONetworkMedium::getMediumWithType(_dictionary, type));
		txrxGate (false);
		startRx();
	} else {
		NVLOG(0, "Network link down.\n");
		setLinkStatus(kIONetworkLinkValid, IONetworkMedium::getMediumWithType(_dictionary, kIOMediumEthernetNone));
		txrxGate (true);
		stopRx();
	}
	
	return retval;
}

void nForceLAN::linkIRQ (void) {
	UInt32 miiStat = readRegister(NvRegMIIStatus);
	writeRegister(NvRegMIIStatus, NVREG_MIISTAT_LINKCHANGE);

	/* reset the link timer if needed */
	if (_nicLinkTimer)
		_nicLinkTimer->setTimeoutMS(LINK_TIMEOUT);
	
	if (miiStat & NVREG_MIISTAT_LINKCHANGE)
		updateLinkSpeed();
}

inline bool nForceLAN::rxCheckFlags_v1 (int rx_index, UInt32 flags, UInt32 *len_ptr) {
	if (!(flags & NV_RX_DESCRIPTORVALID))
		return false;
	
	len_ptr[0] = flags & LEN_MASK_V1;
	
	if (unlikely(flags & NV_RX_ERROR)) {
		if (flags & NV_RX_ERROR4) {
			//      len_ptr[0] = getLength(_rxRing.orig[rx_index].buf, len_ptr[0]);
			if ((int32_t)len_ptr[0] < 0) {
				_stats->inputErrors++;
				return false;
			}
		} else if( flags & NV_RX_FRAMINGERR ) {
			if (flags & NV_RX_SUBTRACT1)
				len_ptr[0]--;
		} else  {
			if (flags & NV_RX_CRCERR)
				_ethernetStats->dot3StatsEntry.fcsErrors++;
			else if (flags & NV_RX_OVERFLOW)
				_ethernetStats->dot3RxExtraEntry.overruns++;
			else if (flags & NV_RX_MISSEDFRAME)
				_ethernetStats->dot3StatsEntry.missedFrames++;
			_stats->inputErrors++;
			return false;
		}
	}

	return true;
}

inline bool nForceLAN::rxCheckFlags_v2 (int rx_index, UInt32 flags, UInt32 *len_ptr) {
	if (!(flags & NV_RX2_DESCRIPTORVALID))
		return false;

	len_ptr[0] = flags & LEN_MASK_V2;

	if (unlikely(flags & NV_RX2_ERROR)) {			
		if (flags & NV_RX2_ERROR4) {
			//      len_ptr[0] = getLength(_rxRing.orig[rx_index].buf, len_ptr[0]);
			if ((int32_t)len_ptr[0] < 0) {
				_stats->inputErrors++;
				return false;
			}
		} else if (flags & NV_RX2_FRAMINGERR) {
			if (flags & NV_RX2_SUBTRACT1)
				len_ptr[0]--;
		} else {
			if (flags & NV_RX2_CRCERR)
				_ethernetStats->dot3StatsEntry.fcsErrors++;
			else if (flags & NV_RX2_OVERFLOW)
				_ethernetStats->dot3RxExtraEntry.overruns++;
			_stats->inputErrors++;
			return false;
		}
	}

	return true;
}

UInt32 nForceLAN::rxProcess (int maxWork) {
  if (_useExtendedRing)
    return rxProcessOptimized (maxWork);
  else
    return rxProcessOrig (maxWork);
}

UInt32 nForceLAN::rxProcessOrig (int maxWork) {
	UInt32 flags, ckResult, len, packetCount;
	bool ret;
	int i, rxWork;
	mbuf_t	pktBuf;

	packetCount = 0;

	rxWork = 0;

	do {
		i = _curRx;

		flags = OSSwapLittleToHostInt32(_rxRing[i].orig->FlagLen);
		if (flags & _rxAvailableFlag)
			break;

		_curRx += likely (_curRx + 1 < _rxRingLen) ? 1 : -_curRx;

		len = descrGetLength(_rxRing[i].orig, _descVers);
    
		if (_descVers == DESC_VER_1)
			ret = rxCheckFlags_v1 (i, flags, &len);
		else
			ret = rxCheckFlags_v2 (i, flags, &len);

		/* process the packet only if the interface is enabled */
		if (likely (_ifEnabled && ret)) {
			NVLOG_DEBUG (3, "forcedeth/rxProcess: packet appears ok. processing packet with flags: %08x, Buffer: %08x...\n",
				     OSSwapLittleToHostInt32(_rxRing[i].orig->FlagLen), OSSwapLittleToHostInt32(_rxRing[i].orig->PacketBuffer));

			pktBuf = allocatePacket (len);
			if (likely (pktBuf)) {
				mbuf_copyback (pktBuf, 0, len, _rxRing[i].v_addr, MBUF_WAITOK);

				if (_descVers == 2 && _hasHWCRC) {
					ckResult = 0;

					if (flags & NV_RX2_CHECKSUM_IP_UDP)
						ckResult |= kChecksumUDP;

					if (flags & NV_RX2_CHECKSUM_IP_TCP)
						ckResult |= kChecksumTCP;

					if (flags & NV_RX2_CHECKSUM_IP)
						ckResult |= kChecksumIP;

					setChecksumResult (pktBuf, kChecksumFamilyTCPIP, ckResult, ckResult);
				}

				OSSynchronizeIO();
				_interface->inputPacket(pktBuf, len);
			} else
				NVLOG (0, "could not allocate packet\n");

			packetCount ++;
		}

		/* zero the buffer for good measure */
		//memset (_rxRing[i].v_addr, 0, len);

		/* mark the ring entry as available */
		setupRingDescriptor (_rxRing, i, _rxBufferSize | _rxAvailableFlag);

		rxWork ++;
	} while (rxWork < maxWork);

	if (_stats)
		_stats->inputPackets += packetCount;

	if (packetCount)
		_interface->flushInputQueue();

	return packetCount;
}

UInt32 nForceLAN::rxProcessOptimized (int maxWork) {
	UInt32 flags, ckResult, len, packetCount;
	bool ret;
	int i, rxWork;
	mbuf_t	pktBuf;

	packetCount = 0;

	rxWork = 0;

	do {
		i = _curRx;

		flags = OSSwapLittleToHostInt32(_rxRing[i].ex->FlagLen);
    		if (flags & NV_RX2_AVAIL)
			break;

		_curRx += likely (_curRx + 1 < _rxRingLen) ? 1 : -_curRx;

		len = descrGetLength(_rxRing[i].ex);    
		ret = rxCheckFlags_v2 (i, flags, &len);

		/* process the packet only if the interface is enabled */
		if (likely (_ifEnabled && ret)) {
			NVLOG_DEBUG(3, "packet appears ok. processing packet with flags: %08x, txvlan: %08x, Buffer: %08x %08x...\n",
				    OSSwapLittleToHostInt32(_rxRing[i].ex->FlagLen), OSSwapLittleToHostInt32(_rxRing[i].ex->TxVlan),
				    OSSwapLittleToHostInt32(_rxRing[i].ex->PacketBufferHigh), OSSwapLittleToHostInt32(_rxRing[i].ex->PacketBufferLow));

			pktBuf = allocatePacket (len);
			if (likely (pktBuf)) {
				mbuf_copyback (pktBuf, 0, len, _rxRing[i].v_addr, MBUF_WAITOK);

				if (unlikely(_rxRing[i].ex->TxVlan & NV_RX3_VLAN_TAG_PRESENT))
					/* get the vlan tag from the hardware */
					setVlanTag (pktBuf, OSSwapLittleToHostInt32(_rxRing[i].ex->TxVlan & ~NV_TX3_VLAN_TAG_PRESENT));

				if (_hasHWCRC) {					
					ckResult = 0;
					if (flags & NV_RX2_CHECKSUM_IP_UDP)
						ckResult |= kChecksumUDP;

					if (flags & NV_RX2_CHECKSUM_IP_TCP)
						ckResult |= kChecksumTCP;

					if (flags & NV_RX2_CHECKSUM_IP)
						ckResult |= kChecksumIP;

					setChecksumResult (pktBuf, kChecksumFamilyTCPIP, ckResult, ckResult);
				}
				
				OSSynchronizeIO();
				_interface->inputPacket(pktBuf, len);
			} else
				NVLOG (0, "could not allocate packet\n");

			packetCount ++;
		}

		/* zero the buffer for good measure */
		//memset (_rxRing[i].v_addr, 0, len);

		/* mark the ring entry as available */
		_rxRing[i].ex->FlagLen          = OSSwapHostToLittleInt32(_rxBufferSize | NV_RX2_AVAIL);
		/* some NICs require we rewrite these values */
		_rxRing[i].ex->PacketBufferLow  = OSSwapHostToLittleInt32(dmaLow ((UInt64)_rxRing[i].p_addr));
		_rxRing[i].ex->PacketBufferHigh = OSSwapHostToLittleInt32(dmaHigh ((UInt64)_rxRing[i].p_addr));
		_rxRing[i].ex->TxVlan           = 0;

		rxWork ++;
	} while (rxWork < maxWork);

	if (_stats)
		_stats->inputPackets += packetCount;

	if (packetCount)
		_interface->flushInputQueue();

	return packetCount;
}

/* initialize RX rings */
inline bool nForceLAN::initRx (void) {
	int i;

	_curRx = 0;
	
	for (i = 0 ; i < _rxRingLen ; i++) {
		_rxRing[i].m_desc = IOBufferMemoryDescriptor::inTaskWithPhysicalMask (kernel_task, kIOMemoryPhysicallyContiguous, _rxBufferSize, _physMemMask);
		if (!_rxRing[i].m_desc)
			return false;

		_rxRing[i].v_addr = _rxRing[i].m_desc->getBytesNoCopy ();
		_rxRing[i].p_addr = _rxRing[i].m_desc->getPhysicalSegment (0, 0);
		_rxRing[i].f_size = _rxBufferSize;
    _rxRing[i].m_buf  = NULL;

		if (unlikely(!_rxRing[i].v_addr)) {
			NVLOG (0, "could not allocate RX buffer\n");

			return false;
		}

		setupRingDescriptor (_rxRing, i, _rxBufferSize | _rxAvailableFlag);
	}

	return true;
}

/* initialize TX rings */
inline bool nForceLAN::initTx (void) {
	_nextTx = _nicTx = 0;

	/* not used but initialized anyway */
	_txPktsInProgress = 0;
	_txChangeOwnerHead = NULL;
	_txChangeOwnerTail = NULL;

	for( int i = 0; i < _txRingLen; i++ ) {
		_txRing[i].m_desc = 0;
		_txRing[i].f_size = 0;
		_txRing[i].v_addr = 0;
		_txRing[i].p_addr = 0;
		_txRing[i].m_buf  = 0;
		_txRing[i].changeOwnerNext = NULL;

		setupRingDescriptor (_txRing, i, 0);
	}

	return true;
}

bool nForceLAN::initRing (void) {
	return initTx() && initRx ();
}

bool nForceLAN::releaseRingEntry (struct io_rings *ring, int index) {	
  bool dropped = false;

 	if (ring[index].m_desc) {
    /* if we allocated our own buffer free it */
		ring[index].m_desc->release();

    dropped = true;
  }

  if (ring[index].m_buf) {
    freePacket (ring[index].m_buf);

    dropped = true;
  }
	
	ring[index].m_desc = 0;
  ring[index].v_addr = 0;
	ring[index].p_addr = 0;
	ring[index].f_size = 0;
	ring[index].m_buf  = 0;

	setupRingDescriptor (ring, index, 0);

  return dropped;
}

inline void nForceLAN::drainTx (void) {
	int i;
	
	for (i = 0; i < _txRingLen; i++) {		
		if (releaseRingEntry (_txRing, i))
			_outputStats->dropCount++;
	}
	
	_txPktsInProgress = 0;
	_txChangeOwnerHead = NULL;
	_txChangeOwnerTail = NULL;
	_nicTx = _nextTx = 0;
}

inline void nForceLAN::drainRx (void) {
	int i;

	for (i = 0 ; i < _rxRingLen ; i++)
    (void)releaseRingEntry (_rxRing, i);

	_curRx = 0;
}

void nForceLAN::drainRing (void) {
	drainTx();
	drainRx();
}

void nForceLAN::startRx (void) {
	UInt rxCtrl = readRegister (NvRegReceiverControl);

	/* Already running? Stop it. */
	if ((rxCtrl & NVREG_RCVCTL_START) && !_macInUse) {
		rxCtrl &= ~NVREG_RCVCTL_START;
		writeRegister (NvRegReceiverControl, rxCtrl);
		pciPush ();
	}

	writeRegister (NvRegLinkSpeed, _linkspeed);
	pciPush ();

	rxCtrl |= NVREG_RCVCTL_START;
	if (_macInUse)
		rxCtrl &= ~NVREG_RCVCTL_RX_PATH_EN;
	writeRegister (NvRegReceiverControl, rxCtrl);
	pciPush ();
}

void nForceLAN::stopRx (void) {
	UInt32 rxCtrl = readRegister (NvRegReceiverControl);

	if (!_macInUse)
		rxCtrl &= ~NVREG_RCVCTL_START;
	else
		rxCtrl |= NVREG_RCVCTL_RX_PATH_EN;
	writeRegister (NvRegReceiverControl, rxCtrl);
	regDelay(NvRegReceiverStatus, NVREG_RCVSTAT_BUSY, 0, NV_RXSTOP_DELAY1, NV_RXSTOP_DELAY1MAX,
		       "receiver status remained busy during attempted shutdown");

	IODelay (NV_RXSTOP_DELAY2);
	if (!_macInUse)
		writeRegister(NvRegLinkSpeed, 0);
}

void nForceLAN::startTx (void) {
	UInt32 txCtrl = readRegister (NvRegTransmitterControl);

	txCtrl |= NVREG_XMITCTL_START;
	if (_macInUse)
		txCtrl &= ~NVREG_XMITCTL_TX_PATH_EN;
	writeRegister (NvRegTransmitterControl, txCtrl);
	pciPush ();
}

void nForceLAN::stopTx (void) {
	UInt32 txCtrl = readRegister (NvRegTransmitterControl);

	if (!_macInUse)
		txCtrl &= ~NVREG_XMITCTL_START;
	else
		txCtrl |= NVREG_XMITCTL_TX_PATH_EN;
	writeRegister (NvRegTransmitterControl, txCtrl);
	regDelay(NvRegTransmitterStatus, NVREG_XMITSTAT_BUSY, 0, NV_TXSTOP_DELAY1, NV_TXSTOP_DELAY1MAX,
		       "transmitter status remained busy during attempted shutdown");

	IODelay (NV_TXSTOP_DELAY2);
	if (!_macInUse)
		writeRegister(NvRegTransmitPoll, readRegister (NvRegTransmitPoll) & NVREG_TRANSMITPOLL_MAC_ADDR_REV);
}

void nForceLAN::txrxReset (void) {
	writeRegister(NvRegTxRxControl, NVREG_TXRXCTL_BIT2 | NVREG_TXRXCTL_RESET | _txrxCtlBits);
	pciPush ();
	IODelay(NV_TXRX_RESET_DELAY);
	writeRegister(NvRegTxRxControl, NVREG_TXRXCTL_BIT2 | _txrxCtlBits);
	pciPush ();
}

void nForceLAN::txrxGate (bool gate) {
	UInt32 powerstate;

	if (!_macInUse && (_deviceFlags & DEV_HAS_POWER_CNTRL)) {
		powerstate = readRegister (NvRegPowerState2);
		if (gate)
			powerstate |= NVREG_POWERSTATE2_GATE_CLOCKS;
		else
			powerstate &= ~NVREG_POWERSTATE2_GATE_CLOCKS;
		writeRegister (NvRegPowerState2, powerstate);
	}
}

void nForceLAN::MACReset (void) {
	UInt32 temp1, temp2, temp3;

	writeRegister (NvRegTxRxControl, NVREG_TXRXCTL_BIT2 | NVREG_TXRXCTL_RESET | _txrxCtlBits);
	pciPush ();

	/* save registers since they will be cleared on reset */
	temp1 = readRegister (NvRegMacAddrA);
	temp2 = readRegister (NvRegMacAddrB);
	temp3 = readRegister (NvRegTransmitPoll);

	writeRegister (NvRegMacReset, NVREG_MAC_RESET_ASSERT);
	pciPush ();
	IODelay (NV_MAC_RESET_DELAY);
	writeRegister (NvRegMacReset, 0);
	pciPush ();
	IODelay (NV_MAC_RESET_DELAY);

	/* restore saved registers */
	writeRegister (NvRegMacAddrA, temp1);
	writeRegister (NvRegMacAddrB, temp2);
	writeRegister (NvRegTransmitPoll, temp3);

	writeRegister (NvRegTxRxControl, NVREG_TXRXCTL_BIT2 | _txrxCtlBits);
	pciPush ();
}

int nForceLAN::getLength(mbuf_t buf, int len) {
	return len;

	/*int hdrLen;

	hdrLen = mbuf_pkthdr_len(buf);

	if( hdrLen <= len ) {
	return hdrLen;
	} else {
	return -1;
	}*/
}

inline void nForceLAN::txFlipOwnership (void) {
	struct io_rings *next;

	_txPktsInProgress--;
	if (_txChangeOwnerHead) {
		next = _txChangeOwnerHead->changeOwnerNext;

		_txChangeOwnerHead->ex->FlagLen |= OSSwapLittleToHostInt32(NV_TX2_VALID);
		_txPktsInProgress++;

		_txChangeOwnerHead->changeOwnerNext = NULL;
		_txChangeOwnerHead = next;

		if (_txChangeOwnerHead)
			_txChangeOwnerHead->changeOwnerPrev = NULL;
		else
			_txChangeOwnerTail = NULL;

		writeRegister (NvRegTxRxControl, NVREG_TXRXCTL_KICK | _txrxCtlBits);
	}
}

void nForceLAN::txCheckFlags_v2 (UInt32 flags) {
	if (flags & NV_TX2_LASTPACKET) {
		if( flags & (NV_TX2_ERROR | NV_TX2_UNDERFLOW | NV_TX2_CARRIERLOST)) {
			_stats->outputErrors++;
			
			if( flags & NV_TX_LATECOLLISION )
				_stats->collisions++;
		} else if ((flags & NV_TX2_RETRYERROR) && !(flags & NV_TX2_RETRYCOUNT_MASK)) {
			_stats->outputErrors++;
			
			if (_deviceFlags & DEV_HAS_GEAR_MODE)
				gearBackoffReseed ();
			else
				legacyBackoffReseed();
		} else
			_stats->outputPackets++;

		if (_txLimit)
			txFlipOwnership ();
	}
}

void nForceLAN::txCheckFlags_v1 (UInt32 flags) {
	if (flags & NV_TX_LASTPACKET) {
		if( flags & (NV_TX_ERROR | NV_TX_UNDERFLOW | NV_TX_CARRIERLOST)) {
			_stats->outputErrors++;

			if( flags & NV_TX_LATECOLLISION )
				_stats->collisions++;
		} else if ((flags & NV_TX_RETRYERROR) && !(flags & NV_TX_RETRYCOUNT_MASK)) {
			_stats->outputErrors++;

			legacyBackoffReseed();
		} else
			_stats->outputPackets++;

		_txPktsInProgress--;
	}
}

UInt32 nForceLAN::txDoneOrig (int maxWork) {
	UInt32 flags, entry;
	bool lastPacket;
	int txWork = 0;

	if (!_useExtendedRing)
		while ((_nicTx != _nextTx) && (txWork < maxWork)) {
			entry = _nicTx;

			flags = OSSwapLittleToHostInt32(_txRing[entry].orig->FlagLen);

			if (_descVers == DESC_VER_1) {
				if (flags & NV_TX_VALID)
					break;

				if (flags & NV_TX_LASTPACKET) {
					txCheckFlags_v1 (flags);
					lastPacket = true;
				} else
					lastPacket = false;
			} else {
				if( flags & NV_TX2_VALID )
					break;

				if (flags & NV_TX2_LASTPACKET) {
					txCheckFlags_v2 (flags);
					lastPacket = true;
				} else
					lastPacket = false;
			}

			NVLOG_DEBUG (3, "%s with packet with flags %x\n", lastPacket ? "done" : "continuing", (unsigned int)flags);

			(void)releaseRingEntry (_txRing, entry);

			/* conditional is much faster than modulus */
			_nicTx += likely (_nicTx + 1 < _txRingLen) ? 1 : -_nicTx;

			txWork ++;
		}

	return txWork;
}

UInt32 nForceLAN::txDoneEx (int maxWork) {
	UInt32 flags, entry;
	int txWork = 0;

	if (_useExtendedRing)
		while ((_nicTx != _nextTx) && (txWork < maxWork)) {
			entry = _nicTx;

			flags = OSSwapLittleToHostInt32(_txRing[entry].ex->FlagLen);

			if (flags & NV_TX2_VALID)
				break;

			if (flags & NV_TX2_LASTPACKET)
				txCheckFlags_v2 (flags);

			NVLOG_DEBUG (3, "%s with packet with flags %x\n", flags & NV_TX2_LASTPACKET ? "done" : "continuing", (unsigned int)flags);
    
			(void)releaseRingEntry (_txRing, entry);

			/* conditional is much faster than modulus */
			_nicTx += likely (_nicTx + 1 < _txRingLen) ? 1 : -_nicTx;

			txWork ++;
		}

	return txWork;
}

UInt32 nForceLAN::txDone (int maxWork) {
	int work;

	if (!_useExtendedRing)
		work = txDoneOrig (maxWork);
	else
		work = txDoneEx (maxWork);

	_outputQueue->service ();

	return work;
}

void nForceLAN::legacyBackoffReseed (void) {
	UInt32 reg, low;
	int txStatus = 0;

	reg = readRegister (NvRegSlotTime) & ~NVREG_SLOTTIME_MASK;
	low = random () & NVREG_SLOTTIME_MASK;
	reg |= low;

	/* Need to stop tx before change takes effect.
	 * Caller has already gained np->lock.
	 */
	txStatus = readRegister (NvRegTransmitterControl) & NVREG_XMITCTL_START;
	if (txStatus)
		stopTx();

	stopRx();
	writeRegister (NvRegSlotTime, reg);

	if (txStatus)
		startTx ();
	startRx ();
}

/* Known Good seed sets */
static const UInt32 main_seedset[BACKOFF_SEEDSET_ROWS][BACKOFF_SEEDSET_LFSRS] = {
	{145, 155, 165, 175, 185, 196, 235, 245, 255, 265, 275, 285, 660, 690, 874},
	{245, 255, 265, 575, 385, 298, 335, 345, 355, 366, 375, 385, 761, 790, 974},
	{145, 155, 165, 175, 185, 196, 235, 245, 255, 265, 275, 285, 660, 690, 874},
	{245, 255, 265, 575, 385, 298, 335, 345, 355, 366, 375, 386, 761, 790, 974},
	{266, 265, 276, 585, 397, 208, 345, 355, 365, 376, 385, 396, 771, 700, 984},
	{266, 265, 276, 586, 397, 208, 346, 355, 365, 376, 285, 396, 771, 700, 984},
	{366, 365, 376, 686, 497, 308, 447, 455, 466, 476, 485, 496, 871, 800,  84},
	{466, 465, 476, 786, 597, 408, 547, 555, 566, 576, 585, 597, 971, 900, 184}};

static const UInt32 gear_seedset[BACKOFF_SEEDSET_ROWS][BACKOFF_SEEDSET_LFSRS] = {
	{251, 262, 273, 324, 319, 508, 375, 364, 341, 371, 398, 193, 375,  30, 295},
	{351, 375, 373, 469, 551, 639, 477, 464, 441, 472, 498, 293, 476, 130, 395},
	{351, 375, 373, 469, 551, 639, 477, 464, 441, 472, 498, 293, 476, 130, 397},
	{251, 262, 273, 324, 319, 508, 375, 364, 341, 371, 398, 193, 375,  30, 295},
	{251, 262, 273, 324, 319, 508, 375, 364, 341, 371, 398, 193, 375,  30, 295},
	{351, 375, 373, 469, 551, 639, 477, 464, 441, 472, 498, 293, 476, 130, 395},
	{351, 375, 373, 469, 551, 639, 477, 464, 441, 472, 498, 293, 476, 130, 395},
	{351, 375, 373, 469, 551, 639, 477, 464, 441, 472, 498, 293, 476, 130, 395}};

void nForceLAN::gearBackoffReseed (void) {
	UInt32 miniseed1, miniseed2, miniseed2_reversed, miniseed3, miniseed3_reversed;
	UInt32 temp, seedset, combinedSeed;
	int i;

	/* Setup seed for free running LFSR */
	/* We are going to read the time stamp counter 3 times
	   and swizzle bits around to increase randomness */
	miniseed1 = random () & 0x0fff;
	if (miniseed1 == 0)
		miniseed1 = 0xabc;

	miniseed2 = random () & 0x0fff;
	if (miniseed2 == 0)
		miniseed2 = 0xabc;
	miniseed2_reversed =
		((miniseed2 & 0xF00) >> 8) |
		(miniseed2 & 0x0F0) |
		((miniseed2 & 0x00F) << 8);
	
	miniseed3 = random () & 0x0fff;
	if (miniseed3 == 0)
		miniseed3 = 0xabc;
	miniseed3_reversed =
		((miniseed3 & 0xF00) >> 8) |
		(miniseed3 & 0x0F0) |
		((miniseed3 & 0x00F) << 8);

	combinedSeed = ((miniseed1 ^ miniseed2_reversed) << 12) |
		(miniseed2 ^ miniseed3_reversed);

	/* Seeds can not be zero */
	if ((combinedSeed & NVREG_BKOFFCTRL_SEED_MASK) == 0)
		combinedSeed |= 0x08;
	if ((combinedSeed & (NVREG_BKOFFCTRL_SEED_MASK << NVREG_BKOFFCTRL_GEAR)) == 0)
		combinedSeed |= 0x8000;

	/* No need to disable tx here */
	temp = NVREG_BKOFFCTRL_DEFAULT | (0 << NVREG_BKOFFCTRL_SELECT);
	temp |= combinedSeed & NVREG_BKOFFCTRL_SEED_MASK;
	temp |= combinedSeed >> NVREG_BKOFFCTRL_GEAR;
	writeRegister (NvRegBackOffControl, temp);

    	/* Setup seeds for all gear LFSRs. */
	seedset = random () % BACKOFF_SEEDSET_ROWS;
	for (i = 1; i <= BACKOFF_SEEDSET_LFSRS; i++) {
		temp = NVREG_BKOFFCTRL_DEFAULT | (i << NVREG_BKOFFCTRL_SELECT);
		temp |= main_seedset[seedset][i-1] & 0x3ff;
		temp |= ((gear_seedset[seedset][i-1] & 0x3ff) << NVREG_BKOFFCTRL_GEAR);
		writeRegister (NvRegBackOffControl, temp);
	}
}

void nForceLAN::setupHWRings (int rxtx_flags) {
	if (!_useExtendedRing) {
		if (rxtx_flags & NV_SETUP_RX_RING)
			writeRegister(NvRegRxRingPhysAddr, (UInt32)_rxRingMem);
		if (rxtx_flags & NV_SETUP_TX_RING)
			writeRegister(NvRegTxRingPhysAddr, (UInt32)_txRingMem);
	} else {
		if (rxtx_flags & NV_SETUP_RX_RING) {
			writeRegister(NvRegRxRingPhysAddr, dmaLow((UInt64)_rxRingMem));
			writeRegister(NvRegRxRingPhysAddrHigh, dmaHigh((UInt64)_rxRingMem));
		}
		if (rxtx_flags & NV_SETUP_TX_RING) {
			writeRegister(NvRegTxRingPhysAddr, dmaLow((UInt64)_txRingMem));
			writeRegister(NvRegTxRingPhysAddrHigh, dmaHigh((UInt64)_txRingMem));
		}
	}
}

bool nForceLAN::initPCIConfigSpace (void) {
	UInt16	reg;
	
	reg  = _device->configRead16 (kIOPCIConfigCommand);
	reg |= (kIOPCICommandIOSpace | kIOPCICommandBusMaster | kIOPCICommandMemorySpace);
	_device->configWrite16 (kIOPCIConfigCommand, reg);

    _device->findPCICapability( kIOPCIPowerManagementCapability, &_pmPCICapabilityPtr );
    if (_pmPCICapabilityPtr) {
      NVLOG_DEBUG (1, "found power management capability at %d\n", _pmPCICapabilityPtr);
      _device->configWrite16 (_pmPCICapabilityPtr + 4, kPCIPMCSPMEStatus | kPCIPMCSPowerStateD0);
      IOSleep (20);
    }

	_device->enablePCIPowerManagement (kPCIPMCSPowerStateD3);
	
	return true;
}
