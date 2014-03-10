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

bool nForceLAN::phyReset (void) {
	UInt32 miiControl;
	
	miiControl = miiRW(_physicalAddress, MII_BMSR, MII_READ) | BMCR_RESET;
	
	return phyReset (miiControl);
}

bool nForceLAN::phyReset (UInt32 miiControl) {
	unsigned int tries = 0;
	
	if (_phyOui== PHY_OUI_MARVELL && _phyModel == PHY_MODEL_MARVELL_E1011)
		saveLEDStats ();
	
	if (miiRW(_physicalAddress, MII_BMCR, miiControl)) {
    NVLOG_DEBUG (0, "failed to write mii BMCR register\n");
    
		return false;
  }
	
	IOSleep(500);
	
	while (miiControl & BMCR_RESET) {
		IOSleep(10);
		miiControl = miiRW(_physicalAddress, MII_BMCR, MII_READ);
		
		/* FIXME: 100 tries IS excessive */
		if (tries++ > 100) {
      NVLOG_DEBUG (0, "too many tries while waiting for the physical to reset\n");
      
			return false;
    }
	}
	
	if (_phyOui== PHY_OUI_MARVELL && _phyModel == PHY_MODEL_MARVELL_E1011)
		restoreLEDStats ();
	
	return true;
}

static inline bool addMediumToDictionary (OSDictionary *dict, IOMediumType type, UInt64 speed, int code, const char *name) {
	IONetworkMedium *medium;
	bool ret = false;
	
	medium = IONetworkMedium::medium(type, speed, 0, code, name);
	if (medium) {
		ret = IONetworkMedium::addMedium(dict, medium);
		medium->release ();
	}
	
	return ret;
}

int nForceLAN::phyInit (void) {
	UInt32 phyInterface, phyReserved, miiStatus, miiControl, miiControlGigabit, reg;
  
	/* phy errata for E3016 phy */
	if (_phyModel == PHY_MODEL_MARVELL_E3016) {
		reg = miiRW(_physicalAddress, MII_NCONFIG, MII_READ) & ~PHY_MARVELL_E3016_INITMASK;
		if (miiRW(_physicalAddress, MII_NCONFIG, reg)) {
		  NVLOG (0, "phy write to E3016 errata reg failed.\n");
			return PHY_ERROR;
		}
	}
  
  do {
    if (_phyOui == PHY_OUI_REALTEK) {
      if (_phyModel == PHY_MODEL_REALTEK_8211 && _phyRev == PHY_REV_REALTEK_8211B) {
        if (miiRW(_physicalAddress, PHY_REALTEK_INIT_REG1, PHY_REALTEK_INIT1))
          break;
        if (miiRW(_physicalAddress, PHY_REALTEK_INIT_REG2, PHY_REALTEK_INIT2))
          break;
        if (miiRW(_physicalAddress, PHY_REALTEK_INIT_REG1, PHY_REALTEK_INIT3))
          break;
        if (miiRW(_physicalAddress, PHY_REALTEK_INIT_REG3, PHY_REALTEK_INIT4))
          break;
        if (miiRW(_physicalAddress, PHY_REALTEK_INIT_REG4, PHY_REALTEK_INIT5))
          break;
        if (miiRW(_physicalAddress, PHY_REALTEK_INIT_REG5, PHY_REALTEK_INIT6))
          break;
        if (miiRW(_physicalAddress, PHY_REALTEK_INIT_REG1, PHY_REALTEK_INIT1)) 
          break;
      }
      if (_phyModel == PHY_MODEL_REALTEK_8211 && _phyRev == PHY_REV_REALTEK_8211C) {
        UInt32 powerstate = readRegister (NvRegPowerState2);
        
        if (0) {
          /* need to perform hw phy reset */
          powerstate |= NVREG_POWERSTATE2_PHY_RESET;
          writeRegister (NvRegPowerState2, powerstate);
          IODelay (25);
          
          powerstate &= ~NVREG_POWERSTATE2_PHY_RESET;
          writeRegister (NvRegPowerState2, powerstate);
          IODelay (25);
        } else
          NVLOG_DEBUG (1, "skipping physical reset for RTL8211C\n");
        
        reg = miiRW (_physicalAddress, PHY_REALTEK_INIT_REG6, MII_READ);
        reg |= PHY_REALTEK_INIT9;
        if (miiRW (_physicalAddress, PHY_REALTEK_INIT_REG6, reg))
          break;
        if (miiRW(_physicalAddress, PHY_REALTEK_INIT_REG1, PHY_REALTEK_INIT10))
          break;
        reg = miiRW(_physicalAddress, PHY_REALTEK_INIT_REG7, MII_READ);
        if (!(reg & PHY_REALTEK_INIT11)) {
          reg |= PHY_REALTEK_INIT11;
          if (miiRW(_physicalAddress, PHY_REALTEK_INIT_REG7, reg))
            break;
        }
        
        if (miiRW (_physicalAddress, PHY_REALTEK_INIT_REG1, PHY_REALTEK_INIT1))
          break;
      }
      if (_phyModel == PHY_MODEL_REALTEK_8201) {
        if (_deviceFlags & DEV_NEED_PHY_INIT_FIX) {
          phyReserved = miiRW(_physicalAddress, PHY_REALTEK_INIT_REG6, MII_READ);
          phyReserved |= PHY_REALTEK_INIT7;
          if (miiRW(_physicalAddress, PHY_REALTEK_INIT_REG6, phyReserved))
            break;
        }
      }
    }
    
    /* set advertise register */
    reg = miiRW(_physicalAddress, MII_ADVERTISE, MII_READ);
    reg |= (ADVERTISE_10HALF|ADVERTISE_10FULL|ADVERTISE_100HALF|ADVERTISE_100FULL|ADVERTISE_PAUSE_ASYM|ADVERTISE_PAUSE_CAP);
    if (miiRW(_physicalAddress, MII_ADVERTISE, reg)) {
      NVLOG (0, "PHY write to advertise failed.\n");
      break;
    }
    
    /* get phy interface type */
    phyInterface = readRegister(NvRegPhyInterface);
    
    /* see if gigabit phy */
    miiStatus = miiRW(_physicalAddress, MII_BMSR, MII_READ);
    if( miiStatus & PHY_GIGABIT ) {
      _gigabit = PHY_GIGABIT;
      miiControlGigabit = miiRW(_physicalAddress, MII_1000BT_CR, MII_READ);
      miiControlGigabit &= ~ADVERTISE_1000HALF;
      if( phyInterface & PHY_RGMII )
        miiControlGigabit |= ADVERTISE_1000FULL;
      else
        miiControlGigabit &= ~ADVERTISE_1000FULL;
      
      if (miiRW(_physicalAddress, MII_1000BT_CR, miiControlGigabit))
        break;
    } else
      _gigabit = 0;
    
    miiControl = miiRW(_physicalAddress, MII_BMCR, MII_READ) | BMCR_ANENABLE;
    
    /* reset the phy
     * (certain phys need bmcr to be setup with reset)
     */
    if (_phyOui == PHY_OUI_REALTEK && _phyModel == PHY_MODEL_REALTEK_8211 && _phyRev == PHY_REV_REALTEK_8211C) {
      /* start autoneg since we already performed hw reset above */
      miiControl |= BMCR_ANRESTART;
      if (miiRW (_physicalAddress, MII_BMCR, miiControl))
        break;
    } else if (!phyReset(miiControl))
    /* reset the phy (certain phys need bmcr to be setup with reset) */
      break;
    
    /* phy vendor specific configuration */
    if ((_phyOui == PHY_OUI_CICADA) && (phyInterface & PHY_RGMII)) {
      phyReserved = miiRW(_physicalAddress, MII_RESV1, MII_READ);
      phyReserved &= ~(PHY_CICADA_INIT1 | PHY_CICADA_INIT2);
      phyReserved |= (PHY_CICADA_INIT3 | PHY_CICADA_INIT4);
      
      if (miiRW(_physicalAddress, MII_RESV1, phyReserved))
        break;
      
      phyReserved = miiRW(_physicalAddress, MII_NCONFIG, MII_READ) | PHY_CICADA_INIT5;
      if (miiRW(_physicalAddress, MII_NCONFIG, phyReserved))
        break;
    }
    if (_phyOui == PHY_OUI_CICADA) {
      phyReserved = miiRW(_physicalAddress, MII_SREVISION, MII_READ) | PHY_CICADA_INIT6;
      if (miiRW(_physicalAddress, MII_SREVISION, phyReserved))
        break;
    }
    if (_phyOui == PHY_OUI_VITESSE) {
      if (miiRW(_physicalAddress, PHY_VITESSE_INIT_REG1, PHY_VITESSE_INIT1))
        break;
      if (miiRW(_physicalAddress, PHY_VITESSE_INIT_REG2, PHY_VITESSE_INIT2))
        break;
      
      phyReserved = miiRW(_physicalAddress, PHY_VITESSE_INIT_REG4, MII_READ);
      if (miiRW(_physicalAddress, PHY_VITESSE_INIT_REG4, phyReserved))
        break;
      
      phyReserved = miiRW(_physicalAddress, PHY_VITESSE_INIT_REG3, MII_READ);
      phyReserved &= ~PHY_VITESSE_INIT_MSK1;
      phyReserved |= PHY_VITESSE_INIT3;
      if (miiRW(_physicalAddress, PHY_VITESSE_INIT_REG3, phyReserved))
        break;
      if (miiRW(_physicalAddress, PHY_VITESSE_INIT_REG2, PHY_VITESSE_INIT4))
        break;
      if (miiRW(_physicalAddress, PHY_VITESSE_INIT_REG2, PHY_VITESSE_INIT5))
        break;
      
      phyReserved = miiRW(_physicalAddress, PHY_VITESSE_INIT_REG4, MII_READ);
      phyReserved &= ~PHY_VITESSE_INIT_MSK1;
      phyReserved |= PHY_VITESSE_INIT3;
      if (miiRW(_physicalAddress, PHY_VITESSE_INIT_REG4, phyReserved))
        break;
      
      phyReserved = miiRW(_physicalAddress, PHY_VITESSE_INIT_REG3, MII_READ);
      if (miiRW(_physicalAddress, PHY_VITESSE_INIT_REG3, phyReserved))
        break;
      if (miiRW(_physicalAddress, PHY_VITESSE_INIT_REG2, PHY_VITESSE_INIT6))
        break;
      if (miiRW(_physicalAddress, PHY_VITESSE_INIT_REG2, PHY_VITESSE_INIT7))
        break;
      
      phyReserved = miiRW(_physicalAddress, PHY_VITESSE_INIT_REG4, MII_READ);
      if (miiRW(_physicalAddress, PHY_VITESSE_INIT_REG4, phyReserved))
        break;
      
      phyReserved = miiRW(_physicalAddress, PHY_VITESSE_INIT_REG3, MII_READ);
      phyReserved &= ~PHY_VITESSE_INIT_MSK2;
      phyReserved |= PHY_VITESSE_INIT8;
      if (miiRW(_physicalAddress, PHY_VITESSE_INIT_REG3, phyReserved))
        break;
      if (miiRW(_physicalAddress, PHY_VITESSE_INIT_REG2, PHY_VITESSE_INIT9))
        break;
      if (miiRW(_physicalAddress, PHY_VITESSE_INIT_REG1, PHY_VITESSE_INIT10))
        break;
    }
    if (_phyOui == PHY_OUI_REALTEK) {
      if (_phyModel == PHY_MODEL_REALTEK_8211 && _phyRev == PHY_REV_REALTEK_8211B) {
        /* reset could have cleared these out, set them back */
        if (miiRW(_physicalAddress, PHY_REALTEK_INIT_REG1, PHY_REALTEK_INIT1))
          break;
        if (miiRW(_physicalAddress, PHY_REALTEK_INIT_REG2, PHY_REALTEK_INIT2))
          break;
        if (miiRW(_physicalAddress, PHY_REALTEK_INIT_REG1, PHY_REALTEK_INIT3))
          break;
        if (miiRW(_physicalAddress, PHY_REALTEK_INIT_REG3, PHY_REALTEK_INIT4))
          break;
        if (miiRW(_physicalAddress, PHY_REALTEK_INIT_REG4, PHY_REALTEK_INIT5))
          break;
        if (miiRW(_physicalAddress, PHY_REALTEK_INIT_REG5, PHY_REALTEK_INIT6))
          break;
        if (miiRW(_physicalAddress, PHY_REALTEK_INIT_REG1, PHY_REALTEK_INIT1))
          break;
      }
      if (_phyModel == PHY_MODEL_REALTEK_8201) {
        if (_deviceFlags & DEV_NEED_PHY_INIT_FIX) {
          phyReserved = miiRW(_physicalAddress, PHY_REALTEK_INIT_REG6, MII_READ);
          phyReserved |= PHY_REALTEK_INIT7;
          if (miiRW(_physicalAddress, PHY_REALTEK_INIT_REG6, phyReserved))
            break;
        }
        if (_phyCross == NV_CROSSOVER_DETECTION_DISABLED) {
          if (miiRW(_physicalAddress, PHY_REALTEK_INIT_REG1, PHY_REALTEK_INIT3))
            break;
          
          phyReserved = miiRW(_physicalAddress, PHY_REALTEK_INIT_REG2, MII_READ);
          phyReserved &= ~PHY_REALTEK_INIT_MSK1;
          phyReserved |= PHY_REALTEK_INIT3;
          if (miiRW(_physicalAddress, PHY_REALTEK_INIT_REG2, phyReserved))
            break;
          if (miiRW(_physicalAddress, PHY_REALTEK_INIT_REG1, PHY_REALTEK_INIT1))
            break;
        }
      }
    }
    
    
    /* some phys clear out pause advertisment on reset, set it back */
    miiRW(_physicalAddress, MII_ADVERTISE, reg);
    
    /* restart auto negotiation, power down phy */
    miiControl = miiRW(_physicalAddress, MII_BMCR, MII_READ);    
    miiControl |= (BMCR_ANRESTART | BMCR_ANENABLE);
    if (_phyPowerDown)
      miiControl |= BMCR_PDOWN;
    
    if (miiRW(_physicalAddress, MII_BMCR, miiControl))
      break;
		
    _dictionary = OSDictionary::withCapacity(5);
    
    (void) addMediumToDictionary (_dictionary, kIOMediumEthernetNone, 0, 0, "None");
    (void) addMediumToDictionary (_dictionary, kIOMediumEthernetAuto, 1, 0, "Auto");
    
    NVLOG (1, "supported media:");
    
    if (miiStatus & BMSR_100FULL2) {
      NVLOGC (1, " 100BaseT2(full)");
      (void) addMediumToDictionary (_dictionary, kIOMediumEthernet100BaseT2 | kIOMediumOptionFullDuplex, 100 * 1000000, 2, "100BaseT2 Full-Duplex");
    }
    
    if (miiStatus & BMSR_100HALF2) {
      NVLOGC (1, " 100BaseT2(half)");
      (void) addMediumToDictionary (_dictionary, kIOMediumEthernet100BaseT2 | kIOMediumOptionHalfDuplex, 100 * 1000000, 3, "100BaseT2");
    }
    
    if (miiStatus & BMSR_10HALF) {
      NVLOGC (1, " 10BaseT(half)");
      (void) addMediumToDictionary (_dictionary, kIOMediumEthernet10BaseT | kIOMediumOptionHalfDuplex, 10 * 1000000, 4, "10BaseT");
      if (_pauseFlags & NV_PAUSEFRAME_TX_CAPABLE)
        (void) addMediumToDictionary (_dictionary, kIOMediumEthernet10BaseT | kIOMediumOptionHalfDuplex | kIOMediumOptionFlowControl, 10 * 1000000, 5, "10BaseT");
    }
    
    if (miiStatus & BMSR_10FULL) {
      NVLOGC (1, " 10BaseT(full)");
      (void) addMediumToDictionary (_dictionary, kIOMediumEthernet10BaseT | kIOMediumOptionFullDuplex, 10 * 1000000, 6, "10BaseT Full-Duplex");
      if (_pauseFlags & NV_PAUSEFRAME_TX_CAPABLE)
        (void) addMediumToDictionary (_dictionary, kIOMediumEthernet10BaseT | kIOMediumOptionFullDuplex | kIOMediumOptionFlowControl, 10 * 1000000, 7, "10BaseT Full-Duplex,Flow Control");
    }
    
    if (miiStatus & BMSR_100FULL) {
      NVLOGC (1, " 100BaseTX(full)");
      (void) addMediumToDictionary (_dictionary, kIOMediumEthernet100BaseTX | kIOMediumOptionFullDuplex, 100 * 1000000, 8, "100BaseTX Full-Duplex");
      if (_pauseFlags & NV_PAUSEFRAME_TX_CAPABLE)
        (void) addMediumToDictionary (_dictionary, kIOMediumEthernet100BaseTX | kIOMediumOptionFullDuplex | kIOMediumOptionFlowControl, 100 * 1000000, 9, "100BaseTX Full-Duplex,Flow Control");
      (void) addMediumToDictionary (_dictionary, kIOMediumEthernet100BaseTX | kIOMediumOptionFullDuplex | kIOMediumOptionLoopback, 100 * 1000000, 16, "100BaseTX Hardware Loopback");
    }
    
    if (miiStatus & BMSR_100HALF) {
      NVLOGC (1, " 100BaseTX(half)");
      (void) addMediumToDictionary (_dictionary, kIOMediumEthernet100BaseTX | kIOMediumOptionHalfDuplex, 100 * 1000000, 10, "100BaseTX");
      if (_pauseFlags & NV_PAUSEFRAME_TX_CAPABLE)
        (void) addMediumToDictionary (_dictionary, kIOMediumEthernet100BaseTX | kIOMediumOptionHalfDuplex | kIOMediumOptionFlowControl, 100 * 1000000, 11, "100BaseTX Flow Control");
    }
    
    if (miiStatus & BMSR_100BASE4) {
      NVLOGC (1, " 100BaseT4");
      (void) addMediumToDictionary (_dictionary, kIOMediumEthernet100BaseT4, 100 * 1000000, 12, "100Base4");
    }
    
    if (_gigabit) {
      NVLOGC (1, " 1000BaseT");
      (void) addMediumToDictionary (_dictionary, kIOMediumEthernet1000BaseT | kIOMediumOptionFullDuplex, 1000 * 1000000, 13, "1000BaseT");
      if (_pauseFlags & NV_PAUSEFRAME_TX_CAPABLE)
        (void) addMediumToDictionary (_dictionary, kIOMediumEthernet1000BaseT | kIOMediumOptionFullDuplex | kIOMediumOptionFlowControl, 1000 * 1000000, 14, "1000BaseT Flow Control");
      (void) addMediumToDictionary (_dictionary, kIOMediumEthernet1000BaseT | kIOMediumOptionFullDuplex | kIOMediumOptionLoopback, 1000 * 1000000, 15, "1000BaseT Hardware Loopback");
    } else
      
      NVLOGC (1, "\n");
    
    publishMediumDictionary(_dictionary);
    setCurrentMedium (IONetworkMedium::getMediumWithType(_dictionary, kIOMediumEthernetAuto));
    
    return 0;    
  } while (0);
  
  NVLOG (0, "PHY init failed\n");
  
  return PHY_ERROR;
}
