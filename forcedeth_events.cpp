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
 * Copyright (c) 2008 Nathan Hjelm (Darwin port)
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

/* event filters */
bool nForceLAN::validEvent (UInt32 irqMask) {
  UInt32 regValue = readRegister (NvRegIrqStatus);
  
  NVLOG_DEBUG (2, "Received an event. Checking %08x vs %08x\n", regValue, irqMask);
  
  return (regValue & irqMask);
}

bool nForceLAN::validEvent (void) {
  /* ignore line interrupts when MSI is enabled */
  if (_msiFlags & NV_MSI_ENABLED)
    return false;
  
  if (readRegister (NvRegIrqStatus) & _irqMask)
    return true;
	
  return false;
}

bool interruptFilter (OSObject *target, IOFilterInterruptEventSource * src) {
  return ((nForceLAN *)target)->validEvent ();
}

bool interruptFilterRx (OSObject *target, IOFilterInterruptEventSource * src) {
  return ((nForceLAN *)target)->validEvent (NVREG_IRQ_RX_ALL);
}

bool interruptFilterTx (OSObject *target, IOFilterInterruptEventSource * src) {
  return ((nForceLAN *)target)->validEvent (NVREG_IRQ_TX_ALL);
}

bool interruptFilterOther (OSObject *target, IOFilterInterruptEventSource * src) {
  return ((nForceLAN *)target)->validEvent (NVREG_IRQ_OTHER);
}
/* END -- event filters */

void nForceLAN::msiWorkaround (void) {

  /* Need to toggle the msi irq mask within the ethernet device,
   * otherwise, future interrupts will not be detected.
   */
  if (_msiFlags & NV_MSI_ENABLED) {
    writeRegister (NvRegMSIIrqMask, 0);
    writeRegister (NvRegMSIIrqMask, _msiEnable);
  }
}

/* MSI multiple interrupt handlers */
void nForceLAN::nicIRQRx (void) {
  UInt32 events;
  int i;

  NVLOG_DEBUG (3, "got RX IRQ\n");

  for (i = 0 ; i < _maxInterruptWork; i++) {
    events = readRegister (NvRegIrqStatus) & NVREG_IRQ_RX_ALL;
    writeRegister (NvRegIrqStatus, NVREG_IRQ_RX_ALL);

    if (!events)
      break;

    msiWorkaround ();

    rxProcess (RX_WORK_PER_LOOP);
  }
}

void nForceLAN::nicIRQRxOptimized (void) {
  UInt32 events;
  int i;

  NVLOG_DEBUG (3, "got RX IRQ\n");

  for (i = 0 ; i < _maxInterruptWork; i++) {
    events = readRegister (NvRegIrqStatus) & NVREG_IRQ_RX_ALL;
    writeRegister (NvRegIrqStatus, NVREG_IRQ_RX_ALL);

    if (!events)
      break;

    msiWorkaround ();

    rxProcessOptimized (RX_WORK_PER_LOOP);
  }
}

void nForceLAN::nicIRQTx (void) {
  UInt32 events;
  int i;

  NVLOG_DEBUG (3, "got TX IRQ\n");
  
  for (i = 0 ; i < _maxInterruptWork ; i++) {
    events = readRegister (NvRegIrqStatus) & NVREG_IRQ_TX_ALL;
    writeRegister (NvRegIrqStatus, NVREG_IRQ_TX_ALL);

    if (!events)
      break;

    msiWorkaround ();

    txDone (TX_WORK_PER_LOOP);

    if (events & (NVREG_IRQ_TX_ERR))
      NVLOG(1, "received irq with events 0x%x. Probably TX fail.\n", (unsigned int)events);
  }
}

void nForceLAN::nicIRQOther (void) {
  UInt32 events;
  int i;

  NVLOG_DEBUG (4, "got other IRQ\n");

  for (i = 0 ; i < _maxInterruptWork ; i++) {
    events = readRegister (NvRegIrqStatus) & NVREG_IRQ_OTHER;
    writeRegister (NvRegIrqStatus, NVREG_IRQ_OTHER);

    if (!events)
      break;

    msiWorkaround ();

    /* check tx in case we reached max loop limit in tx isr */
    txDone (TX_WORK_PER_LOOP);

    if (events & NVREG_IRQ_LINK) {
      linkIRQ();
    }

    if (events & NVREG_IRQ_RECOVER_ERROR) {
      recoverIRQ ();
      break;
    }
  }
}
/* END -- MSI multiple interrupt handlers */

/* Line and single MSI interrupt handler */
void nForceLAN::nicIRQ (void) {
  int i;
  UInt32 events;
  int work, totalWork;

  if (_pmPowerState != kforcedethPowerStateOn && !_wolEnabled)
    return;

  /* removed MSI X register reads. readd them once OSX supports MSI-X */
  events = readRegister (NvRegIrqStatus) & _irqMask;
  writeRegister (NvRegIrqStatus, NVREG_IRQSTAT_MASK);

  if (!events)
    return;

  msiWorkaround ();

  for (i = 0, totalWork = 0 ; i < _maxInterruptWork ; i++) {
    work = rxProcess (RX_WORK_PER_LOOP);
    work += txDone (TX_WORK_PER_LOOP);

    if (!work)
      break;

    totalWork += work;
  }

  if (changeInterruptMode (totalWork))
    /* setup new irq mask */
    writeRegister (NvRegIrqMask, _irqMask);

  if (events & NVREG_IRQ_TX_ALL)
    _ethernetStats->dot3TxExtraEntry.interrupts++;

  if (events & NVREG_IRQ_RX_ALL)
    _ethernetStats->dot3RxExtraEntry.interrupts++;

  if (events & NVREG_IRQ_LINK)
    linkIRQ();
  
  if (events & NVREG_IRQ_RECOVER_ERROR)
    recoverIRQ ();
}

void nForceLAN::recoverIRQ (void) {	
  /* disable interrupts on the nic */
  writeRegister (NvRegIrqMask, readRegister (NvRegIrqMask));
  pciPush ();

  _recoverError = false;
  NVLOG_DEBUG (0, "MAC in recoverable error state\n");
  if (_ifEnabled) {
    /* stop engines */
    stopRxTx ();

    txrxReset ();
    /* drain rx queue */
    drainRing ();
    /* reinit driver view of the rx queue */
    setBufSize ();
    if (!initRing() && !_inShutdown)
      _nicPollTimer->setTimeoutMS (OOM_REFILL);

    /* reinit nic view of the rx queue */
    writeRegister (NvRegOffloadConfig, _rxBufferSize);
    setupHWRings (NV_SETUP_RX_RING | NV_SETUP_TX_RING);
    writeRegister (NvRegRingSizes, ((_rxRingLen-1) << NVREG_RINGSZ_RXSHIFT) + ((_txRingLen-1) << NVREG_RINGSZ_TXSHIFT));
    writeRegister (NvRegTxRxControl, NVREG_TXRXCTL_KICK | _txrxCtlBits);
    
    /* restart rx engine */
    startRxTx ();
  }
	
  /* re-enable interrupts */
  enableHWInterrupts (_irqMask);	
}

/* timer callbacks */
void nForceLAN::doNicPoll() {
	if (!initRing ())
      _nicPollTimer->setTimeoutMS (OOM_REFILL);
}

void nForceLAN::doLinkTimer (void) {
  NVLOG_DEBUG (1, "link timer expired. checking link status.\n");
  
  updateLinkSpeed ();
  
  _nicLinkTimer->setTimeoutMS(LINK_TIMEOUT);
}

void nForceLAN::gatherStats (void) {
  if (_ethernetStats) {
    _ethernetStats->dot3StatsEntry.singleCollisionFrames     += readRegister (NvRegTxOneReXmt);
    _ethernetStats->dot3StatsEntry.multipleCollisionFrames   += readRegister (NvRegTxManyReXmt);
    _ethernetStats->dot3StatsEntry.lateCollisions            += readRegister (NvRegTxLateCol);
    _ethernetStats->dot3StatsEntry.internalMacTransmitErrors += readRegister (NvRegTxUnderflow);
    _ethernetStats->dot3StatsEntry.carrierSenseErrors        += readRegister (NvRegTxLossCarrier);
    _ethernetStats->dot3StatsEntry.excessiveCollisions       += readRegister (NvRegTxExcessDef);
    _ethernetStats->dot3StatsEntry.internalMacTransmitErrors += readRegister (NvRegTxRetryErr);
    _ethernetStats->dot3StatsEntry.internalMacReceiveErrors  += readRegister (NvRegRxFrameErr);
    _ethernetStats->dot3RxExtraEntry.collisionErrors         += readRegister (NvRegRxLateCol);
    _ethernetStats->dot3RxExtraEntry.frameTooShorts          += readRegister (NvRegRxRunt);
    _ethernetStats->dot3StatsEntry.frameTooLongs             += readRegister (NvRegRxFrameTooLong);
    _ethernetStats->dot3RxExtraEntry.overruns                += readRegister (NvRegRxOverflow);
    _ethernetStats->dot3StatsEntry.fcsErrors                 += readRegister (NvRegRxFCSErr);
    _ethernetStats->dot3StatsEntry.alignmentErrors           += readRegister (NvRegRxFrameAlignErr);
    _ethernetStats->dot3StatsEntry.internalMacReceiveErrors  += readRegister (NvRegRxLenErr);
    
    if (_deviceFlags & DEV_HAS_STATISTICS_V2) {
      _ethernetStats->dot3StatsEntry.deferredTransmissions   += readRegister (NvRegTxDef);
      _ethernetStats->dot3StatsEntry.missedFrames            += readRegister (NvRegRxDropFrame);
    }
  }
  
  if (_nicStatTimer)
    _nicStatTimer->setTimeoutMS (STATS_INTERVAL);
}

/* Message Signaled Interrupt (MSI) setup code */
void nForceLAN::addMSIVectorMap (UInt32 vector, UInt32 irqmask, UInt32 *reg0_val, UInt32 *reg1_val) {
  int i;
  UInt32 msimap = 0;

  /* Each interrupt bit can be mapped to a MSI vector (4 bits).
   * MSIMap0/MSIXMap0 represent the first 8 interrupts and MSIMap1/MSIXMap1 represent
   * the remaining 8 interrupts.
   */
  for (i = 0; i < 8; i++)
    if ((irqmask >> i) & 0x1)
      msimap |= vector << (i << 2);

  reg0_val[0] |= msimap;

  msimap = 0;
  for (i = 0; i < 8; i++)
    if ((irqmask >> (i + 8)) & 0x1)
      msimap |= vector << (i << 2);

  reg1_val[0] |= msimap;
}

void nForceLAN::mapInterrupts (void) {
  UInt32 reg0_val, reg1_val;

  if (_msiFlags & NV_MSI_X_ENABLED) {
    /* not supported by 10.4/10.5/10.6 */
  } else if (_msiFlags & NV_MSI_ENABLED) {
    writeRegister (NvRegMSIMap0, 0);
    writeRegister (NvRegMSIMap1, 0);
		
    _msiEnable = NVREG_MSI_VECTOR_0_ENABLED;
    if (_optimizationMode == NV_OPTIMIZATION_MODE_THROUGHPUT) {
      /* map interrupts to vectors 0, 1, and 2 */
      reg0_val = reg1_val = 0;
      addMSIVectorMap (NV_MSI_VECTOR_RX,    NVREG_IRQ_RX_ALL, &reg0_val, &reg1_val);
      addMSIVectorMap (NV_MSI_VECTOR_TX,    NVREG_IRQ_TX_ALL, &reg0_val, &reg1_val);
      addMSIVectorMap (NV_MSI_VECTOR_OTHER, NVREG_IRQ_OTHER , &reg0_val, &reg1_val);

      writeRegister (NvRegMSIMap0, reg0_val);
      writeRegister (NvRegMSIMap1, reg1_val);

      _msiEnable |= NVREG_MSI_VECTOR_1_ENABLED | NVREG_MSI_VECTOR_2_ENABLED;
    }
  }
}
/* END -- Message Signaled Interrupt (MSI) setup code */
bool nForceLAN::changeInterruptMode (int totalWork) {
  if (_optimizationMode == NV_OPTIMIZATION_MODE_DYNAMIC) {
    if (totalWork > NV_DYNAMIC_THRESHOLD) {
			NVLOG_DEBUG(0, "Reached threshold. Switching to CPU optimization mode. totalWork = %d, threshold = %d\n", totalWork, NV_DYNAMIC_THRESHOLD);
      /* transition to poll based interrupts */
      _quietCount = 0;
      if (_irqMask != NVREG_IRQMASK_CPU) {
				_irqMask = NVREG_IRQMASK_CPU;
				return true;
      }
    } else {
      if (_quietCount <= NV_DYNAMIC_MAX_QUIET_COUNT) {
				/* reached a period of low activity, switch
				 to per tx/rx packet interrupts */
				if (_irqMask != NVREG_IRQMASK_THROUGHPUT) {
					_irqMask = NVREG_IRQMASK_THROUGHPUT;
					return true;
				}
      } else
				_quietCount++;
		}
  }

  return false;
}

/* Setup all interrupts and timer events */
bool nForceLAN::setupEventSources (void) {
  int i, type;
  IOReturn ret;
  
  _workLoop = getWorkLoop();
  _workLoop->retain();
	
  if (!_workLoop) {
    NVLOG (0, "failed to get workloop.\n");
    return false;
  }
  
  _msiFlags &= ~(NV_MSI_ENABLED | NV_MSI_X_ENABLED);

  if (!_disableMSI && (_msiFlags & NV_MSI_CAPABLE)) {
    /* try to locate 3 MSI interrupts if we are using multiple interrupts */
    for (i = 0 ; i < 4 ; i++) {
      ret = _device->getInterruptType (i, &type);
      if (ret == kIOReturnNoInterrupt)
        break;
    }
  
    if (i == 1) {
      NVLOG (1, "no DMA interrupts found. using legacy interrupt.\n");
      _disableMSI = true;
    } else if (_optimizationMode == NV_OPTIMIZATION_MODE_THROUGHPUT && i < 3) {
      NVLOG (1, "fewer than 3 interrupts are available. changing optimization mode to cpu.\n");
      _optimizationMode = NV_OPTIMIZATION_MODE_CPU;
    }
  }

  if (_disableMSI || !(_msiFlags & NV_MSI_CAPABLE)) {
    /* use a legacy interrupt */
    _numInterrupts = 1;

    _interrupts[0] = IOFilterInterruptEventSource::filterInterruptEventSource(this, OSMemberFunctionCast(IOInterruptEventSource::Action, this, &nForceLAN::nicIRQ), interruptFilter, _device, 0);
  } else {
    /* use DMA interrupts */
		    
    if (_optimizationMode == NV_OPTIMIZATION_MODE_THROUGHPUT) {
      _numInterrupts = 3;

      NVLOG (1, "using multiple DMA interrupts\n");

      if (_useExtendedRing)
        _interrupts[NV_MSI_VECTOR_RX] = IOFilterInterruptEventSource::filterInterruptEventSource(this, OSMemberFunctionCast(IOInterruptEventSource::Action, this, &nForceLAN::nicIRQRxOptimized), interruptFilterRx, _device, 1);
      else
        _interrupts[NV_MSI_VECTOR_RX] = IOFilterInterruptEventSource::filterInterruptEventSource(this, OSMemberFunctionCast(IOInterruptEventSource::Action, this, &nForceLAN::nicIRQRx), interruptFilterRx, _device, 1);
      
      _interrupts[NV_MSI_VECTOR_TX] = IOFilterInterruptEventSource::filterInterruptEventSource(this, OSMemberFunctionCast(IOInterruptEventSource::Action, this, &nForceLAN::nicIRQTx), interruptFilterTx, _device, 2);
      _interrupts[NV_MSI_VECTOR_OTHER] = IOFilterInterruptEventSource::filterInterruptEventSource(this, OSMemberFunctionCast(IOInterruptEventSource::Action, this, &nForceLAN::nicIRQOther), interruptFilterOther, _device, 3);
    } else {
      _numInterrupts = 1;
			
      NVLOG (1, "using a single DMA interrupt\n");
			
      _interrupts[NV_MSI_VECTOR_ALL] = IOInterruptEventSource::interruptEventSource(this, OSMemberFunctionCast(IOInterruptEventSource::Action, this, &nForceLAN::nicIRQ), _device, 1);
    }

    _msiFlags |= NV_MSI_ENABLED;
  }
  /* MSI-X support when OSX adds support for MSI-X interrupts */

  mapInterrupts();
	
  for (i = 0 ; i < _numInterrupts ; i++) {
    if (_workLoop->addEventSource(_interrupts[i]) != kIOReturnSuccess) {
      NVLOG (0, "could not add an interrupt to the workloop!\n");
      return false;
    }
  }
	
  /* 
     need to enable the interrupt source now to prevent interrupts
     from other devices on the same line being lost.
  */
  _workLoop->enableAllInterrupts ();

  _nicPollTimer = IOTimerEventSource::timerEventSource(this, OSMemberFunctionCast(IOTimerEventSource::Action, this, &nForceLAN::doNicPoll));
  if (_workLoop->addEventSource(_nicPollTimer) != kIOReturnSuccess) {
    NVLOG (0, "cannot set up nic poll timer event source.\n");
    return false;
  }

  if (_deviceFlags & DEV_NEED_LINKTIMER) {
    NVLOG(1, "creating link timer.\n");
    _nicLinkTimer = IOTimerEventSource::timerEventSource(this, OSMemberFunctionCast(IOTimerEventSource::Action, this, &nForceLAN::doLinkTimer));
    if (_workLoop->addEventSource(_nicLinkTimer) != kIOReturnSuccess) {
      NVLOG (0, "cannot set up nic link timer event source.\n");
      return false;
    }
  } else
    _nicLinkTimer = NULL;

  return true;

  _nicStatTimer = IOTimerEventSource::timerEventSource(this, OSMemberFunctionCast(IOTimerEventSource::Action, this, &nForceLAN::gatherStats));
  if (_workLoop->addEventSource(_nicStatTimer) != kIOReturnSuccess) {
    NVLOG (0, "cannot set up nic statistic timer event source.\n");
    return false;
  }

  return true;
}
