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

#include <IOKit/IOLib.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/pci/IOPCIDevice.h>
extern "C" {
#include <sys/kpi_mbuf.h>
}
#include <IOKit/IOService.h>
#include <IOKit/IOFilterInterruptEventSource.h>
#include <IOKit/network/IOEthernetController.h>
#include <IOKit/network/IOMbufMemoryCursor.h>
#include <IOKit/network/IOBasicOutputQueue.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/network/IOEthernetStats.h>
#include <IOKit/network/IOKernelDebugger.h>
#include <IOKit/IOCommandGate.h>

/*extern "C" {
  #include <pexpert/pexpert.h>//This is for debugging purposes ONLY
  }*/

#include "defines.h"
#include "forcedeth_hw.h"

#define FORCEDETH_VERSION "0.64.6"

struct RingDesc {
	UInt32	PacketBuffer;
	UInt32	FlagLen;
};

struct RingDescEx {
	UInt32  PacketBufferHigh;
	UInt32  PacketBufferLow;
	UInt32  TxVlan;
	UInt32  FlagLen;
};

struct io_rings {
	struct RingDesc *orig;
	struct RingDescEx *ex;
	
	IOBufferMemoryDescriptor *m_desc;
	IOPhysicalAddress  p_addr;
	void              *v_addr;
	size_t             f_size;
	mbuf_t             m_buf;
	
	struct io_rings *changeOwnerNext;
	struct io_rings *changeOwnerPrev;	
};

typedef struct mclist {
	UInt32 a, b;
	struct mclist *next;
} multicast_list_t;

/*
 * Crossover Detection
 * Realtek 8201 phy + some OEM boards do not work properly.
 */
enum {
	NV_CROSSOVER_DETECTION_DISABLED,
	NV_CROSSOVER_DETECTION_ENABLED
};

enum {
	NV_OPTIMIZATION_MODE_THROUGHPUT = 0,
	NV_OPTIMIZATION_MODE_CPU,
	NV_OPTIMIZATION_MODE_DYNAMIC
};

/* as suggested by the IOKit fundamentals documentation there are 2 power states (Off and On) */
enum {
  kforcedethPowerStateOff = 0,
  kforcedethPowerStateOn,
  kforcedethPowerStateCount
};

#define NVLOG(l, f, args...)			            \
do {						                        \
    if ((l) <= _logLevel) {                         \
        if (_interface) {                           \
            IOLog ("%s(en%d): " f, getName (), _interface->getUnitNumber (), ## args); \
        } else {                                    \
            IOLog ("%s: " f, getName (), ## args);	\
        }                                           \
    }                                               \
} while (0)

/* log without printing the interface/module id (for continuing log line) */
#define NVLOGC(l, f, args...)			\
do {						            \
    if ((l) <= _logLevel) {             \
        IOLog (f, ## args);             \
    }                                   \
} while (0)

#if defined(DEBUG)
#define NVLOG_DEBUG(l, f, args...)                \
do {                                              \
    if ((l) <= _logLevel) {                       \
        if (_interface) {                         \
            IOLog ("%s(en%d) (%s:%d) DEBUG: " f, getName (), (int)_interface->getUnitNumber (), __func__, __LINE__, ## args); \
        } else {                                  \
            IOLog ("%s (%s:%d) DEBUG: " f, getName (), __func__, __LINE__, ## args); \
        }                                         \
    }                                             \
} while (0)
#else
#define NVLOG_DEBUG(l, f, args...) do {} while (0)
#endif

#define unlikely(x) __builtin_expect(!!(x), 0)
#define likely(x)   __builtin_expect(!!(x), 1)

class nForceLAN : public IOEthernetController
{
	OSDeclareDefaultStructors(nForceLAN);
 private:
	IOMemoryMap              *_map;
	char                     *_baseAddress;
	IOEthernetAddress         _macAddr;
	IOPCIDevice              *_device;
	
	IONetworkInterface       *_interface;
	IONetworkStats           *_stats;
	IOEthernetStats          *_ethernetStats;
	IOOutputQueueStats       *_outputStats;

	OSDictionary             *_dictionary;
	
	UInt16                    _vendorID, _deviceID, _subVendorID, _subDeviceID;
	UInt8                     _revisionID;
	
	IOWorkLoop               *_workLoop;
	IOTimerEventSource       *_nicPollTimer, *_nicStatTimer, *_nicLinkTimer;
	int                       _numInterrupts;
	IOInterruptEventSource   *_interrupts[4];
	
	IOBasicOutputQueue       *_outputQueue;
	
	bool                      _ifEnabled, _promiscuousMode, _multicastMode, _inEnable, _inShutdown;
	UInt32                    _alwaysOn[2], _alwaysOff[2];
	int                       _mcList;
	
	UInt32                    _mtu;
	
	/* General data  */
	UInt32                    _linkspeed;
	bool                      _duplex, _autoneg, _wolEnabled;
	int                       _fixedMode, _physicalAddress;
	UInt16                    _gigabit;

	/* input parameters */
	bool                      _lowPowerSpeed;    /* LowPowerSpeed: set the nic to low speed on power down. Default: true */
	bool                      _useTimerIRQ;      /* UseTimerIRQ: enable timer IRQ on buggy NICs. Default: true */
	bool                      _disableMSI;       /* DisableMSI: use legacy interrupts. Default: false */
	bool                      _disableTimerIRQ;  /* DisableTimerIRQ: don't use the timer IRQ. use this option only if your NIC reliably produces tx done irqs */
	UInt32                    _logLevel;         /* LogLevel: set the detail of debugging messages (0 = None, 1 = Basic, 2-4 = Detailed). Default: 0 */
	UInt32                    _optimizationMode; /* OptimizationMode: 0 = throughput, 1 = cpu. Default: cpu */
	UInt32                    _rxRingLen;        /* RXRingSize: length of rx ring (see defines.h for limits). Default: 512 */
	UInt32                    _txRingLen;        /* TXRingSize: length of tx ring (see defines.h for limits). Default: 1024 */
	int                       _phyCross;         /* CrossoverDetection: enable crossover detection. Default: true */
	
	UInt32                    _deviceFlags; /* flags are defined in defines.h */
	const char               *_deviceString;
	UInt32                    _msiFlags;    /* MSI support is not implemented */
	UInt32                    _vlanCtlBits; /* VLan support is not implemented */
	UInt32                    _pauseFlags, _msiEnable;
	bool                      _txLimit, _useExtendedRing, _recoverError;
	UInt32                    _descVers, _registerSize;
	UInt32                    _macInUse;
	UInt32                    _phyOui, _phyModel, _phyRev;
	UInt32                    _txPktsInProgress;
	bool                      _hasHWCRC;
	UInt32                    _ledStats[3];
	bool                      _haveMgmtSema, _phyPowerDown, _macOverride;
	int                       _mgmtVersion, _quietCount;
	unsigned long long        _physMemMask;

	/* General data: RO fields */
	UInt32                    _irqMask, _txrxCtlBits, _nicPollIRQ;
	UInt32                    _origMac[2];
	
	/* rx/tx */
	IOBufferMemoryDescriptor *_rxMemDesc, *_txMemDesc;
	IOPhysicalAddress         _rxRingMem, _txRingMem;
	struct io_rings          *_rxRing, *_txRing;
	UInt64                    _curRx, _nextTx, _nicTx;
	UInt32                    _rxAvailableFlag, _txFlags;
	
	unsigned int              _rxBufferSize;
	unsigned int              _packetLimit;

	struct io_rings          *_txChangeOwnerHead;
	struct io_rings          *_txChangeOwnerTail;
	
	IOMbufNaturalMemoryCursor *_txMbufCursor;
	
	static const int _maxInterruptWork = 15;
	
	/* power management */
	int            _pmPowerState, _pmSetState;
	bool           _pmPowerStateChanged;
	thread_call_t  _pmThreadCall;
	IOService     *_pmPolicyMaker;
	UInt8          _pmPCICapabilityPtr;
	UInt32         _savedRegPhyInterface;
	
	/* internal functions */
	bool regDelay (int offset, UInt32 mask, UInt32 target, int delay, int delayMax, const char *msg);
	int miiRW (int addr, int miireg, int value);

	/* init */
	bool mapMemoryRegisters (void);
	bool getMACAddressFromHardware (void);
	bool getAndInitializePhy (void);
	void mapInterrupts (void);
	bool setupEventSources (void);

	/* phy */
	bool phyReset();
	bool phyReset(UInt32 miiControl);
	int phyInit (void);
	
	void copyMacToHW();

	inline bool initRx (void);
	inline bool initTx (void);
	bool initRing (void);
	
	bool releaseRingEntry (struct io_rings *ring, int index);
	inline void drainTx (void);
	inline void drainRx (void);
	void drainRing (void);
	
	void setBufSize();
	
	void startRx (void);
	void startTx (void);
	void stopRx (void);
	void stopTx (void);
	void txrxReset();
	void txrxGate (bool gate);
	
	/* process incomming packet */
	int getLength(mbuf_t buf, int len);
	UInt32 rxProcess (int maxWork);
	UInt32 rxProcessOrig (int maxWork);
	UInt32 rxProcessOptimized (int maxWork);
	inline bool rxCheckFlags_v1 (int rx_index, UInt32 flags, UInt32 *len_ptr);
	inline bool rxCheckFlags_v2 (int rx_index, UInt32 flags, UInt32 *len_ptr);
	
	/* link */
	bool updateLinkSpeed();
	void linkIRQ ();
	
	/* legacy interrupts */
	void nicIRQ();
	bool changeInterruptMode (int totalWork);

	/* MSI interrupts */
	void nicIRQRx (void);
	void nicIRQRxOptimized (void);
	void nicIRQTx (void);
	void nicIRQOther (void);
	void addMSIVectorMap (UInt32 vector, UInt32 irqmask, UInt32 *reg0_val, UInt32 *reg1_val);
	
	/* transmission completion */
	UInt32 txDone(int maxWork);
	UInt32 txDoneOrig (int maxWork);
        UInt32 txDoneEx (int maxWork);
	void txCheckFlags_v1 (UInt32 flags);
	void txCheckFlags_v2 (UInt32 flags);
	void txFlipOwnership (void);
	
	void doNicPoll();
	void doLinkTimer (void);
	void recoverIRQ (void);
	void gatherStats (void);
	
	void updateMulticast(UInt32 pff, UInt32 addrA, UInt32 addrB, UInt32 maskA, UInt32 maskB);
	IOReturn setMode (bool multicast, bool promiscuous);

	int managementAcquireSemaphore (void);
	void managementReleaseSemaphore (void);
	bool managementGetVersion (void);

	void MACReset (void);
	void legacyBackoffReseed (void);
	void gearBackoffReseed (void);
	void enableHWInterrupts (UInt32 mask);
	void disableHWInterrupts (UInt32 mask);
	void updatePause (UInt32 pause_flags);
	void setupHWRings (int rxtx_flags);
	void msiWorkaround (void);
	bool initPCIConfigSpace (void);

	/* descriptor functions */
	bool allocateDescriptor (struct io_rings *ring, IOBufferMemoryDescriptor **memDescPtr, IOPhysicalAddress *phyMem, UInt32 ringLen);
	void deallocateDescriptor (IOBufferMemoryDescriptor **ppMemDesc);

	bool allocateRings (void);
	void deallocateRings (void);
	bool getIORing (IOBufferMemoryDescriptor **pMemDesc, IOPhysicalAddress *pAddrPtr, struct io_rings **ringPtr, UInt32 ringLen);

	/* LED */
	void saveLEDStats (void);
	void restoreLEDStats (void);
	void LEDOn (void);
	void LEDOff (void);
	void LEDSetState (UInt32 state);

	/* sleep */
	void resume (void);
	void suspend (void);
	void setLowSpeed (void);

	/* forcedeth_probe.cpp */
	void deviceSetup (void);
	bool deviceDetect (void);

  void restorePHY (void);
  void restoreMAC (void);
  
	/* inline functions */
	inline UInt32 dmaLow (UInt64 address) {
		return (UInt32)(address & 0xffffffffLL);
	}
	
	inline UInt32 dmaHigh (UInt64 address) {
		return (UInt32)(address >> 32);
	}

	inline volatile UInt32 readRegister(UInt16 offset) {
		return OSReadLittleInt32 ((void *)_baseAddress, offset);
	}
	
	inline void writeRegister (UInt16 offset, UInt32 data) {
		OSWriteLittleInt32 ((void *)_baseAddress, offset, data);
	}

	inline UInt32 descrGetLength (RingDesc *prd, UInt32 descVers) {
		return (OSSwapLittleToHostInt32(prd->FlagLen) & (descVers == DESC_VER_1 ? LEN_MASK_V1 : LEN_MASK_V2));
	}
	
	inline UInt32 descrGetLength (RingDescEx *prd) {
		return (OSSwapLittleToHostInt32(prd->FlagLen) & LEN_MASK_V2);
	}
	
	inline void pciPush (void) {
		(void)readRegister (0);
	}

	inline void stopRxTx (void) {
		stopRx ();
		stopTx ();
	}
	
	inline void startRxTx (void) {
		startRx ();
		startTx ();
	}
	
	inline void setupRingDescriptor (struct io_rings *ring, int index, UInt32 flaglen) {
		if (_useExtendedRing) {
			ring[index].ex->FlagLen          = OSSwapHostToLittleInt32(flaglen);
			/* some NICs require we rewrite these values */
			ring[index].ex->PacketBufferLow  = OSSwapHostToLittleInt32(dmaLow ((UInt64)ring[index].p_addr));
			ring[index].ex->PacketBufferHigh = OSSwapHostToLittleInt32(dmaHigh ((UInt64)ring[index].p_addr));
			ring[index].ex->TxVlan           = 0;
		} else {
			ring[index].orig->FlagLen = OSSwapHostToLittleInt32(flaglen);
			ring[index].orig->PacketBuffer = OSSwapHostToLittleInt32((UInt32)ring[index].p_addr);
		}
	}

 protected:
	virtual IOOutputQueue *createOutputQueue();
	/*virtual bool createWorkLoop();*/
		
 public:
	/*virtual IOWorkLoop* getWorkLoop() const;*/
  virtual UInt32 getFeatures (void) const;

	virtual UInt32 outputPacket(mbuf_t buf, void *param);
	virtual bool init(OSDictionary *dictionary = 0);
	virtual void free(void);
	virtual IOService *probe(IOService *provider, SInt32 *score);
	const OSString* newVendorString (void) const;
	const OSString* newModelString (void) const;
	virtual bool start(IOService *provider);
	virtual void stop(IOService *provider);
  virtual IOReturn enable(IONetworkInterface *interface);
  virtual IOReturn disable(IONetworkInterface *interface);

	virtual IOReturn getHardwareAddress(IOEthernetAddress * addrP);
	virtual IOReturn setHardwareAddress(const IOEthernetAddress * addrP);
	virtual IOReturn getChecksumSupport(UInt32 *checksumMask, UInt32 checksumFamily, bool isOutput);
	virtual IOReturn setMulticastMode(bool active);
	virtual IOReturn setPromiscuousMode(bool active);
	virtual IOReturn setMulticastList (IOEthernetAddress *addrs, UInt32 count);

	virtual IOReturn setMaxPacketSize (UInt32 maxSize); 
	virtual IOReturn getMaxPacketSize (UInt32 *maxSize) const; 
	virtual IOReturn getMinPacketSize (UInt32 *minSize) const;

	virtual IOReturn selectMedium (const IONetworkMedium * medium);

	virtual bool configureInterface (IONetworkInterface *interface);
	
	/* power management */
	virtual IOReturn setPowerState (unsigned long powerStateOrdinal, IOService *policyMaker);
	virtual IOReturn registerWithPolicyMaker (IOService *policyMaker);
	virtual IOReturn getPacketFilters (const OSSymbol *group, UInt32 *filters) const;
	virtual IOReturn setWakeOnMagicPacket(bool active);

	virtual IOReturn powerStateWillChangeTo (IOPMPowerFlags capabilities, unsigned long stateNumber, IOService *whatDevice);
	virtual IOReturn powerStateDidChangeTo (IOPMPowerFlags capabilities, unsigned long stateNumber, IOService *whatDevice);

	void setPowerState2 (void);
	bool validEvent (void);
	bool validEvent (UInt32 irqMask);
};
