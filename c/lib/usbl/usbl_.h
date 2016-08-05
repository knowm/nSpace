////////////////////////////////////////////////////////////////////////
//
//										USBL_.H
//
//				Implementation include file for USB library
//
////////////////////////////////////////////////////////////////////////

#ifndef	USBL__H
#define	USBL__H

// Includes
#include "usbl.h"
#include "../../lib/nspcl/nspcl.h"

// External API
#include <winusb.h>

///////////
// Objects
///////////


/////////
// Nodes
/////////

//
// Class - Device.  USB device node.
//

class Device :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Device ( void );										// Constructor

	// Run-time data
	IResource	*pRes;									// Device resource
	WINUSB_INTERFACE_HANDLE		
					hIntf;									// Interface handle
	adtInt		iAltSet;									// Alternate setting number

	// CCL
	CCL_OBJECT_BEGIN(Device)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_RCP(Close)
	DECLARE_EMT(Device)
	DECLARE_EMT(Error)
	DECLARE_RCP(Open)
	DECLARE_CON(Query)
	DECLARE_RCP(Setting)
	DECLARE_RCP(Stream)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Close)
		DEFINE_EMT(Device)
		DEFINE_EMT(Error)
		DEFINE_RCP(Open)
		DEFINE_CON(Query)
		DEFINE_RCP(Setting)
		DEFINE_RCP(Stream)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Internal utilities
	HRESULT getString ( WINUSB_INTERFACE_HANDLE, S32, adtString & );
	};

//
// Class - Endpoint.  USB Endpoint node.
//

class Endpoint :
	public CCLObject,										// Base class
	public Behaviour,									// Interface
	public ITickable										// Interface
	{
	public :
	Endpoint ( void );									// Constructor

	// Run-time data
	WINUSB_INTERFACE_HANDLE		
					hIntf;									// Interface handle
	adtInt		iPipe;									// Pipe Id
	IByteStream	*pStmIo;									// I/O stream
	adtInt		iSzIo;									// I/O size
	U8				*pcBfrPkt;								// Packet buffer
	adtInt		iSzPkt;									// Packet size
	HANDLE		hevWr,hevRd;							// I/O events
	adtBool		bAsync;									// Asynchronous reads ?
	IThread		*pThrd;									// Asynchronous read thread
	HANDLE		hevStop;									// Stop event for read thread

	// 'ITickable' members
	STDMETHOD(tick)		( void );
	STDMETHOD(tickAbort)	( void );
	STDMETHOD(tickBegin)	( void );
	STDMETHOD(tickEnd)	( void );

	// CCL
	CCL_OBJECT_BEGIN(Endpoint)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_RCP(Device)
	DECLARE_RCP(Endpoint)
	DECLARE_EMT(Error)
	DECLARE_CON(Read)
	DECLARE_RCP(Stream)
	DECLARE_CON(Write)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Device)
		DEFINE_RCP(Endpoint)
		DEFINE_EMT(Error)
		DEFINE_CON(Read)
		DEFINE_RCP(Stream)
		DEFINE_CON(Write)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Internal utilities
	HRESULT	pktIo  ( BOOL, DWORD, DWORD, DWORD * );
	void		update ( IDictionary * );					// Update internal state

	};

#endif
