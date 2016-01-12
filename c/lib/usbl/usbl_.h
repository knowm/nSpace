////////////////////////////////////////////////////////////////////////
//
//										WMIL_.H
//
//				Implementation include file for WMI library
//
////////////////////////////////////////////////////////////////////////

#ifndef	WMIL__H
#define	WMIL__H

// Includes
#include	"WMIL.h"
#include "../../lib/nspcl/nspcl.h"

// API
#undef	INITGUID
#include <WbemCli.h>

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
	public IBehaviour										// Interface
	{
	public :
	Device ( void );										// Constructor

	// Run-time data

	// CCL
	CCL_OBJECT_BEGIN(Device)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_RCP(Stream)
	DECLARE_EMT(Error)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Stream)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()
	};

#endif
