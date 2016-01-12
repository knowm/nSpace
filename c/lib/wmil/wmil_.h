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
// Class - Enumerator.  Enumerator node for classo objects and
//								properties within an object.
//

class Enumerator :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Enumerator ( void );									// Constructor

	// Run-time data
	IEnumWbemClassObject
						*pEnum;								// Enumerator object
	IWbemClassObject
						*pObj;								// Class object
	SAFEARRAY		*pNames;								// Enumerated names
	long				lEnumIdx;							// Enumerated index

	// CCL
	CCL_OBJECT_BEGIN(Enumerator)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_RCP(Enumerator)
	DECLARE_RCP(First)
	DECLARE_CON(Next)
	DECLARE_EMT(Last)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Enumerator)
		DEFINE_RCP(First)
		DEFINE_CON(Next)
		DEFINE_EMT(Last)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - Instance.  Node for WBEM class object instance node.
//

class Instance :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Instance ( void );									// Constructor

	// Run-time data
	IWbemClassObject	*pObj;							// Class object
	adtString			strKey;							// Run-time data

	// CCL
	CCL_OBJECT_BEGIN(Instance)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_RCP(Key)
	DECLARE_CON(Load)
	DECLARE_EMT(NotFound)
	DECLARE_RCP(Object)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Key)
		DEFINE_CON(Load)
		DEFINE_EMT(NotFound)
		DEFINE_RCP(Object)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - Locator.  Node for WBEM locator node.
//

class Locator :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Locator ( void );										// Constructor

	// Run-time data

	// CCL
	CCL_OBJECT_BEGIN(Locator)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Connect)
	DECLARE_EMT(Error)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Connect)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - Service.  Management services node.
//

class Service :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Service ( void );										// Constructor

	// Run-time data
	IWbemServices	*pSvc;								// Services object
	adtString		strTxt;								// Query string

	// CCL
	CCL_OBJECT_BEGIN(Service)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_EMT(Error)
	DECLARE_CON(Query)
	DECLARE_RCP(Service)
	DECLARE_RCP(Text)
	BEGIN_BEHAVIOUR()
		DEFINE_EMT(Error)
		DEFINE_CON(Query)
		DEFINE_RCP(Service)
		DEFINE_RCP(Text)
	END_BEHAVIOUR_NOTIFY()
	};

#endif
