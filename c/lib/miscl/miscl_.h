////////////////////////////////////////////////////////////////////////
//
//										MISCL_.H
//
//			Internal include file for the miscellaneous node library
//
////////////////////////////////////////////////////////////////////////

#ifndef	MISCL__H
#define	MISCL__H

// Includes
#include "miscl.h"

#ifndef	_WIN32
#include <sys/types.h>
#include <sys/time.h>
#endif

//
// Class - AsyncEmit.  An asynchronous emission node.
//

class AsyncEmit :
	public CCLObject,										// Base class
	public IBehaviour,									// Interface
	public ITickable										// Interface
	{
	public :
	AsyncEmit ( void );									// Constructor

	// Run-time data
	IThread		*pThrd;									// Thread
	adtValue		vVal,vEmit;								// Emission value
	bool			bEmit;									// Emitting ?

	// 'ITickable' members
	STDMETHOD(tick)		( void );
	STDMETHOD(tickAbort)	( void ) { return S_OK; }
	STDMETHOD(tickBegin)	( void ) { return S_OK; }
	STDMETHOD(tickEnd)	( void ) { return S_OK; }

	// CCL
	CCL_OBJECT_BEGIN(AsyncEmit)
		CCL_INTF(IBehaviour)
		CCL_INTF(ITickable)
	CCL_OBJECT_END()
	virtual void		destruct		( void );		// Destruct object

	// Connections
	DECLARE_CON(Fire)
	DECLARE_RCP(Value)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
		DEFINE_RCP(Value)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - AsyncQ.  An asynchronous queue node.
//

class AsyncQ :
	public CCLObject,										// Base class
	public IBehaviour,									// Interface
	public ITickable										// Interface
	{
	public :
	AsyncQ ( void );										// Constructor

	// Run-time data
	IThread		*pThrd;									// AsyncQ thread
	bool			bRun;										// AsyncQ thread run ?
	IList			*pQw;										// Work queue
	IIt			*pQwIt;									// Work queue iterators
	IDictionary	*pQs;										// Queues by Id
	IDictionary	*pQvs;									// Latest queue values by Id
	adtInt		iMaxSz;									// Maximum queue sizes
	adtBool		bBlock;									// Block on full queue ?
	sysCS			csWork;									// Work protection
	sysEvent		evWork;									// Work event
	adtValue		vQId;										// Queue Id
	adtValue		vIdE,vQE;								// Internal
	adtValue		vQ;										// Queue variable
	adtIUnknown	unkV;										// Internal

	// 'ITickable' members
	STDMETHOD(tick)		( void );
	STDMETHOD(tickAbort)	( void );
	STDMETHOD(tickBegin)	( void ) { return S_OK; }
	STDMETHOD(tickEnd)	( void ) { return S_OK; }

	// CCL
	CCL_OBJECT_BEGIN(AsyncQ)
		CCL_INTF(IBehaviour)
		CCL_INTF(ITickable)
	CCL_OBJECT_END()
	virtual void		destruct		( void );		// Destruct object

	// Connections
	DECLARE_CON(Fire)
	DECLARE_CON(Id)
	DECLARE_RCP(Next)
	DECLARE_RCP(Retry)
	DECLARE_RCP(Start)
	DECLARE_RCP(Stop)
	DECLARE_EMT(Empty)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
		DEFINE_CON(Id)

		DEFINE_RCP(Next)
		DEFINE_RCP(Retry)
		DEFINE_RCP(Start)
		DEFINE_RCP(Stop)
		DEFINE_EMT(Empty)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Internal utilities
	HRESULT getQ ( const adtValue &, IList ** );
	};

//
// Class - Clone.  A node to clone values.
//

class Clone :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Clone ( void );										// Constructor

	// Run-time data
	adtValue		vClone;									// Value to clone
	adtValue		vRes;										// Result value

	// CCL
	CCL_OBJECT_BEGIN(Clone)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Fire)
	DECLARE_RCP(Value)
	DECLARE_EMT(Error)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
		DEFINE_RCP(Value)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()

	private :

	};

//
// Class - Compare.  A node to compare two values.
//

class Compare :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Compare ( void );										// Constructor

	// Run-time data

	//! 'Left' and 'Right' values
	adtValue			vLeft,vRight;						

	// CCL
	CCL_OBJECT_BEGIN(Compare)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	//! \name Connections 
	//@{
	//!	\brief Perform comparison between the left and right values and emit the result.
	//!	If a right side has not been specified, the value fired into the node will be used.
	DECLARE_RCP(Fire)
	//! Specifies the left side of future comparisons
	DECLARE_RCP(Left)
	//! Specifies the right side of future comparisons
	DECLARE_RCP(Right)
	//! Emits the right value if left = right.
	DECLARE_EMT(Equal)
	//! Emits an error if the values cannot be compared
	DECLARE_EMT(Error)
	//! Emits the right side value if left > right.
	DECLARE_EMT(Greater)
	//! Emits the right side value if left < right.
	DECLARE_EMT(Less)
	//! Emits the right side value if left <> right.
	DECLARE_EMT(NotEqual)
	//@}
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Fire)
		DEFINE_RCP(Left)
		DEFINE_RCP(Right)

		DEFINE_EMT(Equal)
		DEFINE_EMT(Error)
		DEFINE_EMT(Greater)
		DEFINE_EMT(Less)
		DEFINE_EMT(NotEqual)
	END_BEHAVIOUR_NOTIFY()

	private :

	};

//
// Class - Create.  A node to create objects.
//

class Create :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Create ( void );										// Constructor

	// Run-time data
	adtString	strId;									// Parameters
	adtIUnknown	unkV;										// Internal
	adtString	strV;										// Internal

	// CCL
	CCL_OBJECT_BEGIN(Create)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Fire)
	DECLARE_RCP(Id)
	DECLARE_EMT(Error)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
		DEFINE_RCP(Id)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()

	private :

	};

//
// Class - Decode.  A node that selects an output from a single input value.
//

class Decode :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Decode ( void );										// Constructor

	// Run-time data
	IDictionary		*pMap;								// Decode dictionary
	adtValue			Select;								// Active selection
	adtValue			Value;								// Active value
	adtIUnknown		unkV;									// Internal
	adtString		strV;									// Internal
	adtValue			vV;									// Internal

	// CCL
	CCL_OBJECT_BEGIN(Decode)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual void		destruct		( void );		// Destruct object

	// Connections
	DECLARE_CON(Fire)
	DECLARE_RCP(Select)
	DECLARE_RCP(Value)
	DECLARE_EMT(Default)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
		DEFINE_RCP(Select)
		DEFINE_RCP(Value)
		DEFINE_EMT(Default)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - Debug.  A node to display debug information.
//

class Debug :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Debug ( void );										// Constructor

	// Run-time data
	adtString	strMsg;									// Parameters
	adtString	strPath;									// Namespace path to this node
	U32			dwT0;										// Start time
	IDictionary	*pDctLog;								// Logging dictionary

	// CCL
	CCL_OBJECT_BEGIN(Debug)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_RCP(Break)
	DECLARE_RCP(Fire)
//	DECLARE_CON(Log)
	DECLARE_RCP(Mark)
	DECLARE_RCP(Reset)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Break)
		DEFINE_RCP(Fire)
//		DEFINE_CON(Log)
		DEFINE_RCP(Mark)
		DEFINE_RCP(Reset)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Internal utilities
	void appendDbg ( const WCHAR * );

	// Logging callback
	static void logCallback	( cLogEntry *, void * );

	};

//
// Class - Demux.  A node that selects an output based
//		on a single input value (keyed from a context).
//

class Demux :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Demux ( void );										// Constructor

	// Run-time data
	IContainer		*pDemux;								// Demultiplexer container
	IDictionary		*pDct;								// Dictionary with value
	IDictionary		*pMap;								// Value map
	adtValue			Key;									// Key of value to select
	adtValue			Value;								// Default value
	adtIUnknown		unkV;									// Internal
	adtString		strV;									// Internal

	// CCL
	CCL_OBJECT_BEGIN(Demux)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_RCP(Dictionary)
	DECLARE_CON(Fire)
	DECLARE_RCP(Key)
	DECLARE_CON(Default)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Dictionary)
		DEFINE_CON(Fire)
		DEFINE_RCP(Key)
		DEFINE_CON(Default)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - DictFormat.  A node to format a dictionary into a string or stream.
//

class DictFormat :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	DictFormat ( void );									// Constructor

	// Run-time data
	IByteStream		*pStm;								// Output stream
	IDictionary		*pDct;								// Dictionary
	IContainer		*pFmt;								// Format
	adtString		strkSize,strkName,strkVal;		// String keys
	adtString		strkMap,strkType,strkSub;		// String keys

	// CCL
	CCL_OBJECT_BEGIN(DictFormat)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_RCP(Dictionary)
	DECLARE_EMT(Error)
	DECLARE_CON(Fire)
	DECLARE_RCP(Format)
	DECLARE_RCP(Stream)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Dictionary)
		DEFINE_EMT(Error)
		DEFINE_CON(Fire)
		DEFINE_RCP(Format)
		DEFINE_RCP(Stream)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Internal utilities
	HRESULT format ( IContainer * );
	};

//
// Class - DictParse.  A node to parse a string or stream into a dictionary.
//

class DictParse :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	DictParse ( void );									// Constructor

	// Run-time data
	IDictionary		*pDict;								// Active dictionary
	IContainer		*pFmt;								// Active format
	IByteStream		*pStm;								// Input stream
//	adtString		strParse;							// Input string

	// CCL
	CCL_OBJECT_BEGIN(DictParse)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Fire)
	DECLARE_RCP(Dictionary)
	DECLARE_RCP(Format)
	DECLARE_RCP(Stream)
	DECLARE_EMT(Error)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
		DEFINE_RCP(Dictionary)
		DEFINE_RCP(Format)
		DEFINE_RCP(Stream)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Internal utilities
	HRESULT parse ( IContainer * );
	};

//
// Class - Dist.  A distribution node emits received values.
//

class Dist :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Dist ( void );											// Constructor

	// Run-time data
	adtValue		vE;										// Value to emit

	// CCL
	CCL_OBJECT_BEGIN(Dist)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Fire)
	DECLARE_RCP(Value)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
		DEFINE_RCP(Value)
	END_BEHAVIOUR_NOTIFY()

	private :

	};

//
// Class - Path.  A node to handle paths.
//

class Path :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Path ( void );											// Constructor

	// Run-time data
	adtString	strPath;									// Current path
	bool			bAbs;										// Path is 'absolute'
	bool			bLoc;										// Path is 'location'
	IList			*pNames;									// Name list
	IIt			*pNamesIt;								// Name iterator

	// CCL
	CCL_OBJECT_BEGIN(Path)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual void		destruct		( void );		// Destruct object

	// Connections
	DECLARE_RCP(Up)
	DECLARE_CON(Path)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Up)
		DEFINE_CON(Path)
	END_BEHAVIOUR_NOTIFY()

	private :

	};

//
// Class - StringFormat.  Formats a string from a dictionary.
//

class StringFormat :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	StringFormat ( void );								// Constructor

	// Run-time data
	IContainer		*pFmt;								// Format specification
	IDictionary		*pDctSrc;							// Source dictionary
	IMemoryMapped	*pBfr;								// String buffer
	WCHAR				*pwBfr;								// String buffer

	// CCL
	CCL_OBJECT_BEGIN(StringFormat)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Fire)
	DECLARE_RCP(Dictionary)
	DECLARE_RCP(Format)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
		DEFINE_RCP(Dictionary)
		DEFINE_RCP(Format)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Internal utilities
	HRESULT formatString	( IUnknown *, WCHAR *, U32 * );

	};

//
// Class - StringOp.  Node for generic string operations.
//

class StringOp :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	StringOp ( void );									// Constructor

	// Run-time data
	adtString		strSrc,strDst,strType;			// Parameters
	adtInt			iFrom,iTo;							// Parameters
	adtString		strRes;								// Result
	bool				bTo;									// 'To' is specified
	IDictionary		*pMap;								// String map

	// CCL
	CCL_OBJECT_BEGIN(StringOp)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual void		destruct		( void );		// Destruct object

	// Connections
	DECLARE_RCP(Destination)
	DECLARE_RCP(From)
	DECLARE_RCP(IndexOf)
	DECLARE_RCP(IsType)
	DECLARE_RCP(LastIndexOf)
	DECLARE_RCP(Length)
	DECLARE_RCP(Replace)
	DECLARE_RCP(Source)
	DECLARE_RCP(Substring)
	DECLARE_RCP(Trailing)
	DECLARE_RCP(To)

	DECLARE_EMT(NotFound)
	DECLARE_EMT(Fire)
	DECLARE_EMT(True)
	DECLARE_EMT(False)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Destination)
		DEFINE_RCP(From)
		DEFINE_RCP(IndexOf)
		DEFINE_RCP(IsType)
		DEFINE_RCP(LastIndexOf)
		DEFINE_RCP(Replace)
		DEFINE_RCP(Source)
		DEFINE_RCP(Length)
		DEFINE_RCP(Substring)
		DEFINE_RCP(Trailing)
		DEFINE_RCP(To)

		DEFINE_EMT(NotFound)
		DEFINE_EMT(Fire)
		DEFINE_EMT(True)
		DEFINE_EMT(False)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - StringParse.  Parser a string with a given format.
//

class StringParse :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	StringParse ( void );								// Constructor

	// Run-time data
	adtString	strParse;								// String to parse
	IContainer	*pFmt;									// Format of string
	IDictionary	*pDctRs;									// Output dictionary

	// CCL
	CCL_OBJECT_BEGIN(StringParse)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Fire)
	DECLARE_RCP(Dictionary)
	DECLARE_RCP(Format)
	DECLARE_RCP(String)
	DECLARE_EMT(Position)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
		DEFINE_RCP(Dictionary)
		DEFINE_RCP(Format)
		DEFINE_RCP(String)
		DEFINE_EMT(Position)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Internal utilities
	HRESULT parseString	( IUnknown *, WCHAR *, U32 * );
	};

//
// Class - StringStream.  Handles conversion of strings to and from streams.
//

class StringStream :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	StringStream ( void );								// Constructor

	// Run-time data
	adtString	sTerm,sFrom,sPre;						// Parameters
	adtBool		bTerm;									// Parameters
	IByteStream	*pStm;									// Active stream
	BYTE			*pbStr;									// Current string
	U32			nalloc,nstr;							// Sizes
	adtString	sCodePage;								// Code page for string

	// CCL
	CCL_OBJECT_BEGIN(StringStream)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual void		destruct		( void );		// Destruct object

	// Connections
	DECLARE_CON(End)
	DECLARE_RCP(From)
	DECLARE_RCP(Prefix)
	DECLARE_RCP(Reset)
	DECLARE_RCP(Stream)
	DECLARE_RCP(String)
	DECLARE_RCP(Terminate)
	DECLARE_RCP(To)
	DECLARE_CON(Fire)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(End)
		DEFINE_RCP(From)
		DEFINE_RCP(Prefix)
		DEFINE_RCP(Reset)
		DEFINE_RCP(Stream)
		DEFINE_RCP(String)
		DEFINE_RCP(Terminate)
		DEFINE_RCP(To)
		DEFINE_CON(Fire)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Internal utilities
	HRESULT	decodeStr ( adtString & );
	};

//
// Class - TimeOp.  A node to deal with time.
//

class TimeOp :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	TimeOp ( void );										// Constructor

	// Run-time data
	adtBool		bLocal;									// Run-time data

	// CCL
	CCL_OBJECT_BEGIN(TimeOp)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Break)
	DECLARE_CON(Now)
	DECLARE_EMT(Day)
	DECLARE_EMT(Month)
	DECLARE_EMT(Year)
	DECLARE_EMT(Hour)
	DECLARE_EMT(Minute)
	DECLARE_EMT(Second)
	DECLARE_EMT(Millisecond)
	DECLARE_EMT(Date)
	DECLARE_EMT(Time)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Break)
		DEFINE_CON(Now)
		DEFINE_EMT(Day)
		DEFINE_EMT(Month)
		DEFINE_EMT(Year)
		DEFINE_EMT(Hour)
		DEFINE_EMT(Minute)
		DEFINE_EMT(Second)
		DEFINE_EMT(Millisecond)
		DEFINE_EMT(Date)
		DEFINE_EMT(Time)
	END_BEHAVIOUR_NOTIFY()

	private :

	};

//
// Class - Timer.  A node that that emits at a given interval.
//

class Timer :
	public CCLObject,										// Base class
	public IBehaviour,									// Interface
	public ITickable										// Interface
	{
	public :
	Timer ( void );										// Constructor

	// Run-time data
	IThread				*pThread;						// Timer thread
	bool					bRun;								// Timer thread run ?
	bool					bFireNow;						// For pre-mature firing
	U32					firet;							// Last fire time
	bool					bArmd;							// Armed ?
	adtBool				bArm;								// Armable ?
	adtInt				uRate;							// Rate
	adtInt				iPri;								// Priority
	adtBool				bSigNow;							// Signal at start
	sysEvent				evWork;							// Worker thread event
	adtInt				iV;								// Internal
	#if					__unix__ || __APPLE__
	struct timeval		t0;								// Reference time
	#endif

	// 'ITickable' members
	STDMETHOD(tick)		( void );
	STDMETHOD(tickAbort)	( void );
	STDMETHOD(tickBegin)	( void );
	STDMETHOD(tickEnd)	( void ) { return S_OK; }

	// CCL
	CCL_OBJECT_BEGIN(Timer)
		CCL_INTF(IBehaviour)
		CCL_INTF(ITickable)
	CCL_OBJECT_END()
	virtual void		destruct		( void );		// Destruct object

	// Connections
	DECLARE_CON(Fire)
	DECLARE_RCP(Arm)
	DECLARE_RCP(Disarm)
	DECLARE_RCP(Rate)
	DECLARE_RCP(Start)
	DECLARE_RCP(Stop)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
		DEFINE_RCP(Arm)
		DEFINE_RCP(Disarm)
		DEFINE_RCP(Rate)
		DEFINE_RCP(Start)
		DEFINE_RCP(Stop)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Internal utilities
	U32		tickCount 	( void );
	HRESULT	wait 			( U32 );

	};

//
// Class - Toggle.  A node to handle a toggle value.
//

class Toggle :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Toggle ( void );										// Constructor

	// Run-time data
	adtBool		bVal;										// Value

	// CCL
	CCL_OBJECT_BEGIN(Toggle)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Fire)
	DECLARE_CON(False)
	DECLARE_RCP(Not)
	DECLARE_CON(True)
	DECLARE_RCP(Value)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
		DEFINE_CON(False)
		DEFINE_RCP(Not)
		DEFINE_CON(True)
		DEFINE_RCP(Value)
	END_BEHAVIOUR_NOTIFY()

	};

//
// Class - TokenIt.  Node to iteratre through substrings.
//

class TokenIt :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	TokenIt ( void );										// Constructor

	// Run-time data
	adtString	strOrg,strCpy,strDelim;				// Parameters
	WCHAR		*ptokLast;								// Last token

	// CCL
	CCL_OBJECT_BEGIN(TokenIt)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(List)
	DECLARE_CON(Count)
	DECLARE_CON(Next)
	DECLARE_RCP(Delimiter)
	DECLARE_RCP(Reset)
	DECLARE_RCP(String)
	DECLARE_EMT(End)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(List)
		DEFINE_CON(Count)
		DEFINE_CON(Next)
		DEFINE_RCP(Delimiter)
		DEFINE_RCP(Reset)
		DEFINE_RCP(String)
		DEFINE_EMT(End)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Internal utilities
	WCHAR	*iwcstok ( WCHAR *, const WCHAR *);
	};

//
// Class - Type.  Determines/queries for value types.
//

class Type :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Type ( void );											// Constructor

	// Run-time data
	adtString		sType;								// Type match
	adtValue			vVal;									// Current value
	adtValue			vL,vConv;							// Internal

	// CCL
	CCL_OBJECT_BEGIN(Type)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Fire)
	DECLARE_CON(Type)
	DECLARE_RCP(Convert)
	DECLARE_RCP(Query)
	DECLARE_RCP(Value)
	DECLARE_EMT(Equal)
	DECLARE_EMT(Error)
	DECLARE_EMT(NotEqual)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
		DEFINE_CON(Type)

		DEFINE_RCP(Convert)
		DEFINE_RCP(Query)
		DEFINE_RCP(Value)

		DEFINE_EMT(Equal)
		DEFINE_EMT(Error)
		DEFINE_EMT(NotEqual)
	END_BEHAVIOUR_NOTIFY()

	// Documentation

//	MACRO("Determine, check, or convert a value from valid types.")

// Fire,"Check if the value has the specified type."

	private :

	// Internal utilities
	HRESULT getType ( const ADTVALUE &, IDictionary ** );

	};

#endif
