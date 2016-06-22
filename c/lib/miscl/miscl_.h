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

#define	WCHAR2HEX(a)																	\
	((a) >= WCHAR('0') && (a) <= WCHAR('9')) ? (a)-WCHAR('0') 		:		\
	((a) >= WCHAR('a') && (a) <= WCHAR('f')) ? (a)-WCHAR('a')+10 	:		\
	((a) >= WCHAR('A') && (a) <= WCHAR('F')) ? (a)-WCHAR('A')+10	: 0
#define	HEX2WCHAR(a)													\
	(((a) < 10) ? ((a) + WCHAR('0')) : ((a) - 10 + WCHAR('A')))

//
// Class - AsyncEmit.  
//! \brief An asynchronous emission node.
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

	//! \name Connections 
	//@{
	//!	\brief Asynchronously emit a value.  If no value was previously specified, the provided value is used.
	DECLARE_CON(Fire)
	//!	\brief Specifies a value for subsequent emissions.
	DECLARE_RCP(Value)
	//@}
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
		DEFINE_RCP(Value)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - AsyncQ.  
//! \brief An asynchronous queue node.
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
	IList			*pQ;										// Value queue
	IIt			*pQIt;									// Iterator for queue
	adtInt		iSzMax;									// Maximum queue size
	adtBool		bBlock;									// Block on full queue ?
	adtInt		iTo;										// Timeout for blocking mode
	sysCS			csWork;									// Work mutex
	sysEvent		evNotEmpty,evNotFull;				// Queue events
	adtValue		vQe;										// Active queue emission

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

	//! \name Connections 
	//@{
	//! \brief	True means incoming values will be blocked until there is room in the queue, 
	//				false means values will be dropped if queue is full.
	DECLARE_CON(Block)
	//! \brief Queue the specified value for asynchronous emission.
	DECLARE_CON(Fire)
	//! \brief Re-emit the current value in the front of the queue.
	DECLARE_RCP(Retry)
	//! \brief Set the maximum size of the queues before values get blocked or dropped
	DECLARE_RCP(Size)
	//! \brief Start accepting and asynchronously emitting values
	DECLARE_RCP(Start)
	//! \brief Shutdown queueing and emission of values.
	DECLARE_RCP(Stop)
	//! \brief If is set to block on full queue, specify a timeout in milliseconds to wait
	DECLARE_RCP(Timeout)
	//! \brief Emit a signal when the queue becomes empty.
	DECLARE_EMT(Empty)
	//@}
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)

		DEFINE_RCP(Block)
		DEFINE_RCP(Retry)
		DEFINE_RCP(Size)
		DEFINE_RCP(Start)
		DEFINE_RCP(Stop)
		DEFINE_RCP(Timeout)
		DEFINE_EMT(Empty)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Internal utilities
	HRESULT qValue ( const ADTVALUE & );
	};

//
// Class - Clone.  
//! \brief A node to clone values.
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

	//@{
	//!	\brief Perform a clone by creating a new object and performing a deep copy.  If no value 
	//!	was previously specified, the provided value is used.
	DECLARE_CON(Fire)
	//!	\brief Specify a value for future cloning.
	DECLARE_RCP(Value)
	//!	\brief Emits a value when cloning is unsuccessful.
	DECLARE_EMT(Error)
	//@}
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
		DEFINE_RCP(Value)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()

	private :

	};

//
// Class - Compare.  
//! \brief A node to compare two values.
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
// Class - Create.  
//! \brief A node to create objects.
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

	//! \name Connections 
	//@{
	//!	\brief Create an object with the specified Id (e.g. "Adt.Dictionary")
	DECLARE_CON(Fire)
	//!	\brief Specifies an object Id for future creations.
	DECLARE_RCP(Id)
	//!	\brief Emits a value if there is an error when creating an object.
	DECLARE_EMT(Error)
	//@}
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
		DEFINE_RCP(Id)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()

	private :

	};

//
// Class - Decode.  
//! \brief A node that selects a single emitter from a single input value.
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

	//! \name Connections 
	//@{
	//!	\brief Emit a value out a specific emitter selected by the latest selection.
	DECLARE_CON(Fire)
	//!	\brief Specifies a value to use when selecting and output.  The string version of
	//!	of this selection is mapped to an emitter name.
	DECLARE_RCP(Select)
	//!	\brief Specifies a value to emit for future decodings.
	DECLARE_RCP(Value)
	//!	\brief Emits the current value if there was no emitter matching the current selection.
	DECLARE_EMT(Default)
	//@}
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
		DEFINE_RCP(Select)
		DEFINE_RCP(Value)
		DEFINE_EMT(Default)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - Debug.  
//! \brief A node to assist in debugging a graph.  Displays values and can initiate a debugger break.
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
	adtLong		lRst,lFreq;								// Reset time
	IDictionary	*pDctLog;								// Logging dictionary

	// CCL
	CCL_OBJECT_BEGIN(Debug)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	//! \name Connections 
	//@{
	//!	\brief Initiate a break in the program flow inside a debugger.  On Windows is calls 'DebugBreak'.
	DECLARE_RCP(Break)
	//!	\brief Print out a string version of the received value to the debug output.
	DECLARE_RCP(Fire)
//	DECLARE_CON(Log)
	//!	\brief Print out the time difference between the last Reset signal and now to the debug output.
	DECLARE_RCP(Mark)
	//!	\brief Reset the initial time used when marking time.
	DECLARE_RCP(Reset)
	//!	\brief Sleep specified number of milliseconds
	DECLARE_RCP(Sleep)
	//@}
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Break)
		DEFINE_RCP(Fire)
//		DEFINE_CON(Log)
		DEFINE_RCP(Mark)
		DEFINE_RCP(Reset)
		DEFINE_RCP(Sleep)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Internal utilities
	void appendDbg ( const WCHAR * );

	// Logging callback
	static void logCallback	( cLogEntry *, void * );

	};

//
// Class - Demux.  
//!	\brief A node that selects an output based on a single input value loaded from a dictionary.
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

	//! \name Connections 
	//@{
	//!	\brief Specifes the dictionary to use for future key lookups.
	DECLARE_RCP(Dictionary)
	//!	\brief Emit a value out a specific emitter matching the current value associated with the key in the dictionary.
	DECLARE_CON(Fire)
	//!	\brief Specifies the key to use for future value lookups in the dictionary.
	DECLARE_RCP(Key)
	//!	\brief Emits the current dictionary if there was no emitter matching the current value.
	DECLARE_CON(Default)
	//@}
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Dictionary)
		DEFINE_CON(Fire)
		DEFINE_RCP(Key)
		DEFINE_CON(Default)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - DictFormat.  
//!	\brief A node to format and write values in a dictionary into a byte stream.
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
// Class - DictParse.  
//!	\brief A node to read bytes from a stream and format into dictionary values 
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
//!	\brief A node that emits a received value unmodified.  A constant value can also be declared and emitted.
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
// Class - StringFormat.  
//!	\brief Formats and writes values from a dictionary into a string.
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
// Class - StringOp.  
//!	\brief A node for generic string operations such as generating substrings and searching existing strings.
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
// Class - StringParse.
//!	\brief A node to extract values into a dictionary from a string using the specified format.
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
// Class - StringStream.  
//!	\brief A node to handles transfers strings to streams and vice verse.
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
	bool			bHex;										// Hex mode ?

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
// Class - TimeOp.
//!	\brief A node to handle dates and times.
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
// Class - Timer.  
//!	\brief A node that asynchronously emits a signal at a given interval.
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
// Class - Toggle.  
//!	\brief A node to manipulate and emit a boolean value.  Values emit out true or false emitters.
//

class Toggle :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Toggle ( void );										// Constructor

	// Run-time data
	adtBool		bVal;										// Value
	adtBool		bValE;									// Value during emission

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
// Class - TokenIt.  
//!	\brief A node to iterate through substrings with a given delimiter.
//

class TokenIt :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	TokenIt ( void );										// Constructor

	// Run-time data
	adtString	strOrg,strCpy,strDelim;				// Parameters
	WCHAR			*ptokLast;								// Last token

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
// Class - Type.  
//!	\brief A node to query and convert value types.
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

	private :

	// Internal utilities
	HRESULT getType ( const ADTVALUE &, IDictionary ** );

	};

#endif
