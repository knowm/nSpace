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
/// \brief An asynchronous emission node.  Values sent into this node
///		will be re-emitted on its own thread.
//

class AsyncEmit :
	public CCLObject,										// Base class
	public Behaviour,										// Interface
	public ITickable										// Interface
	{
	public :
	AsyncEmit ( void );									// Constructor

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
	//!	\brief Specifies the relative priority for asynchronous emissions.  0 = Normal priority, -1 = lower than normal , +1 = higher than normal
	DECLARE_RCP(Priority)
	//!	\brief Specifies a value to use for subsequent emissions.
	DECLARE_RCP(Value)
	//!	\brief Emits an error if cannot emit, such as if the node is already emitting a value.
	DECLARE_EMT(Error)
	//@}
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
		DEFINE_RCP(Priority)
		DEFINE_RCP(Value)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Run-time data
	IThread		*pThrd;									// Thread
	adtValue		vVal,vEmit;								// Emission value
	bool			bRun;										// Running
	sysEvent		evEmit;									// Emit event
	adtInt		iPri;										// Relative priority
	sysCS			csVal;									// Value mutex
	adtBool		bSingle;									// Single shot

	// 'ITickable' members
	STDMETHOD(tick)		( void );
	STDMETHOD(tickAbort)	( void ) { bRun = false; evEmit.signal(); return S_OK; }
	STDMETHOD(tickBegin)	( void );
	STDMETHOD(tickEnd)	( void ) { return S_OK; }

	};

//
// Class - AsyncQ.
//! \brief An asynchronous queue node.
//

class AsyncQ :
	public CCLObject,										// Base class
	public Behaviour,										// Interface
	public ITickable										// Interface
	{
	public :
	AsyncQ ( void );										// Constructor

	// CCL
	public :
	CCL_OBJECT_BEGIN(AsyncQ)
		CCL_INTF(IBehaviour)
		CCL_INTF(ITickable)
	CCL_OBJECT_END()
	virtual void		destruct		( void );		// Destruct object

	//! \name Connections 
	//@{
	//! \brief Place the specified value in current queue and emit out a different thread.
	DECLARE_CON(Fire)
	//! \brief Specify the Id of the active queue
	DECLARE_CON(Id)
	//! \brief Proceed to the next value into the queue if it exists
	DECLARE_RCP(Next)
	//! \brief Re-emit the current value in the front of the queue.
	DECLARE_RCP(Retry)
	//! \brief Start accepting and asynchronously emitting values
	DECLARE_RCP(Start)
	//! \brief Shutdown queueing and emission of values.
	DECLARE_RCP(Stop)
	//! \brief Emit a signal when the queue becomes empty.
	DECLARE_EMT(Empty)
	//@}
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
	adtValue		vIdE, vQE;								// Internal
	adtValue		vQ;										// Queue variable
	adtIUnknown	unkV;										// Internal
	U32			uIn,uOut;								// Debug

	// 'ITickable' members
	STDMETHOD(tick)		( void );
	STDMETHOD(tickAbort)	( void );
	STDMETHOD(tickBegin)	( void );
	STDMETHOD(tickEnd)	( void );

	// Internal utilities
	HRESULT getQ ( const adtValue &, IList ** );
	};

/*
//
// Class - AsyncQ.  
//

class AsyncQ :
	public CCLObject,										// Base class
	public Behaviour,									// Interface
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
*/

//
// Class - Clone.  
//! \brief A node to clone (deep copy) values.
//

class Clone :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Clone ( void );										// Constructor

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

	// Run-time data
	adtValue		vClone;									// Value to clone
	adtValue		vRes;										// Result value
	adtIUnknown	unkV;										// Run-time variable

	};

//
// Class - Compare.  
//! \brief A node to compare two values.
//

class Compare :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Compare ( void );										// Constructor

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

	// Run-time data

	//! 'Left' and 'Right' values
	adtValue			vLeft,vRight;						

	};

//
// Class - Create.  
//! \brief A node to create objects.
//

class Create :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Create ( void );										// Constructor

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

	// Run-time data
	adtString	strId;									// Parameters
	adtIUnknown	unkV;										// Internal
	adtString	strV;										// Internal

	};

//
// Class - Decode.  
//! \brief A node that selects a single emitter from a single input value.
//

class Decode :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Decode ( void );										// Constructor

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

	private :

	// Run-time data
	IDictionary		*pMap;								// Decode dictionary
	adtValue			Select;								// Active selection
	adtValue			Value;								// Active value
	adtIUnknown		unkV;									// Internal
	adtString		strV;									// Internal
	adtValue			vV;									// Internal

	};

//
// Class - Debug.  
//! \brief A node to assist in debugging a graph.  Displays values and can initiate a debugger break.
//

class Debug :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Debug ( void );										// Constructor

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

	// Run-time data
	adtString	strMsg;									// Parameters
	adtString	strPath;									// Namespace path to this node
	adtLong		lRst,lFreq;								// Reset time
	IDictionary	*pDctLog;								// Logging dictionary

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
	public Behaviour										// Interface
	{
	public :
	Demux ( void );										// Constructor

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

	private :

	// Run-time data
	IContainer		*pDemux;								// Demultiplexer container
	IDictionary		*pDct;								// Dictionary with value
	IDictionary		*pMap;								// Value map
	adtValue			Key;									// Key of value to select
	adtValue			Value;								// Default value
	adtIUnknown		unkV;									// Internal
	adtString		strV;									// Internal

	};

//
// Class - DictFormat.  
//!	\brief A node to format and write values in a dictionary into a byte stream.
//

class DictFormat :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	DictFormat ( void );									// Constructor

	// CCL
	CCL_OBJECT_BEGIN(DictFormat)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	//! \name Connections 
	//@{
	//!	\brief Specifes the dictionary to use for value lookups.
	DECLARE_RCP(Dictionary)
	//!	\brief Emits error code on failure
	DECLARE_EMT(Error)
	//!	\brief Emits formatted byte stream.
	DECLARE_CON(Fire)
	//!	\brief Specify the specification for the format.
	DECLARE_RCP(Format)
	//!	\brief Specify the stream to receive formatted bytes.
	DECLARE_RCP(Stream)
	//@}
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Dictionary)
		DEFINE_EMT(Error)
		DEFINE_CON(Fire)
		DEFINE_RCP(Format)
		DEFINE_RCP(Stream)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Run-time data
	IByteStream		*pStm;								// Output stream
	IDictionary		*pDct;								// Dictionary
	IContainer		*pFmt;								// Format
	adtString		strkSize,strkName,strkVal;		// String keys
	adtString		strkMap,strkType,strkSub;		// String keys
	adtBool			bEndianBig;							// Big endian ?

	// Internal utilities
	HRESULT format ( IContainer * );
	};

//
// Class - DictParse.  
//!	\brief A node to read bytes from a stream and format into dictionary values 
//

class DictParse :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	DictParse ( void );									// Constructor

	// CCL
	CCL_OBJECT_BEGIN(DictParse)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	//! \name Connections 
	//@{
	//!	\brief Peform parsing
	DECLARE_CON(Fire)
	//!	\brief Specifies the dictionary that will receive the parsed values
	DECLARE_RCP(Dictionary)
	//!	\brief Specify the specification for the format.
	DECLARE_RCP(Format)
	//!	\brief Specify the stream that contains the bytes to parse.
	DECLARE_RCP(Stream)
	//!	\brief Emits error code on failure
	DECLARE_EMT(Error)
	//@}
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
		DEFINE_RCP(Dictionary)
		DEFINE_RCP(Format)
		DEFINE_RCP(Stream)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Run-time data
	IDictionary		*pDict;								// Active dictionary
	IContainer		*pFmt;								// Active format
	IByteStream		*pStm;								// Input stream
	adtBool			bEndianBig;							// Big endian ?
//	adtString		strParse;							// Input string

	// Internal utilities
	HRESULT parse ( IContainer * );
	};

//
// Class - Dist.  A distribution node emits received values.
//!	\brief A node that emits a received value unmodified.  A constant value can also be declared and emitted.
//

class Dist :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Dist ( void );											// Constructor

	// CCL
	CCL_OBJECT_BEGIN(Dist)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	//! \name Connections 
	//@{
	//!	\brief Receive and emit value
	DECLARE_CON(Fire)
	//!	\brief Specify a default value to emit instead of received value
	DECLARE_RCP(Value)
	//@}
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
		DEFINE_RCP(Value)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Run-time data
	adtValue		vE;										// Value to emit

	};

//
// Class - Path.  A node to handle paths.
//!	\brief A node that handles the manipulation of paths.
//

class Path :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Path ( void );											// Constructor

	// CCL
	CCL_OBJECT_BEGIN(Path)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	//! \name Connections 
	//@{
	//!	\brief Move 'up' one level in the current path
	DECLARE_RCP(Up)
	//!	\brief Specify the active path
	DECLARE_CON(Path)
	//@}
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Up)
		DEFINE_CON(Path)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Run-time data
	adtString	strPath;									// Current path
	bool			bAbs;										// Path is 'absolute'
	bool			bLoc;										// Path is 'location'
	IList			*pNames;									// Name list
	IIt			*pNamesIt;								// Name iterator

	virtual void		destruct		( void );		// Destruct object
	};

//
// Class - StringFormat.  
//!	\brief Formats and writes values from a dictionary into a string.
//

class StringFormat :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	StringFormat ( void );								// Constructor

	// CCL
	CCL_OBJECT_BEGIN(StringFormat)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	//! \name Connections 
	//@{
	//!	\brief Format string from dictionary and emit result
	DECLARE_CON(Fire)
	//!	\brief Specify dictionary that contains the values to format
	DECLARE_RCP(Dictionary)
	//!	\brief Specify the specification for the format.
	DECLARE_RCP(Format)
	//@}
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
		DEFINE_RCP(Dictionary)
		DEFINE_RCP(Format)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Run-time data
	IContainer		*pFmt;								// Format specification
	IDictionary		*pDctSrc;							// Source dictionary
	IMemoryMapped	*pBfr;								// String buffer
	WCHAR				*pwBfr;								// String buffer

	// Internal utilities
	HRESULT formatString	( IUnknown *, WCHAR *, U32 * );

	};

//
// Class - StringOp.  
//!	\brief A node for generic string operations such as generating substrings and searching existing strings.
//

class StringOp :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	StringOp ( void );									// Constructor

	// CCL
	CCL_OBJECT_BEGIN(StringOp)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual void		destruct		( void );		// Destruct object

	//! \name Connections 
	//@{
	//!	\brief Specify string to use as destination
	DECLARE_RCP(Destination)
	//!	\brief Specifies index from which cetain functions begin.
	DECLARE_RCP(From)
	//!	\brief Find the index of the first occurence of source string in destination.
	DECLARE_RCP(IndexOf)
	//!	\brief Determine if the character at the from index in source string is of the specified type (e.g. "Digit").
	DECLARE_RCP(IsType)
	//!	\brief Find the last index of the first occurence of source string in destination.
	DECLARE_RCP(LastIndexOf)
	//!	\brief Compute the length of the source string
	DECLARE_RCP(Length)
	//!	\brief Replace source string with destination in the received string.
	DECLARE_RCP(Replace)
	//!	\brief Specify string to use as source
	DECLARE_RCP(Source)
	//!	\brief Create a substring from the source string using the from and to indices
	DECLARE_RCP(Substring)
	//!	\brief Strip the trailing characters specified in the source string from the destination
	DECLARE_RCP(Trailing)
	//!	\brief Specifies index to which certain functions end.
	DECLARE_RCP(To)

	//!	\brief Emit the source string from which a search failed.
	DECLARE_EMT(NotFound)
	//!	\brief Emit a resulting string
	DECLARE_EMT(Fire)
	//!	\brief Emits if character is of the specified type
	DECLARE_EMT(True)
	//!	\brief Emits if character is not of the specified type
	DECLARE_EMT(False)
	//@}
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

	private :

	// Run-time data
	adtString		strSrc,strDst,strType;			// Parameters
	adtInt			iFrom,iTo;							// Parameters
	adtString		strRes;								// Result
	bool				bTo;									// 'To' is specified
	IDictionary		*pMap;								// String map

	};

//
// Class - StringParse.
//!	\brief A node to extract values into a dictionary from a string using the specified format.
//

class StringParse :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	StringParse ( void );								// Constructor

	// CCL
	CCL_OBJECT_BEGIN(StringParse)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	//! \name Connections 
	//@{
	//!	\brief Parse string into dictionary using the format specification
	DECLARE_CON(Fire)
	//!	\brief Specifies the dictionary that will receive the parsed values.
	DECLARE_RCP(Dictionary)
	//!	\brief Specify the specification for the format.
	DECLARE_RCP(Format)
	//!	\brief Specifies the string to parse.
	DECLARE_RCP(String)
	//!	\brief Emits the position of the string at which parsing ended.
	DECLARE_EMT(Position)
	//@}
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
		DEFINE_RCP(Dictionary)
		DEFINE_RCP(Format)
		DEFINE_RCP(String)
		DEFINE_EMT(Position)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Run-time data
	adtString	strParse;								// String to parse
	IContainer	*pFmt;									// Format of string
	IDictionary	*pDctRs;									// Output dictionary

	// Internal utilities
	HRESULT parseString	( IUnknown *, WCHAR *, U32 * );
	};

//
// Class - StringStream.  
//!	\brief A node to handles transfers strings to streams and vice verse.
//

class StringStream :
	public CCLObject,										// Base class
	public Behaviour										// Interface
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

	//! \name Connections 
	//@{
	//!	\brief Signal an end to the binary data and emit remaining string
	DECLARE_CON(End)
	//!	\brief Initiate extraction of string from the stream
	DECLARE_RCP(From)
	//!	\brief An optional prefix to match
	DECLARE_RCP(Prefix)
	//!	\brief Reset the internal state of the extraction logic
	DECLARE_RCP(Reset)
	//!	\brief Specify the byte stream
	DECLARE_RCP(Stream)
	//!	\brief Specify the string to put into a byte stream
	DECLARE_RCP(String)
	//!	\brief An optional termination that ends string extraction
	DECLARE_RCP(Terminate)
	//!	\brief Initiate placement of string into stream
	DECLARE_RCP(To)
	//!	\brief Emits extracted strings
	DECLARE_EMT(Fire)
	//@}
	BEGIN_BEHAVIOUR()
		DEFINE_CON(End)
		DEFINE_RCP(From)
		DEFINE_RCP(Prefix)
		DEFINE_RCP(Reset)
		DEFINE_RCP(Stream)
		DEFINE_RCP(String)
		DEFINE_RCP(Terminate)
		DEFINE_RCP(To)
		DEFINE_EMT(Fire)
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
	public Behaviour										// Interface
	{
	public :
	TimeOp ( void );										// Constructor

	// CCL
	CCL_OBJECT_BEGIN(TimeOp)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	//! \name Connections 
	//@{
	//!	\brief Break apart the current time into its components (month, year, etc)
	DECLARE_CON(Break)
	//!	\brief Emit the current date/time.
	DECLARE_CON(Now)
	//!	\brief The day portion of the time.
	DECLARE_EMT(Day)
	//!	\brief The month portion of the time.
	DECLARE_EMT(Month)
	//!	\brief The year portion of the time.
	DECLARE_EMT(Year)
	//!	\brief The hour portion of the time.
	DECLARE_EMT(Hour)
	//!	\brief The minute portion of the time.
	DECLARE_EMT(Minute)
	//!	\brief The second portion of the time.
	DECLARE_EMT(Second)
	//!	\brief The millisecond portion of the time.
	DECLARE_EMT(Millisecond)
	//!	\brief The date (calendar) portion of the time.
	DECLARE_EMT(Date)
	//!	\brief The time (without calendar) portion of the time.
	DECLARE_EMT(Time)
	//@}
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

	// Run-time data
	adtBool		bLocal;									// Run-time data

	};

//
// Class - Timer.  
//!	\brief A node that asynchronously emits a signal at a given interval.
//

class Timer :
	public CCLObject,										// Base class
	public Behaviour,									// Interface
	public ITickable										// Interface
	{
	public :
	Timer ( void );										// Constructor

	// CCL
	CCL_OBJECT_BEGIN(Timer)
		CCL_INTF(IBehaviour)
		CCL_INTF(ITickable)
	CCL_OBJECT_END()
	virtual void		destruct		( void );		// Destruct object

	//! \name Connections 
	//@{
	//!	\brief Fire timer immediately
	DECLARE_CON(Fire)
	//!	\brief Arm timer for firing
	DECLARE_RCP(Arm)
	//!	\brief Disarm (disable) timer
	DECLARE_RCP(Disarm)
	//!	\brief Specify the rate of the timer in milliseconds
	DECLARE_RCP(Rate)
	//!	\brief Start asynchronous timer
	DECLARE_RCP(Start)
	//!	\brief Stop asynchronous timer
	DECLARE_RCP(Stop)
	//@}
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
		DEFINE_RCP(Arm)
		DEFINE_RCP(Disarm)
		DEFINE_RCP(Rate)
		DEFINE_RCP(Start)
		DEFINE_RCP(Stop)
	END_BEHAVIOUR_NOTIFY()

	private :

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
	public Behaviour										// Interface
	{
	public :
	Toggle ( void );										// Constructor

	// CCL
	CCL_OBJECT_BEGIN(Toggle)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	//! \name Connections 
	//@{
	//!	\brief Fire value in a way that reflects the state of the node
	DECLARE_CON(Fire)
	//!	\brief Set current state to false
	DECLARE_CON(False)
	//!	\brief Invert the current toggle state
	DECLARE_RCP(Not)
	//!	\brief Set current state to true
	DECLARE_CON(True)
	//!	\brief Set the true/false state directly via value
	DECLARE_RCP(Value)
	//@}
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
		DEFINE_CON(False)
		DEFINE_RCP(Not)
		DEFINE_CON(True)
		DEFINE_RCP(Value)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Run-time data
	adtBool		bVal;										// Value
	adtBool		bValE;									// Value during emission


	};

//
// Class - TokenIt.  
//!	\brief A node to iterate through substrings with a given delimiter.
//

class TokenIt :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	TokenIt ( void );										// Constructor

	// CCL
	CCL_OBJECT_BEGIN(TokenIt)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	//! \name Connections 
	//@{
	//!	\brief Emit a new list of parsed tokens
	DECLARE_CON(List)
	//!	\brief Emit the number of parser tokens
	DECLARE_CON(Count)
	//!	\brief Emit the next token
	DECLARE_CON(Next)
	//!	\brief Specify the set of delimiters between the tokens
	DECLARE_RCP(Delimiter)
	//!	\brief Reset iteration of tokens
	DECLARE_RCP(Reset)
	//!	\brief Specify the string from which to extract tokens
	DECLARE_RCP(String)
	//!	\brief Signal indcating the end of the tokens has been reached.
	DECLARE_EMT(End)
	//@}
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

	// Run-time data
	adtString	strOrg,strCpy,strDelim;				// Parameters
	WCHAR			*ptokLast;								// Last token


	// Internal utilities
	WCHAR	*iwcstok ( WCHAR *, const WCHAR *);
	};

//
// Class - Type.  
//!	\brief A node to query and convert value types.
//

class Type :
	public CCLObject,										// Base class
	public Behaviour										// Interface
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

	//! \name Connections 
	//@{
	//!	\brief Determine if the value is of the current type
	DECLARE_CON(Fire)
	//!	\brief Specify a string version of the desired type (e.g. "Integer", "Float", etc)
	DECLARE_CON(Type)
	//!	\brief Convert the value to the current type
	DECLARE_RCP(Convert)
	//!	\brief Emit the type of the value
	DECLARE_RCP(Query)
	//!	\brief Specify a value for use.
	DECLARE_RCP(Value)
	//!	\brief Emits a signal if the type matches
	DECLARE_EMT(Equal)
	//!	\brief Emits a value when cloning is unsuccessful.
	DECLARE_EMT(Error)
	//!	\brief Emits a signal if the type does not match
	DECLARE_EMT(NotEqual)
	//@}
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

//
// Class - UUIDOp.  Node to create a handle universally unique Ids (UUID).
//

class UUIDOp :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	UUIDOp ( void );										// Constructor

	// Run-time data

	// CCL
	CCL_OBJECT_BEGIN(UUIDOp)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Fire)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
	END_BEHAVIOUR()

	private :

	};

#endif
