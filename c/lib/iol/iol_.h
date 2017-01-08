////////////////////////////////////////////////////////////////////////
//
//										IOL_.H
//
//				Implementation include file for the I/O library
//
////////////////////////////////////////////////////////////////////////

#ifndef	IOL__H
#define	IOL__H

// Includes
#include	"iol.h"
#ifndef	_WIN32
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#endif

// nSpace
#include "../../lib/nspcl/nspcl.h"

#if	defined(_WIN32)
#include <setupapi.h>
#endif

// XML processing flags
//#define	__NOSAX__
#if	!defined(__NOSAX__)
#define	__USESAX__
#include <msxml2.h>
#else
#undef	__USESAX__
#endif

#define	WM_USER_DESTROY	(WM_USER+1)

///////////
// Objects
///////////

//
// Class - ByteCache.  Stream based byte buffer cache to be placed in 
//								front of another byte stream.
//

class ByteCache :
	public CCLObject,										// Base class
	public IByteStream,									// Interface
	public IResource										// Interface
	{
	public :
	ByteCache ( void );									// Constructor

	// Run-time data
	IByteStream		*pStm;								// Source/destination stream
	IByteStream		*pStmC;								// Internal cache stream
	bool				bRO;									// Read only
	U32				nAllocd;								// Allocated buffer size
	U32				nCache;								// Number of valid bytes in cache

	// 'IResource' members
	STDMETHOD(close)		( void );
	STDMETHOD(getResId)	( ADTVALUE & );
	STDMETHOD(open)		( IDictionary * );

	// 'IByteStream' members
	STDMETHOD(available)	( U64 * );
	STDMETHOD(copyTo)		( IByteStream *, U64, U64 * );
	STDMETHOD(flush)		( void );
	STDMETHOD(read)		( void *, U64, U64 * );
	STDMETHOD(seek)		( S64, U32, U64 * );
	STDMETHOD(setSize)	( U64 );
	STDMETHOD(write)		( void const *, U64, U64 * );

	// CCL
	CCL_OBJECT_BEGIN(ByteCache)
		CCL_INTF(IResource)
		CCL_INTF(IByteStream)
	CCL_OBJECT_END()
	virtual void		destruct		( void );		// Destruct object
	};

//
// Class - Lock.  Lock synchronization object.
//

class Lock :
	public CCLObject,										// Base class
	public IResource										// Interface
	{
	public :
	Lock ( void );											// Constructor

	// Run-time data
	sysCS			lock;										// Lock object

	// 'IResource' members
	STDMETHOD(close)		( void );
	STDMETHOD(getResId)	( ADTVALUE & );
	STDMETHOD(open)		( IDictionary * );

	// CCL
	CCL_OBJECT_BEGIN(Lock)
		CCL_INTF(IResource)
	CCL_OBJECT_END()

	private :

	};

//
// Class - MemoryBlock.  Implementation of a memory block.
//
class MemoryBlock :
	public CCLObject,										// Base class
	public ICloneable,									// Interface
	public IMemoryMapped									// Interface
	{
	public :
	MemoryBlock ( void );								// Constructor

	// 'IMemoryMapped' members
	STDMETHOD(getSize)	( U32 * );
	STDMETHOD(lock)		( U32, U32, void **, U32 * );
	STDMETHOD(setSize)	( U32 );
	STDMETHOD(stream)		( IByteStream ** );
	STDMETHOD(unlock)		( void * );

	// 'ICloneable' members
	STDMETHOD(clone)		( IUnknown ** );

	// CCL
	CCL_OBJECT_BEGIN(MemoryBlock)
		CCL_INTF(IMemoryMapped)
		CCL_INTF(ICloneable)
	CCL_OBJECT_END()
	virtual void destruct		( void );			// Destruct object

	private :

	// Run-time data
	U8			*pcBlk;										// Memory block
	U32		szBlk;										// Size of memory block
	U32		szAllocd;									// Amount of memory allocated
	HANDLE	hMap;											// File mapping
	friend	class StmMemory;							// Stream access
	};

//
// Class - StmSrcFile.  File system based stream source.
//

class StmSrcFile :
	public CCLObject,										// Base class
	public ILocations										// Interface
	{
	public :
	StmSrcFile ( void );									// Constructor

	// Run-time data
	IUnknown			*punkDct;							// Object dictionary
	IDictionary		*pDct,*pStmOpts;					// Object dictionary
	adtString		strRoot;								// Root location
	adtString		strkLoc,strkRO;					// String keys
	adtString		strkCr,strkTr,strkAsync;		// String keys

	// 'ILocations' members
	STDMETHOD(open)		( IDictionary *,	IUnknown ** );
	STDMETHOD(locations)	( const WCHAR *,	IIt ** );
	STDMETHOD(resolve)	( const WCHAR *,	bool, ADTVALUE & );
	STDMETHOD(status)		( const WCHAR *,	IDictionary * );

	// CCL
	CCL_OBJECT_BEGIN(StmSrcFile)
		CCL_INTF(ILocations)
		CCL_INTF_AGG(IDictionary,punkDct)
	CCL_OBJECT_END()
	virtual HRESULT	construct	( void );		// Construct object
	virtual void		destruct		( void );		// Destruct object

	private :

	// Internal utilities
	HRESULT	createLoc	( const WCHAR * );		// Create stream location
	HRESULT	defaultRoot	( void );					// Default filesystem root
	HRESULT	toPath		( const WCHAR *, adtString & );
	};

//
// Class - StmFileRes.  File resource.
//

class StmFileRes :
	public CCLObject										// Base class
	{
	public :
	#ifdef	_WIN32
	StmFileRes ( HANDLE );								// Constructor
	#else
	StmFileRes ( int );									// Constructor
	#endif

	// Run-time data
	#ifdef		_WIN32
	HANDLE		hFile;									// Handle to file
	HANDLE		hevRd,hevWr;							// Asynchronous I/O event handles
	#else
	int			fd;										// File descriptor
	#endif
	sysCS			csRd,csWr;								// Thread protection
	bool			bAsync;									// Async I/O ?

	// Extractor
	#ifdef		_WIN32
	inline operator HANDLE() const { return hFile; }
	#else
	inline operator int() const { return fd; }
	#endif

	// CCL
	CCL_OBJECT_BEGIN_INT(StmFileRes)
	CCL_OBJECT_END()
	virtual void		destruct		( void );		// Destruct object

	private :
	StmFileRes ( void );									// Default constructor
	};

//
// Class - StmFile.  File system based stream.
//

class StmFile :
	public CCLObject,										// Base class
	public IByteStream,									// Interface
	public IResource										// Interface
	{
	public :
	StmFile ( void );										// Constructor

	// Run-time data
	StmFileRes	*pFile;									// File resource
	adtString	strkLoc,strkRO;						// String keys
	adtString	strkCr,strkTr,strkAsync;			// String keys

	// 'IResource' members
	STDMETHOD(close)		( void );
	STDMETHOD(getResId)	( ADTVALUE & );
	STDMETHOD(open)		( IDictionary * );

	// 'IByteStream' members
	STDMETHOD(available)	( U64 * );
	STDMETHOD(copyTo)		( IByteStream *, U64, U64 * );
	STDMETHOD(flush)		( void );
	STDMETHOD(read)		( void *, U64, U64 * );
	STDMETHOD(seek)		( S64, U32, U64 * );
	STDMETHOD(setSize)	( U64 );
	STDMETHOD(write)		( void const *, U64, U64 * );

	// CCL
	CCL_OBJECT_BEGIN(StmFile)
		CCL_INTF(IResource)
		CCL_INTF(IByteStream)
	CCL_OBJECT_END()
	virtual void		destruct		( void );		// Destruct object
	};

//
// Class - StmMemory.  Implementation of a byte stream 
//		interface on a block of memory.
//

class StmMemory :
	public CCLObject,										// Base class
	public ICloneable,									// Interface
	public IByteStream									// Interface
	{
	public :
	StmMemory ( void );									// Constructor

	// Run-time data
	MemoryBlock		*pBlock;								// Owner
	U64				pos;									// Current position

	// 'IByteStream' members
	STDMETHOD(available)	( U64 * );
	STDMETHOD(copyTo)		( IByteStream *, U64, U64 * );
	STDMETHOD(flush)		( void );
	STDMETHOD(read)		( void *, U64, U64 * );
	STDMETHOD(seek)		( S64, U32, U64 * );
	STDMETHOD(setSize)	( U64 );
	STDMETHOD(write)		( void const *, U64, U64 * );

	// 'ICloneable' members
	STDMETHOD(clone)		( IUnknown ** );

	// CCL
	CCL_OBJECT_BEGIN(StmMemory)
		CCL_INTF(IByteStream)
		CCL_INTF(ICloneable)
	CCL_OBJECT_END()
	virtual HRESULT	construct	( void );		// Construct object
	virtual void		destruct		( void );		// Destruct object

	private :

	};

//
// Class - StmPrsBin.  Binary stream value parser.
//

class StmPrsBin :
	public CCLObject,										// Base class
	public IStreamPersist									// Interface
	{
	public :
	StmPrsBin ( void );									// Constructor

	// Run-time data

	// 'IStreamPersist' members
	STDMETHOD(load)	( IByteStream *, ADTVALUE & );
	STDMETHOD(save)	( IByteStream *, const ADTVALUE & );

	// CCL
	CCL_OBJECT_BEGIN(StmPrsBin)
		CCL_INTF(IStreamPersist)
	CCL_OBJECT_END()

	private :

	// Internal utilities
	STDMETHOD(read)			( IByteStream *, void *, U32 );
	STDMETHOD(write)			( IByteStream *, void const *, U32 );
	STDMETHOD(read)			( IByteStream *, U16 * );
	STDMETHOD(read)			( IByteStream *, U32 * );
	STDMETHOD(read)			( IByteStream *, U64 * );
	STDMETHOD(readStr)		( IByteStream *, adtString & );
	STDMETHOD(write)			( IByteStream *, const U16 * );
	STDMETHOD(write)			( IByteStream *, const U32 * );
	STDMETHOD(write)			( IByteStream *, const U64 * );
	STDMETHOD(writeStr)		( IByteStream *, const WCHAR * );
	};

//
// Class - StmPrsXML.  XML stream value parser.
//

class StmPrsXML :
	public CCLObject,										// Base class
	#ifdef	__USESAX__
	public ISAXContentHandler,							// Interface
	public ISAXErrorHandler,							// Interface
	#endif
	public IStreamPersist								// Interface
	{
	public :
	StmPrsXML ( void );									// Constructor

	// Run-time data
	#ifdef	__USESAX__
	IUnknown				*punkRdr;						// SAX reader
	adtValue				vSAXFrom;						// SAX value
	adtString			sSAXFrom;						// SAX string
	IList					*pSAXStk;						// Object stack
	IIt					*pSAXStkIt;						// Iterator
	VALUETYPE			tSAX;								// Type
	adtString			strResv;							// Reserved string
	#endif
	#ifdef				_WIN32
	IHaveValue			*pStmStm;						// Stream on byte stream
	#endif

	// 'IStreamPersist' members
	STDMETHOD(load)	( IByteStream *, ADTVALUE & );
	STDMETHOD(save)	( IByteStream *, const ADTVALUE & );

	#ifdef	__USESAX__
	// 'ISAXContentHandler' members
	STDMETHOD(putDocumentLocator)		( ISAXLocator * );
	STDMETHOD(startDocument)			( void );
	STDMETHOD(endDocument)				( void );
	STDMETHOD(startPrefixMapping)		( const wchar_t *, int, const wchar_t *, int );
	STDMETHOD(endPrefixMapping)		( const wchar_t *, int );
	STDMETHOD(startElement)				( const wchar_t *, int, const wchar_t *, int,
													const wchar_t *, int, ISAXAttributes * );
	STDMETHOD(endElement)				( const wchar_t *, int, const wchar_t *, int,
													const wchar_t *, int );
	STDMETHOD(characters)				( const wchar_t *, int );
	STDMETHOD(ignorableWhitespace)	( const wchar_t *, int );
	STDMETHOD(processingInstruction)	( const wchar_t *, int, const wchar_t *, int );
	STDMETHOD(skippedEntity)			( const wchar_t *, int );

	// 'ISAXErrorHandler' members
	STDMETHOD(error)						( ISAXLocator *, const wchar_t *, HRESULT );
	STDMETHOD(fatalError)				( ISAXLocator *, const wchar_t *, HRESULT );
	STDMETHOD(ignorableWarning)		( ISAXLocator *, const wchar_t *, HRESULT );
	#endif

	// CCL
	CCL_OBJECT_BEGIN(StmPrsXML)
		CCL_INTF(IStreamPersist)
		#ifdef	__USESAX__
		CCL_INTF(ISAXContentHandler)
		CCL_INTF(ISAXErrorHandler)
		#endif
	CCL_OBJECT_END()
	virtual HRESULT	construct	( void );		// Construct object
	virtual void		destruct		( void );		// Destruct object

	private :

	// Internal utilities
	HRESULT	emit		( IByteStream *, const WCHAR *, bool = false );
	HRESULT	writeAll	( IByteStream *, const void *, U32 );
	};

//
// Class - StmOnByteStm.  Lays an 'IStream' interface on top
//		of our 'IByteStream'.  Useful for APIs that require 'IStream'.
//

class StmOnByteStm :
	public CCLObject,										// Base class
	public IStream,										// Interface
	public IHaveValue										// Interface
	{
	public :
	StmOnByteStm ( void );								// Constructor

	// 'IStream' members
	STDMETHOD(Clone)			( IStream ** );
	STDMETHOD(Commit)			( DWORD );
	STDMETHOD(CopyTo)			( IStream *, ULARGE_INTEGER,
										ULARGE_INTEGER *, ULARGE_INTEGER * );
	STDMETHOD(LockRegion)	( ULARGE_INTEGER, ULARGE_INTEGER, DWORD );
	STDMETHOD(Revert)			( void );
	STDMETHOD(Seek)			( LARGE_INTEGER, DWORD, ULARGE_INTEGER * );
	STDMETHOD(SetSize)		( ULARGE_INTEGER );
	STDMETHOD(Stat)			( STATSTG *, DWORD );
	STDMETHOD(UnlockRegion)	( ULARGE_INTEGER, ULARGE_INTEGER, DWORD );

	// 'ISequentialStream' members
	STDMETHOD(Read)	( void *, ULONG, ULONG * );
	STDMETHOD(Write)	( void const *, ULONG, ULONG * );

	// 'IHaveValue' members
	STDMETHOD(getValue)	( ADTVALUE & );
	STDMETHOD(setValue)	( const ADTVALUE & );

	// CCL
	CCL_OBJECT_BEGIN(StmOnByteStm)
		CCL_INTF(IHaveValue)
		CCL_INTF(IStream)
		CCL_INTF(ISequentialStream)
	CCL_OBJECT_END()
	virtual void		destruct	( void );			// Destruct object

	private :

	// Run-time data
	IByteStream		*pStm;								// Current stream

	};

/////////
// Nodes
/////////

#if	defined(_WIN32)
//
// Class - EnumDevices.  Node to enumerator through system devices.
//

class EnumDevices :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	EnumDevices ( void );								// Constructor

	// Run-time data
	GUID			guidClass;								// Class ID
	HANDLE		hEnum;									// Enumeration handle
	U32			idx;										// Current device index
	sysDl			dl;										// Library

	// CCL
	CCL_OBJECT_BEGIN(EnumDevices)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_RCP(Class)
	DECLARE_EMT(End)
	DECLARE_RCP(First)
	DECLARE_EMT(Name)
	DECLARE_CON(Next)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Class)
		DEFINE_EMT(End)
		DEFINE_RCP(First)
		DEFINE_EMT(Name)
		DEFINE_CON(Next)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Library functions (dynamic to support Windows NT)
	#if	!defined(UNDER_CE)
	BOOL		(WINAPI *sdddil)	( HDEVINFO );
	BOOL		(WINAPI *sdedi)	( HDEVINFO, PSP_DEVINFO_DATA, const GUID *,
											DWORD, PSP_DEVICE_INTERFACE_DATA );
	HDEVINFO	(WINAPI *sdgcd)	( const GUID *, PCTSTR, HWND, DWORD );
	BOOL		(WINAPI *sdgdid)	( HDEVINFO, PSP_DEVICE_INTERFACE_DATA,
											PSP_DEVICE_INTERFACE_DETAIL_DATA,
											DWORD, DWORD *, PSP_DEVINFO_DATA );
	HKEY		(WINAPI *sdodrk)	( HDEVINFO, PSP_DEVINFO_DATA, DWORD, DWORD,
											DWORD, REGSAM );
	#endif
	};
#endif

//
// Class - File.  General file I/O node.
//

class File :
	public CCLObject,										// Base class
	public Behaviour,									// Interface
	public ITickable										// Interface
	{
	public :
	File ( void );											// Constructor

	// Run-time data
	#ifdef		_WIN32
	HANDLE		hFile;									// File handle
	HANDLE		hevWr,hevRd;							// I/O events
	HANDLE		hevStop;									// Stop event for read thread
	#endif
	IByteStream	*pStmIo;									// I/O stream
	adtInt		iSzIo;									// I/O size
	adtBool		bAsync;									// Asynchronous reads ?
	IThread		*pThrd;									// Asynchronous read thread
	U8				*pcBfr;									// I/O buffer
	adtInt		iSzBfr;									// Buffer size

	// 'ITickable' members
	STDMETHOD(tick)		( void );
	STDMETHOD(tickAbort)	( void );
	STDMETHOD(tickBegin)	( void );
	STDMETHOD(tickEnd)	( void );

	// CCL
	CCL_OBJECT_BEGIN(File)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual HRESULT	construct	( void );		// Construct object
	virtual void		destruct		( void );		// Destruct object

	// Connections
	DECLARE_EMT(Error)
	DECLARE_RCP(File)
	DECLARE_CON(Read)
	DECLARE_RCP(Stream)
	DECLARE_CON(Write)
	BEGIN_BEHAVIOUR()
		DEFINE_EMT(Error)
		DEFINE_RCP(File)
		DEFINE_CON(Read)
		DEFINE_RCP(Stream)
		DEFINE_CON(Write)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Internal utilities
	HRESULT	fileIo	( BOOL, DWORD, DWORD, DWORD * );
	};

#if	defined(_WIN32)

//
// Class - NotifyDevices.  Node to notify contains of system device changes.
//

class NotifyDevices :
	public CCLObject,										// Base class
	public Behaviour,									// Interface
	public ITickable										// Interface
	{
	public :
	NotifyDevices ( void );								// Constructor

	// Run-time data
	IThread		*pThrd;									// Running window thread
	HWND			hWndDev;									// Handle to hidden window
	HDEVNOTIFY	hDev;										// Device notification
	IDictionary	*pDctN;									// Notification dictionary
	GUID			guidClass;								// Class ID

	// 'ITickable' members
	STDMETHOD(tick)		( void );
	STDMETHOD(tickAbort)	( void );
	STDMETHOD(tickBegin)	( void );
	STDMETHOD(tickEnd)	( void );

	// CCL
	CCL_OBJECT_BEGIN(NotifyDevices)
		CCL_INTF(IBehaviour)
		CCL_INTF(ITickable)
	CCL_OBJECT_END()

	// Connections
	DECLARE_RCP(Start)
	DECLARE_RCP(Stop)
	DECLARE_EMT(Fire)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Start)
		DEFINE_RCP(Stop)
		DEFINE_EMT(Fire)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Internal utilties
	void	notify ( void ) { _EMT(Fire,adtIUnknown(pDctN)); }

	// Win32 window callback
	static LRESULT CALLBACK windowProc	( HWND, UINT, WPARAM, LPARAM );
	};

#endif

//
// Class - Persist.  Node to save/load values to/from streams.
//

class Persist :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Persist ( void );										// Constructor

	// Run-time data
	adtValue			vSave;								// Value to save
	IStreamPersist	*pPrs;								// Persistence parser
	IByteStream		*pStm;								// Active stream

	// CCL
	CCL_OBJECT_BEGIN(Persist)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual void		destruct	( void );			// Destruct object

	// Connections
	DECLARE_CON(Load)
	DECLARE_CON(Save)
	DECLARE_RCP(Parser)
	DECLARE_RCP(Stream)
	DECLARE_RCP(Value)
	DECLARE_EMT(Error)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Load)
		DEFINE_CON(Save)
		DEFINE_RCP(Parser)
		DEFINE_RCP(Stream)
		DEFINE_RCP(Value)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - Resource.  Node to access a generic resource.
//

class Resource :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Resource ( void );									// Constructor

	// Run-time data
	IDictionary		*pOpt;								// Parameters
	IResource		*pRes;								// Parameters
	adtValue			vRes;									// Resource value

	// CCL
	CCL_OBJECT_BEGIN(Resource)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual void		destruct	( void );			// Destruct object

	// Connections
	DECLARE_RCP(Close)
	DECLARE_CON(Open)
	DECLARE_RCP(Options)
	DECLARE_RCP(Resource)
	DECLARE_EMT(Error)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Close)
		DEFINE_CON(Open)
		DEFINE_RCP(Options)
		DEFINE_RCP(Resource)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - Serial.  Node to setup a serial port stream.
//

class Serial :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Serial ( void );										// Constructor

	// Run-time data
	adtInt		iBaud;									// Baud rate
	adtInt		iBits;									// Data bits
	adtString	strParity;								// Parity
	adtFloat		fStop;									// Stop bits

	// CCL
	CCL_OBJECT_BEGIN(Serial)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual void		destruct	( void );			// Destruct object

	// Connections
	DECLARE_CON(Fire)
	DECLARE_EMT(Error)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - StreamOp.  Node to perform operations on a stream.
//

class StreamOp :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	StreamOp ( void );									// Constructor

	// Run-time data
	IByteStream		*pStm;								// Active stream
	adtInt			iOff;									// Offset
	S32				iOrg;									// Origin

	// CCL
	CCL_OBJECT_BEGIN(StreamOp)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual void		destruct	( void );			// Destruct object

	// Connections
	DECLARE_CON(Available)
	DECLARE_RCP(Offset)
	DECLARE_RCP(Origin)
	DECLARE_RCP(Seek)
	DECLARE_CON(Size)
	DECLARE_RCP(Stream)
	DECLARE_EMT(Position)
	DECLARE_EMT(Error)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Available)
		DEFINE_RCP(Offset)
		DEFINE_RCP(Origin)
		DEFINE_RCP(Seek)
		DEFINE_CON(Size)
		DEFINE_RCP(Stream)
		DEFINE_EMT(Position)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - StreamCopy.  Node to perform operations on a stream.
//

class StreamCopy :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	StreamCopy ( void );									// Constructor

	// Run-time data
	IByteStream		*pStmDst,*pStmSrc;				// Active stream
	adtInt			iSz;									// Offset

	// CCL
	CCL_OBJECT_BEGIN(StreamCopy)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual void		destruct	( void );			// Destruct object

	// Connections
	DECLARE_CON(Fire)
	DECLARE_RCP(Destination)
	DECLARE_RCP(Size)
	DECLARE_RCP(Source)
	DECLARE_EMT(Error)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
		DEFINE_RCP(Destination)
		DEFINE_RCP(Size)
		DEFINE_RCP(Source)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - StreamSource.  Node to access streams in a stream source.
//

class StreamSource :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	StreamSource ( void );								// Constructor

	// Run-time data
	IDictionary		*pOpt;								// Parameters
	ILocations		*pSrc;								// Parameters
	IByteStream		*pStm;								// Parameters
	adtString		strLoc,strRefLoc;					// Stream location
	IIt				*pStmsIt;							// Streams iteration

	// CCL
	CCL_OBJECT_BEGIN(StreamSource)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual void		destruct	( void );			// Destruct object

	// Connections
	DECLARE_CON(Open)
	DECLARE_CON(Next)
	DECLARE_CON(Resolve)
	DECLARE_CON(Status)
	DECLARE_RCP(First)
	DECLARE_RCP(Location)
	DECLARE_RCP(Options)
	DECLARE_RCP(Source)
	DECLARE_EMT(End)
	DECLARE_EMT(Error)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Open)
		DEFINE_CON(Next)
		DEFINE_CON(Resolve)
		DEFINE_CON(Status)

		DEFINE_RCP(First)
		DEFINE_RCP(Location)
		DEFINE_RCP(Options)
		DEFINE_RCP(Source)

		DEFINE_EMT(End)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()
	};

#endif
