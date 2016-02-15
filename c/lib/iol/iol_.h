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
	public IStreamPersist								// Interface
	{
	public :
	StmPrsXML ( void );									// Constructor

	// Run-time data

	// 'IStreamPersist' members
	STDMETHOD(load)	( IByteStream *, ADTVALUE & );
	STDMETHOD(save)	( IByteStream *, const ADTVALUE & );

	// CCL
	CCL_OBJECT_BEGIN(StmPrsXML)
		CCL_INTF(IStreamPersist)
	CCL_OBJECT_END()

	private :

	// Internal utilities
	HRESULT	emit		( IByteStream *, const WCHAR *, bool = false );
	HRESULT	writeAll	( IByteStream *, const void *, U32 );
	};

/////////
// Nodes
/////////

#if	defined(_WIN32)
//
// Class - EnumDevClass.  Node to iterate through system devices
//		of a particular class.
//

class EnumDevClass :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	EnumDevClass ( void );								// Constructor

	// Run-time data
	GUID			guidClass;								// Class ID
	HANDLE		hEnum;									// Enumeration handle
	U32			idx;										// Current device index
	sysDl			dl;										// Library

	// CCL
	CCL_OBJECT_BEGIN(EnumDevClass)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_RCP(Class)
	DECLARE_RCP(First)
	DECLARE_CON(Next)
	DECLARE_EMT(End)
	DECLARE_EMT(Name)
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
// Class - Persist.  Node to save/load values to/from streams.
//

class Persist :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
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
	public IBehaviour										// Interface
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
// Class - StreamOp.  Node to perform operations on a stream.
//

class StreamOp :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
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
	public IBehaviour										// Interface
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
	public IBehaviour										// Interface
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
