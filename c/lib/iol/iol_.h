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
//!	\brief Stream based byte buffer cache to be placed in front of another byte stream.
//!	\nodetag ByteStream
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

#ifdef 	_WIN32

//
//!	\brief Contains the information necessary to interact
//!		with an interface IDispatch object.  Can also serve as an event sink.
//!	\nodetag Dispatch
//
class Dispatch;											// Forward dec.
class DispIntf :
	public CCLObject										// Base class
	{
	public :
	DispIntf ( Dispatch * );							// Constructor

	// Run-time data
	Dispatch		*prParent;								// Reference to parent
	IDispatch	*pDisp;									// Dispatch interface
	IDictionary	*pDctFuncs;								// Functions dictionary

	// Utilities
	HRESULT	assign	( IDispatch * );				// Assign interface
	HRESULT	invoke	( const WCHAR *,				// Invoke method
								IDictionary *,
								adtValue & );
	void		unassign ( void );						// Unassign active interface

	// CCL
	CCL_OBJECT_BEGIN_INT(DispIntf)
	CCL_OBJECT_END()
	virtual void destruct		( void );			// Destruct object
	};
#endif

//
//!	\brief Lock synchronization resource object. 
//!	\nodetag Synchronization
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
//!	\brief A block of memory supporting the memory mapped interface.
//!	\nodetag Memory
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
//!	\brief A file system based stream source. 
//!	\nodetag File ByteStream
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
//!	\brief A Win32 compound storage based stream source.
//!	\nodetag Storage ByteStream
//

#ifdef	_WIN32
class StmSrcStg :
	public CCLObject,										// Base class
	public ILocations										// Interface
	{
	public :
	StmSrcStg ( void );									// Constructor

	// Run-time data
	IUnknown			*punkDct;							// Object dictionary
	IDictionary		*pDct;								// Object dictionary
	adtString		strFile;								// Root storage file
	adtString		strRoot;								// Root path
	IDictionary		*pCache;								// Storage/stream cache
	IStorage			*pStg;								// Root storage
	BOOL				bStgRead;							// Storage read only ?

	// 'ILocations' members
	STDMETHOD(open)		( IDictionary *,	IUnknown ** );
	STDMETHOD(locations)	( const WCHAR *,	IIt ** );
	STDMETHOD(resolve)	( const WCHAR *,	bool, ADTVALUE & );
	STDMETHOD(status)		( const WCHAR *,	IDictionary * );

	// CCL
	CCL_OBJECT_BEGIN(StmSrcStg)
		CCL_INTF(ILocations)
		CCL_INTF_AGG(IDictionary,punkDct)
	CCL_OBJECT_END()
	virtual HRESULT	construct	( void );		// Construct object
	virtual void		destruct		( void );		// Destruct object

	private :

	// Internal utilities
	HRESULT	toPath		( const WCHAR *, adtString & );
	HRESULT	validate		( adtString & );
	};
#endif

//
//!	\brief Wrapper object for a file resource.
//!	\nodetag File
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
//!	\brief A file system based stream.
//!	\nodetag File
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
//!	\brief A compound document storage stream.
//!	\nodetag File Storage ByteStream
//

#ifdef _WIN32
class StmStg :
	public CCLObject,										// Base class
	public IByteStream,									// Interface
	public IResource										// Interface
	{
	public :
	StmStg ( IStream * );								// Constructor

	// Run-time data
	IStream			*pStm;								// Active stream object

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
	CCL_OBJECT_BEGIN_INT(StmStg)
		CCL_INTF(IResource)
		CCL_INTF(IByteStream)
	CCL_OBJECT_END()
	virtual void		destruct		( void );		// Destruct object
	};
#endif

//
//!	\brief Implements a byte stream interface on a block of memory.
//!	\nodetag ByteStream MemoryMapped
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
//!	\brief A binary stream value parser
//!	\nodetag Parser
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
//!	\brief An XML stream value parser
//!	\nodetag Parser XML
//

class StmPrsXML :
	public CCLObject,										// Base class
	public IStreamPersist								// Interface
	{
	public :
	StmPrsXML ( void );								// Constructor

	// 'IStreamPersist' members
	STDMETHOD(load)	( IByteStream *, ADTVALUE & );
	STDMETHOD(save)	( IByteStream *, const ADTVALUE & );

	// CCL
	CCL_OBJECT_BEGIN(StmPrsXML)
		CCL_INTF(IStreamPersist)
	CCL_OBJECT_END()
	virtual HRESULT	construct	( void );		// Construct object
	virtual void		destruct		( void );		// Destruct object

	private :

	// Run-time data
	IByteStream			*pStmDoc;						// Document stream
	#ifdef				_WIN32
	IHaveValue			*pStmStm;						// Stream on byte stream
	IUnknown				*pXMLDocLoad;					// XML load document
	IUnknown				*pXMLDocNode;					// Active XML document node
	LONG					lChild;							// Child index
	#elif					__unix__
	void					*pXMLDocChild;					// Active XML child node
	#endif

	// Internal utilities
	HRESULT	valueLoad	( ADTVALUE & );			// Load child
	HRESULT	valueSave	( const ADTVALUE & );	// Save child
	HRESULT	emit			( const WCHAR *,			// Emit character to output stream
									bool = false );
	};

#ifdef	_WIN32
//
//!	\brief Lays an 'IStream' interface on top
//!		of 'IByteStream'.  Useful for APIs that require 'IStream'.
//!	\nodetag ByteStream
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
#endif

//
//!	\brief A ZLib compression/decompression based stream.
//!	\nodetag ByteStream
//

class StmZlib :
	public CCLObject,										// Base class
	public IByteStream,									// Interface
	public IResource										// Interface
	{
	public :
	StmZlib ( void );										// Constructor

	// Run-time data
	IByteStream		*pStm;								// Destination stream
	IMemoryMapped	*pMemWr,*pMemRd;					// Memory buffers
	void				*pvMemWr,*pvMemRd;				// Memory buffers
	adtBool			bReadOnly;							// Current mode
	void				*pvstm;								// Zlib stream ptr

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
	CCL_OBJECT_BEGIN(StmZlib)
		CCL_INTF(IResource)
		CCL_INTF(IByteStream)
	CCL_OBJECT_END()
	virtual HRESULT	construct	( void );		// Construct object
	virtual void		destruct		( void );		// Destruct object

	private :

	// Internal utilities
	HRESULT	flushRead	( void );
	HRESULT	flushWrite	( void );
	HRESULT	writeAll		( IByteStream *, void const *, U64 );
	};

/////////
// Nodes
/////////

#if	defined(_WIN32)

//
//!	\brief Implentation of IDispatch usage (Windows specific).
//!	\nodetag Dispatch Windows
//

class Dispatch :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Dispatch ( void );									// Constructor

	// Run-time data
	DispIntf			*pIntf;								// Interface object
	IDictionary		*pDctP;								// Parameters
	adtString		strName;								// Member name
	IList				*pIntfs;								// Open interfaces

	// CCL
	CCL_OBJECT_BEGIN(Dispatch)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual void		destruct	( void );			// Destruct object

	// Connections
	//! \name Connections 
	//@{
	//!	\brief Close all interfaces for the node.
	DECLARE_CON(Close)
	//!	\brief Invoke method/property action.
	DECLARE_CON(Fire)
	//!	\brief Set the active IDispatch interface
	DECLARE_RCP(Iface)
	//!	\brief Open IDispatch interfac on a new object instance
	DECLARE_CON(Open)
	//!	\brief Set active dictionary of method parameters
	DECLARE_RCP(Params)
	//!	\brief Signaled when an error is countered
	DECLARE_EMT(Error)
	//@}
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Close)
		DEFINE_RCP(Iface)
		DEFINE_CON(Fire)
		DEFINE_CON(Open)
		DEFINE_RCP(Params)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()
	};

//
//!	\brief Node to enumerate through system devices.
//!	\nodetag Iterate Windows
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

	//! \name Connections 
	//@{
	//!	\brief GUID for the type of device to enumerate
	DECLARE_RCP(Class)
	//!	\brief Signals at the end of enumeration
	DECLARE_EMT(End)
	//!	\brief Emit the first enumerated device
	DECLARE_RCP(First)
	//!	\brief Unique system name of the device
	DECLARE_EMT(Name)
	//!	\brief Emit the next enumerated device
	DECLARE_CON(Next)
	//@}
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
//!	\brief General file I/O node.  Only used for direct (possibly asynchronous I/O) access to a 'file' resource without IByteStream
//!	\nodetag File
//

class File :
	public CCLObject,										// Base class
	public Behaviour,										// Interface
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
		CCL_INTF(ITickable)
	CCL_OBJECT_END()
	virtual HRESULT	construct	( void );		// Construct object
	virtual void		destruct		( void );		// Destruct object

	//! \name Connections 
	//@{
	//!	\brief Signals on an error state
	DECLARE_EMT(Error)
	//!	\brief Set active file resource
	DECLARE_RCP(File)
	//!	\brief Explicitly perform a read (when synchronous)
	DECLARE_CON(Read)
	//!	\brief Set the active input or output byte stream for data
	DECLARE_RCP(Stream)
	//!	\brief Explicity perform a write
	DECLARE_CON(Write)
	//@}
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
//!	\brief Provides notifications of system device changes.
//!	\nodetag Asynchronous System Devices
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

	//! \name Connections 
	//@{
	//!	\brief Begin monitoring system for device changes
	DECLARE_RCP(Start)
	//!	\brief End monitoring system for device changes
	DECLARE_RCP(Stop)
	//!	\brief Output dictionary containing information about a device change
	DECLARE_EMT(Fire)
	//@}
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
//!	\brief Save/load values to/from byte streams using provided parser.
//!	\nodetag Persist 
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

	//! \name Connections 
	//@{
	//!	\brief Load and emit a value from the stream
	DECLARE_CON(Load)
	//!	\brief Save and emit a value to the stream
	DECLARE_CON(Save)
	//!	\brief Specifies parser object to use during persistence
	DECLARE_RCP(Parser)
	//!	\brief Specifies byte stream for input/output.
	DECLARE_RCP(Stream)
	//!	\brief Sets the active value for saving.
	DECLARE_RCP(Value)
	//!	\brief Signals on error
	DECLARE_EMT(Error)
	//@}
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
//!	\brief Node to access a generic resource.
//!	\nodetag Resource
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

	//! \name Connections 
	//@{
	//!	\brief Close the active resource
	DECLARE_RCP(Close)
	//!	\brief Open the active resource with the active options.
	DECLARE_CON(Open)
	//!	\brief Specify options dictionary for opening resources.
	DECLARE_RCP(Options)
	//!	\brief Specify the active resource
	DECLARE_RCP(Resource)
	//!	\brief Signals on error
	DECLARE_EMT(Error)
	//@}
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Close)
		DEFINE_CON(Open)
		DEFINE_RCP(Options)
		DEFINE_RCP(Resource)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()
	};

//
//!	\brief Provide serial port specific functionality.
//!	\nodetag Serial
//

class Serial :
	public CCLObject,										// Base class
	public ITickable,										// Interface
	public Behaviour										// Interface
	{
	public :
	Serial ( void );										// Constructor

	// Run-time data
	adtInt		iBaud;									// Baud rate
	adtInt		iBits;									// Data bits
	adtString	strParity;								// Parity
	adtFloat		fStop;									// Stop bits
	IIt			*pItEv;									// Monitoring event list
	IThread		*pThrd;									// Monitoring thread
	bool			bRun;										// Thread should run
	#ifdef		_WIN32
	HANDLE		hPort;									// Handle to serial port
	HANDLE		hevWait;									// Wait event handle
	#endif

	// 'ITickable' members
	STDMETHOD(tick)		( void );
	STDMETHOD(tickAbort)	( void );
	STDMETHOD(tickBegin)	( void );
	STDMETHOD(tickEnd)	( void );

	// CCL
	CCL_OBJECT_BEGIN(Serial)
		CCL_INTF(IBehaviour)
		CCL_INTF(ITickable)
	CCL_OBJECT_END()
	virtual void		destruct	( void );			// Destruct object

	//! \name Connections 
	//@{
	//!	\brief Emits a signal when a monitoring state changes
	DECLARE_EMT(Change)
	//!	\brief Apply serial settings
	DECLARE_CON(Fire)
	//!	\brief Signals on an error
	DECLARE_EMT(Error)
	//!	\brief Specifies the active serial port resource.
	DECLARE_RCP(Port)
	//!	\brief Begin monitoring serial line for changes
	DECLARE_RCP(Start)
	//!	\brief End monitoring serial line for changes
	DECLARE_RCP(Stop)
	//@}
	BEGIN_BEHAVIOUR()
		DEFINE_EMT(Change)
		DEFINE_CON(Fire)
		DEFINE_EMT(Error)
		DEFINE_RCP(Port)
		DEFINE_RCP(Start)
		DEFINE_RCP(Stop)
	END_BEHAVIOUR_NOTIFY()
	};

//
//!	\brief Perform operations on a byte stream.
//!	\nodetag ByteStream
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

	//! \name Connections 
	//@{
	//!	\brief Emits the number of bytes available for reading in the active byte stream
	DECLARE_CON(Available)
	//!	\brief Specify the active offset for byte stream operations
	DECLARE_RCP(Offset)
	//!	\brief Specify the origin of seek operations, valid strings are : Set, End, or Current
	DECLARE_RCP(Origin)
	//!	\brief Seeks to the active offset from the active origin  within the byte stream
	DECLARE_RCP(Seek)
	//!	\brief Set the size of the active byte stream
	DECLARE_CON(Size)
	//!	\brief Set the active byte stream
	DECLARE_RCP(Stream)
	//!	\brief Emits the active stream position after a seek
	DECLARE_EMT(Position)
	//!	\brief Signals on error
	DECLARE_EMT(Error)
	//@}
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
//!	\brief Copyies between byte streams and/or memory mapped bytes
//!	\nodetag ByteStream MemoryMapped
//

class StreamCopy :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	StreamCopy ( void );									// Constructor

	// Run-time data
	IUnknown			*punkDst,*punkSrc;				// Active objects
	adtInt			iSz;									// Offset

	// CCL
	CCL_OBJECT_BEGIN(StreamCopy)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()
	virtual void		destruct	( void );			// Destruct object

	//! \name Connections 
	//@{
	//!	\brief Perform a copy between the source and destination
	DECLARE_CON(Fire)
	//!	\brief Specifies the destination byte stream or memory mapped object
	DECLARE_RCP(Destination)
	//!	\brief Provide a specific number of bytes to copy
	DECLARE_RCP(Size)
	//!	\brief Specifies the source byte stream or memory mapped object
	DECLARE_RCP(Source)
	//!	\brief Signals on error
	DECLARE_EMT(Error)
	//@}
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
		DEFINE_RCP(Destination)
		DEFINE_RCP(Size)
		DEFINE_RCP(Source)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()
	};

//
//!	\brief Node to access byte streams from inside a stream source.
//!	\nodetag Locations ByteStream
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

	//! \name Connections 
	//@{
	//!	\brief Open and emit a byte stream from within the stream source
	DECLARE_CON(Open)
	//!	\brief Emit the next available byte stream location
	DECLARE_CON(Next)
	//!	\brief Resolve and emit a given location to a fully qualified location within the stream source
	DECLARE_CON(Resolve)
	//!	\brief Emit a dictionary containing information about the active byte stream location
	DECLARE_CON(Status)
	//!	\brief Emit the first available byte stream location
	DECLARE_RCP(First)
	//!	\brief Specify the active byte stream location for future operations
	DECLARE_RCP(Location)
	//!	\brief Specify an options dictionary when opening the next stream
	DECLARE_RCP(Options)
	//!	\brief Specifies a stream source object
	DECLARE_RCP(Source)
	//!	\brief Signals when there are no more byte streams
	DECLARE_EMT(End)
	//!	\brief Signals on an error
	DECLARE_EMT(Error)
	//@}
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
