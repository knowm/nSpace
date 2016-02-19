////////////////////////////////////////////////////////////////////////
//
//										SYSL.H
//
//					Underlying system specific library
//
////////////////////////////////////////////////////////////////////////

#ifndef	SYSL_H
#define	SYSL_H

// System include
#if		defined(_WIN32)
#include <Windows.h>
#include <OCIdl.h>
#endif
#if defined(__APPLE__) || defined(__unix__)
#include <stdlib.h>
#include <wchar.h>
#include <unistd.h>
#if defined(__APPLE__)
#include <libkern/OSAtomic.h>
#endif
#include <pthread.h>
#include <memory.h>
#include <errno.h>
#endif

// Portable data types
#if		defined(_WIN32)
typedef unsigned	char		U8;
typedef signed		char		S8;
typedef unsigned	short		U16;
typedef signed		short		S16;
typedef unsigned	int		U32;
typedef signed		int		S32;
typedef unsigned	__int64	U64;
typedef signed		__int64	S64;
typedef U16						WCHAR;
#endif

#if defined(__APPLE__) || defined(__unix__)
typedef unsigned	char			U8;
typedef signed		char			S8;
typedef unsigned	short			U16;
typedef signed		short			S16;
typedef unsigned	int			U32;
typedef signed		int			S32;
typedef unsigned	long long	U64;
typedef signed		long long	S64;
#ifdef	__LP64__
typedef S64							UINT_PTR;
#else
typedef S32							UINT_PTR;
#endif

// Win32 types
typedef wchar_t		WCHAR;
typedef S32				HRESULT;
typedef S8				BOOL;
typedef void *			HANDLE;
typedef void *			HINSTANCE;
typedef double			DATE;
typedef U32				LCID;
typedef const WCHAR *LPCWSTR;
typedef const WCHAR *PCWSTR;
typedef U8				BYTE;
typedef U32				ULONG;
typedef S32				LONG;
typedef WCHAR			*BSTR;
typedef U32				DWORD;
typedef U16				WORD;
typedef void *			HWND;
typedef U32				UINT;
typedef U32				WPARAM;
typedef U32				LPARAM;

#endif

//
// Platform specific definitions
//

#if	defined(_WIN32)
// Case-insensitive wide string compare
#define	WCSCPY(a,b,c)		wcscpy_s ( a, b, c )
#define	WCSCAT(a,b,c)		wcscat_s ( a, b, c )
#define	WCASECMP				_wcsicmp
#define	WCASENCMP			_wcsnicmp
#define	WCSTOK				wcstok_s
#define	SWPF(a,b)			(a),(b)
#define	SWSCANF				swscanf_s
#define	SSCANF				sscanf_s
#define	STRCPY(a,b,c)		strcpy_s ( a, b, c )
#define	STRCAT(a,b,c)		strcat_s ( a, b, c )
#define	SPF					sprintf_s
#endif

#if defined(__APPLE__) || defined(__unix__)

#define	WCSCPY(a,b,c)		wcscpy ( a, c )
#define	WCSCAT(a,b,c)		wcscat ( a, c )
#define	WCASECMP				wcscasecmp
#define	WCASENCMP			wcsncasecmp
#define	WCSTOK				wcstok
#define	SWPF(a,b)			(a),(b)
#define	SWSCANF				swscanf
#define	SSCANF				sscanf
#define	STRCPY(a,b,c)		strcpy ( a, c )
#define	STRCAT(a,b,c)		strcat ( a, c )
#define	SPF					sprintf
#define	WINAPI
#define	STDAPI				 extern "C" HRESULT
#define	__NOGDI__							// GDI not available

// Error codes
#define	S_OK								0
#define	S_FALSE							1
#define	E_INVALIDARG					0x80070057
#define	E_NOTIMPL						0x80004001
#define	CLASS_E_NOAGGREGATION		0x80004002
#define	E_FAIL							0x80004005
#define	CLASS_E_CLASSNOTAVAILABLE	0x80040111
#define	E_UNEXPECTED					0x8000FFFF
#define	E_OUTOFMEMORY					0x8007000E
#define	E_NOINTERFACE					0x80004002
#define	REGDB_E_CLASSNOTREG			0x80040154
#define	ERROR_INVALID_OPERATION		0x8009802C
#define	ERROR_NOT_FOUND				1168
#define	ERROR_NO_MATCH					1169
#define	ERROR_TIMEOUT					1460L
#define	ERROR_INVALID_STATE			5023
#define	LOCALE_USER_DEFAULT			0x00000400
#define	MAXDWORD							0xffffffff
#define	INFINITE							MAXDWORD
#define	STREAM_SEEK_SET				0
#define	STREAM_SEEK_CUR				1
#define	STREAM_SEEK_END				2
#define	CP_ACP							0
#define	INVALID_HANDLE_VALUE			MAXDWORD

//
// COM
//

#define	DECLARE_INTERFACE_(i,bi)	struct i : public bi
#define	DECLARE_INTERFACE(i)			struct i
#define	STDMETHOD(method)				virtual HRESULT method
#define	STDMETHOD_(type,method)		virtual type method
#define	PURE								=0

// Creation context values
typedef enum tagCLSCTX
	{
	CLSCTX_INPROC_SERVER		= 0x01,
	CLSCTX_INPROC_HANDLER	= 0x02,
	CLSCTX_LOCAL_SERVER		= 0x04,
	CLSCTX_REMOTE_SERVER		= 0x10
	} CLSCTX;

#define	CLSCTX_ALL		(	CLSCTX_INPROC_SERVER	| \
									CLSCTX_INPROC_HANDLER| \
									CLSCTX_LOCAL_SERVER	| \
									CLSCTX_REMOTE_SERVER)

// GUID definition from Win32
typedef struct _GUID
	{
	U32	Data1;
	U16	Data2;
	U16	Data3;
	U8		Data4[8];
	} GUID;
typedef GUID				IID;
typedef GUID				CLSID;
typedef const IID		&	REFIID;
typedef const CLSID	&	REFCLSID;

#define DEFINE_GUID(name, l, w1, w2, b1, b2, b3, b4, b5, b6, b7, b8 ) \
	const GUID (name) = { (l), (w1), (w2), { (b1), (b2), (b3), (b4), (b5), (b6), (b7), (b8) } }

DEFINE_GUID	(	GUID_NULL, 0x00000000, 0x0000, 0x0000, 0x00, 0x00,
					0x00, 0x00, 0x00, 0x00, 0x00, 0x00 );

#define	CLSID_NULL	GUID_NULL

typedef enum tagCOINIT
	{
	COINIT_APARTMENTTHREADED	= 0x2,
	COINIT_MULTITHREADED			= 0x0,
	} COINIT;

//
// Class - IUnknown.  Base interface for all other interfaces.
//

DEFINE_GUID	(	IID_IUnknown, 0x00000000, 0x0000, 0x0000, 0xc0, 0x00,
					0x00, 0x00, 0x00, 0x00, 0x00, 0x46 );

DECLARE_INTERFACE(IUnknown)
	{
	public :
	STDMETHOD(QueryInterface)	( REFIID, void ** )	PURE;
	STDMETHOD_(ULONG,AddRef)	( void )					PURE;
	STDMETHOD_(ULONG,Release)	( void )					PURE;
	};

//
// Class - IClassFactory.  Interface for a class factory.
//

DEFINE_GUID	(	IID_IClassFactory, 0x00000001, 0x0000, 0x0000, 0xc0, 0x00,
					0x00, 0x00, 0x00, 0x00, 0x00, 0x46 );

DECLARE_INTERFACE_(IClassFactory,IUnknown)
	{
	public :
	STDMETHOD(CreateInstance)	( IUnknown *, REFIID, void ** )	PURE;
	STDMETHOD(LockServer)		( BOOL )									PURE;
	};

//
// Class - IPersist.  Interface for an object that can be stored
//		persistently.
//

DEFINE_GUID	( IID_IPersist, 0x0000010c, 0x0000, 0x0000, 0xc0, 0x00,
					0x00, 0x00, 0x00, 0x00, 0x00, 0x46 );

DECLARE_INTERFACE_(IPersist,IUnknown)
	{
	public :
	STDMETHOD(GetClassID)	( CLSID * ) PURE;
	};

#if		!defined(InlineIsEqualGUID)
#define	InlineIsEqualGUID(rguid1,rguid2)								\
		      (																	\
				((U32 *) &rguid1)[0] == ((U32 *) &rguid2)[0] &&		\
		      ((U32 *) &rguid1)[1] == ((U32 *) &rguid2)[1] &&		\
				((U32 *) &rguid1)[2] == ((U32 *) &rguid2)[2] &&		\
		      ((U32 *) &rguid1)[3] == ((U32 *) &rguid2)[3]			\
				)
#endif

// Inline functions
inline BOOL IsEqualGUID ( const GUID &id1, const GUID &id2 )
	{
	return ( (id1.Data1 == id2.Data1) &&
				(id1.Data2 == id2.Data2) &&
				(id1.Data3 == id2.Data3) &&
				( *((U32 *)(&id1.Data4[0])) == *((U32 *)(&id2.Data4[0])) ) &&
				( *((U32 *)(&id1.Data4[4])) == *((U32 *)(&id2.Data4[4])) ) );
	}	// IsEqualGUID

inline HRESULT GetLastError()
	{
	return errno;
	}	// GetLastError

// Win32 functions
extern "C"
	{
	HRESULT	CLSIDFromProgID		( const WCHAR *, CLSID * );
	HRESULT	CLSIDFromString		( const WCHAR *, CLSID * );
	HRESULT	CoCreateInstance		( REFCLSID, IUnknown *, U32, REFIID, void ** );
	HRESULT	CoGetClassObject		( REFCLSID, U32, void *, REFIID, void ** );
	HRESULT	CoInitializeEx			( void *, U32 );
	void		CoUninitialize			( void );
	S32		MultiByteToWideChar	( U32, S32, const char *, S32, WCHAR *, S32 );
	HRESULT	StringFromCLSID		( REFCLSID, WCHAR ** );
	HRESULT	VarDateFromStr			( WCHAR *, LCID, U32, DATE * );
	S32		WideCharToMultiByte	( U32, S32, const WCHAR *, S32, char *, S32,
												const char *, BOOL * );
	}

#if defined(__APPLE__)

// Atomic operations
#define	InterlockedCompareExchange(a,b,c) \
		OSAtomicCompareAndSwap64 ( c, (int64_t) b, (int64_t *) a )

#define	InterlockedIncrement(a)	OSAtomicIncrement32 ( a )
#define	InterlockedDecrement(a)	OSAtomicDecrement32 ( a )

#else

#define	InterlockedCompareExchange(a,b,c) __sync_val_compare_and_swap ( a, c, b )
#define	InterlockedIncrement(a)	__sync_fetch_and_add( (a), 1 )
#define	InterlockedDecrement(a)	__sync_fetch_and_sub( (a), 1 )

#endif

#endif

//
// Memory management
//

//
// Structure - sysALLOC.  Function ptrs. for custom memory management.
//

typedef struct
	{
	void *(*alloc)		( U32 );							// Allocate memory
	void	(*free)		( void * );						// Free memory
	void *(*realloc)	( void *, U32 );				// Reallocate memory
	} sysALLOC;

//
// Structure - sysSTRING.  A reference counted string.  Structure and string
//		are allocated as one so string can be treated as a single 'value'.
//

typedef struct
	{
	LONG		refcnt;										// Reference count
	LONG		nalloc;										// Size of string
	} sysSTRING;

// Macros
#define	_ALLOCMEM(a)		sysMemAlloc ( (a) )
#define	_FREEMEM(a)			{ sysMemFree ( (a) ); (a) = NULL; }
#define	_REALLOCMEM(a,b)	sysMemRealloc ( (a), (b) )
#define	_ADDREFSTR(a)		sysStringAddRef ( (a) )
#define	_RELEASESTR(a)		{ sysStringRelease ( (a) ); (a) = NULL; }
#define	_FREEBSTR(a)		if ((a) != NULL) { SysFreeString ( (a) ); (a) = NULL; }
#define	_FREETASKMEM(a)	{ CoTaskMemFree ( (a) ); (a) = NULL; }

// Not defined on all platforms
#ifndef	FALSE
#define	FALSE		0
#endif
#ifndef	TRUE
#define	TRUE		1
#endif

// Prototypes - Memory
void			*sysMemAlloc			( U32 );
void			sysMemFree				( void * );
void			*sysMemRealloc			( void *, U32 );
void			sysMemUse				( sysALLOC * );
S32			sysStringAddRef		( sysSTRING * );
S32			sysStringRelease		( sysSTRING * );
sysSTRING	*sysStringAlloc		( const WCHAR * );
sysSTRING	*sysStringReallocLen	( sysSTRING *, U32 );
sysSTRING	*sysStringAllocLen	( const WCHAR *, U32 );

//////////////
// Interfaces
//////////////

//
// Interface - ICloneable.  Interface for cloning an object.
//

DEFINE_GUID	(	IID_ICloneable, 0x2534d01b, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DECLARE_INTERFACE_(ICloneable,IUnknown)
	{
	public :
	STDMETHOD(clone)	( IUnknown ** )	PURE;
	};

//
// Interface - IThis.  Interface for obtaining a ptr. to an object.
//

DEFINE_GUID	(	IID_IThis, 0x2534d08a, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DECLARE_INTERFACE_(IThis,IUnknown)
	{
	public :
	STDMETHOD(getThis)	( void ** )	PURE;
	};

//
// Interface - ITickable.  Interface to an object that supports 'ticking'.
//

DEFINE_GUID	(	IID_ITickable, 0x2534d022, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DECLARE_INTERFACE_(ITickable,IUnknown)
	{
	public :
	STDMETHOD(tick)		( void )		PURE;
	STDMETHOD(tickAbort)	( void )		PURE;
	STDMETHOD(tickBegin)	( void )		PURE;
	STDMETHOD(tickEnd)	( void )		PURE;
	};

//
// Interface - IThread.  Interface to a thread.
//

DEFINE_GUID	(	IID_IThread, 0x2534d04f, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DECLARE_INTERFACE_(IThread,IUnknown)
	{
	public :
	STDMETHOD(threadJoin)	( U32 )					PURE;
	STDMETHOD(threadSelf)	( void )					PURE;
	STDMETHOD(threadStart)	( ITickable *, U32 )	PURE;
	STDMETHOD(threadStop)	( U32 )					PURE;
	};

///////////
// Classes
///////////

DEFINE_GUID	(	CLSID_Thread, 0x2534d056, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

DEFINE_GUID	(	CLSID_UtilUUID, 0x2534d05a, 0x8628, 0x11d2, 0x86, 0x8c,
					0x00, 0x60, 0x08, 0xad, 0xdf, 0xed );

///////////
// Objects
///////////

//
// Class - sysEvent.  Win32-like event.
//

class sysEvent
	{
	public :
	sysEvent ( void );									// Constructor
	~sysEvent ( void );									// Destructor

	// Utilities
	bool init 	( void );								// Initialize event
	bool reset	( void );								// Reset event
	bool signal	( void );								// Signal event
	bool wait 	( U32 );									// Wait for signal

	// Run-time data
	#ifdef	_WIN32
	HANDLE				hEv;								// Event handle
	#elif		__unix__ || __APPLE__
	pthread_mutex_t	mtx;								// Event mutex
	pthread_cond_t		cnd;								// Event condition variable
	#endif
	};

//
// Class - sysCS.  Critical section.
//

class sysCS : private sysEvent
	{
	public :
	sysCS ( void );										// Constructor
	~sysCS ( void );										// Destructor

	// Utilities
	bool	enter		( void );							// Enter critical section
	bool	leave		( void );							// Leave critical section

	// Run-time data
	#ifdef	_WIN32
	LONG			lOwnerThrd;								// Owner thread
	#elif		__APPLE__ || __unix__
	pthread_t	lOwnerThrd;								// Owner thread
	#endif
	LONG			uLockCount;								// Lock count
	LONG			uWaitCount;								// Wait count
	};

//
// Class - sysDl.  Dynamic library object.
//

class sysDl
	{
	public :
	sysDl	( const WCHAR * );							// Constructor
	~sysDl( void );										// Destructor

	// Utilities
	virtual LONG AddRef	();							// Add reference to library
	virtual LONG Release ();							// Remove reference from library

	// Extractors
	#ifdef	_WIN32
	inline operator HMODULE() const { return hLib; }
	#elif				__unix__ || __APPLE__
	inline operator void *() const { return pvLib; }
	#endif

	protected :
	// Run-time data
	LONG				lRefCnt;								// Reference count
	const wchar_t	*pwLib;								// Path to library
	sysCS				cs;									// Critical section
	#ifdef			_WIN32
	HMODULE			hLib;									// Handle to library
	#elif				__unix__ || __APPLE__
	void				*pvLib;								// Library
	#endif
	};

// Prototypes
extern "C"
	{
	int		dbgprintf	( const WCHAR *, ... );
	}

// Logging system
#define		LOG_DBG				0
#define		LOG_INFO				1
#define		LOG_WARN				2
#define		LOG_ERR				3
#define		LOG_FATAL			4
#ifdef		_WIN32
#define		LOG_WIDE(x)			L ## x
#else
#define		LOG_WIDE(x)			L## # x
#endif
#define		LOG_WIDE2(x)		LOG_WIDE(x)
#define		lprintf(a,b,...)	logPrintf ( LOG_WIDE2(__FILE__), __LINE__, LOG_WIDE2(__FUNCTION__), a, b, ##__VA_ARGS__ )

//
// Class - cLogEntry.  Log entry.
//

class cLogEntry
	{
	public :
	cLogEntry ( const WCHAR *, int,					// Constructor
					const WCHAR *, int, 
					const WCHAR * );
	virtual ~cLogEntry ( void );						// Destructor

	// Run-time data
	sysSTRING		*file;								// Source file
	int				line;									// Line number
	sysSTRING		*func;								// Source function
	int				level;								// Log level
	sysSTRING		*str;									// Logging text
	cLogEntry		*next;								// Next log entry
	};

// Callback function
typedef void (*logCallback)	( cLogEntry *, void * );

//
// Class - cLogInfo.  Global log information.
//

class cLogInfo
	{
	public :
	cLogInfo				( void );						// Constructor
	virtual ~cLogInfo ( void );						// Destructor

	// Utilities
	void flush ( cLogEntry ** );						// Flush entry to callback

	// Run-time data
	WCHAR			wLogBfr[32768];						// Buffers
	WCHAR			wLogFmt[32768];						// Buffers
	sysCS			csLog;									// Critical section
	logCallback	pCB;										// Callback
	void			*pvCB;									// Callback parameter
	cLogEntry	*pHead;									// Head of log list
	bool			bBusy;									// Logging system busy
	};

extern "C"
	{
	int	logPrintf	( const WCHAR *, int, const WCHAR *, int, const WCHAR *, ... );
	void	logSink		( logCallback, void * );
	}

#endif

