////////////////////////////////////////////////////////////////////////
//
//										SYSL_.H
//
//				Implementaiton include file for the system library
//
////////////////////////////////////////////////////////////////////////

#ifndef	SYSL__H
#define	SYSL__H

// Includes
#include	"sysl.h"
#include	"../../lib/ccl/ccl.h"

//#include "../../lib/nspcl/nspcl.h"

///////////
// Objects
///////////

//
// Class - Thread.  Thread control class.
//

class Thread :
	public CCLObject,										// Base class
	public IThread											// Interface
	{
	public :
	Thread ( void );										// Constructor

	// 'IThread' members
	STDMETHOD(threadJoin)	( U32 );
	STDMETHOD(threadSelf)	( void );
	STDMETHOD(threadStart)	( ITickable *, U32 );
	STDMETHOD(threadStop)	( U32 );

	// CCL
	CCL_OBJECT_BEGIN(Thread)
		CCL_INTF(IThread)
	CCL_OBJECT_END()
	virtual HRESULT	construct	( void );		// Construct object
	virtual void		destruct		( void );		// Destruct object

	private :

	// Run-time data
	ITickable	*pTick;									// Tickable for thread
	BOOL			bTick;									// Keep ticking ?
	sysEvent		evStart;									// 'Start' event
	#ifdef		_WIN32
	HANDLE		hThread;									// Handle to thread
	#elif			__unix__ || __APPLE__
	pthread_t	sThread;									// Thread Id
	#endif

	// Internal utilities
	#ifdef	UNDER_CE
	DWORD			dwThreadId;								// Thread Id
	static DWORD WINAPI ThreadProc ( PVOID );			// Thread entry point
	#elif 	_WIN32
	unsigned		dwThreadId;								// Thread Id
	static unsigned __stdcall ThreadProc ( PVOID );	// Thread entry point
	#elif		defined(__unix__) || defined(__APPLE__)
	static void *ThreadProc ( void * );					// Thread entry point
	#endif
	};

#endif
