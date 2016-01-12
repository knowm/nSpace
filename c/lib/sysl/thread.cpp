////////////////////////////////////////////////////////////////////////
//
//									THREAD.CPP
//
//					Implementation of the thread class
//
////////////////////////////////////////////////////////////////////////

// Minimum OS support
#ifndef	_WIN32_WINNT
#define	_WIN32_WINNT		0x0400					// Win95+
#endif

#include "sysl_.h"
#include <stdio.h>
#if	defined(_WIN32) && !defined(UNDER_CE)
#include <process.h>
#endif

Thread :: Thread ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IThread
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	// Setup
	bTick			= FALSE;
	pTick			= NULL;
	#ifdef	_WIN32
	hThread		= NULL;
	dwThreadId	= 0;
	#elif		__unix__ || __APPLEE__
	memset ( &sThread, 0, sizeof(sThread) );
	#endif
	}	// Thread

HRESULT Thread :: construct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called to construct the object.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr			= S_OK;

	// Initialize event
	CCLTRYE ( evStart.init() == TRUE, E_UNEXPECTED );

	return hr;
	}	// construct

void Thread :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed.
	//
	////////////////////////////////////////////////////////////////////////
	threadStop(0);
	}	// destruct

#ifdef	UNDER_CE
DWORD Thread :: ThreadProc ( LPVOID pvoid )
#elif		_WIN32
unsigned __stdcall Thread :: ThreadProc ( VOID *pvoid )
#elif		defined(__unix__) || defined(__APPLE__)
void *Thread :: ThreadProc ( void *pvoid )
#endif
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Entry point for thread
	//
	//	PARAMETERS
	//		-	pvoid is a ptr. to the parent object
	//
	////////////////////////////////////////////////////////////////////////
	Thread		*pThis	= (Thread *) pvoid;
	ITickable	*pTick	= pThis->pTick;
	HRESULT		hr			= S_OK;

	// Keep a reference on our parent until exit
	pTick->AddRef();
	pThis->AddRef();

	// Start of thread
	CCLTRY ( pTick->tickBegin() );

	// Thread has started
	pThis->evStart.signal();

	// Keep running until stopped
	while (pThis->bTick && hr == S_OK)
		hr = pTick->tick();

	// End of thread
	pTick->tickEnd();

	// Done
	pTick->Release();
	pThis->Release();

	// Terminate thread
	#if	defined(UNDER_CE)
	ExitThread(0);
	#elif	defined(_WIN32)
	_endthreadex(0);
	#endif
	return 0;
	}	// ThreadProc

HRESULT Thread :: threadJoin ( U32 numms )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IThread
	//
	//	PURPOSE
	//		-	Waits the specified amount of time for the thread to shutdown.
	//
	//	PARAMETERS
	//		-	numms specifies how long to wait for thread to stop.
	//
	//	RETURN VALUE
	//		S_OK is successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr		= S_OK;
	bool		bSelf	= false;

	// Same thread ?  Sanity check...
	#ifdef	_WIN32
	bSelf = (GetCurrentThreadId() == dwThreadId);
	#elif		__unix__ || __APPLE__
	bSelf = (pthread_self() == sThread);
	#endif

	// Wait for completion
	if (!bSelf)
		{
		#ifdef	_WIN32
		if (hThread != NULL)
			{
			CCLTRYE ( (WaitForSingleObject ( hThread, numms ) == WAIT_OBJECT_0),
							S_FALSE );
			}	// if
		#elif		__unix__ || __APPLE__
		// Thread terminate
		pthread_join ( sThread, NULL );
		#endif
		}	// if
	else hr = E_UNEXPECTED;

	return hr;
	}	// threadJoin

HRESULT Thread :: threadSelf ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IThread
	//
	//	PURPOSE
	//		-	Determines if calling thread is the same as 'this' thread.
	//
	//	RETURN VALUE
	//		S_OK is the same, S_FALSE if not the same.
	//
	////////////////////////////////////////////////////////////////////////
	bool		bSelf	= false;

	// Same thread ?
	#ifdef	_WIN32
	bSelf = (GetCurrentThreadId() == dwThreadId);
	#elif		__unix__ || __APPLE__
	bSelf = (pthread_self() == sThread);
	#endif

	return (bSelf) ? S_OK : S_FALSE;
	}	// threadSelf

HRESULT Thread :: threadStart ( ITickable *_pTick, U32 ms )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IThread
	//
	//	PURPOSE
	//		-	Starts the thread.
	//
	//	PARAMETERS
	//		-	_pTick is the interface to 'tick' along in the thread.
	//		-	ms is the # of milliseconds to wait for thread to 'start'
	//			0 means do not wait
	//
	//	RETURN VALUE
	//		S_OK is successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Store tickable
	if (hr == S_OK)
		{
		pTick = _pTick; //_ADDREF(pTick);
		hr = (pTick != NULL) ? S_OK : E_INVALIDARG;
		}	// if

	// Create thread
	CCLOK ( bTick	= TRUE; )
	#ifdef	UNDER_CE
	if (hr == S_OK)
		{
		hThread	= CreateThread ( NULL, 0, ThreadProc, this, 0, &dwThreadId );
		hr = (hThread != NULL) ? S_OK : GetLastError();
		}	// if
	#elif		_WIN32
	if (hr == S_OK)
		{
		hThread = (HANDLE) _beginthreadex ( NULL, 0, ThreadProc, this, 0, &dwThreadId );
		hr = (hThread != (HANDLE)-1) ? S_OK : errno;
		}	// if
	#elif		__unix__ || __APPLE__
	pthread_attr_t			attr;
	int						ret;

	// Setup
	CCLTRYE	( (ret = pthread_attr_init ( &attr )) == 0, ret );

	// Enable priorities for thread.
	// 50 = "normal"
	// Causes VMWARE to lock up, unneeded anyway ?
//	struct sched_param	sp;
//	CCLOK		( sp.sched_priority = 50; )
//	CCLTRYE	( (ret = pthread_attr_setschedpolicy ( &attr, SCHED_RR )) == 0, ret );
//	CCLTRYE	( (ret = pthread_attr_setschedparam  ( &attr, &sp )) == 0, ret );

	// Create the thread
	CCLTRYE	( (ret = pthread_create ( &sThread, &attr, ThreadProc, this )) == 0,
					ret );
	if (hr != S_OK)
		dbgprintf ( L"Thread::threadStart:hr 0x%x, Id 0x%x\r\n", hr, sThread );
	#endif

	// Wait for start ?
	if (hr == S_OK && ms > 0)
		{
		CCLTRYE ( evStart.wait ( ms ) == TRUE, ERROR_TIMEOUT );
		}	// if

	// Clean up
//	if (hr != S_OK) _RELEASE(pTick);

	return hr;
	}	// threadStart

HRESULT Thread :: threadStop ( U32 numms )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IThread
	//
	//	PURPOSE
	//		-	Stops the thread.
	//
	//	PARAMETERS
	//		-	numms specifies how long to wait for thread to stop.
	//
	//	RETURN VALUE
	//		S_OK is successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr		= S_OK;
	bool		bSelf	= false;

	// Support shutting down 'self'.  Sometimes it is useful to generate
	// the thread stop from the thread itself.  Only difference
	// it just we do not wait for the thread shutdown to occur (i.e. join).
	#ifdef	_WIN32
	bSelf = (GetCurrentThreadId() == dwThreadId);
	#elif		__unix__ || __APPLE__
	bSelf = (pthread_self() == sThread);
	#endif

	// Note to own thread routine
	bTick	= FALSE;

	// Tell tickable
	if (pTick != NULL)
		{
		// Abort 'tick'ing
		pTick->tickAbort();

		// Perform join only if different thread
		if (!bSelf)
			{
			// Wait for completion
			#ifdef	_WIN32
			if (hThread != NULL)
				{
				// Wait for shutdown
				CCLTRYE ( (WaitForSingleObject ( hThread, numms ) == WAIT_OBJECT_0),
								S_FALSE );

				// If thread did not shutdown, force it
				if (hr != S_OK && numms > 0)
					{
					dbgprintf ( L"Thread::threadStop:Warning, thread not terminating after %d ms (Thread %d terminating thread %d)\r\n",
									numms, GetCurrentThreadId(), dwThreadId );
					#ifdef	_DEBUG
					DebugBreak();
					#endif
					TerminateThread ( hThread, S_FALSE );
					hr = S_OK;
					}	// if
				}	// if
			#elif	__unix__ || __APPLE__
			// Thread terminate
			pthread_join ( sThread, NULL );
			#endif
			}	// if
		else
			dbgprintf ( L"Thread::threadStop:Joining own thread\r\n" );
		}	// if

	// Clean up
	pTick = NULL;
//	_RELEASE(pTick);

	return hr;
	}	// threadStop
