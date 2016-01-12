////////////////////////////////////////////////////////////////////////
//
//									Timer.CPP
//
//					Implementation of the timer node
//
////////////////////////////////////////////////////////////////////////

#include "miscl_.h"
#include <stdio.h>
#ifndef	_WIN32
#include	<unistd.h>
#endif
#if      defined(__unix__) || defined(__APPLE__)
#include <errno.h>
#endif

Timer :: Timer ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the CD Player node
	//
	////////////////////////////////////////////////////////////////////////
	pThread	= NULL;
	bRun		= false;
	bArm		= false;
	firet		= 0;
	bArmd		= false;
	bFireNow	= false;
	bSigNow	= false;
	uRate		= 1000;
	iPri		= 0;
	#if		__unix__ || __APPLE__
	gettimeofday 			( &t0, NULL );
	#endif
	}	// Timer

void Timer :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		CCLObject
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed
	//
	////////////////////////////////////////////////////////////////////////

	// Shutdown thread
	if (pThread != NULL)
		{
		pThread->threadStop(10000);
		pThread->Release();
		pThread = NULL;
		}	// if

	}	// destruct

HRESULT Timer :: onAttach ( bool bAttach )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when this behaviour is assigned to a node
	//
	//	PARAMETERS
	//		-	bAttach is true for attachment, false for detachment.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////

	// Attach
	if (bAttach)
		{
		adtValue	v;

		// Default states
		if (pnDesc->load ( adtString(L"Rate"), v ) == S_OK)
			uRate = v;
		if (pnDesc->load ( adtString(L"Priority"), v ) == S_OK)
			iPri = v;
		if (pnDesc->load ( adtString(L"Arm"), v ) == S_OK)
			bArm = v;

		// Initialize event
		evWork.init();
		}	// if

	// Detach
	else if (!bAttach && pThread != NULL)
		{
		// Shutdown thread
		pThread->threadStop(10000);
		pThread->Release();
		pThread = NULL;
		}	// else if

	return S_OK;
	}	// onAttach

HRESULT Timer :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	The node has received a value on the specified receptor.
	//
	//	PARAMETERS
	//		-	pr is the receptor
	//		-	v is the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Start
	if (_RCP(Start))
		{
		// Already started ?
		CCLTRYE ( bRun == false && pThread == NULL, E_UNEXPECTED );

		// Start timed emissions
		CCLTRY(COCREATE(L"Sys.Thread", IID_IThread, &pThread ));
		CCLOK (bRun = true;)
		CCLTRY(pThread->threadStart ( this, 5000 ));
		}	// else if

	// Stop
	else if (_RCP(Stop))
		{
		// Shutdown thread
		if (pThread != NULL)
			{
			// Make a copy so pThread can be NULL.  This stops pontentially
			// mulitple 'receiveStop' messages from entering at once
			IThread *pTmp 	= pThread;
			pThread 			= NULL;
			pTmp->threadStop(10000);
			pTmp->Release();
			}	// if
		}	// else if

	// Arm
	else if (_RCP(Arm))
		{
		// Arm Timer
		if (!bArmd)
			{
			bArmd	= true;
			evWork.signal();
			}	// if
		}	// if

	// Disarm
	else if (_RCP(Disarm))
		{
		// Disarm Timer
		if (bArmd)
			{
			bArmd	= false;
			evWork.signal();
			}	// if
		}	// else if

	// Fire
	else if (_RCP(Fire))
		{
		// Used to fire the Timer immediately
		if (!bFireNow)
			{
			bFireNow	= true;
			evWork.signal();
			}	// if
		}	// else if

	// Context
	else if (_RCP(Rate))
		{
		// Set a new rate
		if (v.vtype == VTYPE_I4 && v.vint > 0)
			{
			// New rate
			uRate = v.vint;

			// Wake to reset timer
			evWork.signal();
			}	// if
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

HRESULT Timer :: tick ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITickable
	//
	//	PURPOSE
	//		-	Perform one 'tick's worth of work.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	U32	now,to,ret,dt;

	// Compute remaining time
	now	= tickCount();
	to		= uRate;
	dt		= now-firet;
	if (dt >= to)	to = 0;
	else				to -= dt;

	// Signal immediately ?
	if (bArm == false && bSigNow.vbool == TRUE)
		{
		to					= 0;
		bSigNow.vbool	= FALSE;
		}	// if

	// Wait forever if armable and not armed.
	else if (bArm == true && bArmd == false)
		to = (U32)-1;

	// Wait for remainder of time or a signal
	ret	= wait ( to );
	firet	= tickCount();

	// Shutting down ?
	if (!bRun) 
		return S_FALSE;

	// If we timed out, emit signal
	if (	bFireNow ||
			(ret == ERROR_TIMEOUT && (!bArm || (to != (U32)-1))) )
		{
		bArmd		= false;									// This first because fire might re-arm
//		dbgprintf ( L"Timer::tick\r\n" );
		_EMT(Fire,iV);
		bFireNow	= false;									// Do this after in case we were already firing
		}	// if

	return S_OK;
	}	// tick

HRESULT Timer :: tickAbort ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITickable
	//
	//	PURPOSE
	//		-	Notifies the object that 'ticking' should abort.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	bRun = false;
	evWork.signal();
	return S_OK;
	}	// tickAbort

HRESULT Timer :: tickBegin ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITickable
	//
	//	PURPOSE
	//		-	Notifies the object that it should get ready to 'tick'.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;
	adtValue		v;

	// Re load parameters
	if (pnDesc->load ( adtString(L"EmitAtZero"), v ) == S_OK)
		bSigNow = v;

	// Priority
	#ifdef	_WIN32
	if (	hr == S_OK && iPri != 0 )
		{
		if (!SetThreadPriority ( GetCurrentThread(),
				(iPri.vint == 2)	?	THREAD_PRIORITY_HIGHEST :
				(iPri.vint == 1)	?	THREAD_PRIORITY_ABOVE_NORMAL :
				(iPri.vint == -1)	?	THREAD_PRIORITY_BELOW_NORMAL :
				(iPri.vint == -2)	?	THREAD_PRIORITY_LOWEST :
											THREAD_PRIORITY_NORMAL ))
			dbgprintf ( L"Timer::tickBegin:Unable to set priority\n" );
		}	// if
	#endif

	#if	defined(__unix__) || defined(__APPLE__)
	if (hr == S_OK && iPri != 0)
		{
		pthread_t	self;
		sched_param	sp;
		int			p,ret;

		// Get current priority and adjust by the specified amount
		CCLOK		( self = pthread_self(); )
		CCLTRYE	( (ret = pthread_getschedparam ( self, &p, &sp )) == 0, ret );
		CCLOK		( sp.sched_priority += ((S32)iPri); )
		CCLTRYE	( (ret = pthread_setschedparam ( self, p, &sp )) == 0, ret );
		}	// if
	#endif

	// Initialize variables
	firet	= tickCount();

	return S_OK;
	}	// tickBegin

U32 Timer :: tickCount ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Returns the # of milliseconds elapsed since an arbitrary time.
	//
	//	RETURN VALUE
	//		# of milliseconds since 't zero'
	//
	////////////////////////////////////////////////////////////////////////
	U32	ret = 0;

	#ifdef	_WIN32
	ret = GetTickCount();
	#endif

	#if	defined(__unix__) || defined(__APPLE__)
	struct timeval		now;

	// Compute relative elapsed time since t0
	if (gettimeofday ( &now, NULL ) == 0)
		{
		// Watch for negative usec
		if (now.tv_usec > t0.tv_usec)
			ret = (now.tv_usec - t0.tv_usec) / 1000;
		else
			ret = 1000 - ((t0.tv_usec - now.tv_usec)/1000);

		// Seconds
		ret += (now.tv_sec - t0.tv_sec) * 1000;
		}	// if
	#endif

	return ret;
	}	// tickCount

HRESULT Timer :: wait ( U32 to )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Waits for the specified amount of time or until we are
	//			interrupted.
	//
	//	PARAMETERS
	//		-	to is the timeout value in ms.  -1 = forever
	//
	//	RETURN VALUE
	//		S_OK if event signalled, ERROR_TIMEOUT if timeout expored.
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr	= S_OK;

	// Any time ?
	if (!to)
		return ERROR_TIMEOUT;

	// Wait
	CCLTRYE ( evWork.wait ( (to != (U32)-1) ? to : INFINITE ), ERROR_TIMEOUT );

	return hr;
	}	// wait
