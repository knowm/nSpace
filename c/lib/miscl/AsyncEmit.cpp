////////////////////////////////////////////////////////////////////////
//
//									ASYNCE.CPP
//
//				Implementation of the asynchronous emission node
//
////////////////////////////////////////////////////////////////////////

#include "miscl_.h"
#include <stdio.h>

AsyncEmit :: AsyncEmit ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the CD Player node
	//
	////////////////////////////////////////////////////////////////////////
	pThrd		= NULL;
	bRun		= false;
	iPri		= 0;
	bSingle	= false;
	evEmit.init();
	}	// AsyncEmit

void AsyncEmit :: destruct ( void )
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
	onAttach(false);
	}	// destruct

HRESULT AsyncEmit :: onAttach ( bool bAttach )
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
	HRESULT	hr = S_OK;

	// Attach
	if (bAttach)
		{
		adtValue vL;

		// Default states
		pnDesc->load ( adtString(L"Value"), vVal );
		if (pnDesc->load ( adtString(L"Single"), vL ) == S_OK)
			bSingle = vL;
		}	// if

	// Detach
	else if (!bAttach)
		{
		// Shutdown thread
		if (pThrd != NULL)
			{
			bRun = false;
			pThrd->threadStop(5000);
			_RELEASE(pThrd);
			}	// if
		}	// else if

	return hr;
	}	// onAttach

HRESULT AsyncEmit :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Emit
	if (_RCP(Fire))
		{
		// Single shot ?
		if (bSingle == true && pThrd != NULL)
			{
			bRun = false;
			pThrd->threadStop(5000);
			_RELEASE(pThrd);
			}	// if

		// Create thread if necessary
		if (hr == S_OK && pThrd == NULL)
			{
			CCLTRY(COCREATE(L"Sys.Thread", IID_IThread, &pThrd ));
			CCLOK (bRun = true;)
			CCLTRY(pThrd->threadStart ( this, 5000 ));
			}	// if

		// Copy value to emit
		if (hr == S_OK)
			{
			// Thread safety
			csVal.enter();

			// Available ?
			CCLTRYE ( adtValue::empty(vEmit) == true, E_UNEXPECTED );

			// New value
			CCLTRY ( adtValue::copy ( adtValue::empty(vVal) ? v : vVal, vEmit ) );

			// Thread safety
			csVal.leave();

			// Signal availability
			CCLOK ( evEmit.signal(); )
			}	// if

		}	// if

	// State
	else if (_RCP(Value))
		adtValue::copy ( v, vVal );
	else if (_RCP(Priority))
		iPri = v;
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

HRESULT AsyncEmit :: tick ( void )
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
	HRESULT		hr		= S_OK;
	bool			bEmit = false;
	adtValue		vEmitNow;

	// Wait for value to emit
	CCLTRYE ( evEmit.wait(-1), ERROR_TIMEOUT );

	// Still running ?
	CCLTRYE ( bRun == true, S_FALSE );

	// Emit value
	if (hr == S_OK && !adtValue::empty(vEmit))
		{
		// Make local copy so next value can be readied immediately
		csVal.enter();
		CCLTRY ( adtValue::copy ( vEmit, vEmitNow ) );

		// Clear value to signal slot available
		CCLOK ( adtValue::clear ( vEmit ); )
		csVal.leave();

		// Emit
		CCLOK ( _EMT(Fire,vEmitNow); )

		// No need to exit thread due to emission error
		hr = S_OK;
		}	// if

	// Single shot ?
	if (bSingle == true)
		hr = S_FALSE;

	return hr;
	}	// tick

HRESULT AsyncEmit :: tickBegin ( void )
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

	return S_OK;
	}	// tickBegin
