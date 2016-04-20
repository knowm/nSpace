////////////////////////////////////////////////////////////////////////
//
//									ASYNCQ.CPP
//
//				Implementation of the asynchronous queue node
//
////////////////////////////////////////////////////////////////////////

#include "miscl_.h"
#include <stdio.h>

AsyncQ :: AsyncQ ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the CD Player node
	//
	////////////////////////////////////////////////////////////////////////
	pThrd		= NULL;
	bRun		= false;
	pQ			= NULL;
	pQIt		= NULL;
	iSzMax	= 0;
	bBlock	= false;
	iTo		= -1;
	}	// AsyncQ

void AsyncQ :: destruct ( void )
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

HRESULT AsyncQ :: onAttach ( bool bAttach )
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
		adtValue	v;

		// Create value queue
		CCLTRY ( COCREATE ( L"Adt.Queue", IID_IList, &pQ ) );
		CCLTRY ( pQ->iterate ( &pQIt ) );

		// Manual reset events
		CCLTRYE ( evNotEmpty.init(true) == true, GetLastError() );
		CCLTRYE ( evNotFull.init(true) == true, GetLastError() );

		// Default states
		if (pnDesc->load ( adtString(L"Size"), v ) == S_OK)
			iSzMax = v;
		if ( pnDesc->load ( adtString(L"Block"),v ) == S_OK)
			bBlock = v;
		if ( pnDesc->load ( adtString(L"Timeout"),v ) == S_OK)
			iTo = v;
		}	// if

	// Detach
	else if (!bAttach)
		{
		// Shutdown thread
		if (pThrd != NULL)
			{
			pThrd->threadStop(10000);
			pThrd->Release();
			pThrd = NULL;
			}	// if

		// Clean up
		_RELEASE(pQ);
		_RELEASE(pQIt);
		}	// else if

	return hr;
	}	// onAttach

HRESULT AsyncQ :: qValue ( const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Place a value into the queue.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT hr = S_OK;

	// Add to queue
	hr = pQ->write ( v );

	// Queue no longer empty
	evNotEmpty.signal();

	return hr;
	}	// qValue

HRESULT AsyncQ :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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
		CCLTRYE ( bRun == false && pThrd == NULL, E_UNEXPECTED );

		// Empty queue
		CCLTRY( pQ->clear() );
		CCLOK ( evNotEmpty.reset(); )
		CCLOK ( evNotFull.signal(); )

		// Start timed emissions
		CCLTRY(COCREATE(L"Sys.Thread", IID_IThread, &pThrd ));
		CCLOK (bRun = true;)
		CCLTRY(pThrd->threadStart ( this, 5000 ));
	
//		dbgprintf ( L"AsyncQ::receive:Start:0x%x\r\n", hr );
		}	// else if

	// Stop
	else if (_RCP(Stop))
		{
		// Shutdown thread
		if (pThrd != NULL)
			{
			// Make a copy so pThrd can be NULL.  This stops pontentially
			// mulitple 'receiveStop' messages from entering at once
			IThread *pTmp 	= pThrd;
			pThrd 			= NULL;
			pTmp->threadStop(10000);
			pTmp->Release();

			// Should not happen but empty queue just in case
			pQ->clear();
			}	// if
		}	// else if

	// Queue value
	else if (_RCP(Fire))
		{
		bool		bRoom = true;
//		dbgprintf ( L"Misc:::AsyncQ::receive:Fire {\r\n" );

		// Attempt to queue value
		if (hr == S_OK && csWork.enter())
			{
			U32		sz;

			// Room in queue for another item ?
			bRoom = (iSzMax == 0 || (pQ->size ( &sz ) == S_OK && sz < iSzMax));

			// Add value if room
			if (bRoom)	hr = qValue(v);

			// Otherwise queue is full (not 'not full')
			else			evNotFull.reset();

			// Clean up
			csWork.leave();
			}	// if

		// Check if caller blocks on full queue
		if (hr == S_OK && bRoom == false && bBlock == true)
			{
			// Wait for queue to be 'not full' and then add the value
			if (evNotFull.wait(iTo) && csWork.enter())
				{
				// Queue value while protecting state
				CCLTRY ( qValue(v) );
				csWork.leave();
				}	// if
			else				
				{
				// Debug
				dbgprintf ( L"AsyncQ::receive:Fire:Blocked queue timed out when queueing value\r\n" );
				hr = S_FALSE;
				}	// if

			}	// if

//		dbgprintf ( L"} Misc:::AsyncQ::receive:Fire\r\n" );
		}	// else if

	// State
	else if (_RCP(Block))
		bBlock = adtBool(v);
	else if (_RCP(Size))
		iSzMax = adtInt(v);
	else if (_RCP(Timeout))
		iTo = adtInt(v);
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

HRESULT AsyncQ :: tick ( void )
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
	HRESULT	hr		= S_OK;
	bool		bEmit	= false;

//	dbgprintf ( L"AsyncQ::tick {\r\n" );

	// Wait for a queue to be not empty
	CCLTRYE ( evNotEmpty.wait ( -1 ), ERROR_TIMEOUT );
	
	// Keep running ?
	CCLTRYE ( bRun == true, S_FALSE );

	// Emit next item from queue
	if (csWork.enter())
		{
		// Next item
		CCLTRY ( pQIt->begin() );
		if (hr == S_OK && pQIt->read ( vQe ) == S_OK)
			{
			// Will emit value
			bEmit = true;

			// Queue cannot be full now
			evNotFull.signal();

			// Is queue empty now ?
			if (pQIt->next() != S_OK)
				evNotEmpty.reset();
			}	// if

		// Clean up
		csWork.leave();
		}	// if

	// Emit value
	if (hr == S_OK && bEmit)
		_EMT(Fire,vQe);

//	dbgprintf ( L"} AsyncQ::tick\r\n" );
	return hr;
	}	// tick

HRESULT AsyncQ :: tickAbort ( void )
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

	// Make sure all events are signalled so any waits are satisfied.
	evNotEmpty.signal();
	evNotFull.signal();

	return S_OK;
	}	// tickAbort

