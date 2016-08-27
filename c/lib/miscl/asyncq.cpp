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
	pQw		= NULL;
	pQwIt		= NULL;
	pQs		= NULL;
	pQvs		= NULL;
	iMaxSz	= 0;
	bBlock	= false;

	// Default Id in case node is being used in single queue mode
	adtValue::copy ( adtInt(0), vQId );
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

HRESULT AsyncQ :: getQ ( const adtValue &vId, IList **ppQ )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Retrieve/create a queue with the specified Id.
	//
	//	PARAMETERS
	//		-	pr is the receptor
	//		-	v is the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;

	// Access queue
	if (hr == S_OK && pQs->load ( vId, vQ ) != S_OK)
		{
		// First time accessing queue
		CCLTRY ( COCREATE ( L"Adt.Queue", IID_IList, ppQ ) );
		CCLTRY ( pQs->store ( vId, (unkV=(*ppQ)) ) );
		}	// if
	else
		hr = _QISAFE((unkV = vQ),IID_IList,ppQ);

	return hr;
	}	// getQ

HRESULT AsyncQ :: onAttach ( bool bAttach )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//! \brief Called when a behaviour is attached/detached to a location.
	//! \param bAttach is true on attachment, false on detachment
	//! \return S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Attach
	if (bAttach)
		{
		adtValue	v;

		// Create work queue
		CCLTRY ( COCREATE ( L"Adt.Queue", IID_IList, &pQw ) );
		CCLTRY ( pQw->iterate ( &pQwIt ) );

		// Create dictionary for queues and queue values
		CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pQs ) );
		CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pQvs ) );

		// Resources
		CCLTRYE ( evWork.init() == true, GetLastError() );

		// Default states
		if (pnDesc->load ( adtString(L"Size"), v ) == S_OK)
			iMaxSz = v;
		if ( pnDesc->load ( adtString(L"Block"),v ) == S_OK)
			bBlock = v;
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
		_RELEASE(pQw);
		_RELEASE(pQwIt);
		_RELEASE(pQs);
		_RELEASE(pQvs);
		}	// else if

	return hr;
	}	// onAttach

HRESULT AsyncQ :: onReceive ( IReceptor *pr, const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//! \brief A location has received a value on the specified receptor.
	//! \param pr is a ptr. to the receptor that received the value
	//! \param v is the received value
	//! \return S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Start
	if (_RCP(Start))
		{
		// Already started ?
		CCLTRYE ( bRun == false && pThrd == NULL, E_UNEXPECTED );

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
			// Timeout is tricky since output may be busy
//			pTmp->threadStop(10000);
			pTmp->threadStop(0);
			pTmp->Release();
			}	// if
		}	// else if

	// Queue value
	else if (_RCP(Fire))
		{
//		dbgprintf ( L"Misc:::AsyncQ::receive:Fire {\r\n" );
		// Attempt to queue value
		if (hr == S_OK && csWork.enter())
			{
			IList		*pQ	= NULL;
			U32		sz;

			// Access queue with current Id
			CCLTRY ( getQ ( vQId, &pQ ) );

			// Add to queue if there is room
			if (iMaxSz == 0 || (pQ->size ( &sz ) == S_OK && sz < iMaxSz))
				{
				// Add value to queue
//				dbgprintf ( L"AsyncQ::receive:Size %d\r\n", sz );
				CCLTRY ( pQ->write ( v ) );

				// If there is no active value, signal thread to emit it.
				if (hr == S_OK && pQvs->load ( vQId, vQ ) != S_OK)
					{
					// Set active value
					CCLTRY ( pQvs->store ( vQId, v ) );

					// Queue work
					CCLTRY ( pQw->write ( vQId ) );
					CCLOK  ( evWork.signal(); )
					}	// if
				}	// if

			// TODO: Implement blocking option ?
//			else
//				dbgprintf ( L"AsyncQ::receive:WARNING Queue full\r\n" );

			// Clean up
			_RELEASE(pQ);
			csWork.leave();
			}	// if

//		dbgprintf ( L"} Misc:::AsyncQ::receive:Fire\r\n" );
		}	// else if

	// Next
	else if (_RCP(Next))
		{
		// Proceed to the next value in the queue (if present)
		if (hr == S_OK && csWork.enter())
			{
			IList		*pQ	= NULL;
			IIt		*pQIt	= NULL;
			U32		sz;

			// Access queue with current Id
			CCLTRY ( getQ ( vQId, &pQ ) );

			// Current value is no longer valid
			CCLOK ( pQvs->remove ( vQId ); )

			// Is there another value available ?
			if (	pQ->size ( &sz ) == S_OK		&& 
					sz > 0								&& 
					pQ->iterate ( &pQIt ) == S_OK &&
					pQIt->next() == S_OK				&&
					pQIt->read ( vQ ) == S_OK) 
				{
				// Set active value
				CCLTRY ( pQvs->store ( vQId, vQ ) );

				// Queue work
				CCLTRY ( pQw->write ( vQId ) );
				CCLOK  ( evWork.signal(); )
				}	// if

			// Clean up
			_RELEASE(pQIt);
			_RELEASE(pQ);
			csWork.leave();
			}	// if

		}	// else if

	// Retry
	else if (_RCP(Retry))
		{
		// Re-emit the current value
		if (hr == S_OK && csWork.enter())
			{
			// Signal only if there is a valid value
			if (pQvs->load ( vQId, vQ ) == S_OK)
				{
				// Queue work
				CCLTRY ( pQw->write ( vQId ) );
				CCLOK  ( evWork.signal(); )
				}	// if

			// Clean up
			csWork.leave();
			}	// if

		}	// else if

	// State
	else if (_RCP(Id))
		{
		// Set new Id
		if (csWork.enter())
			{
			adtValue::copy ( v, vQId );
			csWork.leave();
			}	// if
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

HRESULT AsyncQ :: tick ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//! \brief Perform one 'tick's worth of work.
	//! \return S_OK if ticking should continue
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr		= S_OK;
	bool		bEmit;

//	dbgprintf ( L"AsyncQ::tick {\r\n" );
	
	// Keep running ?
	CCLTRYE ( bRun == true, S_FALSE );

	// Process queued work
	while (hr == S_OK && pQwIt->read ( vIdE ) == S_OK)
		{
		// Work for current queue
		bEmit = false;
		if (csWork.enter())
			{
			// Valid value ?
			bEmit = (pQvs->load ( vIdE, vQE ) == S_OK);
			csWork.leave();
			}	// if

		// Emit current value if valid
		if (bEmit)
			{
			// Emit current Id and value
//			dbgprintf ( L"AsyncQ::tick:emit\r\n" );
			_EMT(Id,vIdE);
			_EMT(Fire,vQE);
			}	// if

		// Clean up
		pQwIt->next();
		}	// while

	// Wait for additional work
	CCLTRYE ( evWork.wait ( -1 ), ERROR_TIMEOUT );
		
//	dbgprintf ( L"} AsyncQ::tick\r\n" );
	
	return hr;
	}	// tick

HRESULT AsyncQ :: tickAbort ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//! \brief Notifies the object that 'ticking' should stop.
	//! \return S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	bRun = false;
	evWork.signal();
	return S_OK;
	}	// tickAbort

HRESULT AsyncQ:: tickBegin ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//! \brief Notifies the object that it should prepare to 'tick'.
	//! \return S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	return S_OK;
	}	// tickBegin

HRESULT AsyncQ:: tickEnd ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//! \brief Notifies the object that 'ticking' has stopped.
	//! \return S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	return S_OK;
}	// tickEnd

/*
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

HRESULT AsyncQ :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

*/
