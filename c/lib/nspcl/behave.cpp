////////////////////////////////////////////////////////////////////////
//
//									BEHAVE.CPP
//
//					Implementation of the behaviour wrapper class
//
////////////////////////////////////////////////////////////////////////

#include "nspcl_.h"

Behaviour :: Behaviour ( IBehaviour *_pBehave )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////

	// Setup
	pBehave		= _pBehave;
	_ADDREF(pBehave);
	pBehaveR		= NULL;
	bReceiving	= false;
	bReceive		= false;

	// Auto add-ref self
	AddRef();
	}	// Behaviour

HRESULT Behaviour :: attach ( IDictionary *pnLoc, bool bAttach )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	FROM	IBehaviour
	//
	//	PURPOSE
	//		-	Called when the behaviour is attached to a location.
	//
	//	PARAMETERS
	//		-	pnLoc is the location
	//		-	bAttach is true to attach, false to detach
	//
	//	RETURN Behaviour
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT hr = S_OK;

	// Thread safety
	csRx.enter();

	// Receive flag
	if (pBehave == NULL)
		bReceive = false;

	// Contained object
	hr = (pBehave != NULL) ? pBehave->attach ( pnLoc, bAttach ) : E_UNEXPECTED;

	// Receive flag
	if (pBehave != NULL && hr == S_OK)
		bReceive = true;

	// Thread safety
	csRx.leave();

	return hr;
	}	// attach

void Behaviour :: destruct ( void )
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

	// Shutting down
	if (csInt.enter())
		{
		bReceive = false;
		csInt.leave();
		}	// if

	// Clean up
	_RELEASE(pBehave);
	}	// destruct

HRESULT Behaviour :: receive ( IReceptor *pr, const WCHAR *pl, 
											const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	FROM	IReceptor
	//
	//	PURPOSE
	//		-	A location has received a Behaviour on the specified receptor.
	//
	//	PARAMETERS
	//		-	pr is the receptor
	//		-	pl is the location
	//		-	v is the Behaviour
	//
	//	RETURN Behaviour
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT hr = S_OK;

	// Block other threads into behaviour until reception on active
	// thread is complete.
	if (bReceive == false || !csRx.enter())
		return E_UNEXPECTED;

	// Receive value
	if (csInt.enter())
		{
		// If behaviour is already receiving a value, queue it up for later.
		// The queue is not created until now because only a small number
		// will end up having to deal with this situation (iterators, etc).
		if (bReceiving)
			{
			// Need a queue ?
			if (pRxQ == NULL)
				{
				// Create the reception value queue and set up iterator
				CCLTRY(COCREATE(L"Adt.Queue",IID_IList,&pRxQ));
				CCLTRY(pRxQ->iterate ( &pRxIt ) );
				}	// if

			// Queue information for later processing
			CCLTRY ( pRxQ->write ( adtIUnknown(pr) ) );
			CCLTRY ( pRxQ->write ( adtString(pl) ) );
			CCLTRY ( pRxQ->write ( v ) );

			// Debug
			if (hr == S_OK)
				{
				U32	sz;
				pRxQ->size ( &sz );
				if (!(sz % 10))
					lprintf ( LOG_WARN, L"Queue:%p:%d\r\n", pBehave, sz );
				}	// if

			// Done
			csInt.leave();
			}	// if

		// Not busy, receive
		else
			{
			U32 sz;

			// Obtain a behaviour reference for duration of reception
			pBehaveR = pBehave;
			_ADDREF(pBehaveR);

			// Busy now
			bReceiving = true;
			csInt.leave();

			// Receive the value
			pBehaveR->receive ( pr, pl, v );

			// Still ok to receive ?
			CCLTRYE(bReceive == true, ERROR_INVALID_STATE);

			// Any queue receptions to process ?
			if (	hr == S_OK						&&
					csInt.enter()					&& 
					pRxQ != NULL					&&
					pRxQ->size ( &sz ) == S_OK && 
					sz > 0 )
				{
				adtValue	vSrc,vLoc,vV;

				// Process queue values
				while (hr == S_OK && pRxIt->read ( vSrc ) == S_OK)
					{
					// Read location and value
					CCLOK  ( pRxIt->next(); )
					CCLTRY ( pRxIt->read ( vLoc ) );
					CCLOK  ( pRxIt->next(); )
					CCLTRY ( pRxIt->read ( vV ) );
					CCLOK  ( pRxIt->next(); )

					// Release internal lock
					csInt.leave();

					// Receive value, direct cast ok since this function queued values.
					pBehaveR->receive ( (IReceptor *)vSrc.punk, vLoc.pstr, vV );

					// Still ok to receive ?
					CCLTRYE(bReceive == true, ERROR_INVALID_STATE);

					// Internal lock
					csInt.enter();
					}	// while

				}	// if

			// No longer receiving
			_RELEASE(pBehaveR);
			bReceiving = false;
			csInt.leave();
			}	// else

		}	// if

	// Receiving thread complete
	csRx.leave();

	return hr;
	}	// receive

