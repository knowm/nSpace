////////////////////////////////////////////////////////////////////////
//
//									BEHAVE.CPP
//
//				Implementation of the behaviour base class
//
////////////////////////////////////////////////////////////////////////

#include "nspcl_.h"

Behaviour :: Behaviour ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////

	// Setup
	bReceiving	= false;
	bReceive		= false;
	}	// Behaviour

HRESULT Behaviour :: attach ( IDictionary *_pnLoc, bool bAttach )
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

	// Flag for receiving
	bReceive = bAttach;

	// Remember location
	pnLoc	= _pnLoc;

	// Information about environment
	if (bAttach)
		{
		adtValue		v;
		adtIUnknown	unkV;

		// Namespace
		CCLTRY(pnLoc->load(strnRefNspc,v));
		CCLTRY(_QISAFE(v.punk,IID_INamespace,&pnSpc));

		// Descriptor
		CCLTRY(pnLoc->load(strnRefDesc,v));
		CCLTRY(_QISAFE((unkV=v),IID_IDictionary,&pnDesc));

		// Name
		CCLTRY(pnLoc->load(strnRefName,v));
		CCLOK(strnName = v;)

		// Clean up, no reference counts to avoid circular reference
		if (pnSpc != NULL)	pnSpc->Release();
		if (pnDesc != NULL)	pnDesc->Release();
		}	// if

	// Detach
	else
		{
		// Remove all connections
		if (pnSpc != NULL && pnLoc != NULL)
			pnSpc->connection ( pnLoc, L"", L"", this, NULL );
		}	// else

	// Thread safety
	csRx.leave();

	return hr;
	}	// attach

HRESULT Behaviour :: onAttach ( bool bAttach )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	FROM	Behaviour
	//
	//	PURPOSE
	//		-	Called when behaviour wants to be notified of attachment.
	//
	//	PARAMETERS
	//		-	pnLoc is the location
	//		-	bAttach is true to attach, false to detach
	//
	//	RETURN Behaviour
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	return S_OK;
	}	// onAttach

HRESULT Behaviour :: onReceive (	IReceptor *pr, const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	A location has received a value on the specified receptor.
	//
	//	PARAMETERS
	//		-	pr is the receptor
	//		-	v is the Behaviour
	//
	//	RETURN Behaviour
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	return S_OK;
	}	// onReceive

HRESULT Behaviour :: receive (	IReceptor *pr, const WCHAR *pl, 
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
					lprintf ( LOG_WARN, L"Queue:%d\r\n", sz );
				}	// if

			// Done
			csInt.leave();
			}	// if

		// Not busy, receive
		else
			{
			U32 sz;

			// Busy now
			bReceiving = true;
			csInt.leave();

			// Receive the value
			prl = pl;
			onReceive ( pr, v );

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
					prl = vLoc.pstr;
					onReceive ( (IReceptor *)vSrc.punk, vV );

					// Still ok to receive ?
					CCLTRYE(bReceive == true, ERROR_INVALID_STATE);

					// Internal lock
					csInt.enter();
					}	// while

				}	// if

			// No longer receiving
			bReceiving = false;
			csInt.leave();
			}	// else

		}	// if

	// Receiving thread complete
	csRx.leave();

	return hr;
	}	// receive

