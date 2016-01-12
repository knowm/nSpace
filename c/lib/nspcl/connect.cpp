////////////////////////////////////////////////////////////////////////
//
//									CONNECT.CPP
//
//					Implementation of the location connect node
//
////////////////////////////////////////////////////////////////////////

#include "nspcl_.h"

Connect :: Connect ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pRcpLoc	= NULL;
	pDctMk	= NULL;
	pDctLoc	= NULL;
	pSpc		= NULL;
	bActive	= false;
	}	// Connect

HRESULT Connect :: connect ( IDictionary *pEmit )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Process any connections for the location.
	//
	//	PARAEMTERS
	//		-	pEmit is an optional dictionary that, if specified, will
	//			cause function to only make connections that match the 
	//			emitter path.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	IDictionary	*pConn	= NULL;
	IIt			*pIt		= NULL;
	adtValue		v;
	adtIUnknown	unkV;
	adtString	strE;

	// Does location have connection location ?
	CCLTRY(pDctLoc->load ( strnRefConn, v ));
	CCLTRY(_QISAFE((unkV=v),IID_IDictionary,&pConn));

	// Is an emitter specified ?
	if (hr == S_OK && pEmit != NULL)
		{
		CCLTRY	( pEmit->load ( strnRefFrom, v ) );
		CCLTRYE	( (strE = v).length() > 0, E_UNEXPECTED );
		}	// if

	// Since the connection array exists in the namespace the indexes
	// are stored as strings, to ensure proper connection order, order
	// the indicies first.
	CCLTRY ( pDctSeq->clear() );
	CCLTRY ( pConn->keys ( &pIt ) );
	while (hr == S_OK && pIt->read(v) == S_OK)
		{
		adtInt	iIdx(v);

		// Store integer version of key in sequential dicitonary
		if (iIdx > 0 && pConn->load ( v, v ) == S_OK)
			hr = pDctSeq->store ( iIdx, v );

		// Next entry
		pIt->next();
		}	// while
	_RELEASE(pIt);

	// Connections are stored in the order desired.  Connect/disconnect
	// each location.  
	CCLTRY ( pDctSeq->keys ( &pIt ) );
	while (hr == S_OK && pIt->read(v) == S_OK)
		{
		IDictionary	*pDct	= NULL;
		adtString	strF,strT;

		// String version of integer index
		adtString	strKey(v);

		// Connection information
		CCLTRY ( pDctSeq->load ( v, v ) );
		CCLTRY ( _QISAFE((unkV=v),IID_IDictionary,&pDct) );
		CCLTRY ( pDct->load ( strnRefFrom, v ) );
		CCLTRYE( (strF = v).length() > 0, E_UNEXPECTED );
		CCLTRY ( pDct->load ( strnRefTo, v ) );
		CCLTRYE( (strT = v).length() > 0, E_UNEXPECTED );
//		CCLOK ( dbgprintf ( L"Location %d:%s -> %s\r\n", idx, (LPCWSTR)strF, (LPCWSTR)strT ); )
//		if (hr == S_OK && !WCASECMP(strF,L"TupleTest/A1/Fire"))
//			dbgprintf ( L"Hi\r\n" );

		// Process all connections or only ones with specified emitter
		if (hr == S_OK &&
				(pEmit == NULL || !WCASECMP(strE,strF)) )
			connect ( strF, strT, bActive );

		// Add/remove connection to/from internal dictionary
		if (hr == S_OK)
			{
			if (bActive)
				hr = pDctMk->store ( strKey, adtIUnknown(pDct) );
			else
				pDctMk->remove ( strKey );
			}	// if

		// Clean up
		_RELEASE(pDct);
		pIt->next();
		}	// for

	// Clean up
	_RELEASE(pIt);
	_RELEASE(pConn);

	return hr;
	}	// connect

HRESULT Connect :: connect (	const WCHAR *wFrom,
										const WCHAR *wTo, bool bConnect )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IGraph
	//
	//	PURPOSE
	//		-	Connects two locations
	//
	//	PARAMETERS
	//		-	wFrom is the 'from' location
	//		-	wTo is the 'to' location
	//		-	bConnect is true to connect, false to disconnect
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	ILocation	*pLoc	= NULL;
	IReceptor	*pR	= NULL;
	adtValue		vF,vT;
	adtIUnknown	unkV;

	// A relative path can be retrieved from the specified graph.
	// An 'external' connection will have to be retrieved from the namespace.

	// State check
	CCLTRYE ( pDctLoc != NULL && pSpc != NULL, ERROR_INVALID_STATE );

	// Debug
//	dbgprintf ( L"%s : %s -> %s\r\n", (bConnect) ? L"connect" : L"disconnect", wFrom, wTo );
//	if (!WCASECMP(wFrom,L"Initialize/OnFire") && !WCASECMP(wTo,L"Debug/Fire"))
//	if (!WCASECMP(wTo,L"ReceiveVal/Value"))
//	if (!WCASENCMP(wTo,L"StoreImgKeys",12))
//		dbgprintf ( L"Hi\r\n" );

	// 'From'
	if (hr == S_OK)
		{
		// Local connection
		if (wFrom[0] != '/')
			hr = nspcLoadPath ( pDctLoc, wFrom, vF );

		// Non-local
		else
			hr = pSpc->get ( wFrom, vF, NULL );
		}	// if

	// 'To'
	if (hr == S_OK)
		{
		// Local connection
		if (wTo[0] != '/')
			hr = nspcLoadPath ( pDctLoc, wTo, vT );

		// Non-local
		else
			hr = pSpc->get ( wTo, vT, NULL );
		}	// if

	// Debug.  Sanity check in case a non-emitter is connecting to a non-receptor.
/*
	if (hr == S_OK)
		{
		IDictionary	*pDctTst	= NULL;
		adtValue		vType;

		// Emitter ?
		CCLTRY ( _QISAFE((unkV = vF),IID_IDictionary,&pDctTst) );
		if (	hr == S_OK												&& 
				pDctTst->load ( strnRefType, vType ) == S_OK &&
				WCASECMP(vType.pstr,L"Emitter"))
			dbgprintf ( L"Location::connect:Source not emitter:%s\r\n", wFrom );
		_RELEASE(pDctTst);			

		// Receptor ?
		CCLTRY ( _QISAFE((unkV = vT),IID_IDictionary,&pDctTst) );
		if (	hr == S_OK												&& 
				pDctTst->load ( strnRefType, vType ) == S_OK &&
				WCASECMP(vType.pstr,L"Receptor"))
			dbgprintf ( L"Location::connect:Destination not receptor:%s\r\n", wTo );
		_RELEASE(pDctTst);			
		}	// if
*/

	// Make connection
	CCLTRY ( _QISAFE((unkV = vF),IID_ILocation,&pLoc) );
	CCLTRY ( _QISAFE((unkV = vT),IID_IReceptor,&pR) );
	CCLTRY ( pLoc->connect ( pR, bConnect, false ) );

	// Debug
	if (hr != S_OK)// && bConnect)
		lprintf ( LOG_WARN, L"Missing connection:%s:%s:%s:0x%x",
						wFrom, wTo, (bConnect) ? L"connect" : L"disconnect", hr );

	// Clean up
	_RELEASE(pR);
	_RELEASE(pLoc);

	return hr;
	}	// connect

HRESULT Connect :: construct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		CCLObject
	//
	//	PURPOSE
	//		-	Called when the object is being created.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr	= S_OK;

	// Create run-time connections dictionary
	CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDctMk ) );

	// Dictionary for ordering connections
	CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDctSeq ) );

	return hr;
	}	// construct

void Connect :: destruct ( void )
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
	_RELEASE(pDctMk);
	_RELEASE(pDctSeq);
	}	// destruct

HRESULT Connect :: emitBrk ( IDictionary *pDct )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Break all connections for the emitter specified in the 
	//			connection dictionary for previously 'made' connections.
	//
	//	PARAMETERS
	//		-	pDct contains the connection specification
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	IIt			*pIt	= NULL;
	adtString	strF;
	adtValue		vL;

	// Access the 'from' connection
	CCLTRY	( pDct->load ( strnRefFrom, vL ) );
	CCLTRYE	( (strF = vL).length() > 0, E_UNEXPECTED );

	// Iterate existing connections
	CCLTRY ( pDctMk->keys ( &pIt ) );
	while (hr == S_OK && pIt->read ( vL ) == S_OK)
		{
		IDictionary		*pDctChk	= NULL;
		bool				bNxt		= true;
		adtIUnknown		unkV;
		adtValue			vKey;

		// Remember key
		CCLTRY ( adtValue::copy ( vL, vKey ) );

		// Access connection information
		if (	hr == S_OK									&&
				pDctMk->load ( vKey, vL ) == S_OK	&&
				(IUnknown *)(NULL) != (unkV = vL)	&&
				_QI(unkV,IID_IDictionary,&pDctChk) == S_OK)
			{
			adtString	strFchk,strT;

			// Check for matching 'from'
			if (	pDctChk->load ( strnRefFrom, vL ) == S_OK &&
					(strFchk = vL).length() > 0					&&
					!WCASECMP(strF,strFchk) )
				{
				// Matching connection, break it
				CCLTRY	( pDctChk->load ( strnRefTo, vL ) );
				CCLTRYE	( (strT = vL).length() > 0, E_UNEXPECTED );
				CCLOK    ( connect ( strF, strT, false ); )

				// Remove from internal list
				pDctMk->remove ( vKey );

				// Iteration must restart 
				pIt->begin();
				bNxt	= false;
				}	// if
			}	// if

		// Clean up
		_RELEASE(pDctChk);
		if (bNxt)
			pIt->next();
		}	// while

	// Clean up
	_RELEASE(pIt);

	return hr;
	}	// emitBrk

HRESULT Connect :: emitBrk ( const WCHAR *pwIdx )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Break a single previosly made connection.
	//
	//	PARAMETERS
	//		-	pwIdx is the index to break.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	IDictionary	*pDct	= NULL;
	adtString	strIdx(pwIdx),strF,strT;
	adtValue		vL;
	adtIUnknown	unkV;

	// Access connection information
	CCLTRY	( pDctMk->load ( strIdx, vL ) );
	CCLTRY	( _QISAFE((unkV=vL),IID_IDictionary,&pDct) );
	CCLTRY	( pDct->load ( strnRefFrom, vL ) );
	CCLTRYE	( (strF = vL).length() > 0, E_UNEXPECTED );
	CCLTRY	( pDct->load ( strnRefTo, vL ) );
	CCLTRYE	( (strT = vL).length() > 0, E_UNEXPECTED );

	// Disconnect connection
	CCLOK    ( connect ( strF, strT, false ); )

	// Remove from internal list
	pDctMk->remove ( strIdx );

	// Clean up
	_RELEASE(pDct);

	return hr;
	}	// emitBrk

HRESULT Connect :: receive ( IReceptor *pr, const WCHAR *pl, 
										const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	FROM	IReceptor
	//
	//	PURPOSE
	//		-	A location has received a Connect on the specified receptor.
	//
	//	PARAMETERS
	//		-	pr is the receptor
	//		-	pl is the location
	//		-	v is the Connect
	//
	//	RETURN Connect
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;
	adtInt	iIdx;

	// Debug
//	dbgprintf ( L"Connect::receive:%s\r\n", pl );

	// Active state ?
	if (!WCASECMP(pl,STR_NSPC_ACTIVE))
		{
		adtValue		vL;

		// New active state
		bActive = adtBool(v);

		// Previous state
		pDctLoc	= NULL;
		pSpc		= NULL;

		// Setup
		pRcpLoc	= pr;
		_QISAFE(pRcpLoc,IID_IDictionary,&pDctLoc);
		if (pDctLoc != NULL)
			{
			// Do not hold reference count on location
			pDctLoc->Release();

			// Namespace object
			if (pDctLoc->load ( strnRefNspc, vL ) == S_OK)
				pSpc = (INamespace *)(IUnknown *)(vL.punk);
			}	// if

		// Process all connections
		connect(NULL);
		}	// if

	// Numbered state.  Real-time updates are only necessary if location is active
	else if (bActive																&&
				adtValue::fromString ( pl, VTYPE_I4, iIdx ) == S_OK	&& 
				iIdx > 0)
		{
		IDictionary		*pDct	= NULL;
		adtIUnknown		unkV(v);

		// Debug
		dbgprintf ( L"Connect::receive:Change while active at idx %d!\r\n", (U32)iIdx );

		// Specification
		CCLTRY(_QISAFE(unkV,IID_IDictionary,&pDct));

		// Storing new connection
		if (!adtValue::empty(v))
			{
			// Since connection order is important, existing connections must be disconnected
			// so that the new connections can be propertly connected in order.

			// Break 'made' connections from emitter in internal list
			emitBrk ( pDct );

			// Make all connections for same emitter in the stored connection list
			connect ( pDct );
			}	// if

		// Removing connection
		else
			{
			// Remove make connection from position
			emitBrk ( pl );
			}	// else

		// Clean up
		_RELEASE(pDct);
		}	// else if

	return hr;
	}	// receive

