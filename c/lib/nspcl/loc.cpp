////////////////////////////////////////////////////////////////////////
//
//								LOC.CPP
//
//					Implementation of the namespace location node.
//
////////////////////////////////////////////////////////////////////////

#include "nspcl_.h"

// Globals
extern GlobalNspc	nspcglb;

Location :: Location ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	pDctc		= NULL;
	pDctOut	= NULL;
	pLocOut	= NULL;
	pRcpOut	= NULL;
	pPar		= NULL;
	pDctPar	= NULL;
	pSpc		= NULL;
	bActive	= false;
	bRx		= false;
	bRcp		= false;
	bEmt		= false;
	pRxQ		= NULL;
	pRxIt		= NULL;
	pBehave	= NULL;
	bBehaveV	= false;
	bLocThis	= false;
	pRcpConn = NULL;
	}	// Location

HRESULT Location :: active ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Process the active state.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;

	// Debug
//	dbgprintf ( L"Location::active:%s:%d\r\n", (LPCWSTR)strName, bActive );

	// Send uninit signal down the chain
	if (hr == S_OK && !bActive)
		receive ( pDctOut, L"Uninitialize/Fire", strName, false );

	// Notify sub-locations
	CCLTRY ( activeNotify(bActive) );

	// Process descriptor and connections, order depends on active state
	if (hr == S_OK)
		{
		// If active, initialize descriptors first then connect
		if (bActive)
			desc();
		else if (pRcpConn != NULL)
			pRcpConn->receive ( this, STR_NSPC_ACTIVE, adtBool(bActive) );

		// If inactive, disconnect then shutdown descriptors
		if (bActive)
			{
			if (pRcpConn != NULL)
				pRcpConn->receive ( this, STR_NSPC_ACTIVE, adtBool(bActive) );
			}	// if
		else
			desc();
		}	// if

	// Initialize graphs as they become active
	if (hr == S_OK && bActive)
		receive ( pDctOut, L"Initialize/Fire", strName );

	return hr;
	}	// active

HRESULT Location :: activeNotify ( bool bA )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Notify sub-locations of active state
	//
	//	PARAMETERS
	//		-	bA is the active state
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	ILocation	*pL	= NULL;
	IDictionary	*pLd	= NULL;
	IIt			*pIt	= NULL;
	adtValue		v;
	adtIUnknown	unkV;
	adtBool		bAct(bA);

	// Do not propagate notifications beyond the connector level to avoid
	// possible false signalling to other graph instances that might be in
	// the connector value itself.
	if (bRcp || bEmt)
		{
		// To avoid possible recursive reference counts, clear any stored/cached
		// values for emitters
		if (bLocThis)
			{
			// Remove the value from the connector
			pDctc->remove ( strnRefVal );

			// Clear cached value
			bLocThis = false;
			adtValue::clear ( vLocThis );
			}	// if

		// Done
		return S_OK;
		}	// if

	// Propagate active state to sub-locations
	CCLTRY ( keys ( &pIt ) );
	while (hr == S_OK && pIt->read ( v ) == S_OK)
		{
		adtString	strKey(v);

		// Send active state to sub-location (do not go 'up' to parent)
		if (	hr == S_OK										&&
				(	strKey[0] != '_' ||
					!WCASECMP(strKey,STR_NSPC_CONN) )	&&
				load( strKey, v ) == S_OK					&&
				(IUnknown *)(NULL) != (unkV=v)			&&
				_QI(unkV,IID_ILocation,&pL) == S_OK		&&
				_QI(unkV,IID_IDictionary,&pLd) == S_OK )
			hr = pLd->store ( strnRefAct, bAct );

		// Clean up
		_RELEASE(pL);
		_RELEASE(pLd);
		pIt->next();
		}	// while

	// Clean up
	_RELEASE(pIt);

	return hr;
	}	// activeNotify

HRESULT Location :: clear ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IContainer
	//
	//	PURPOSE
	//		-	Resets the container.
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;
	IList		*pKeys	= NULL;
	IIt		*pIt		= NULL;
	adtValue	vN;

	// To avoid restarting iteration every time there is a 'remove', 
	// pre-generate key list before removal.
	CCLTRY ( COCREATE ( L"Adt.List", IID_IList, &pKeys ) );
	CCLTRY ( keys ( &pIt ) );
	while (hr == S_OK && pIt->read ( vN ) == S_OK)
		{
		// Add key to list
		CCLTRY ( pKeys->write ( vN ) );

		// Next entry
		pIt->next();
		}	// while	

	// Clean up
	_RELEASE(pIt);

	// Remove sub-locations before everything else
	CCLTRY ( pKeys->iterate ( &pIt ) );
	while (hr == S_OK && pIt->read ( vN ) == S_OK)
		{
		ILocation	*pSub	= NULL;
		adtString	strKey (vN);
		adtIUnknown	unkV;

		// Debug
//		dbgprintf ( L"Location::clear:%s:%s\r\n", (LPCWSTR)strName, (LPCWSTR)strKey );

		// Remove locations
		if (	strKey[0] != '_'									&&
				load ( vN, vN ) == S_OK							&&
				(IUnknown *)(NULL) != (unkV=vN)				&&
				_QI (unkV,IID_ILocation,&pSub) == S_OK )
			remove ( strKey );

		// Clean up
		adtValue::clear(vN);
		adtValue::clear(unkV);
		_RELEASE (pSub);
		pIt->next();
		}	// while

	// Clean up
	_RELEASE(pIt);
	_RELEASE(pKeys);

	// Ensure all items are removed
	pDctc->clear();

	// Flush existing value
	bLocThis = false;
	adtValue::clear(vLocThis);

	return hr;
	}	// clear

HRESULT Location :: connect ( IReceptor *pR, bool bC, bool bM )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IReceptor
	//
	//	PURPOSE
	//		-	Connect a receptor to this location.
	//
	//	PARAMETERS
	//		-	pr is the receptor
	//		-	bC is true to connect, false to disconnect
	//		-	bM is true to mirror locations, false for single direction
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	ILocation	*prL	= NULL;

	//
	// Connect
	//
	if (bC)
		{
		// Add receptor to list
		CCLTRY ( pConns->add ( pR, true ) );

		// When a new connection is made, receive the current state of
		// values in the branch into the target.
		CCLTRY ( reflect ( L"", pR ) );

		// Notify other side of connection and mirror mode
		if (hr == S_OK && _QI(pR, IID_ILocation, &prL) == S_OK)
			hr = prL->connected(pRcpOut, true, bM);
		_RELEASE(prL);
		}	// if

	//
	// Disconnect
	//
	else
		{
		// Find specified receptor
		if (pR != NULL)
			{
			// Ensure other end is disconnected
			if (hr == S_OK && _QI(pR,IID_ILocation,&prL) == S_OK)
				prL->connected ( pRcpOut, false, false );
			_RELEASE(prL);

			// Disconnect from here
			pConns->remove ( pR );
			}	// if

		// Special case, disconnect all receptors
		else if (pR == NULL)
			{
			CNNE	*c;

			// Remove all connections
			while ( (c = pConns->head()) != NULL )
				{
				IReceptor *r = (IReceptor *)(c->pConn);

				// Notify and remove
				if (hr == S_OK && _QI(r,IID_ILocation,&prL) == S_OK)
					prL->connected ( pRcpOut, false, false );
				_RELEASE(prL);
				pConns->remove ( r );
				}	// while
			}	// else if

		}	// else

	return hr;
	}	// connect

HRESULT Location :: connected ( IReceptor *pR, bool bC, bool bM )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IReceptor
	//
	//	PURPOSE
	//		-	Notifies when this location is now connected to another.
	//
	//	PARAMETERS
	//		-	pr is the receptor
	//		-	bC is true for connected, false for disconnected
	//		-	bM is true to mirror location, false for single direction
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Connected/disconnected
	if (bC)
		pConns->add ( pR, bM );
	else
		pConns->remove ( pR );

	return hr;
	}	// connected

HRESULT Location :: construct ( void )
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
	HRESULT		hr		= S_OK;

	// Create a contained dictionary for storing items
//	if (nspcglb.pcfDct != NULL)
//		hr = nspcglb.pcfDct->CreateInstance ( NULL, IID_IDictionary, (void **) &pDctc );
//	else
		hr = COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDctc );

	// Create run-time receptor object
	CCLTRYE ( (pConns = new ConnList()) != NULL, E_OUTOFMEMORY );
	CCLOK   ( pConns->AddRef(); )
	CCLTRY  ( pConns->construct() );

	// Obtain outer dictionary interface in case of aggregation
	CCLTRY ( _QI(this,IID_IDictionary,&pDctOut) );
	CCLOK  ( pDctOut->Release(); )
	CCLTRY ( _QI(this,IID_ILocation,&pLocOut) );
	CCLOK  ( pLocOut->Release(); )
	CCLTRY ( _QI(this,IID_IReceptor,&pRcpOut) );
	CCLOK  ( pRcpOut->Release(); )

	return hr;
	}	// construct

HRESULT Location :: create ( const WCHAR *pwKey, ILocation **ppLoc )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ILocation
	//
	//	PURPOSE
	//		-	Called to create a new location of the same type.
	//
	//	PARAMETERS
	//		-	pwKey is the key name that will be used for the location.
	//		-	ppLoc will receive the location
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	IDictionary	*pDctLoc	= NULL;

	// Need to create a location object ? (allows for outer object to create location)
	if (hr == S_OK && *ppLoc == NULL)
		{
		Location		*pLoc		= NULL;

		// Create new location object
		CCLTRYE( (pLoc = new Location()) != NULL, E_OUTOFMEMORY );
		CCLOK  ( pLoc->AddRef(); )
		CCLTRY ( pLoc->construct() );

		// Result
		CCLTRY ( _QI(pLoc,IID_ILocation,ppLoc) );

		// Clean up
		_RELEASE(pLoc);
		}	// if

	// Set state
	CCLTRY ( _QISAFE(*ppLoc,IID_IDictionary,&pDctLoc) );
	CCLTRY ( pDctLoc->store ( strnRefNspc,	adtIUnknownRef(pSpc) ) );
	CCLTRY ( pDctLoc->store ( strnRefPar,	adtIUnknownRef((ILocation *)this) ) );
	CCLTRY ( pDctLoc->store ( strnRefName, adtString(pwKey) ));

	// Clean up
	_RELEASE(pDctLoc);

	return hr;
	}	// create

HRESULT Location :: desc ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Process the descriptor for the location.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	IDictionary	*pDsc	= NULL;
	adtValue		v;
	adtIUnknown	unkV;
	adtString	strType,strDscName;

	// Does location have a descriptor ?
	if (pDctOut->load ( strnRefDesc, v ) != S_OK)
		return S_OK;

	// Descriptor information
	CCLTRY ( _QISAFE((unkV=v),IID_IDictionary,&pDsc) );
	CCLTRY ( pDsc->load ( strnRefType, v ) );
	CCLTRYE( (strType = v).length() > 0, E_UNEXPECTED );

	// Debug
	CCLTRY ( pDctOut->load ( strnRefName, v ) );
	CCLTRYE( (strDscName = v).length() > 0, E_UNEXPECTED );

	//
	// Behaviour
	//
	if (hr == S_OK && !WCASECMP(strType,L"Behaviour"))
		{
		adtString	strB;

		// Attach
		if (bActive)
			{
			// Obtain the behaviour
			CCLTRY ( pDsc->load ( strnRefBehave, v ) );
			CCLTRYE( (strB = v).length() > 0, E_UNEXPECTED );
//			dbgprintf ( L"Location::%s:%s:%s\r\n", (LPCWSTR) strDscName, (LPCWSTR) strType, 
//							(LPCWSTR) strB );

			// Create the behaviour for the node
			// Object class Id
			CCLTRY ( cclCreateObject ( strB, NULL, IID_IBehaviour, (void **) &pBehave ) );

			// Attach using the wrapper as the outer receptor
			CCLTRY ( pBehave->attach ( this, true ) );

			// nSpace value behaviour ?
			CCLOK ( bBehaveV = !WCASECMP(strB,L"nSpc.Value"); )

			// Clean up
			if (hr != S_OK)
				{
				lprintf ( LOG_WARN, L"Unable to attach behaviour : %s\r\n", (LPCWSTR) strB );
				strB = L"";
				_RELEASE(pBehave);
				}	// if
			}	// if

		// Detach
		else if (pBehave != NULL)
			{
			// Obtain the behaviour (DEBUG)
			CCLTRY ( pDsc->load ( strnRefBehave, v ) );
			CCLTRYE( (strB = v).length() > 0, E_UNEXPECTED );
//			CCLOK  ( dbgprintf ( L"Location::desc:detach:%s:%s:%s\r\n", (LPCWSTR) strDscName, (LPCWSTR) strType, (LPCWSTR) strB ); )

			// Detach from location
			pBehave->attach ( this, false );

			// Clean up
			strB = L"";
			_RELEASE(pBehave);
			}	// else

		}	// if

	//
	// Location
	//
	else if (hr == S_OK && !WCASECMP(strType,L"Location"))
		{
//		adtString	strSubName;

		// Name of location
//		CCLTRY ( pDsc->load ( strnRefName, v ) );
//		CCLTRYE( (strSubName = v).length() > 0, E_UNEXPECTED );

		// Active
		if (bActive)
			{
			ILocation	*pLocRef	= NULL;
			adtString	strPath,strRef(LOC_NSPC_REF);

			// Obtain the reference location
			CCLTRY ( pDsc->load ( strnRefLocn, v ) );
			CCLTRYE( (strPath = v).length() > 0, E_UNEXPECTED );

//			CCLOK  ( dbgprintf ( L"Location::%s:%s:%s:%s\r\n", 
//						(LPCWSTR) strDscName, (LPCWSTR) strType, (LPCWSTR) strName, (LPCWSTR) strPath); )

			// Debug
//			if (hr == S_OK && !WCASECMP(strPath,L"render/widget/dict/"))
//				dbgprintf ( L"Hi\r\n" );

			//
			// Place an instance of the specified reference location at this location.
			//

			// Obtain the reference information
			CCLTRY ( strRef.append ( strPath ) );
			CCLTRY ( pSpc->get ( strRef, v, NULL ) );
			
			// Create a non-mirrored link between source and this location
			CCLTRY ( _QISAFE((unkV=v),IID_ILocation,&pLocRef) );
			CCLTRY ( pLocRef->connect ( pRcpOut, true, false ) );

			// Move location into active state
			CCLTRY ( activeNotify ( true ) );

			// Receive path to reference into location so listeners are notified
			// of activation, leave off the 'ref/' prefix
			CCLTRY ( receive ( pRcpOut, strnRefLocn, strPath ) );
//			CCLTRY ( pDctOut->store ( strnRefLocn, strRef ) );

			// Debug
			if (hr != S_OK)
				lprintf ( LOG_WARN, L"Unable to access reference path : %s\r\n", (LPCWSTR) strPath );

			// Clean up
			_RELEASE(pLocRef);
			}	// if

		// In active
		else
			{
			// No need to notify sublocation of active change since it will
			// be handled by this location.
			}	// else

		}	// if

	// Clean up
	_RELEASE(pDsc);

	return hr;
	}	// desc

void Location :: destruct ( void )
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
//	if (strName.length() > 0)
//		dbgprintf(L"Location::destruct:%s:%d {\r\n", (LPCWSTR)strName,bActive);
//	if (!WCASECMP ( strName, L"Leap" ))
//		dbgprintf(L"Hi\r\n" );

	// Protect from double deletion and deativate location
	AddRef();

	// If parent is still valid, remove to shutdown location
	if (pPar != NULL)
		store ( strnRefPar, adtIUnknown(NULL) );

	// Clear items from location
	clear();

	// Clean up
	_RELEASE(pRcpConn);
	_RELEASE(pBehave);
	_RELEASE(pRxQ);
	_RELEASE(pRxIt);
	_RELEASE(pConns);
	_RELEASE(pDctc);

//	if (strName.length ( ) > 0)
//		dbgprintf ( L"} Location::destruct:%s:%d\r\n", (LPCWSTR)strName, bActive );
	}	// destruct

HRESULT Location :: load ( const ADTVALUE &vKey, ADTVALUE &vValue )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITemporalLoc
	//
	//	PURPOSE
	//		-	Loads a value from the TemporalLoc with the given key.
	//
	//	PARAMETERS
	//		-	vKey is the key
	//		-	vValue will receive the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;

	// Common 'value' is cached
	if (	adtValue::type(vKey) == VTYPE_STR && 
			!WCASECMP(vKey.pstr,L"Value") &&
			!adtValue::empty(vLocThis) )
		hr = adtValue::copy ( vLocThis, vValue );
	else
		hr = pDctc->load ( vKey, vValue );

	return hr;
	}	// load

HRESULT Location :: receive ( IDictionary *pDct, 
										const WCHAR *wPath, const ADTVALUE &v,
										bool bRec )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Receive a value into a receptor.
	//
	//	PARAMETERS
	//		-	pDct is the root dictionary
	//		-	wPath is the path to the receptor
	//		-	v is the value to receive
	//		-	bRec is true to receive recursively.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	IReceptor	*pRecep	= NULL;
	adtValue		vL,vN;
	adtIUnknown	unkV;

	// Send to sub-locations
	if (hr == S_OK && bRec)
		{
		ILocation	*pL	= NULL;
		IDictionary	*pLd	= NULL;
		IIt			*pIt	= NULL;

		// Propagate active state to sub-locations
		CCLTRY ( pDct->keys ( &pIt ) );
		while (hr == S_OK && pIt->read ( vL ) == S_OK)
			{
			adtString	strKey(vL);

			// Send active state to sub-location (do not go 'up' to parent)
			if (	hr == S_OK									&&
					WCASECMP(strKey,STR_NSPC_PARENT)		&&
					pDct->load ( vL, vN ) == S_OK &&
					(IUnknown *)(NULL) != (unkV=vN)		&&
					_QI(unkV,IID_ILocation,&pL) == S_OK &&
					_QI(unkV,IID_IDictionary,&pLd) == S_OK )
				receive ( pLd, wPath, v, bRec );

			// Clean up
			_RELEASE(pL);
			_RELEASE(pLd);
			pIt->next();
			}	// while

		// Clean up
		_RELEASE(pIt);
		}	// if

	// Load specified receptor
	CCLTRY ( nspcLoadPath ( pDct, wPath, vL ) );
	CCLTRY ( _QISAFE((unkV = vL),IID_IReceptor,&pRecep) );

	// Send signal
	if (hr == S_OK)
		{
//		dbgprintf ( L"Location::receive:%s:%s\r\n", wPath, (LPCWSTR)strName );
		pRecep->receive ( pRcpOut, L"Value", v );
		}	// if

	// Clean up
	_RELEASE(pRecep);

	return hr;
	}	// receive

HRESULT Location :: receive ( IReceptor *prSrc, const WCHAR *pwLoc, 
										const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IReceptor
	//
	//	PURPOSE
	//		-	Called to store a value at a location.
	//
	//	PARAMETERS
	//		-	prSrc is the source of the reception
	//		-	pwLoc is the location
	//		-	v is the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	bool			bVal	= !WCASECMP(pwLoc,L"Value");
	const WCHAR	*pw	= NULL;

	// Sanity check for double referenced values.  This was a bug
	// early on, check it in case something goes wrong.
	#ifdef	_DEBUG
	if	(	v.vtype == (VTYPE_VALUE|VTYPE_BYREF) &&
			v.pval != NULL &&
			v.pval->vtype == (VTYPE_VALUE|VTYPE_BYREF) )
		DebugBreak();
	#endif

	// SPECIAL CASE: Connectors only receive 'value' locations/keys.
	if ((bRcp || bEmt) && !bVal)
		return S_OK;

	// A received value goes 'down' into the levels until a single
	// key/value pair is left.  No slash means a single key.
	if (bVal || (pw = wcschr ( pwLoc, '/')) == NULL)
		{
		// Ok to store just locations
		if (*pwLoc != '\0')
			{
			// Store/remove the value at this level
			if (!adtValue::empty(v))
				hr = pDctOut->store ( (bVal) ? strnRefVal : adtString(pwLoc), v );
			else
				hr = receiveEmpty ( prSrc, pwLoc );
			}	// if

		// Notification
		CCLOK ( pLocOut->stored ( pLocOut, bRcp, prSrc, pwLoc, v ); )
		}	// if

	// Key is still a path, move down to next level
	else
		{
		IReceptor	*pRcp	= NULL;
		adtIUnknown	unkV;
		adtString	strKey(pwLoc);
		adtValue		vL;

		// Isolate key
		strKey.at(pw-pwLoc) = '\0';

		// Load sub-location
		if (hr == S_OK && load ( strKey, vL ) != S_OK)
			{
			ILocation	*pLoc		= NULL;

			// Debug
			if (pBehave != NULL)
				dbgprintf ( L"Location::receiveInt:Behaviour but unknown connector:%s/%s(%s)\r\n",
								(LPCWSTR) strName, (LPCWSTR) strKey, pwLoc );

			// Create a new sub-location in order to continue storage
			CCLTRY ( pLocOut->create ( strKey, &pLoc ) );

			// Loaded value
			CCLTRY ( adtValue::copy ( adtIUnknown((IReceptor *)pLoc), vL ) );

			// Store sublocation at this location
			CCLTRY ( pDctOut->store ( strKey, vL ) );

			// Clean up
			_RELEASE(pLoc);
			}	// if

		// Receive remaining key path into the next level
		if (hr == S_OK)
			{
			CCLTRY ( _QISAFE((unkV=vL),IID_IReceptor,&pRcp) );
			CCLTRY ( pRcp->receive ( prSrc, pw+1, v ) );
			}	// if

		// Clean up
		_RELEASE(pRcp);
		}	// if

	return hr;
	}	// receive

HRESULT Location :: receiveEmpty ( IReceptor *prSrc, const WCHAR *pwKey )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ILocation
	//
	//	PURPOSE
	//		-	Called when an empty value has been received.
	//
	//	PARAMETERS
	//		-	prSrc is the source receptor
	//		-	pwKey is the key being emptied
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr		= S_OK;
	ILocation		*pLoc	= NULL;
	adtValue			vL;
	adtIUnknown		unkV;
	adtString		strKey(pwKey);

//	dbgprintf ( L"Location::receiveEmpty:%s\r\n", (LPCWSTR)strKey );

	// If key is a location, all of its keys must be removed via 'receive'
	// in order for notifications to propogate throughout hierarchy
	if (	hr == S_OK								&&
			load ( strKey, vL ) == S_OK		&&
			(IUnknown *)(NULL) != (unkV=vL)	&&
			_QI(unkV,IID_ILocation,&pLoc) == S_OK )
		{
		IDictionary		*pDct	= NULL;
		IIt				*pIt	= NULL;
		IReceptor		*pRcp	= NULL;

		// In order to disassemble a hierarchy from the bottom, take two
		// passes where the first pass empties locations and the second
		// the remaining keys.
		for (int i = 0;hr == S_OK && i < 2;++i)
			{
			// Search keys for locations
			CCLTRY ( _QI(pLoc,IID_IReceptor,&pRcp) );
			CCLTRY ( _QI(pLoc,IID_IDictionary,&pDct) );
			CCLTRY ( pDct->keys ( &pIt ) );
			while (hr == S_OK && pIt->read ( vL ) == S_OK)
				{
				ILocation	*pLocSub	= NULL;
				bool			bT			= false;
				adtString	strKeySub(vL);

				// Certain internal, local, run-time only keys are not touched
				bT = (	(	strKeySub[0] != '_' ) || 
							(	WCASECMP(strKeySub,STR_NSPC_PARENT)	&&
								WCASECMP(strKeySub,STR_NSPC_NAME)	&&
								WCASECMP(strKeySub,STR_NSPC_ACTIVE) &&
								WCASECMP(strKeySub,STR_NSPC_TYPE) &&
								WCASECMP(strKeySub,STR_NSPC_NSPC) ) );

				// On the first pass, only empty value if it is a location
				if (bT && i == 0)
					bT = (pDct->load ( vL, vL ) == S_OK		&&
							(IUnknown *)(NULL) != (unkV=vL)	&&
							_QI(unkV,IID_ILocation,&pLocSub) == S_OK);
				if (bT)
					{
					// Empty contents
					pRcp->receive ( prSrc, strKeySub, adtValue() );

					// Removal affects container state
					pIt->begin();
					}	// if

				// Next entry
				else
					pIt->next();

				// Clean up
				_RELEASE(pLocSub);
				}	// while

			// Clean up
			_RELEASE(pIt);
			_RELEASE(pDct);
			_RELEASE(pRcp);
			}	// for

		}	// if

	// Remove key from dictionary
	hr = pDctOut->remove ( strKey );

	// Clean up
	_RELEASE(pLoc);

	return hr;
	}	// receiveEmpty

HRESULT Location :: reflect ( const WCHAR *pwLoc, IReceptor *prDst )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ILocation
	//
	//	PURPOSE
	//		-	Reflects the current level to the provided receptor.
	//
	//	PARAMETERS
	//		-	pwLoc is the current location
	//		-	prDst will receive the updates.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	IIt			*pIt	= NULL;
	ILocation	*pLoc	= NULL;
	U32			nA		= 0;
	U32			nR		= 0;
	U32			sz;
	adtValue		vK,vV;
	adtIUnknown	unkV;
	adtString	strLoc(pwLoc),strKeyLoc;

	// SPECIAL OPTIMIZATION: Currently no need to reflect locations with
	// non-nSpace value behaviours or receptors.
	if ((pBehave != NULL && !bBehaveV) || bRcp)
		return S_OK;

	// Empty location ?
	if (pDctOut->size ( &sz ) != S_OK || sz == 0)
		return S_OK;

	// Debug
//	if (hr == S_OK && prDst != NULL)
//		{
//		IDictionary *pTst = NULL;
//		if (_QI(prDst,IID_IDictionary,&pTst) == S_OK)
//			pTst->remove ( adtString(L"Value") );
//		_RELEASE(pTst);
//		}	// if

	// Iterate the key/values for this location
	CCLTRY ( pDctOut->keys ( &pIt ) );
	while (hr == S_OK && pIt->read ( vK ) == S_OK)
		{
		adtString	strKey(vK);
		bool			bR;

		// Debug
//		if (!WCASECMP(strKey,L"TreeVis"))
//			dbgprintf ( L"Hi\r\n" );

		// Certain internal, local, run-time only keys are not reflected
		bR = (	(	strKey[0] != '_' ) || 
					(	WCASECMP(strKey,STR_NSPC_PARENT)	&&
						WCASECMP(strKey,STR_NSPC_NAME)	&&
						WCASECMP(strKey,STR_NSPC_ACTIVE) &&
						WCASECMP(strKey,STR_NSPC_TYPE) &&
						WCASECMP(strKey,STR_NSPC_NSPC) ) );
		if (hr == S_OK && bR)
			{
			// Generate current path
			CCLTRY ( adtValue::copy ( strLoc, strKeyLoc ) );
			CCLTRY ( strKeyLoc.append ( strKey ) );

			// Access value for key
			CCLTRY ( pDctOut->load ( vK, vV ) );

			// Sub-location ? 
			if (	hr == S_OK &&
					(IUnknown *)(NULL) != (unkV=vV) &&
					_QI(unkV,IID_ILocation,&pLoc) == S_OK )
				{
				IDictionary	*pDctSub	= NULL;

				// A location with a behaviour always reflects its locations
				// even if they are empty.  For non-behaviours, do not 
				// reflect empty locations to handle cases such as a deleted temporal location.
				CCLTRY ( _QI(pLoc,IID_IDictionary,&pDctSub) );
				if (hr == S_OK && ( (pBehave != NULL) || (pDctSub->isEmpty() != S_OK) ) )
					{
					// Sub-location path
					CCLTRY ( strKeyLoc.append ( L"/" ) );

					// Continue reflection in sub-location
					CCLTRY ( pLoc->reflect ( strKeyLoc, prDst ) );
					}	// if

				// Clean up
				_RELEASE(pDctSub);
				_RELEASE(pLoc);
				}	// if

			// Value, reflect
			else if (hr == S_OK && !adtValue::empty(vV))
				prDst->receive ( pRcpOut, strKeyLoc, vV );

			// Key reflected
			CCLOK ( ++nR; )
			}	// if

		// Clean up
		++nA;
		pIt->next();
		}	// while

	// Ensure that 'this' location exists.  This handles the 
	// case where the location is necessary (e.g. connection) 
	// but has no contents yet.
	if (hr == S_OK && nR == 0 && nA != 0 && *pwLoc != '\0')
		prDst->receive ( pRcpOut, pwLoc, adtValue() );

	// Clean up
	_RELEASE(pIt);

	return hr;
	}	// reflect

HRESULT Location :: remove ( const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IContainer
	//
	//	PURPOSE
	//		-	Removes an item from the container identified by the specified
	//			value.
	//
	//	PARAMETERS
	//		-	v identifies the key to remove
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	adtValue	vL;

	// Access item
	if (load ( v, vL ) == S_OK)
		{
		IDictionary	*pDct	= NULL;
		ILocation	*pLoc	= NULL;
		adtIUnknown	unkV (vL);

		// Parent now invalid.
		if (	(IUnknown *)(NULL) != unkV					&&
				adtValue::type(v) == VTYPE_STR			&&
				v.pstr[0] != '_'								&&
				_QI(unkV,IID_ILocation,&pLoc) == S_OK	&&
				_QI(unkV,IID_IDictionary,&pDct) == S_OK )
			pDct->store(strnRefPar, adtIUnknown(NULL));

		// Clean up
		_RELEASE(pDct);
		_RELEASE(pLoc);
		}	// if

	// If key is 'Value', remove reference
	if (v.vtype == VTYPE_STR && v.pstr != NULL && !WCASECMP(v.pstr,L"Value"))
		{
		// Remove reference
		pDctc->remove ( strnRefVal );

		// Value is removed
		bLocThis = false;
		}	// if

	// Remove item from internal dictionary
	return pDctc->remove ( v );
	}	// remove

HRESULT Location :: shutdown ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Shut down this location for operation.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr		= S_OK;

	// Move to inactive state
	if (bActive)
		{
		bActive = false;
		active ();
		}	// if

	// Clear items from location
	clear();

	return hr;
	}	// shutdown

HRESULT Location :: store ( const ADTVALUE &vKey, const ADTVALUE &vValue )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IDictionary
	//
	//	PURPOSE
	//		-	Stores a value in the dictionary with the given key.
	//
	//	PARAMETERS
	//		-	vKey is the key
	//		-	vValue is the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	adtString	strKey(vKey);

	// Default behaviour
	if (strKey[0] != '_')
		{
		// SPECIAL CASE: 'Value's are updated frequently so copy updated
		// value to internally referenced value.
		if (!WCASECMP(strKey,L"Value"))
			{
			// Cache copy of value
			CCLTRY ( adtValue::copy ( vValue, vLocThis ) );

			// Store if necessary
			if (!bLocThis)
				{
				// Store reference to local copy
				CCLTRY ( pDctc->store ( strnRefVal, adtValue(&vLocThis) ) );

				// Valid is stored
				CCLOK ( bLocThis = true; )
				}	// if
			}	// if

		// Store value
		else 
			hr = pDctc->store ( strKey, vValue );
		}	// if

	// Internal/reserved keys
	else
		{
		// Default is to store for self
		bool	bStInt	= true;

		// Parent
		if (!WCASECMP(strKey,STR_NSPC_PARENT))
			{
			adtIUnknown	unkV(vValue);

			// If parent is being removed, shutdown location with current parent first.
			if (unkV.punk == NULL && pPar != NULL)
				{
				// De-activate if necessary
				if (bActive)
					{
					bActive = false;
					active();
					}	// if

				// Propogate false activation to children regardless of state of parent
				// to ensure entire branch is shutdown
				else
					activeNotify ( false );

				// Ensure all connections are disconnected since no parent leaves
				// location is disconnected state.
				connect ( NULL, false, false );
				}	// if

			// Parent location
			pPar		= NULL;
			pDctPar	= NULL;
			if ((IUnknown *)(NULL) != unkV && _QI(unkV,IID_ILocation,&pPar) == S_OK)
				pPar->Release();
			if ((IUnknown *)(NULL) != unkV && _QI(unkV,IID_IDictionary,&pDctPar) == S_OK)
				pDctPar->Release();
			}	// if

		// Namespace
		else if (!WCASECMP(strKey,STR_NSPC_NSPC))
			{
			adtIUnknown	unkV(vValue);

			// Namespace
			pSpc = NULL;
			if ((IUnknown *)(NULL) != unkV && _QI(unkV,IID_INamespace,&pSpc) == S_OK)
				pSpc->Release();
			}	// else if

		// Type
		else if (!WCASECMP(strKey,STR_NSPC_TYPE))
			{
			adtString	strV(vValue);

			// Connector ?
			bRcp = (!WCASECMP(L"Receptor",strV));
			bEmt = (!WCASECMP(L"Emitter",strV));
			}	// else if

		// Active 
		else if (!WCASECMP(strKey,STR_NSPC_ACTIVE))
			{
			adtBool	bV(vValue);

			// Active state has changed
			if (bActive != bV)
				{
				// Process new state
				bActive = bV;
//				dbgprintf ( L"Location::store:Active %d\r\n", bActive );
				active();
				}	// if

			// Propogate false activation to children regardless of state of parent
			// to ensure entire branch is shutdown
			else if (bV == false)
				activeNotify ( bV );
			}	// else if

		// Name
		else if (!WCASECMP(strKey,STR_NSPC_NAME))
			strName = vValue;
//{
//			hr = adtValue::toString(vValue,strName);
//strName.at();
//}

		// Descriptor
		else if (!WCASECMP(strKey,STR_NSPC_DESC))
			{
			ICloneable	*pClone	= NULL;
			IUnknown		*pDesc	= NULL;
			adtIUnknown	unkV(vValue);

			// A clone of the descriptor must be used to ensure each location has
			// its own default attributes.
			CCLTRY ( _QISAFE(unkV,IID_ICloneable,&pClone) );
			CCLTRY ( pClone->clone ( &pDesc ) );

			// Store clone in dictionary instead of original
			CCLOK  ( bStInt = false; )
			CCLTRY ( pDctc->store ( vKey, adtIUnknown(pDesc) ) );

			// Descriptor change while active ?
			if (bActive)
				{
//				dbgprintf ( L"Location::store:Descriptor(Active %d)\r\n", bActive );
				}	// if

			// Clean up
			_RELEASE(pDesc);
			_RELEASE(pClone);
			}	// else if

		// Connections
		else if (!WCASECMP(strKey,STR_NSPC_CONN))
			{
			ILocation		*pLocC	= NULL;
			adtIUnknown		unkV(vValue);

			// In order to manage connections, create and attach a connect receptor object.

			// Valid ?
			if ((IUnknown *)(NULL) != unkV && _QI(unkV,IID_ILocation,&pLocC) == S_OK)
				{
				// Create connect object
				if (pRcpConn == NULL)
					{
					Connect	*pObj	= NULL;

					// Create/initialize object
					CCLTRYE ( (pObj = new Connect()) != NULL, E_OUTOFMEMORY );
					CCLOK   ( pObj->AddRef(); )
					CCLTRY  ( pObj->construct() );

					// Interface
					CCLTRY	( _QI(pObj,IID_IReceptor,&pRcpConn) );

					// Clean up
					_RELEASE(pObj);
					}	// if

				// Connect the connection manager to location
				CCLTRY ( pLocC->connect ( pRcpConn, true, false ) );

				// Clean up
				_RELEASE(pLocC);
				}	// if

			}	// else if

		// Store in own dictionary
		if (hr == S_OK && bStInt)
			hr = pDctc->store ( vKey, vValue );
		}	// else

	return hr;
	}	// store

HRESULT Location :: stored ( ILocation *pAt, bool bAtRcp, IReceptor *prSrc,
										const WCHAR *pwKey, const ADTVALUE &vValue )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ILocation
	//
	//	PURPOSE
	//		-	Called when a value is stored at a location.
	//
	//	PARAMETERS
	//		-	pAt is the location at which the value was stored 
	//		-	bAtRcp is true if original location is a receptor.
	//		-	prSrc is the original source receptor of the store.
	//		-	pwKey is the key path at source location
	//		-	v is the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Debug
//	if (hr == S_OK)
//		{
//		IDictionary	*pAtDct	= NULL;
//		if (_QI(pAt,IID_IDictionary,&pAtDct) == S_OK)
//			{
//			adtString strLoc;
//			CCLTRY ( nspcPathTo ( pAtDct, pwKey, strLoc, this ) );
//			dbgprintf ( L"Location::stored:%s\r\n", (LPCWSTR)strLoc );
//			_RELEASE(pAtDct);
//			if (!WCASECMP(strLoc,L"apps/auto/default/GDI/This/Cache/TagGet/Fire/Value"))
//				dbgprintf ( L"Hi\r\n" );
//			}	// if
//		}	// if

	// Does this location have any listeners ?
	if (pConns->head() != NULL)
		{
		IDictionary	*pAtDct	= NULL;
		CNNE			*n,*e		= NULL;
		adtString	*pstrLoc	= NULL;
//		adtString	strLoc(pwKey);
		adtValue		vRx;

		// Optimization, no need to generate path if at same level
		if (pAt != pLocOut)
			{
			// Need own buffer for full path
			CCLTRYE ( (pstrLoc = new adtString()) != NULL, E_OUTOFMEMORY );

			// Generate the path from the source level to this level.
			CCLTRY ( _QISAFE(pAt,IID_IDictionary,&pAtDct) );
//			CCLTRY ( nspcPathTo ( pAtDct, pwKey, strLoc, this ) );
			CCLTRY ( nspcPathTo ( pAtDct, pwKey, *pstrLoc, pDctOut ) );
			}	// if

		// Debug
//		if (pDctPar != NULL && pDctPar->load ( strnRefName, vL ) == S_OK)
//			{
//			adtString	strPar(vL);
//			dbgprintf ( L"Location::stored:Parent %s:Name %s:Loc %s:Head %p\r\n", 
//								(LPCWSTR)strPar,
//								(LPCWSTR)strName, (LPCWSTR)strLoc, pConns->head() );
//			if (!WCASECMP(strPar,L"Uninitialize"))
//				dbgprintf ( L"Hi\r\n" );
//			}	// if

		// Debug
//		if (wcsstr ( strLoc, L"TagGet/" ) != NULL)
//			dbgprintf ( L"Location::stored:%s:%s:%s\r\n", (LPCWSTR)strName, pwKey, (LPCWSTR) strLoc );

		// Receive store into connected receptors.  Ignore original source
		// of store to avoid loops.
		for (e = pConns->head(),n = NULL;e != NULL;e = n)
			{
			// Can avoid QI for speed
			IReceptor	*r = (IReceptor *)(e->pConn);

			// Remember next entry
			n = pConns->next(e);

			// Own reference in case receptor is destroyed/disconnected during reception
			r->AddRef();

			// Reception if not source and enabled
			if (r != prSrc && e->bRx)
				r->receive ( pRcpOut, ((pstrLoc != NULL) ? ((const WCHAR *)*pstrLoc) : pwKey), vValue );
//				r->receive ( this, strLoc, vValue );
//				connect ( r, false );

			// Clean up
			r->Release();
			}	// for

		// Clean up
		_RELEASE(pAtDct);
		if (pstrLoc != NULL)
			delete pstrLoc;
		}	// if

	// Notify up the chain.
	if (pPar != NULL)
		{
		// SPECIAL OPTIMIZATION: 
		// Currently no need to propagate values from receptors of behaviours or
		// from non-nSpace value behaviours.
		if (pBehave == NULL || (bBehaveV && !bAtRcp))
			pPar->stored ( pAt, bAtRcp, prSrc, pwKey, vValue );
		}	// if

	return hr;
	}	// stored

//
// Default functionality
//

HRESULT Location :: copyTo ( IContainer *pCont )
	{
	return pDctc->copyTo(pCont);
	}	// copyTo

HRESULT Location :: isEmpty ( void )
	{
	return pDctc->isEmpty();
	}	// isEmpty

HRESULT Location :: iterate ( IIt **ppIt )
	{
	return pDctc->iterate ( ppIt );
	}	// keys

HRESULT Location :: keys ( IIt **ppIt )
	{
	return pDctc->keys ( ppIt );
	}	// keys

HRESULT Location :: size ( U32 *pusz )
	{
	return pDctc->size ( pusz );
	}	// size

