////////////////////////////////////////////////////////////////////////
//
//									SpcX.CPP
//
//			Implementation of the external nSpace namespace object
//
////////////////////////////////////////////////////////////////////////

#include "nshl_.h"

NamespaceX :: NamespaceX ( ShellX *_pShellX, IDictionary *_pDctCmd )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	//	PARAMETERS
	//		-	_pShellX is the owner object
	//		-	_pDctCmd are the command line options
	//
	////////////////////////////////////////////////////////////////////////
	pShellX		= _pShellX;
	pThrd			= NULL;
	pDctCmd		= _pDctCmd;
	pListens		= NULL;
	pAutoUn		= NULL;
	_ADDREF(pDctCmd);
	}	// NamespaceX

HRESULT NamespaceX :: construct ( void )
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
	NamespaceX_t	*pSpct	= NULL;

	// Create dictionary to keep track of listened paths
	CCLTRY(COCREATE(L"Adt.Dictionary",IID_IDictionary,&pListens));

	// Auto un-listen list during errors on delivery
	CCLTRY(COCREATE(L"Adt.Queue",IID_IList,&pAutoUn));

	// Create thread object
	CCLTRYE ( (pSpct = new NamespaceX_t (this)) != NULL, E_OUTOFMEMORY );
	CCLOK   ( pSpct->AddRef(); )
	CCLTRY  ( pSpct->construct() );

	// Start a thread to host a shell object for the namespace.
	CCLTRY ( COCREATE ( L"Sys.Thread", IID_IThread, &pThrd ) );
	CCLTRY ( pThrd->threadStart ( pSpct, 60000 ) );

	// Clean up
	_RELEASE(pSpct);

	return hr;
	}	// construct

void NamespaceX :: destruct ( void )
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
	IIt	*pIt	= NULL;

	// Protection from recursive deletion
	AddRef();

	// Thread protection
	csListen.enter();

	// From outstanding listens
	if (pListens != NULL && pListens->keys ( &pIt ) == S_OK)
		{
		adtValue		vL;
		adtIUnknown	unkV;

		// Unlisten until empty
		while (pIt->begin() == S_OK && pIt->read ( vL ) == S_OK)
			{
			IDictionary	*pDct		= NULL;
			IIt			*pItL		= NULL;
			BSTR			bstrPath	= NULL;

			// First key is the path 
			adtString	strPath(vL);

			// Obtain the receptor dictionary for the path
			if (	strPath.toBSTR ( &bstrPath ) == S_OK		&&
					pListens->load ( strPath, vL ) == S_OK		&&
					(IUnknown *)(NULL) != (unkV = vL)			&&
					_QI(unkV,IID_IDictionary,&pDct) == S_OK	&&
					pDct->keys ( &pItL ) == S_OK)
				{
				// Unlisten all entries
				while (pItL->read ( vL ) == S_OK)
					{
					// Perform unlisten
					unlisten ( bstrPath, (IListenX *)(vL.punk) );

					// Next entry
					pItL->next();
					}	// while

				// Clean up
				_RELEASE(pItL);
				}	// if

			// Clean up
			_RELEASE(pDct);
			_FREEBSTR(bstrPath);

			// Ensure removed from dictionary
			pListens->remove ( strPath );
			}	// while

		// Clean up
		_RELEASE(pIt);
		}	// if
	_RELEASE(pListens);
	_RELEASE(pAutoUn);

	// Thread protection
	csListen.leave();

	// This object is released when the external entity releases it so
	// shutdown the shell when that happens.
	if (pThrd != NULL)
		{
//		pThrd->threadStop(10000);
//		pThrd->threadJoin(10000);
		pThrd->threadStop(30000);
//		pThrd->threadJoin(30000);
		_RELEASE(pThrd);
		}	// if
	_RELEASE(pShell);

	// Notify parent
	if (pShellX != NULL)
		{
		pShellX->down ( this );
		pShellX = NULL;
		}	// if

	// Clean up
	_RELEASE(pDctCmd);
	}	// destruct

HRESULT NamespaceX :: listen ( BSTR bstrPath, IListenX *pR )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		INamespaceX
	//
	//	PURPOSE
	//		-	Listen to a location in the namespace.
	//
	//	PARAMETERS
	//		-	bstrPath is the namespace path.
	//		-	pR is the callback interface that receives persisted values.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr			= S_OK;
	IDictionary		*pPath	= NULL;
	ILocation		*pLoc		= NULL;
	ReceptorX		*pnR		= NULL;
	adtValue			vL;
	adtString		strPath(bstrPath);
	adtIUnknown		unkV;
	U32				len;

	// Debug
//	dbgprintf ( L"NamespaceX::listen:bstrPath %s:pR %p\r\n", bstrPath, pR );

	// State check
	CCLTRYE ( pShell != NULL, ERROR_INVALID_STATE );

	//
	// Location
	//

	// Allow caller to leave off the slash, e.g. XXX/OnFire = XXX/OnFire/
	if (hr == S_OK && strPath[(len = strPath.length())-1] != '/')
		hr = strPath.append ( L"/" );

	// Load specified path from namespace
	CCLTRY ( pShell->pSpc->get ( strPath, vL, NULL ) );

	// Path must specify a location for listening
	CCLTRY ( _QISAFE((unkV=vL),IID_ILocation,&pLoc) );

	//
	// Multiple listens on same location ok, obtain dictionary for location.
	//

	// Obtain dictionary for path
	if (hr == S_OK)
		{
		// Load
		CCLTRY(pListens->load ( strPath, vL ));
		CCLTRY(_QISAFE((unkV=vL),IID_IDictionary,&pPath));

		// Create dictionary for path if necessay
		if (hr != S_OK)
			{
			// New path dictionary
			hr = COCREATE(L"Adt.Dictionary",IID_IDictionary,&pPath);

			// Store under path
			CCLTRY(pListens->store ( strPath, adtIUnknown(pPath) ));
			}	// if

		// State check
		CCLTRYE ( pPath != NULL, E_UNEXPECTED );
		}	// if

	//
	// Create receptor and listen to location.
	//

	// Is listen already active for listener ?
	CCLTRYE ( pPath->load ( adtIUnknown(pR), vL ) != S_OK, E_UNEXPECTED );

	// Set up a listen on location
	if (hr == S_OK)
		{
		// New listener object
		CCLTRYE ( (pnR = new ReceptorX ( this, strPath, pLoc )) != NULL, E_OUTOFMEMORY );
		CCLTRY  ( pnR->construct() );

		// Thread protection
		csListen.enter();

		// Store listen before actually listening because current values will be
		// reflected immediately.
		CCLTRY ( pPath->store ( adtIUnknown(pR), adtIUnknown((IReceptor *)pnR) ) );

		// Thread protection
		csListen.leave();

		// Ok to listen
		CCLTRY ( pLoc->connect ( pnR, true, false ) );
		}	// if

	// Clean up
	if (hr != S_OK)
		dbgprintf ( L"NamespaceX::listen:bstrPath %s:pLoc %p:hr 0x%x\r\n", bstrPath, pLoc, hr );
	_RELEASE(pnR);
	_RELEASE(pPath);
	_RELEASE(pLoc);

	return hr;
	}	// listen

HRESULT NamespaceX :: load ( BSTR bstrPath, VARIANT *var )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		INamespaceX
	//
	//	PURPOSE
	//		-	Load a value from the namespace.  Value will be persisted
	//			into provided interface.
	//
	//	PARAMETERS
	//		-	strPath is the namespace path.
	//		-	var will receive the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr			= S_OK;
	adtValue			vL;
	adtString		strPath(bstrPath);
	U32				len;

	// State check
	CCLTRYE ( pShell != NULL, ERROR_INVALID_STATE );

	// Path must currently be a 'root' path (current working path ?)
	// Minimum path length : /X/OnFire
	CCLTRYE (	(len = strPath.length()) > 8 && bstrPath[0] == WCHAR('/'), 
					E_INVALIDARG );

	// Since loading only values (not locations) is supported through this interface,
	// allow caller to just specify '/OnFire' at the end of the path.
	// Full path needs to be 'XXX/OnFire/Value/'
	if (hr == S_OK && strPath[len-1] != '/')
		{
		CCLTRY ( strPath.append ( L"/" ) );
		CCLOK  ( ++len; )
		}	// if
	if (hr == S_OK && !WCASECMP(&strPath[len-7],L"OnFire/"))
		hr = strPath.append ( L"Value/" );

	// Load the value from the path
	dbgprintf ( L"NamespaceX::load:strPath %s:hr 0x%x\r\n", (LPCWSTR)strPath, hr );
	CCLTRY ( pShell->pSpc->get ( strPath, vL, NULL ) );
//hr = S_FALSE;

	// Convert to variant and copy
	CCLOK ( varL = vL; )
	CCLTRY( VariantCopy ( var, &varL ) );

	// Clean up
	if (hr != S_OK)
		dbgprintf ( L"NamespaceX::load:bstrPath %s:hr 0x%x\r\n", bstrPath, hr );

	return hr;
	}	// load

HRESULT NamespaceX :: received ( const WCHAR *pwRoot, const WCHAR *pwLoc,
											VARIANT *pv )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called from a receptor object when a value is received.
	//
	//	PARAMETERS
	//		-	pwRoot is the root of the location
	//		-	pwLoc is the relative location of the value
	//		-	pv is the received value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr			= S_OK;
	IDictionary		*pPath	= NULL;
	IIt				*pKeys	= NULL;
	IListenX			*pL		= NULL;
	BSTR				bstrRoot	= NULL;
	BSTR				bstrLoc	= NULL;
	U32				iUn		= 0;
	adtValue			vL;
	adtIUnknown		unkV;

	// Debug
//	dbgprintf ( L"NamespaceX::received!:Path %s:%s:%d\r\n", pwRoot, pwLoc, v.vtype );
//	if (v.vtype == VTYPE_R8)
//		dbgprintf ( L"%g\r\n", v.vdbl );

	// Thread protection
	csListen.enter();

	// Load the dictionary for the path
	CCLTRY ( pListens->load (adtString(pwRoot),vL) );
	CCLTRY ( _QISAFE((unkV=vL),IID_IDictionary,&pPath ) );

	// Convert to variant
//	CCLOK  ( varX = v; )

	// Generate BSTR paths
	CCLTRYE ( (bstrRoot	= SysAllocString ( pwRoot )) != NULL, E_OUTOFMEMORY );
	CCLTRYE ( (bstrLoc	= SysAllocString ( pwLoc )) != NULL, E_OUTOFMEMORY );

	// Iterate all listeners to be notified
	CCLTRY ( pPath->keys ( &pKeys ) );
	while (hr == S_OK && pKeys->read ( vL ) == S_OK)
		{
		// Current listener
		CCLTRY ( _QISAFE((unkV=vL),IID_IListenX,&pL));

		// Thread protection
		csListen.leave();

		// Notify
		if (hr == S_OK && (hr = pL->receive ( bstrRoot, bstrLoc, pv )) != S_OK)
			{
			// Most common error is "RPC server is unavailable".  That means the target 
			//	interface/process  is gone (due to crash, etc.)
			// Auto-unlisten to this path on behalf of that now gone client to avoid sending
			// updates in the future.
			if (	hr == 0x800706BA ||					// RPC server unavailble
					hr == 0x800706BE ||					// The remote procedure call failed
					true )									// Other types of errors ? (for now any error means unlisten)
				{
				// Debug
				lprintf ( LOG_DBG, L"Error delivering value, unlistening to : 0x%x : %s\r\n",
												hr, pwRoot );

				// Store for auto unlisten later.  Cannot unlisten here since it would
				// alter the state of the container that is currently being iterated.
				pAutoUn->write ( adtString(bstrRoot) );
				pAutoUn->write ( adtIUnknown(pL) );
				++iUn;
				}	// if

			// Do not error out delivery to other listeners
			hr = S_OK;
			}	// if

		// Thread protection
		csListen.enter();

		// Clean up
		_RELEASE(pL);
		pKeys->next();
		}	// while

	// Thread protection
	csListen.leave();

	// Any auto-unlistens ?
	if (hr == S_OK && iUn > 0)
		{
		IIt		*pIt = NULL;
		adtValue	vP,vUn;

		// Auto-unlisten listeners
		CCLTRY ( pAutoUn->iterate ( &pIt ) );
		while (hr == S_OK && pIt->read ( vP ) == S_OK)
			{
			// Listener
			CCLOK ( pIt->next(); )
			CCLTRY( pIt->read ( vUn ));
			CCLOK ( pIt->next(); )

			// Unlisten
			unlisten ( (BSTR) (LPCWSTR) adtString(vP), (IListenX *)vUn.punk );
			}	// while

		// Clean up
		_RELEASE(pIt);
		}	// if

	// Clean up
	if (bstrLoc != NULL)
		SysFreeString( bstrLoc );
	if (bstrRoot != NULL)
		SysFreeString( bstrRoot );
	_RELEASE(pKeys);
	_RELEASE(pPath);

	// Debug
	if (hr != S_OK)
		lprintf ( LOG_DBG, L"Error sending out received value : 0x%x : %s : %s\r\n",
									hr, pwRoot, pwLoc );

	return hr;
	}	// received

HRESULT NamespaceX :: store ( BSTR bstrPath, VARIANT *var )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		INamespaceX
	//
	//	PURPOSE
	//		-	Store a value into the namespace.  Value is assumed to
	//			be in persisted buffer.
	//
	//	PARAMETERS
	//		-	bstrPath is the namespace path.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	IReceptor	*pRecep	= NULL;
	adtValue		vR,vSt;
	adtIUnknown	unkV;
//	adtString	strVar;
	adtString	strPath;
	U32			len;

	// State check
	CCLTRYE ( pShell != NULL, ERROR_INVALID_STATE );

	// Storing is done from the root location so ignore any leading slashes
	CCLOK ( strPath = (bstrPath != NULL && bstrPath[0] == '/') ? 
								bstrPath+1 : bstrPath; )
	
	// Since storing only values (not locations) is supported through this interface,
	// allow caller to just specify '/Fire' at the end of the path.
	// Full path needs to be 'XXX/Fire/Value'
	if (hr == S_OK && strPath[(len=strPath.length())-1] == '/')
		strPath.at(--len) = '\0';
	if (hr == S_OK && WCASECMP(&strPath[len-6],L"/Value"))
		hr = strPath.append ( L"/Value" );

	// Perform a 'load' first on the path before the store.  This allows any
	// auto-instancing to occur before the store.  Let the store fail, not this.
//	if (hr == S_OK)
//		hr = pShell->pSpc->get ( strPath, vL, NULL );

	// Access root namespace location/receptor
	CCLTRY(pShell->pSpc->get(L"/",vR,NULL));
	CCLTRY ( _QISAFE((unkV=vR),IID_IReceptor,&pRecep) );

	// Convert to local value type
	CCLOK ( varS = var; )
	CCLTRY( varS.toValue(vSt) );

	// Debug
//	adtValue::toString ( vSt, strVar );
//	dbgprintf ( L"NamespaceX::store:%s:%s\r\n", bstrPath, (LPCWSTR) strVar );

	// Receive value into namespace
	CCLTRY ( pRecep->receive ( NULL, strPath, vSt ) );

	// Clean up
	if (hr != S_OK)
		dbgprintf ( L"NamespaceX::store:0x%x:%s\r\n", hr, bstrPath );
	_RELEASE(pRecep);
	adtValue::clear(vR);

	return hr;
	}	// store

HRESULT NamespaceX :: unlisten (	BSTR bstrPath, IListenX *pR )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		INamespaceX
	//
	//	PURPOSE
	//		-	Stop listening to an emitter in the namespace.
	//
	//	PARAMETERS
	//		-	bstrPath is the namespace path.
	//		-	pR is the callback interface that receives persisted values.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	IDictionary	*pPath	= NULL;
	ILocation	*pLoc		= NULL;
	IReceptor	*pRecep	= NULL;
	adtValue		vL,v;
	adtString	strPath(bstrPath);
	adtIUnknown	unkV;

	// State check
	CCLTRYE ( pShell != NULL, ERROR_INVALID_STATE );

	// Allow caller to leave off the slash, e.g. XXX/OnFire = XXX/OnFire/
	if (hr == S_OK && strPath[strPath.length()-1] != '/')
		hr = strPath.append ( L"/" );

	// Load specified path from catalog
	CCLTRY ( pShell->pSpc->get ( bstrPath, vL, NULL ) );

	// Path must specify a location to obtain the value
	CCLTRY ( _QISAFE((unkV=vL),IID_ILocation,&pLoc) );

	// Obtain dictionary for path
	CCLTRY(pListens->load ( strPath, v ));
	CCLTRY(_QISAFE(v.punk,IID_IDictionary,&pPath));

	// Obtain receptor object for listener
	CCLTRY(pPath->load ( adtIUnknown(pR), v ) );
	CCLTRY(_QISAFE(v.punk,IID_IReceptor,&pRecep));

	// Disconnect receptor from location
	CCLOK (pLoc->connect(pRecep,false,false);)

	// Thread protection
	csListen.enter();

	// Remove listener from path
	if (pPath != NULL)
		pPath->remove ( adtIUnknown(pR) );

	// If last listener, remove path
	if (pPath != NULL && pPath->isEmpty() == S_OK)
		pListens->remove ( strPath );

	// Thread protection
	csListen.leave();

	// Clean up
//	dbgprintf ( L"NamespaceX::unlisten:bstrPath %s pLoc %p pRecep %p hr 0x%x\r\n", 
//					bstrPath, pLoc, pRecep, hr );
	_RELEASE(pRecep);
	_RELEASE(pPath);
	_RELEASE(pLoc);

	return hr;
	}	// unlisten

////////////////
// NamespaceX_t
////////////////

NamespaceX_t :: NamespaceX_t ( NamespaceX *_pSpc )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	//	PARAMETERS
	//		-	_pSpc is the parent object
	//
	////////////////////////////////////////////////////////////////////////
	pSpc	= _pSpc;
	}	// NamespaceX_t

HRESULT NamespaceX_t :: tickAbort ( void )
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
	return (pSpc->pShell != NULL) ? pSpc->pShell->tickAbort() : S_OK;
	}	// tickAbort

HRESULT NamespaceX_t :: tick ( void )
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
	HRESULT			hr		= S_OK;

	// Execute tick on shell object
	CCLTRY ( pSpc->pShell->tick() );

	return hr;
	}	// tick

HRESULT NamespaceX_t :: tickBegin ( void )
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
	HRESULT		hr				= S_OK;

	// Create shell object for command line
	CCLTRYE	( (pSpc->pShell = new Shell(pSpc->pDctCmd)) != NULL, E_OUTOFMEMORY );
	CCLTRY	( pSpc->pShell->construct() );
	CCLOK		( pSpc->pShell->AddRef(); )

	// Execute startup on shell
	CCLTRY ( pSpc->pShell->tickBegin() );

	// Clean up
	if (hr != S_OK)
		{
		_RELEASE(pSpc->pShell);
		}	// if

	return hr;
	}	// tickBegin

HRESULT NamespaceX_t :: tickEnd ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITickable
	//
	//	PURPOSE
	//		-	Notifies the object that 'ticking' is to stop.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr = S_OK;

	// Clean up shell
	if (pSpc->pShell != NULL)
		{
		pSpc->pShell->tickEnd();
		_RELEASE(pSpc->pShell);
		}	// if

	return hr;
	}	// tickEnd

