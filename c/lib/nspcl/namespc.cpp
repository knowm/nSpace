////////////////////////////////////////////////////////////////////////
//
//									NAMESPACE.CPP
//
//						Implementation of a namespace.
//
////////////////////////////////////////////////////////////////////////

#include "nspcl_.h"
#include <stdio.h>

// Globals
extern GlobalNspc	nspcglb;

Namespace :: Namespace ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	pTmp		= NULL;
	pDctRt	= NULL;
//	pDctLnk	= NULL;
	pDctStat	= NULL;
	pDctRec	= NULL;
	pDctOpts	= NULL;
	}	// Namespace

HRESULT Namespace :: connection (	IDictionary *pLoc, const WCHAR *wName, 
												const WCHAR *wType,
												IReceptor *pRcp, IReceptor **ppRcp )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		INamespace
	//
	//	PURPOSE
	//		-	Enable or disable connection to the named sublocation for the 
	//			specified location.
	//
	//	PARAMETERS
	//		-	pLoc is the location under which the sublocation is stored.
	//		-	wName is the name of the sub-location
	//		-	wType is the type of the connection (receptor/emitter)
	//		-	pRcp is the receptor that will receive location notifications
	//		-	ppRcp will receive the receptor for the location, or NULL
	//			to disable connection.  BY REFRENCE, no count.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	ILocation	*pLocSub	= NULL;
	adtIUnknown	unkV;
	adtValue		vL;
	adtString	strName(wName);

	//
	// Enable
	//
	if (ppRcp != NULL)
		{
		// Allow location recycling 
		if (	hr == S_OK									&& 
				pLoc->load ( strName, vL ) == S_OK	&&
				(IUnknown *)(NULL) != (unkV = vL) )
			{
			// Location
			hr = _QI(unkV,IID_ILocation,&pLocSub);
			}	// if

		// New connector
		else if (hr == S_OK)
			{
			Location		*obj	= NULL;

			// Create a new location (create directly for speed)
			CCLTRYE( (obj = new Location()) != NULL, E_OUTOFMEMORY );
			CCLOK  ( obj->AddRef(); )
			CCLTRY ( obj->construct() );

			// Initialize sub-location
			CCLTRY ( obj->store ( strnRefPar,		adtIUnknownRef(pLoc) ) );
			CCLTRY ( obj->store ( strnRefNspc,		adtIUnknownRef((INamespace *)this) ) );
			CCLTRY ( obj->store ( strnRefName,		strName ) );
			CCLTRY ( obj->store ( strnRefType,		adtString(wType) ) );

			// Assign location
			pLocSub	= obj;
			_ADDREF(pLocSub);

			// Store under parent
			CCLTRY ( pLoc->store ( strName, adtIUnknown(pLocSub) ) );

			// Clean up
			_RELEASE(obj);
			}	// else if

		// Connect receptor to location, non-mirror mode
		CCLTRY ( pLocSub->connect ( pRcp, true, false ) );

		// Receptor for location
		CCLTRY ( _QI(pLocSub,IID_IReceptor,ppRcp));

		// By reference
		if (hr == S_OK)
			(*ppRcp)->Release();
		}	// if

	//
	// Disable
	//
	else
		{
		// If name is null string then all connections are to be disabled
		if (!strName.length ())
			{
			IDictionary	*pDct		= NULL;
			IIt			*pIt		= NULL;
			IDictionary	*pDctSubA= NULL;
			ILocation	*pLocSubA= NULL;
			adtValue		vK, vV;
			adtIUnknown	unkV;

			// Iterate connections
			CCLTRY (_QISAFE (pLoc,IID_IDictionary,&pDct));
			CCLTRY (pLoc->keys (&pIt));
			while (hr == S_OK && pIt->read ( vK ) == S_OK)
				{
				adtString	strKey ( vK );

				// Location with type specification ?
				if (	hr == S_OK												&&
						strKey[0] != '_'										&&
						pLoc->load ( vK, vV ) == S_OK						&&
						(IUnknown *)(NULL) != (unkV=vV)					&&
						_QI (unkV,IID_ILocation,&pLocSubA) == S_OK	&&
						_QI (unkV,IID_IDictionary,&pDctSubA) == S_OK	&&
						pDctSubA->load ( strnRefType, vV ) == S_OK)
					{
//					dbgprintf (L"Namespace::connection:%s\r\n", (LPCWSTR)strKey );
					connection ( pLoc, strKey, L"", pRcp, NULL );
					}	// if

				// Clean up
				adtValue::clear(vV);
				_RELEASE (pDctSubA);
				_RELEASE (pLocSubA);
				pIt->next ();
				}	// while

			// Clean up
			_RELEASE (pIt);
			_RELEASE (pDct);
			}	// if

		// Disable single connection
		else
			{
			// Load the specified location
			CCLTRY	( pLoc->load ( strName, vL ) );

			// Interfaces
			CCLTRY ( _QISAFE((unkV=vL),IID_ILocation,&pLocSub); )

			// Disconnect receptor
			CCLOK ( pLocSub->connect ( pRcp, false, false ); )
			}	// else

		}	// if

/*
	// If the owner has the persistence flag set, record/unrecord emitter.
	if (	hr		== S_OK										&& 
			pEmt	!= NULL										&&
			pDct->load ( strnRefPers, vL ) == S_OK	&&
			adtBool(vL) == true )
		{
		adtString	strPath;

		// Generate the full path to the connector for adding
		if (hr == S_OK && ppC != NULL)
			{
			// Full path to connector
			CCLTRY ( nspcToAbs ( pDct, L"./", strPath ) );

			// Plus name
			CCLTRY ( strPath.append ( wName ) );
			}	// if

		// Record/unrecord
		CCLTRY ( record ( strPath, pEmt, (ppC != NULL) ) );

		// Don't fail connector creation on persistence error (?)
		hr = S_OK;
		}	// if
*/

	// Clean up
	_RELEASE(pLocSub);

	return hr;
	}	// connection

HRESULT Namespace :: construct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	FROM	CCLObject
	//
	//	PURPOSE
	//		-	Called to construct the object.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr		= S_OK;

	// Aggregated dictionary
	CCLTRY ( COCREATEA ( L"Adt.Dictionary", (INamespace *) this, &punkDct ) );
	CCLTRY(_QI(punkDct,IID_IDictionary,&pDctNs));
	CCLOK (pDctNs->Release();)							// Cancel 'QI'

	// Dictionary to keep track of recordings
	CCLTRY(COCREATE(L"Adt.Dictionary",IID_IDictionary,&pDctRec));

	// Dictionary to obtain status
	CCLTRY(COCREATE(L"Adt.Dictionary",IID_IDictionary,&pDctStat));

	// Dictionary options
	CCLTRY(COCREATE(L"Adt.Dictionary",IID_IDictionary,&pDctOpts));

	return hr;
	}	// construct

void Namespace :: destruct ( void )
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

	// Protect namespace object during destruction of namespace
	AddRef();

	// Clean up
	_RELEASE(pDctRt);
	_RELEASE(pDctRec);
	_RELEASE(pDctStat);
	_RELEASE(pDctOpts);
	_RELEASE(pTmp);
	_RELEASE(punkDct);
	}	// destruct

HRESULT Namespace :: get ( const WCHAR *wPath, ADTVALUE &vGet, 
									const WCHAR *wRef )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		INamespace
	//
	//	PURPOSE
	//		-	Get an object from the namespace.
	//
	//	PARAMETERS
	//		-	wPath is the namespace location, 
	//			Format <Definition path>/<Instance name>/<Path to object>
	//		-	vGet will receive the object
	//		-	wRef optionally specifies the graph reference to use for an 
	//			instance that should be created if not present.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr				= S_OK;
	IDictionary	*pDctSub		= NULL;
	IDictionary	*pDct			= NULL;
	bool			bRef			= false;
	bool			bRefGet		= false;
	bool			bInst			= false;
	adtValue		v,vAt;
	adtString	strName,strPath,strGet,strDef,strRes,strRef;
	U32			cnt,i;
	adtIUnknown	unkV;
	WCHAR			*token,*n;

	// Thread safety
//	csGet.enter();

	// Start at root 
//	dbgprintf ( L"Namespace::get:%s {\r\n", wPath );
	CCLTRY ( adtValue::copy ( adtIUnknown(pDctRt), vAt ) );
//	if (!WCASECMP(wPath,L"/graph/interface/browse/inst/"))
//	if (!wcsncmp ( wPath,L"/graph/interface/browse/"))
//	if (!WCASECMP( wPath,L"/Lib/Image/Cache/Client/Default/State/"))
//		dbgprintf ( L"Hi\r\n" );

	// Reference specified ?
	CCLOK ( strRef = wRef; )
	CCLOK ( bRef = (strRef.length() > 0); )

	// Graph reference ?
	CCLOK ( bRefGet = !WCASENCMP(wPath,LOC_NSPC_REF,wcslen(LOC_NSPC_REF)); )

	// Case insensitive
	CCLOK ( strGet = wPath; )
	CCLOK ( strGet.toLower(); )

	// Count number of tokens in path
	n = NULL;
	for (	token = WCSTOK ( &strGet.at(0), L"/", &n ),cnt = 0;
			token != NULL;
			token = WCSTOK ( NULL, L"/", &n ))
		++cnt;

	// Case insensitive
	CCLOK ( strGet = wPath; )
	CCLOK ( strGet.toLower(); )

	// Retrieve each level of the path in the namespace, 
	// loading/creating values from temporal database as needed.
	CCLOK ( strPath = L""; )
	for (	token = WCSTOK ( &strGet.at(0), L"/", &n ),i = 0;
			token != NULL && hr == S_OK;
			token = WCSTOK ( NULL, L"/", &n ),++i)
		{
		// Next name
		CCLOK  ( strName = token; )

		// Update current path
		CCLTRY ( strPath.append ( strName ) );
		CCLTRY ( strPath.append ( L"/" ) );

		// Current value must act like a dictionary in order for other locations to be stored inside
		CCLTRY ( _QISAFE((unkV = vAt),IID_IDictionary,&pDct) );

		// Check if name at current level is has already been loaded
		if (hr == S_OK && pDct->load ( strName, vAt ) != S_OK)
			{
			bool			bFolder	= false;

			// Need new 'at'
			CCLOK ( adtValue::clear ( vAt ); )

			//
			// Instance
			//

			// Instance if reference is specified and at last location
			if (!bInst) bInst = (bRef && (i+1) == cnt);

			// Time to instance at current location
			if (hr == S_OK && bInst)
				{
				ILocation	*pLoc	= NULL;
				IReceptor	*pRcp	= NULL;
				adtString	strRefInst(LOC_NSPC_REF);

				// State check
				CCLTRYE ( strRef.length() > 0, ERROR_INVALID_STATE );

				// Create new location and source location of reference
				CCLTRY ( COCREATE ( L"Nspc.Location", IID_IDictionary, &pDctSub ) );
				CCLTRY ( pDctSub->store ( strnRefLocn, strRef ) );

				// Store information about level
				CCLTRY ( pDct->store ( strName, adtIUnknown(pDctSub) ) );
				CCLTRY ( pDctSub->store ( strnRefName,	adtString(strName) ) );
				CCLTRY ( pDctSub->store ( strnRefPar,	adtIUnknownRef(pDct) ) );
				CCLTRY ( pDctSub->store ( strnRefNspc,	adtIUnknownRef((INamespace *)this) ) );

				//
				// Same logic as 'Location::desc'
				//	VVVV
				//

				// Obtain the reference information
//				CCLTRY ( strRef.prepend ( LOC_NSPC_REF ) );
				CCLTRY ( strRefInst.append ( strRef ) );
				CCLTRY ( get ( strRefInst, v, NULL ) );

				// Create a non-mirrored link between source and destination
				CCLTRY ( _QISAFE((unkV=v),IID_ILocation,&pLoc) );
				CCLTRY ( _QISAFE(pDctSub,IID_IReceptor,&pRcp) );
				CCLTRY ( pLoc->connect ( pRcp, true, false ) );
				adtValue::clear(v);

				// Move location into active state
//				CCLOK ( dbgprintf ( L"Namespace::get:Active:%s\r\n", (LPCWSTR) strPath ); )
				CCLTRY ( pDctSub->store ( strnRefAct, adtBool(true) ) );

				// Receive path to reference into location so listeners are notified
				// of activation, leave off the 'ref/' prefix
				CCLTRY ( pRcp->receive ( pRcp, strnRefLocn, strRef ) );
//				CCLTRY ( receive ( (IReceptor *) this, strnRefLocn, strPath ) );
//				CCLTRY ( pDctOut->store ( strnRefLocn, strRef ) );

				//
				//	^^^^
				// Same logic as 'Location::desc'
				//	

				// Initialize temporal state
				CCLTRY ( record ( strPath, false, true ) );

				// Now at graph
				CCLTRY ( adtValue::copy ( adtIUnknown(pDctSub), vAt ) );

				// Instance off
				bInst = false;

				// Clean up
				_RELEASE(pRcp);
				_RELEASE(pLoc);
				_RELEASE(pDctSub);
				}	// if

			// If no reference has been encountered yet, assume valid path ?.
			// TODO: Fix creation of invalid empty paths.
			if (hr == S_OK && !bRef && !bRefGet)
				bFolder = true;

			// If a reference is specified, allow creation at arbitrary locations,
			// so prepare path to instance by creating locations as needed.
			if (hr == S_OK && adtValue::empty(vAt) && !bFolder && bRef)
				bFolder = true;

			//
			// Folder
			//
			if (hr == S_OK && bFolder)
				{
				// Create location 
				CCLTRY ( COCREATE ( L"Nspc.Location", IID_IDictionary, &pDctSub ) );
				CCLTRY ( adtValue::copy ( adtIUnknown(pDctSub), vAt ) );
				CCLTRY ( pDct->store ( strName, vAt ) );

				// Store name and parent for this location
				CCLTRY ( pDctSub->store ( strnRefName, adtString(strName) ) );
				CCLTRY ( pDctSub->store ( strnRefPar, adtIUnknownRef(pDct) ) );

				// Clean up
				_RELEASE(pDctSub);
				}	// if

			}	// if

		// To support auto instancing :
		// If no reference is specified and not at last name, 
		// determine if the current path 
		// represents a valid definition, if it does the graph
		// will be instanced at the next name.
		if (hr == S_OK && !bRef && !bRefGet && (i+1) < cnt)
			{
			adtString	strDesc;

			// Obtain path to what would be a the reference
			CCLTRY ( adtValue::copy ( strPath, strDesc ) );
			CCLTRY ( strDesc.prepend ( LOC_NSPC_REF ) );
			CCLTRY ( get ( strDesc, v, NULL ) );

			// Expecting a dictionary
			if (	hr == S_OK &&
					(IUnknown *)(NULL) != (unkV=v) &&
					_QI(unkV,IID_IDictionary,&pDctSub) == S_OK )
				{
				HRESULT		hrd	= S_OK;
				adtString	strType;

				// Does layer have a reference value ?
				hrd = pDctSub->load ( strnRefRef, v );

				// If last string in path, reference layer must exist
				if (hrd != S_OK && (i+1) == cnt)
					hr = ERROR_NOT_FOUND;

				// Reference to use is path up to this point
				else if (hrd == S_OK)
					{
					CCLTRY ( adtValue::copy ( strPath, strRef ) );
					CCLOK  ( bRef = true; )
					CCLOK  ( bInst = true; )
					}	// else if

				}	// if

			// Clean up
			_RELEASE(pDctSub);
			}	// if

		// Clean up
		_RELEASE(pDct);
		}	// for

	// Result
	CCLTRY ( adtValue::copy ( vAt, vGet ) );

	// Thread safety
//	csGet.leave();

	// Clean up
//	dbgprintf ( L"} Namespace::get:%s:0x%x\r\n", wPath, hr );

/*
	bool					bDef			= true;
	bool					bAutoInst	= true;

	// Setup
	adtValue::clear(vGet);

	// SPECIAL CASE: Definitions are stored in a predefined path that are not loaded
	//	into the namespace directly.
//	if (	hr == S_OK &&
//			(	(wPath[0] != '/' && !WCASENCMP(LOC_NSPC_DEFS,wPath,wcslen(LOC_NSPC_DEFS))) ||
//				(wPath[0] == '/' && !WCASENCMP(LOC_NSPC_DEFS,wPath+1,wcslen(LOC_NSPC_DEFS))) ) )
//		return definition ( wPath, vGet );

	// Debug
//	if (!WCASECMP(wPath,L"/driver/instrument/manage/inst/default/inst/state/instance/B29001_1/config/States/Fire"))
//		dbgprintf ( L"Hi\r\n" );


	// Handle any redirects
//	CCLOK ( dbgprintf ( L"Namespace::resolve:%s\r\n", wPath ); )
//	CCLTRY ( resolve ( wPath, strPath ) );
//	CCLOK ( dbgprintf ( L"Namespace::resolve:%s\r\n", (LPCWSTR)strPath ); )

	// Retrieve each level of the path in the namespace, 
	//	loading/creating values dynamically as needed.
	for (i = 0;hr == S_OK && i < cnt;++i)
		{
		// Obtain next name
		CCLTRY ( pNamesIt->read ( v ) );
		CCLOK  ( strName = v; )

		// Update current path
		CCLTRY ( strPath.append ( strName ) );
		CCLTRY ( strPath.append ( L"/" ) );
		CCLOK  ( len		= strPath.length(); )

		// Current value must act like a dictionary in order for other locations to be stored inside
		CCLTRY ( _QISAFE((unkV = vAt),IID_IDictionary,&pDct) );

		// Check if name at current level is has already been loaded
		if (hr == S_OK && pDct->load ( strName, vAt ) != S_OK)
			{
			// If still within the definition prefix, check if the current
			// path is identified with a valid definition.
			if (hr == S_OK && bDef)
				{
				bool bNs		= false;
				bool bTmp	= false;

				// Definitions are stored as states in the namespace (State/Graph/Graph/XXXX)
				CCLOK  ( strDef = LOC_NSPC_DEFS; )
				CCLTRY ( strDef.append ( strPath ) );
				bNs = (nspcLoadPath ( pDctDef, strDef, v ) == S_OK);

				// Check if path is in temporal dataase
				if (!bNs && pDctStat->clear() == S_OK)
					bTmp = (pTmp->status ( strDef, pDctStat ) == S_OK);

				// If definition path up to this point is valid...
				if (hr == S_OK && (bNs || bTmp))
					{
					// Valid paths are always created
					CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDctSub ) );
					CCLTRY ( adtValue::copy ( adtIUnknown(pDctSub), vAt ) );
					CCLTRY ( pDct->store ( strName, vAt ) );

					// Store information about level so paths to root can be generated
					CCLTRY ( pDctSub->store ( adtString(L"_Name"), adtString(strName) ) );
					CCLTRY ( pDctSub->store ( adtString(L"_Parent"), adtLong((U64)pDct) ) );

					// A valid definition in the namespace has a last modified value.
					bool bAtDef = false;
					if (bNs)
						{
						IDictionary	*pDctD = NULL;
						adtString	strDefE(strDef);

						// Modified emitter
						CCLTRY ( strDefE.append ( L"Modified/Fire" ) );

						// Check value
						CCLTRY ( _QISAFE((unkV = v),IID_IDictionary,&pDctD) );
						CCLOK  ( bAtDef = (pDctD->load ( adtString(L"Modified"), v ) == S_OK); )

						// Clean up
						_RELEASE(pDctD);
						}	// if

					// Check for modified emitter in database
					else if (bTmp)
						{
						adtString	strDefE(strPath);

						// Check emitter
						CCLTRY ( strDefE.append ( L"Modified/Fire" ) );
						CCLTRY ( pDctStat->clear() );
						CCLOK  ( bAtDef = (	pTmp->status ( strDefE, pDctStat ) == S_OK &&
													pDctStat->load ( adtString(L"Emitter"), v ) ); )
						}	// if

					// At definition ?
					if (hr == S_OK && bAtDef)
						{
						// Current path is a valid definition, ensure definition is
						// fully loaded and signal end of definition path
						if (!bNs) hr = definition ( strDef );
						CCLTRY( pDctSub->store ( adtString(L"_Definition"), strDef ) );
						}	// if

					// Clean up
					_RELEASE(pDctSub);
					}	// if

				// Invalid definition path
				else
					hr = ERROR_NOT_FOUND;
				}	// if

			// If definition has been found but top level instance is missing,
			// instance the graph for the current definition.
			else if (hr == S_OK && !bDef && bAutoInst)
				{
//if (!WCASECMP(strDef,L"Lib/Network/Ip/Tcp/Http/Browser/"))
//if (!WCASECMP(wPath,L"/Apps/Auto/Test/Test/Fire"))
//	dbgprintf ( L"Hi\r\n" );
				// Instance the graph
//				CCLTRY ( instance ( strPath, strDef, vAt ) );
				}	// else if

			// If on the last name in the path and a definition is specified,
			// an instance can be created and placed there.
			else if (hr == S_OK && !bDef && !bAutoInst && (i+1) == cnt && wDef != NULL)
				{
				// Instance the graph
//				CCLTRY ( instance ( strPath, wDef, vAt ) );
				}	// else if

			// Otherwise item does not exist
			else if (hr == S_OK)
				hr = ERROR_NOT_FOUND;
			}	// if

		// Opportunity for auto instance has passed
		if (	hr == S_OK	&&
				!bDef			&&
				bAutoInst )
			bAutoInst = false;

		// End of the definition portion of the path ?
		if (	hr == S_OK												&&
				bDef														&&
				(IUnknown *)(NULL) != (unkV = vAt)				&&
				_QI(unkV,IID_IDictionary,&pDctSub) == S_OK	&&
				pDctSub->load ( adtString(L"_Definition"), v ) == S_OK)
			{
			// Definition now available
			bDef = false;

			// Assign used definition
			CCLOK ( strDef = strPath; )
			}	// if
		_RELEASE(pDctSub);

		// Clean up
		_RELEASE(pDct);
		CCLOK ( pNamesIt->next(); )
		}	// for

	// Result
	CCLTRY ( adtValue::copy ( vAt, vGet ) );

	// Clean up
*/
	return hr;
	}	// get

HRESULT Namespace :: load ( const WCHAR *wPath, IDictionary **ppRef )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		INamespace
	//
	//	PURPOSE
	//		-	Load the refefence location for the specified path.
	//
	//	PARAMETERS
	//		-	wPath is the source path.
	//		-	ppRef will receive the reference location.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr			= S_OK;
	IUnknown			*pLoc		= NULL;
	adtValue			v;
	adtIUnknown		unkV;
	adtStringSt		strPath(wPath);

	// See if it is already loaded/cached
	if (nspcLoadPath ( pDctRt, wPath, v ) == S_OK)
		{
		// Access interface
		CCLTRY(_QISAFE((unkV=v),IID_IDictionary,ppRef) );
		return hr;
		}	// if

	// Attempt to open up a temporal location for the reference.
	CCLTRY ( pDctOpts->store ( strnRefRO, adtBool(true) ) );
	CCLTRY ( pDctOpts->store ( strnRefLoc, adtStringSt(wPath) ) );
	CCLTRY ( pTmp->open ( pDctOpts, &pLoc ) );

	// If successful, place in namespace
	CCLTRY ( pDctRt->store ( strPath, adtIUnknown(pLoc) ) );

	// Result
	CCLTRY ( _QISAFE(pLoc,IID_IDictionary,ppRef) );

	// Debug
	if (hr != S_OK)
		dbgprintf ( L"Namespace::load:Error:%s:0x%x\r\n", wPath, hr );

	// Clean up
	_RELEASE(pLoc);

	return hr;
	}	// load

/*
HRESULT Namespace :: iterate	( const WCHAR *wPath, const SEQRNG *pRng, 
											IEmittertIt **ppIt )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Iterate values at a location across a range of sequences.
	//
	//	PARAMETERS
	//		-	wPath is the emitter path for iteration
	//		-	pRng is the range of sequence numbers to use to bound the 
	//			iteration
	//		-	ppIt will receive the emitter iterator
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	IEmittert	*pEmitt	= NULL;
	adtValue		v;

	// Check status on emitter to see if its valid
	CCLTRY ( pDctStat->clear() );
	CCLTRY ( pTmp->status ( wPath, pDctStat, pRng ) );

	// Valid emitter ?
	if (hr == S_OK && pDctStat->load ( adtString(L"Emitter"), v ) == S_OK)
		{
		// Access temporal emitter for path
		CCLTRY ( pTmp->emitter ( wPath, &pEmitt ) );

		// Perform iteration
		CCLTRY ( pEmitt->iterate ( ppIt, pRng ) );

		// Clean up
		_RELEASE(pEmitt);
		}	// if

	return hr;
	}	// iterate
*/

HRESULT Namespace :: link ( const WCHAR *wPathDst, const WCHAR *wPathSrc,
										bool bLink )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called to link a destination subgraph to a source subgraph.
	//
	//	PARAMETERS
	//		-	wPathDst is the path to the destination subgraph
	//		-	wPathSrc is the path to the source subgraph
	//		-	bLink is true to link, false to unlink
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	IDictionary	*pLocDst	= NULL;
	IReceptor	*pLocRcp	= NULL;
	ILocation	*pLocSrc	= NULL;
	adtValue		v;
	adtIUnknown	unkV;

	// Obtain destination
	if (hr == S_OK)
		{
		CCLTRY ( get ( wPathDst, v, NULL ) );
		CCLTRY ( _QISAFE((unkV=v),IID_IDictionary,&pLocDst) );
		CCLTRY ( _QISAFE((unkV=v),IID_IReceptor,&pLocRcp) );

		// Debug
		if (hr != S_OK)
			dbgprintf ( L"Namespace::link:Destination error:0x%x:%s\r\n", hr, wPathDst );
		}	// if

	// Obtain source	
	if (hr == S_OK)
		{
		CCLTRY ( get ( wPathSrc, v, NULL ) );
		CCLTRY ( _QISAFE((unkV=v),IID_ILocation,&pLocSrc) );

		// Debug
		if (hr != S_OK)
			dbgprintf ( L"Namespace::link:Source error:0x%x:%s\r\n", hr, wPathSrc );
		}	// if

	// Debug
	if (hr == S_OK && !bLink)
		{
		hr = S_OK;
		}	// if
//		dbgprintf ( L"Hi\r\n" );

	// Optional unlink signal
	if (hr == S_OK && !bLink)
		receive ( pLocDst, L"~Unlink/OnFire", adtInt(0) );

	// If linking, deactivate the destination since sharing just locations
	if (hr == S_OK && bLink)
		hr = pLocDst->store ( strnRefAct, adtBool(false) );

	// Share location information by setting up a mirrored link
	CCLTRY ( pLocSrc->connect ( pLocRcp, bLink, true ) );

	// If unlinking, destination can be re-activated to function as normal
	if (hr == S_OK && !bLink)
		hr = pLocDst->store ( strnRefAct, adtBool(true) );

	// Optional link signal
	if (hr == S_OK && bLink)
		receive ( pLocDst, L"~Link/OnFire", adtInt(0) );

	// Clean up
	_RELEASE(pLocSrc);
	_RELEASE(pLocRcp);
	_RELEASE(pLocDst);

	// Debug
	if (hr != S_OK)
		dbgprintf ( L"Namespace::link:Error:%s -> %s\r\n", wPathSrc, wPathDst );

	return hr;
	}	// link

HRESULT Namespace :: open ( ILocations *_pTmp )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		INamespace
	//
	//	PURPOSE
	//		-	Open a new namespace.
	//
	//	PARAMETERS
	//		-	_pTmp is the temporal locations that backs the namespace.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	IDictionary	*pDctRef	= NULL;
	IUnknown		*pLoc		= NULL;
	adtValue		v;

	// Temporal database
	pTmp	= _pTmp;
	_ADDREF(pTmp);

	// Create root location for namespace
	CCLTRY ( COCREATE ( L"Nspc.Location", IID_IDictionary, &pDctRt ) );

	// Open up a temporal location for the reference layers.
	CCLTRY ( pDctOpts->store ( strnRefLoc, adtString(LOC_NSPC_REF) ) );
	CCLTRY ( pDctOpts->store ( strnRefRO, adtBool(true) ) );
	CCLTRY ( pTmp->open ( pDctOpts, &pLoc ) );
	CCLTRY ( _QI(pLoc,IID_IDictionary,&pDctRef) );

	// Initialize reference layer and store under root
	CCLTRY ( pDctRef->store ( strnRefNspc,	adtIUnknownRef((INamespace *)this) ) );
	CCLTRY ( pDctRef->store ( strnRefPar,	adtIUnknownRef(pDctRt) ) );
	CCLTRY ( pDctRef->store ( strnRefName,	adtString(L"ref") ) );
	CCLTRY ( pDctRt->store ( adtString(L"ref"), adtIUnknown(pDctRef) ) );

	// Clean up
	_RELEASE(pDctRef);
	_RELEASE(pLoc);

	return hr;
	}	// open

HRESULT Namespace :: receive ( IDictionary *pDct,  const WCHAR *wPath, 
											const ADTVALUE &v )
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
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	IReceptor	*pRecep	= NULL;
	adtValue		vL;
	adtIUnknown	vUnk;

	// Load specified receptor
	if (	nspcLoadPath ( pDct, wPath, vL ) == S_OK	&&
			(IUnknown *)(NULL) != (vUnk = vL)			&&
			_QI(vUnk,IID_IReceptor,&pRecep) == S_OK )
		pRecep->receive ( NULL, L"Value", v );

	// Clean up
	_RELEASE(pRecep);

	return hr;
	}	// receive

HRESULT Namespace :: record ( const WCHAR *wPath, bool bRec )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		INamespace
	//
	//	PURPOSE
	//		-	Called to turn on or off recording for a portion of the
	//			namespace.
	//
	//	PARAMETERS
	//		-	wPath is the namespace path to record.
	//		-	bRec is true to record, false to turn off recording.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
/*
	IEmitter		*pEmit	= NULL;
	INode			*pNode	= NULL;
	IDictionary	*pDct		= NULL;
	adtValue		vV;
	adtIUnknown	unkV;

	// Determine if caller is requesting recording on emitter, node or
	// an entire subgraph.

	// Access the object at path
	CCLTRY	( nspcLoadPath ( pDctRt, wPath, vV ) );
	CCLTRYE	( (IUnknown *)NULL != (unkV = vV), E_UNEXPECTED );

	// Emitter ?
	if (hr == S_OK && _QI(unkV,IID_IEmitter,&pEmit) == S_OK)
		{
		// Record/unrecord emitter
		CCLTRY ( record ( wPath, pEmit, bRec ) );
		}	// if

	// Node ?
	else if (hr == S_OK && _QI(unkV,IID_INode,&pNode) == S_OK)
		{
		hr = E_NOTIMPL;
		}	// if

	// Subgraph
	else if (hr == S_OK && _QI(unkV,IID_IDictionary,&pDct) == S_OK)
		{
		// Record subgraph
		CCLTRY ( record ( wPath, bRec, false ) );
		}	// if

	// Clean up
	_RELEASE(pDct);
	_RELEASE(pNode);
	_RELEASE(pEmit);
*/
	return hr;
	}	// record

HRESULT Namespace :: record ( const WCHAR *wPath, bool bRec, bool bInit )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Internal utility to initialize and turn on or off recording 
	//			for a portion of the namespace.
	//
	//	PARAMETERS
	//		-	wPath is the namespace path to record.
	//		-	bRec is true to force record, false to turn off any recording.
	//		-	bInit is true if initializing an instance for the first time.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	IDictionary	*pLoc		= NULL;
	IIt			*pIt		= NULL;
	adtValue		vK,vV;
	adtIUnknown	unkV;
	adtString	strPathLoc(wPath);

	// This function requires trailing slash
	if (hr == S_OK && strPathLoc[strPathLoc.length()-1] != '/')
		hr = strPathLoc.append ( L"/" );

	// Access the location
	CCLTRY ( nspcLoadPath ( pDctRt, strPathLoc, vV ) );
	CCLTRY ( _QISAFE((unkV = vV),IID_IDictionary,&pLoc) );

	// Debug
//	if (!WCASECMP(wPath,L"/apps/auto/test/editor/interface/BrowseTo/interface/IntfNodes/"))
//		dbgprintf ( L"Hi\r\n" );

	// Iterate the sub-locations
	CCLTRY ( pLoc->keys ( &pIt ) );
	while (hr == S_OK && pIt->read ( vK ) == S_OK)
		{
		IDictionary	*pDesc	= NULL;
		IDictionary	*pLocDct	= NULL;
		IDictionary	*pLocRec	= NULL;
		ILocation	*pLocLoc	= NULL;
		adtIUnknown	unkV;
		adtValue		v;
		adtString	strV,strPathLocSub;
		adtString	strName(vK);
		adtString	strType;

		// Path to this sublocation
		CCLOK	( adtValue::copy ( strPathLoc, strPathLocSub ); )
		CCLTRY( strPathLocSub.append ( strName ) );
		CCLTRY( strPathLocSub.append ( L"/" ) );

		// Debug
//		if (hr == S_OK && !WCASECMP(strName,L"TupleTest"))
//			dbgprintf ( L"Hi\r\n" );

		// Check if item is a location
		if (	hr == S_OK &&
				WCASECMP(strName,STR_NSPC_PARENT)					&&
				pLoc->load ( vK, vV )				== S_OK			&&
				(IUnknown *)(NULL)					!= (unkV = vV)	&&
				_QI(unkV,IID_ILocation,&pLocLoc)	== S_OK )
			{
			// Check if location has a descriptor with an attached behaviour
			if (	hr == S_OK													&&
					_QI(unkV,IID_IDictionary,&pLocDct)	== S_OK		&&
					pLocDct->load ( strnRefDesc, v )		== S_OK		&&
					(IUnknown *)(NULL) != (unkV = v)						&&
					_QI(unkV,IID_IDictionary,&pDesc) == S_OK			&&
					pDesc->load ( strnRefType, v ) == S_OK				&&
					(strType = v).length() > 0								&&
					!WCASECMP(strType,L"Behaviour")						&&
					pDesc->load ( strnRefBehave, v ) == S_OK			&&
					(strV = v).length() > 0 )
				{
				// nSpace value ?
				if (!WCASECMP(strV,L"Nspc.Value"))
					{
					adtString	strLoct(strPathLocSub);

					// Temporal location of values
					CCLTRY ( strLoct.append ( L"OnFire/" ) );

					// Prepare to receive stats
					CCLOK ( pDctStat->clear(); )

					// Enable recording for pre-existing values and forced recordings.
					if (	hr == S_OK &&

								// Temporal values already exist
							(	(	pTmp->status ( strLoct, pDctStat ) == S_OK &&
									pDctStat->load ( strnRefLoc, v ) == S_OK ) ||

								// Caller is requsting forced recording
								(bRec && !bInit) ) )
						{
						// Access location
						CCLTRY ( pLocDct->load ( adtStringSt(L"OnFire"), vV ) );
						CCLTRY ( _QISAFE((unkV = vV),IID_IDictionary,&pLocRec) );

						// Persistence
						CCLOK ( record ( strLoct, pLocRec, true ); )

						// Clean up
						_RELEASE(pLocRec);
						}	// if

					}	// if (!WCASECMP(strV,L"Nspc.Value"))

				}	// if

			// Process sub locations
			else
				hr = record ( strPathLocSub, bRec, bInit );
			}	// if

		// Clean up
		_RELEASE(pDesc);
		_RELEASE(pLocDct);
		_RELEASE(pLocLoc);

		// Clean up
		pIt->next();
		}	// while

	// If caller explicity requested to turn on/off recording, flag the graph
	if (hr == S_OK && !bInit)
		{
		// Persist enable/disable
//		if (bRec)
//			hr = pGr->store ( strnRefPers, adtBool(true) );
//		else
//			pGr->remove ( strnRefPers );
		}	// if

	// Clean up
	_RELEASE(pIt);
	_RELEASE(pLoc);

	return hr;
	}	// record

HRESULT Namespace :: record ( const WCHAR *wPath, IDictionary *pLoc,
										bool bRec )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		INamespace
	//
	//	PURPOSE
	//		-	Record/stop recording a location.
	//
	//	PARAMETERS
	//		-	wPath is the path of the location
	//		-	pLoc is the location
	//		-	bRec is true to record, false to stop recording.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr	= S_OK;
	adtValue		vL;
	adtIUnknown	unkV;

	// Debug
	dbgprintf ( L"Namespace::record:%p:%d:%s\r\n", pLoc, bRec, wPath );

	// Record 
	if (hr == S_OK && bRec)
		{
		// Ensure not already recorded
		if (pDctRec->load ( adtIUnknownRef(pLoc), vL ) != S_OK)
			{
			ILocation	*pLocSrc	= NULL;
			IReceptor	*pRcp		= NULL;
			IUnknown		*punkLoc	= NULL;

			// Open temporal dictionary for location.
			CCLTRY ( pDctOpts->store ( strnRefRO, adtBool(true) ) );
			CCLTRY ( pDctOpts->store ( strnRefLoc, adtStringSt(wPath) ) );
			CCLTRY ( pTmp->open ( pDctOpts, &punkLoc ) );

			// Set-up a mirrored connection between temporal location and
			// namespace location.
			CCLTRY ( _QISAFE(punkLoc,IID_ILocation,&pLocSrc) );
			CCLTRY ( _QISAFE(pLoc,IID_IReceptor,&pRcp) );
			CCLTRY ( pLocSrc->connect ( pRcp, true, true ) );

			// Store as recording 
			CCLTRY ( pDctRec->store ( adtIUnknownRef(pLoc), adtIUnknown(punkLoc) ) );

			// Clean up
			_RELEASE(pRcp);
			_RELEASE(pLocSrc);
			_RELEASE(punkLoc);
			}	// if
//		else
//			dbgprintf ( L"Namespace::record:Already recorded %s\r\n", wPath );

		}	// if
	else if (hr == S_OK && !bRec)
		{
		// Remove recording from dictionary.  This should clean up the temporal receptor, etc.
		pDctRec->remove ( adtIUnknownRef(pLoc) );
		}	// else

	// Debug
//	dbgprintf ( L"} Namespace::record:%p:%d:%s:0x%x\r\n", pEmit, bRec, wPath, hr );

	return hr;
	}	// record

HRESULT Namespace :: relink ( const WCHAR *wPath, IDictionary *pGr )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Scan the provided graph for subgraph that might need to
	//			be reattached to a different state based on the information
	//			in the link layer.
	//
	//	PARAMETERS
	//		-	wPath is the namespace path of the graph
	//		-	pGr is the graph
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
/*
if (true)
	return hr;
	IIt			*pIt	= NULL;
	adtValue		vK,vV;
	adtString	strPath;

	// Search items in graph
//	dbgprintf ( L"Namespace::relink:%s\r\n", (LPCWSTR)wPath );
	CCLTRY ( pGr->keys ( &pIt ) );
	while (hr == S_OK && pIt->read ( vK ) == S_OK)
		{
		IDictionary	*pSubDct	= NULL;
		IGraph		*pSubGr	= NULL;
		adtIUnknown	unkV;
		adtValue		v;
		adtString	strV,strSub;

		// Access the value at the key
		CCLTRY ( pGr->load ( vK, vV ) );

		// Subgraph ?
		if (	(IUnknown *)(NULL) != (unkV = vV)					&&
				_QI(unkV,IID_IGraph,&pSubGr) == S_OK				&&
				_QI(unkV,IID_IDictionary,&pSubDct) == S_OK		&&
				pSubDct->load ( strnRefName, v ) == S_OK		&&
				(strSub = v).length() > 0 )
			{
			adtString	strPath(wPath),strRes;

			// Generate path to subgraph
			CCLTRY ( strPath.append ( strSub ) );
			CCLTRY ( strPath.append ( L"/" ) );

			// Resolve path
//if (strPath.length() > 26 && !WCASENCMP(strPath,L"Lib/Editor/Editor/Default/",26))
//	dbgprintf ( L"Hi\r\n" );
			CCLTRY ( resolve ( strPath, strRes ) );

			// If the subgraph has been redirected attach new graph
			if (hr == S_OK && WCASECMP(strPath,strRes))
				{
				adtValue	vSub;
				lprintf ( LOG_INFO, L"Subgraph redirect:%s:%s",
									(LPCWSTR)strPath, (LPCWSTR)strRes );
//				if (!WCASECMP(strRes,L"/Lib/Editor/Editor/Default/Nodes/ListTags/State/"))
//					dbgprintf ( L"Hi\r\n" );

				// Link source to destination
				CCLTRY ( link ( strPath, strRes, true ) );
				}	// if

			// Subgraph stays put, process as own graph
			else if (hr == S_OK)
				hr = relink ( strPath, pSubDct );
			}	// if

		// Clean up
		pIt->next();
		_RELEASE(pSubGr);
		_RELEASE(pSubDct);
		}	// while

	// Clean up
	_RELEASE(pIt);
*/
	return hr;
	}	// relink

HRESULT Namespace :: resolve ( const WCHAR *strFrom, adtString &strTo )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called to resolve a path using the link layer.  If the path
	//			is not link the same path is returned.
	//
	//	PARAMETERS
	//		-	strFrom is the path to resolve
	//		-	strTo will receive the resolve path.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
/*
	IDictionary	*pE	= NULL;
	adtValue		v;
	adtIUnknown	unkV;

	// Start with original path
	CCLOK ( strTo = strFrom; )

	// Follow the links until the final destination is found.
	// Use temporal nSpace to fill in missing links if necessary
	while (hr == S_OK)
		{
		bool	bLink	= true;

		// Attempt to load path from existing link layer
//		bLink = (nspcLoadPath ( pDctLnk, strTo, v ) == S_OK);
		hr = E_FAIL;

		// If path does not exist in link layer, check temporal nSpace.
		if (!bLink)
			{
			adtString	strFromL(L"Link/");

			// Prepare to receive stats
			pDctStat->clear();

			// Full path for temporal nSpace
			CCLTRY ( strFromL.append ( strTo ) );

			// Attempt stat. on location
			bLink					= (pTmp->status ( strFromL, pDctStat ) == S_OK);

			// Ensure location is an emitter
			if (bLink) bLink	= (pDctStat->load ( adtString(L"Emitter"), v ) == S_OK);

			// Attempt load on latest value
			if (bLink) bLink	= (tmpLoad ( strFromL, v ) == S_OK);

			// Value must be a non-empty string for the link to still be valid
			if (bLink) bLink	= (!adtValue::empty(v) && adtValue::type(v) == VTYPE_STR && v.pstr != NULL);

			// Links are simple values in the namespace
			if (bLink)
				{
				IEmitter	*pEmit	= NULL;

				// Create a namespace value for the link
				CCLTRY ( COCREATE ( L"Nspc.NamespaceValue", IID_IEmitter, &pEmit ) );

				// Set latest link value
				CCLTRY ( pEmit->emit ( v ) );

				// Store value in tree
				CCLTRY ( nspcStoreValue ( pDctRt, strFromL, adtIUnknown(pEmit) ) );

				// Emitter value
				CCLTRY ( adtValue::copy ( adtIUnknown(pEmit), v ) );

				// Clean up
				_RELEASE(pEmit);
				}	// if

			}	// if

		// Stop on final link
		if (!bLink)
			break;

		// The value at a path in the link layer is a namespace value
		// containing the target path.
		CCLTRY ( _QISAFE((unkV=v),IID_IDictionary,&pE) );
		CCLTRY ( pE->load(adtString(L"Value"),v) );

		// Value contains path
		CCLOK	 ( strTo = v; )
		CCLTRYE( strTo.length() > 0, E_UNEXPECTED );

		// Clean up
		_RELEASE(pE);
		}	// while
*/
	return hr;
	}	// resolve
/*
HRESULT Namespace :: temporal ( ITemporalImpl **ppTmp )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		INamespace
	//
	//	PURPOSE
	//		-	Return ptr. of current temporal interface.
	//
	//	PARAMETERS
	//		-	ppTmp will receive the interface.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	*ppTmp = pTmp;
	_ADDREF((*ppTmp));
	return ((*ppTmp) != NULL) ? S_OK : S_FALSE;
	}	// temporal
*/
/*
HRESULT Namespace :: tmpInit ( IDictionary *pGr, bool bInit )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Scan the provided graph for nodes that need to be
	//			temporally (un)initialized.
	//
	//	PARAMETERS
	//		-	wPath is the namespace path of the graph
	//		-	pGr is the graph
	//		-	bInit is true to initialize, false to uninitalize
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	IContainer	*pNms		= NULL;
	IDictionary	*pDct		= NULL;
	IDictionary	*pDef		= NULL;
	IIt			*pNmsIt	= NULL;
	IIt			*pIt		= NULL;
	adtValue		vK,vV;
	adtString	strPathGr;
	adtIUnknown	unkV;

	// Generate the full path to the graph.  This is done because where
	// the graph is instanced from may not match where the original state exists.
	CCLTRY ( nspcToAbs ( pGr, L"./", strPathGr ) );
//	dbgprintf ( L"Namespace::tmpInit:%s\r\n", (LPCWSTR)strPathGr );

	// Debug
//	if (!WCASECMP(wPath,L"Apps/Auto/Test/Tags/State/Instance/Dictionary/"))
//		dbgprintf ( L"Hi\r\n" );

	// Obtain the static list of ordered names in the list from the definition
	CCLTRY ( definition ( pGr, &pDef ) );
	CCLTRY ( pDef->load ( adtString(L"Names"), vV ) );
	CCLTRY ( _QISAFE((unkV=vV),IID_IContainer,&pNms) );

	// Iterate the named items in the graph
	CCLTRY ( pNms->iterate ( &pNmsIt) );
	while (hr == S_OK && pNmsIt->read ( vK ) == S_OK)
		{
		INode			*pNode	= NULL;
		IDictionary	*pAttr	= NULL;
		IEmitter		*pEmt		= NULL;
		adtIUnknown	unkV;
		adtValue		v;
		adtString	strV;

		// Access the item in the graph with the current name
		CCLTRY ( pGr->load ( vK, vV ) );

		// Dictionary object ?
		if (	(IUnknown *)(NULL)					!= (unkV = vV)			&&
				_QI(unkV,IID_IDictionary,&pDct)	== S_OK )
			{
			// Node ?
			if (	_QI(unkV,IID_INode,&pNode) == S_OK						&&
					pDct->load ( adtString(L"Attributes"), v ) == S_OK	&&
					(IUnknown *)(NULL) != (unkV = v)							&&
					_QI(unkV,IID_IDictionary,&pAttr) == S_OK				&&
					pAttr->load ( adtString(L"Behaviour"), v ) == S_OK	&&
					(strV = v).length() > 0 )
				{
				// nSpace value ?
				if (!WCASECMP(strV,L"Nspc.Value"))
					{
					adtString	strEmitt(strPathGr),strNode(vK);

					// Node path
					CCLTRY ( strEmitt.append ( strNode ) );
					CCLTRY ( strEmitt.append ( L"/Fire" ) );

					// Prepare to receive stats
					CCLOK ( pDctStat->clear(); )

					// Does an emitter exist for this value ?
					if (	hr == S_OK &&
							pTmp->status ( strEmitt, pDctStat ) == S_OK &&
							pDctStat->load ( adtString(L"Emitter"), v ) == S_OK)
						{
						// Access emitter
						CCLTRY ( pDct->load ( adtString(L"Fire"), vV ) );
						CCLTRY ( _QISAFE((unkV = vV),IID_IEmitter,&pEmt) );

						// Persistence
						CCLOK ( record ( strEmitt, pEmt, true ); )

						// Clean up
						_RELEASE(pEmt);
						}	// if

					}	// if (!WCASECMP(strV,L"Nspc.Value"))

				// nSpace values (dynamic) ?
				else if (!WCASECMP(strV,L"Nspc.Values"))
					{
					IIt			*pLocIt		= NULL;
					U32			nRec			= 0;
					adtString	strNodePath(strPathGr),strNode(vK);

					// Node path
					CCLTRY ( strNodePath.append ( strNode ) );
					CCLTRY ( strNodePath.append ( L"/" ) );

					// To support dynamic emitter values, scan for emitters in temporal database 
					CCLTRY ( pTmp->locations ( strNodePath, &pLocIt ) );
					while (hr == S_OK && pLocIt->read ( vV ) == S_OK)
						{
						adtString	strEmit(vV);

						// Emitter ?
						if (hr == S_OK && strEmit.length() > 0 && strEmit[strEmit.length()-1] != '/')
							{
//if (!WCASECMP(strEmit,L"entry_2"))
//	dbgprintf ( L"Hi\r\n" );
							// Does emitter need to be added to node ? (dynamic emitter)
							if (pDct->load ( strEmit, vV ) != S_OK)
								hr = pNode->addConnector ( strEmit, NULL, &pEmt );
							else
								hr = _QISAFE((unkV = vV),IID_IEmitter,&pEmt);

							// Persistence
							if (hr == S_OK)
								{
								adtString	strEmitt;

								// Full path to emitter
								CCLOK ( strEmitt = (LPCWSTR)strNodePath; )
								CCLOK ( strEmitt.append ( strEmit ); )

								// Persistence
								CCLOK ( record ( strEmitt, pEmt, true ); )

								// Emitter recorded
								CCLOK ( ++nRec; )
								}	// if

							// Clean up
							_RELEASE(pEmt);
							}	// if

						// Do no fail remaining load on error
						if (hr != S_OK)
							{
							dbgprintf ( L"Namespace::tmpInit:Dynamic emitter load failed:%s\r\n", (LPCWSTR)strEmit );
							hr = S_OK;
							}	// if
//						else
//							dbgprintf ( L"Namespace::tmpInit:Dynamic emitter load:%s\r\n", (LPCWSTR)strEmit );

						// Clean up
						pLocIt->next();
						}	// while

					// Clean up
					_RELEASE(pLocIt);

					// If node had temporalized emitters then flag it as such.
					if (hr == S_OK && nRec > 0)
						hr = pDct->store ( adtString(L"_Persist"), adtBool(true) );
					}	// else if (!WCASECMP(strV,L"Nspc.Values"))
				}	// if

			// Subgraph ?
			else if (	pDct->load ( adtString(L"_Name"), v ) == S_OK	&&
							(strV = v).length() > 0 )
				{
				// Initialize temporal activity for subgraph
				CCLTRY ( tmpInit ( pDct, bInit ) );
				}	// if

			}	// if

		// Clean up
		pNmsIt->next();
		_RELEASE(pNode);
		_RELEASE(pAttr);
		_RELEASE(pDct);
		}	// while

	// Clean up
	_RELEASE(pDef);
	_RELEASE(pNms);
	_RELEASE(pNmsIt);

	return hr;
	}	// tmpInit
*/
HRESULT Namespace :: tmpLoad ( const WCHAR *wPath, ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Load the latest value for the specified emitter.
	//
	//	PARAMETERS
	//		-	wPath is the emitter path
	//		-	v will receive the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	IDictionary	*pEmitt	= NULL;
	IUnknown		*pLoc		= NULL;

	// DEBUG
//	DWORD dwThen = GetTickCount();

	// Access location
	CCLTRY ( pDctOpts->store ( strnRefLoc, adtString(wPath) ) );
	CCLTRY ( pDctOpts->store ( strnRefRO, adtBool(true) ) );
	CCLTRY ( pTmp->open ( pDctOpts, &pLoc ) );
	CCLTRY ( _QISAFE(pLoc,IID_IDictionary,&pEmitt) );

	// Check if there is an emitted value stored
	CCLTRY ( pEmitt->load ( strnRefVal, v ) );

	// Debug
//	DWORD dwNow = GetTickCount();
//	dbgprintf ( L"tmpLoad:%d ms\r\n", (dwNow-dwThen) );

	// Clean up
	_RELEASE(pEmitt);
	_RELEASE(pLoc);

	return hr;
	}	// tmpLoad

HRESULT Namespace :: tmpLocs ( const WCHAR *wPath, IIt **ppIt )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Return an iterator for all of the emitter locations under
	//			the provided path.
	//
	//	PARAMETERS
	//		-	wPath is the parent location
	//		-	ppIt will receive the iterator
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	adtValue		v;

	// Setup
	(*ppIt)	= NULL;

	// First step is to ensure the provided path is a valid location
	CCLTRY ( pDctStat->clear() );
	CCLTRY ( pTmp->status ( wPath, pDctStat ) );
	CCLTRY ( pDctStat->load ( strnRefLoc, v ) );

	// Obtain iterator for locations at location
	CCLTRY ( pTmp->locations ( wPath, ppIt ) );

	return hr;
	}	// tmpLocs

HRESULT Namespace :: tokens ( const WCHAR *wStr, const WCHAR *wDelim,
										IContainer **ppC )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Generate a list of tokens in the string separated by the
	//			delimiters.
	//
	//	PARAMETERS
	//		-	wStr is the string to search
	//		-	wDelim is the list of delimiters
	//		-	ppC will receive the list
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	IList			*pLst	= NULL;
	WCHAR			*token,*n	= NULL;
	adtString	strSrch(wStr);

	// Setup
	(*ppC)	= NULL;

	// Create list to receive the tokens
	CCLTRY ( COCREATE ( L"Adt.List", IID_IList, &pLst ) );

	// Add tokens to list
	if (hr == S_OK)
		for (token = WCSTOK ( &strSrch.at(0), wDelim, &n );
				token != NULL;
				token = WCSTOK ( NULL, wDelim, &n ))
			pLst->write ( adtString(token) );

	// Result
	CCLOK ( (*ppC) = pLst; )
	_ADDREF(*ppC);

	// Clean up
	_RELEASE(pLst);

	return hr;
	}	// tokens
/*
HRESULT Namespace :: unload ( const WCHAR *wPath )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		INamespace
	//
	//	PURPOSE
	//		-	Unload a location in the namespace.
	//
	//	PARAMETERS
	//		-	wPath is the namespace location to unload.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	IDictionary	*pDctGr	= NULL;
	IDictionary	*pDctP	= NULL;
	size_t		len		= wcslen(wPath);
	adtValue		vL;
	adtIUnknown	unkV;
	adtString	strType;
	adtString	strName;

	// Access location from namespace
	CCLTRY ( nspcLoadPath ( pDctRt, wPath, vL ) );

	// Must at least support dictionary interface
	CCLTRY ( _QISAFE((unkV=vL),IID_IDictionary,&pDctGr) );

	// Parent graph
	if (hr == S_OK && pDctGr->load ( strnRefPar, vL ) == S_OK)
		{
		const WCHAR	*pw;

		// Dictionary interface for parent
		CCLTRY( _QISAFE((unkV=vL),IID_IDictionary,&pDctP) );

		// Isolate name from path
		if (hr == S_OK && len > 2)
			{
			// Extract the last name of the path
			for (pw = &(wPath[len- 2]);
				pw != wPath && *pw != '/';
				--pw) {}

			// Generate name
			strName	= pw + 1;
			len		= strName.length();
			if (len > 0 && strName[len - 1] == '/')
				strName.at(len - 1) = '\0';
	
			// Remove location from parent
			CCLOK ( pDctP->remove(strName); )
			}	// if

		}	// if

	// Clean up
	_RELEASE(pDctP);
	_RELEASE(pDctGr);

	return hr;
	}	// unload
*/
