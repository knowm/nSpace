////////////////////////////////////////////////////////////////////////
//
//								THIS.CPP
//
//					Implementation of the 'this' node.
//
////////////////////////////////////////////////////////////////////////

#include "nspcl_.h"

This :: This ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	strLoc	= L"";
	pDctF		= NULL;
	pDctT		= NULL;
	pLocPar	= NULL;
//	pItt		= NULL;

	// Default range (all values)
//	rng.lower	= MIN_SEQIDX;
//	rng.upper	= MAX_SEQIDX;
	}	// This

HRESULT This :: onAttach ( bool bAttach )
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
	HRESULT		hr		= S_OK;
	adtValue		v;

	// Attach
	if (bAttach)
		{
		adtIUnknown	unkV;

		// Attributes
		if (pnDesc->load(adtStringSt(L"Location"),v) == S_OK)
			strLoc = v;
		if (pnDesc->load(adtStringSt(L"Definition"),v) == S_OK)
			strDef = v;

		// Obtain reference to parent location
		CCLTRY ( pnLoc->load ( strnRefPar, v ) );
		CCLTRY ( _QISAFE((unkV=v),IID_IDictionary,&pLocPar) );
		if (hr == S_OK) pLocPar->Release();
		}	// if

	// Detach
	else
		{
		// Clean up
//		_RELEASE(pItt);
		pLocPar = NULL;
		_RELEASE(pDctF);
		_RELEASE(pDctT);
		}	// else

	return hr;
	}	// onAttach

HRESULT This :: onReceive ( IReceptor *pr, const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	FROM	IReceptor
	//
	//	PURPOSE
	//		-	A location has received a value on the specified receptor.
	//
	//	PARAMETERS
	//		-	pr is the receptor
	//		-	pl is the location
	//		-	v is the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;
//	static	DWORD dwThen = 0;

	// Local graph information
	if (_RCP(Fire))
		{
		IDictionary	*pDct	= NULL;
		adtValue		vL;

		// Reference to graph object (avoid circular reference counts)
//		CCLOK ( _EMT(Graph,adtIUnknownRef(pnGr)); )

		// Namespace information
		CCLTRY ( _QI(pnSpc,IID_IDictionary,&pDct) );
		if (hr == S_OK && pDct->load ( adtStringSt(L"StreamSource"), vL ) == S_OK)
			_EMT(Source,vL);

		// Clean up
		_RELEASE(pDct);
		}	// if

	// Load object
	else if (_RCP(Load))
		{
		adtValue		vLd;
		adtString	strLocAbs;

		// State check
		CCLTRYE ( strLoc.length() > 0,	ERROR_INVALID_STATE );

//if (!WCASECMP(this->strnName,L"LoadRender"))
//	dbgprintf ( L"Hi\r\n" );

		// Generate full path
		CCLTRY ( nspcPathTo ( (pDctF != NULL) ? pDctF : pLocPar, strLoc, strLocAbs ) );

//if (!WCASECMP(strLoc,L"./Browser/Interface/"))
//if (!WCASECMP(strLocAbs,L"/Apps/Auto/Default/ValueTest/Fire/Value/You/"))
//if (!WCASECMP(this->strnName,L"LoadNode"))
//	dbgprintf ( L"Hi\r\n" );
//if (!WCASECMP(this->strnName,L"GetSt"))
//	dbgprintf ( L"Hi\r\n" );
//if (strDef.length() > 0)
//if (!WCASECMP(strLocAbs,L"/Apps/Auto/Default/Direct/State/Color/Down"))
//if (!WCASECMP(strDef,L"Editor/Visual/Node/Node/"))
//{
//	dbgprintf ( L"%s:%s\r\n", (LPCWSTR)strLoc, (LPCWSTR)strLocAbs );
//	dbgprintf ( L"\r\n" );
//}

		// Attempt to load value with the given root
		CCLTRY ( pnSpc->get ( strLocAbs, vLd, strDef ) );

		// Debug
//		DWORD dwNow = GetTickCount();
//		dbgprintf ( L"This::receive:Load:%s:%d ms\r\n", (LPCWSTR)strLocAbs, dwNow-dwThen );
//		dwThen = GetTickCount();

		// Result
		if (hr == S_OK)
			_EMT(Load,vLd);
		else if (strLoc.length() > 0)
			{
			dbgprintf ( L"This::receive:load failed:0x%x:%s:%s:%s\r\n", hr, 
									(LPCWSTR)strnName, (LPCWSTR)strLocAbs, (LPCWSTR)strDef );
			_EMT(NotFound,strLocAbs);
			}	// else

		}	// else if

	// Store location
	else if (_RCP(Store))
		{
		IReceptor			*pRecep	= NULL;
		const ADTVALUE		*pvUse	= (adtValue::empty(vValue)) ? &v : &vValue;
		adtString			strLocAbs,strPathSt;
		U32					len;
		adtValue				vR;
		adtIUnknown			unkV;

		// NOTE: Using the same logic as the NamespaceX::store function to
		// maximize flexibility.

		// State check
		CCLTRYE ( strLoc.length() > 0,	ERROR_INVALID_STATE );

		// Ensure an absolute namespace path is used
		CCLTRY ( nspcPathTo ( (pDctF != NULL) ? pDctF : pLocPar, strLoc, strLocAbs ) );

		// Storing is done from the root location so ignore any leading slashes
		CCLOK ( strPathSt = (strLocAbs[0] == '/') ? &strLocAbs[1] : &strLocAbs[0]; )

		// Ensure path is not a location
		if (hr == S_OK && strPathSt[(len=strPathSt.length())-1] == '/')
			strPathSt.at(--len) = '\0';

		// For the pending load, remove trailing '/Value' specification
		// NOTE: Handle .../Value/Value
		if (	hr == S_OK && 
				(len > 6 && !WCASECMP(&strPathSt[len-6],L"/Value")) &&
				(len > 12 && WCASECMP(&strPathSt[len-12],L"/Value/Value")) )
			strPathSt.at(len-6) = '\0';

		// Perform a 'load' first on the path before the store.  This allows any
		// auto-instancing to occur before the store.
		CCLTRY ( pnSpc->get ( strPathSt, vR, NULL ) );
		CCLTRY ( _QISAFE((unkV=vR),IID_IReceptor,&pRecep) );

		// Receive value into location
		CCLTRY ( pRecep->receive ( NULL, L"Value", *pvUse ) );

		// Clean up
		_RELEASE(pRecep);

		// Result
		if (hr == S_OK)
			_EMT(Store,strPathSt);
		else if (strLoc.length() > 0)
			{
			dbgprintf ( L"This::receive:store failed:0x%x:%s:%s\r\n", hr, 
									(LPCWSTR)strLocAbs, (LPCWSTR)strDef );
			_EMT(NotFound,strLocAbs);
			}	// else

		}	// else if

	// Test - Test for existing namespace location.
	else if (_RCP(Test))
		{
		IDictionary	*pDctFNs	= NULL;
		adtString	strLocAbs;
		adtIUnknown	unkV;
		adtValue		vGet;

		// State check
		CCLTRYE ( strLoc.length() > 0,	ERROR_INVALID_STATE );

		// Ensure an absolute namespace is used
		CCLTRY ( nspcPathTo ( (pDctF != NULL) ? pDctF : pLocPar, strLoc, strLocAbs ) );

		// Obtain root of entire namespace
		CCLTRY ( pnSpc->get ( L"/", vGet, NULL ) );
		CCLTRY ( _QISAFE((unkV = vGet),IID_IUnknown,&pDctFNs) );

		// Test load
		CCLTRY ( nspcLoadPath ( pDctFNs, strLocAbs, vGet ) );

		// Clean up
		_RELEASE(pDctFNs);

		// Result
		if (hr == S_OK)
			_EMT(Test,strLocAbs);
		else
			_EMT(NotFound,strLocAbs);
		}	// else if

	// Remove/unload
	else if (_RCP(Remove))
		{
		IDictionary	*pDctFNs	= NULL;
		IDictionary *pDct		= NULL;
		IDictionary	*pDctPar	= NULL;
		IReceptor	*pRcp		= NULL;
		const WCHAR *pw		= NULL;
		adtValue		vRm;
		adtString	strLocAbs;
		adtIUnknown	unkV;
		adtString	strName;
		U32			len;

		// State check
		CCLTRYE ( strLoc.length() > 0,	ERROR_INVALID_STATE );

		// Ensure an absolute namespace is used
		CCLTRY ( nspcPathTo ( (pDctF != NULL) ? pDctF : pLocPar, strLoc, strLocAbs ) );

		// Obtain instance from the root namespace to avoid auto-instancing
		// invalid locations.

		// Obtain root of entire namespace
		CCLTRY ( pnSpc->get ( L"/", vRm, NULL ) );
		CCLTRY ( _QISAFE((unkV = vRm),IID_IUnknown,&pDctFNs) );

		// Retrieve instance
		CCLTRY ( nspcLoadPath ( pDctFNs, strLocAbs, vRm ) );

		// If instance is valid, remove from parent
		CCLTRY ( _QISAFE((unkV = vRm),IID_IDictionary,&pDct) );
		CCLTRY ( pDct->load ( strnRefPar, vRm ) );
		CCLTRY ( _QISAFE((unkV = vRm),IID_IDictionary,&pDctPar) );

		// Retrieve name of instance by isolating the last string in path
		if (hr == S_OK)
			{
			// Remove trailing slash
			if (strLocAbs[(len = strLocAbs.length())-1] == '/')
				strLocAbs.at(len-1) = '\0';

			// Name after last slash
			if ((pw = wcsrchr ( &strLocAbs.at(), '/' )) != NULL)
				strName = pw+1;
			else
				hr = adtValue::copy ( strLocAbs, strName );
			}	// if

		// Receive empty value into parent at name
		if (hr == S_OK && _QI(pDctPar,IID_IReceptor,&pRcp) == S_OK)
			pRcp->receive ( this, strName, adtValue() );

		// Clean up
		_RELEASE(pRcp);
		_RELEASE(pDctPar);
		_RELEASE(pDct);
		_RELEASE(pDctFNs);

		// Result
		if (hr == S_OK)
			_EMT(Remove,strLocAbs);
		else
			_EMT(NotFound,strLocAbs);
		}	// else if

	// Resolve path
	else if (_RCP(Resolve))
		{
		adtString	strLocAbs;

		// State check
		CCLTRYE ( strLoc.length() > 0, ERROR_INVALID_STATE );

//if (!WCASECMP(this->strnName,L"HitResv"))
//	dbgprintf ( L"Hi\r\n" );

		// Generate absolute namespace path
		CCLTRY ( nspcPathTo ( (pDctF != NULL) ? pDctF : pLocPar, strLoc, strLocAbs, pDctT ) );

		// Result
		if (hr == S_OK)
			_EMT(Resolve,strLocAbs);
		else
			_EMT(NotFound,strLoc);
		}	// else if

	// Temporal
/*	else if (prFt == pR)
		{
		adtString	strLocAbs;
		adtBool		bRec(v);

		// Previous iteration
		_RELEASE(pItt);

		// State check
		CCLTRYE ( strLoc.length() > 0,	ERROR_INVALID_STATE );

		// Ensure an absolute namespace is used
		CCLTRY ( nspcToAbs ( (pDctF != NULL) ? pDctF : pLocPar, strLoc, strLocAbs ) );

		// Iterate location
		CCLTRY ( pnSpc->iterate ( strLocAbs, &rng, &pItt ) );

		// Start at beginning
		CCLTRY ( pItt->begin() );

		// Next value
		if (hr == S_OK)	hr = receive ( prNt, v );
		else					peE->emit(adtInt(hr));
		}	// else if
	else if (prNt == pR)
		{
		EMITHDR	hdr;
		adtValue	vIt;

		// Obtain the next header and value
		CCLTRY(pItt->header ( &hdr ));
		CCLTRY(pItt->read ( vIt ) );

		// Next position
		CCLOK(pItt->next();)

		// Result
		if (hr == S_OK)
			{
			peSq->emit(adtLong(hdr.sequence));
			peNt->emit(vIt);
			}	// if
		else
			peE->emit(adtInt(hr));
		}	// else if
	else if (prTe == pR)
		{
//		ITemporal	*pTmp	= NULL;
//		IEmittert	*
		// State check
//		CCLTRYE ( strLoc.length() > 0,	ERROR_INVALID_STATE );

		// Access temporal interface

		// Access emitter

		// Result

		}	// else if
*/
	// State
	else if (_RCP(From))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pDctF);
		hr = _QISAFE(unkV,IID_IDictionary,&pDctF);
		}	// else if
	else if (_RCP(To))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pDctT);
		hr = _QISAFE(unkV,IID_IDictionary,&pDctT);
		}	// else if
	else if (_RCP(Value))
		hr = adtValue::copy ( v, vValue );
	else if (_RCP(Location))
		strLoc = v;
	else if (_RCP(Definition))
		strDef = v;
//	else if (prL == pR)
//		rng.lower	= adtLong(v);
//	else if (prU == pR)
//		rng.upper	= adtLong(v);
	else
		hr = ERROR_NO_MATCH;

/*
	// Emit local graph information
	if (prFire == pR)
		{
//		IADTDictionary	*pnCtxParent	= NULL;
//		IADTDictionary	*pnCtxShell		= NULL;
//		IADTDictionary	*pnCtxCatalog	= NULL;
//		adtIUnknownRef	unkrefV;
//		adtIUnknown		unkV;

		// Reference to graph object (avoid circular reference counts)
		CCLOK ( peGr->emit ( adtIUnknownRef(pnGr) ); )

//		CCLTRY ( _QISAFE(unkrefV,IID_IADTDictionary,&pnCtxShell) );
//		CCLOK  ( peShell->emit ( adtIUnknown(pnCtxShell) ); )
//		CCLTRY ( pnCtxShell->load ( strRefCatalog, unkV ) );
//		CCLTRY ( _QISAFE(unkV,IID_IADTDictionary,&pnCtxCatalog) );
//		CCLOK  ( peCat->emit ( adtIUnknown(pnCtxCatalog) ); )

		// Default stream source for environment
//		CCLTRY ( pnCtxShell->load ( strRefStreamSource, unkV ) );
//		CCLOK	 ( peStms->emit ( unkV ); )

		// Clean up
//		_RELEASE(pnCtxParent);
//		_RELEASE(pnCtxCatalog);
//		_RELEASE(pnCtxShell);
		}	// if

	// Get location
	else if (prGt == pR)
		{
		IUnknown		*pVal	= NULL;

		// State check
		CCLTRYE ( (pnSpc != NULL),			ERROR_INVALID_STATE );
		CCLTRYE ( (strLoc.length() > 0), ERROR_INVALID_STATE );

		// Debug
//		if (hr == S_OK && strLoc[0] == '.')
//			dbgprintf ( L"Hi\r\n" );

		// Request load
		CCLTRY ( pnSpc->get ( strLoc, &pVal, (pDctF != NULL) ? pDctF : pnGr, bCreate ) );

		// Result
		if (hr == S_OK)
			peGt->emit ( adtIUnknown(pVal) );
		else
			peNotF->emit ( strLoc );

		// Clean up
		_RELEASE(pVal);
		}	// if

	// Create a graph instance at 'strLoc' under 'pDctF' of definition 'strDef'
	else if (prCr == pR)
		{
		IUnknown		*pLoc	= NULL;
		IDictionary	*pDct	= NULL;
		adtString	strLocAbs;
		adtValue		vL;

		// State check
		CCLTRYE ( strDef.length() > 0,					ERROR_INVALID_STATE );
		CCLTRYE ( strLoc.length() > 0,					ERROR_INVALID_STATE );
		CCLTRYE ( strLoc[strLoc.length()-1] == '/',	ERROR_INVALID_STATE );
		CCLTRYE ( pnSpc != NULL,							ERROR_INVALID_STATE );

		// Ensure an absolute namespace is used
//		CCLTRY ( nspcToAbs ( (pDctF != NULL) ? pDctF : pnGr, strLoc, strLocAbs ) );

		// Debug
//		if (hr == S_OK && !WCASECMP(strLocAbs,L"/Instance/State/Root/Window/Viewer/Render/Instance/Elements/Instance_Description/"))
//			dbgprintf ( L"Hi" );

		// Obtain requested location
		CCLTRY ( pnSpc->get ( strLoc, &pLoc, (pDctF != NULL) ? pDctF : pnGr, true ) );

		// Remove the initialized flag and store the definition at the instance location
		CCLTRY ( _QI(pLoc,IID_IDictionary,&pDct) );
		CCLOK  ( pDct->remove( adtString(L"_Location") ); )
		CCLTRY ( pDct->store	( adtString(L"_Definition"), strDef ) );
		_RELEASE(pDct);
		_RELEASE(pLoc);

		// Obtain location again, this will trigger the creation
		CCLTRY ( pnSpc->get ( strLoc, &pLoc, (pDctF != NULL) ? pDctF : pnGr, true ) );

		// Result
		if (hr == S_OK)
			peGt->emit ( adtIUnknown(pLoc) );
		else
			{
			dbgprintf ( L"This::receive:Create:Error creating graph:%s\r\n", (LPCWSTR) strLocAbs );
			peNotF->emit ( strLocAbs );
			}	// else

		// Clean up
		_RELEASE(pLoc);
		}	// else if

	// Observe a location
	else if (prOb == pR)
		{
		IObserver	*pOb	= NULL;
		IObserved	*pObd	= NULL;
		adtIUnknown	unkV(v);

		// State check
		CCLTRYE ( (IUnknown *)(NULL) != unkV, ERROR_INVALID_STATE );

		// Wrap value in an observer
		CCLTRY ( COCREATE ( CLSID_Observer, IID_IObserver, &pOb ) );
		CCLTRY ( _QI(unkV,IID_IObserved,&pObd) );
		CCLTRY ( pOb->lookAt ( pObd ) );

		// Store observation in dictionary
		CCLTRY ( pDctOb->store ( unkV, adtIUnknown(pOb) ) );

		// Clean up
		_RELEASE(pObd);
		_RELEASE(pOb);
		}	// else if

	// Unobserve a location
	else if (prUnb == pR)
		{
		adtIUnknown	unkV(v);

		// State check
		CCLTRYE ( (IUnknown *)(NULL) != unkV, ERROR_INVALID_STATE );

		// Remove in case value is being observed
		CCLOK ( pDctOb->remove ( unkV ); )
		}	// else if

	// Unobserve all locations
	else if (prUnAll == pR)
		{
		// Release all observations
		pDctOb->clear();
		}	// else if

	// Get definition
	else if (prGtDf == pR)
		{
		IUnknown		*pVal		= NULL;
		IEmitter		*pEmit	= NULL;
		adtString	strDefLoc(LOC_NSPC_DEFS);
		adtValue		vDef;

		// State check
		CCLTRYE ( pnSpc != NULL,			ERROR_INVALID_STATE );
		CCLTRYE ( strDef.length() > 0,	ERROR_INVALID_STATE );
//		CCLOK ( dbgprintf ( L"This::receive::Load def:%s\r\n", (LPCWSTR)strDef ); )

		// Definitions are stored in a predefined layer of the namespace
		CCLTRY ( strDefLoc.append ( strDef ) );
		CCLOK  ( strDefLoc.at(strDefLoc.length()-1) = '\0'; )
//		CCLTRY ( strDefLoc.append ( L"OnFire" ) );
		CCLTRY ( pnSpc->get ( strDefLoc, &pVal, NULL, false ) );

		// Extract the definition
		CCLTRY ( _QI(pVal,IID_IEmitter,&pEmit) );
		CCLTRY ( pEmit->value ( vDef ) );

		// Clean up
		_RELEASE(pEmit);
		_RELEASE(pVal);

		// Result
		if (hr == S_OK)
			peDef->emit ( vDef );
		else
			peNotF->emit ( strDef );
		}	// else if

	// Resolve path
	else if (prRsv == pR)
		{
		adtString	strLocAbs;

		// State check
		CCLTRYE ( strLoc.length() > 0, ERROR_INVALID_STATE );

		// Generate absolute namespace path
		CCLTRY ( nspcToAbs ( (pDctF != NULL) ? pDctF : pnGr, strLoc, strLocAbs ) );

		// Result
		if (hr == S_OK)
			peRsv->emit ( strLocAbs );
		else
			peNotF->emit ( strLoc );
		}	// else if
*/
/*
	// Emit local graph information
	if (prFire == pR)
		{
//		IADTDictionary	*pnCtxParent	= NULL;
//		IADTDictionary	*pnCtxShell		= NULL;
//		IADTDictionary	*pnCtxCatalog	= NULL;
//		adtIUnknownRef	unkrefV;
//		adtIUnknown		unkV;

		// Load a reference to the graph object (avoid circular reference counts)
		CCLOK ( peGr->emit ( adtIUnknownRef((IUnknown **)&pnGr) ); )

//		CCLTRY ( _QISAFE(unkrefV,IID_IADTDictionary,&pnCtxShell) );
//		CCLOK  ( peShell->emit ( adtIUnknown(pnCtxShell) ); )
//		CCLTRY ( pnCtxShell->load ( strRefCatalog, unkV ) );
//		CCLTRY ( _QISAFE(unkV,IID_IADTDictionary,&pnCtxCatalog) );
//		CCLOK  ( peCat->emit ( adtIUnknown(pnCtxCatalog) ); )

		// Default stream source for environment
//		CCLTRY ( pnCtxShell->load ( strRefStreamSource, unkV ) );
//		CCLOK	 ( peStms->emit ( unkV ); )

		// Clean up
//		_RELEASE(pnCtxParent);
//		_RELEASE(pnCtxCatalog);
//		_RELEASE(pnCtxShell);
		}	// if

	// Load namespace location
	else if (prLd == pR)
		{
		adtValue		vL;
		adtIUnknown	vUnk;
		adtString	strLocAbs;

		// State check
		CCLTRYE ( strLoc.length() > 0, ERROR_INVALID_STATE );
		CCLTRYE ( pSpcRef != NULL, ERROR_INVALID_STATE );

//		if (hr == S_OK && wcsstr ( strLoc, L"OnValue" ) != NULL)
//			dbgprintf ( L"Hi!\r\n" );
//		if (strDef.length() > 0 && !WCASECMP ( strDef, L"Render/GDI/Viewer/" ))
//			dbgprintf ( L"Hi!\r\n" );

		// Ensure an absolute namespace is used
		CCLTRY ( nspcToAbs ( (pDctF != NULL) ? pDctF : pnGr, strLoc, strLocAbs ) );
		CCLOK ( dbgprintf ( L"This::receive:Load:%s (%s) (%s)\r\n", (LPCWSTR) strLocAbs, (LPCWSTR)strLoc, (LPCWSTR)strnName ); )
//		if (strLocAbs.length() > 0 && !WCASECMP ( strLocAbs, L"/Instance/State/" ))
//		if (!WCASECMP(strnName,L"LoadInstChk"))
//			dbgprintf ( L"Hi!\r\n" );
		// Request load
		CCLTRY ( pSpcRef->get ( strLocAbs, vL ) );

		// Result
		if (hr == S_OK)
			peLd->emit ( vL );
		else
			{
			dbgprintf ( L"This::receive:Location not found:%s\r\n", (LPCWSTR) strLocAbs );
			peNotF->emit ( strLocAbs );
			}	// else

		// Clean up
		}	// else if

	// Unload namespace location
	else if (prUn == pR)
		{
		// State check
		CCLTRYE ( strLoc.length() > 0, ERROR_INVALID_STATE );
		CCLTRYE ( pSpcRef != NULL, ERROR_INVALID_STATE );

		// Unload the location
		CCLTRY ( pSpcRef->unload ( strLoc ) );
		}	// else if


	// Resolve location
	else if (prRsv == pR)
		{
		adtString	strLocAbs;

		// State check
		CCLTRYE ( strLoc.length() > 0, ERROR_INVALID_STATE );

		// Generate absolute namespace path
		CCLTRY ( nspcToAbs ( (pDctF != NULL) ? pDctF : pnGr, strLoc, strLocAbs ) );

		// Result
		if (hr == S_OK)
			peRsv->emit ( strLocAbs );
		else
			peNotF->emit ( strLoc );
		}	// else if


	// Notifications
	else if (prNt == pR)
		{
		// State check
		CCLTRYE ( pSpcRef != NULL, ERROR_INVALID_STATE );

		// Set up namespace notifications
		CCLTRY ( pSpcRef->notify ( this, true ) );
		}	// else if
*//*
	else if (prDef == pR)
		adtValue::copy ( adtString(v), strDef );
*/


	return hr;
	}	// receive


/*
		// Creation location if desired
		// NOTE: Calling 'get' right away might create the wrong kind of graph if it is a location
		// that is mirrored to state space.  Ensure the proper graph instance is created
		// based on the specified definition.
		if (hr == S_OK && bCreate)
			{
			IDictionary	*pDct = NULL;
			adtString	strName,strPath;
			adtIUnknown	unkV;
			int			iIdx;

			// A graph definition is required for it to be placed in the namespace.
			CCLTRYE ( strDef.length() > 0, ERROR_INVALID_STATE );

			// Find the starting position of the last name in the path
			for (iIdx = strLocAbs.length()-2;hr == S_OK && iIdx >= 0;--iIdx)
				if (strLocAbs[iIdx] == '/')
					break;

			// Found slash ?
			CCLTRYE ( iIdx >= 0, ERROR_NOT_FOUND );

			// Generate name (minus trailing slash)
			CCLOK ( strName = &(strLocAbs[iIdx+1]); )
			CCLOK ( strName.at(strName.length()-1) = '\0'; )

			// Generate path up to name
			CCLTRY ( strLocAbs.substring ( 0, iIdx+1, strPath ) );

			// Ensure the namespace path exists up to the name
			CCLTRY ( pSpcRef->put ( strPath, adtValue() ) );

			// Obtain the location for use as parent of instance
			CCLTRY ( pSpcRef->get ( strPath, vL ) );
			CCLTRY ( _QISAFE((unkV = vL),IID_IDictionary,&pDct) );

			// Instance the graph at its location
			if (hr == S_OK)
				{
				// Only if location does not exist
				if (pDct->load ( strName, vL ) != S_OK)
					{
					dbgprintf ( L"This::receive:Create:%s at %s\r\n", (LPCWSTR)strDef, (LPCWSTR)strLocAbs );
					hr = pSpcRef->instance ( pDct, strLocAbs, strDef );
					}	// if
//				else
//					dbgprintf ( L"This::receive:Create:NOTE:Location already exists\r\n" );
				}	// if

			// Clean up
			_RELEASE(pDct);
			}	// if

*/
