////////////////////////////////////////////////////////////////////////
//
//								TEMPORAL.CPP
//
//					Implementation of the temporal node.
//
////////////////////////////////////////////////////////////////////////

#include "nspcl_.h"

Temporal :: Temporal ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	strLoc	= L"";
	pRoot		= NULL;
	pLocPar	= NULL;
	bRead		= true;
	}	// Temporal

HRESULT Temporal :: onAttach ( bool bAttach )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when Temporal behaviour is assigned to a node
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
		if (pnDesc->load(adtString(L"Location"),v) == S_OK)
			strLoc = v;
		if (pnDesc->load(adtString(L"ReadOnly"),v) == S_OK)
			bRead = adtBool(v);

		// Obtain reference to parent location
		CCLTRY ( pnLoc->load ( strnRefPar, v ) );
		CCLTRY ( _QISAFE((unkV=v),IID_IDictionary,&pLocPar) );
		if (hr == S_OK) pLocPar->Release();
		}	// if

	// Detach
	else
		{
		// Clean up
		pLocPar = NULL;
		_RELEASE(pRoot);
		}	// else

	return hr;
	}	// onAttach

HRESULT Temporal :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

	// Test - Test for existing temporal emitter.
	if (_RCP(Test))
		{
/*
		IDictionary	*pRootNs	= NULL;
		adtString	strLocAbs;
		adtIUnknown	unkV;
		adtValue		vGet;

		// State check
		CCLTRYE ( strLoc.length() > 0,	ERROR_INVALID_STATE );

		// Ensure an absolute namespace is used
		CCLTRY ( nspcToAbs ( (pRoot != NULL) ? pRoot : pnGr, strLoc, strLocAbs ) );

		// Obtain root of entire namespace
		CCLTRY ( pnSpc->get ( L"/", vGet, NULL ) );
		CCLTRY ( _QISAFE((unkV = vGet),IID_IUnknown,&pRootNs) );

		// Test load
		CCLTRY ( nspcLoadPath ( pRootNs, strLocAbs, vGet ) );

		// Clean up
		_RELEASE(pRootNs);

		// Result
		if (hr == S_OK)
			peTst->emit(strLocAbs);
		else
			peNotF->emit(strLocAbs);
*/
		}	// else if
/*
	// Temporal emitter access
	else if (_RCP(Emitter))
		{
		ITemporalImpl	*pTmp	= NULL;
		IDictionary		*pDct	= NULL;
		adtString		strLocAbs;

		// State check
		CCLTRYE ( strLoc.length() > 0,	ERROR_INVALID_STATE );

		// Ensure an absolute namespace is used
		CCLTRY ( nspcToAbs ( (pRoot != NULL) ? pRoot : pnGr, strLoc, strLocAbs ) );

		// Temporal interface
		CCLTRY ( pnSpc->temporal ( &pTmp ) );

		// Access location
		CCLTRY ( pTmp->location ( strLocAbs, !bRead, &pDct ) );

		// Result
		if (hr == S_OK)
			_EMT(Emitter,adtIUnknown(pDct));
		else
			_EMT(NotFound,strLocAbs);

		// Clean up
		_RELEASE(pDct);
		}	// else if
*/
	// Turn on/off recording of path
	else if (_RCP(Record))
		{
		adtString	strLocAbs;
		adtBool		bRec(v);

		// State check
		CCLTRYE ( strLoc.length() > 0,	ERROR_INVALID_STATE );

		// Ensure an absolute namespace is used
		CCLTRY ( nspcPathTo ( (pRoot != NULL) ? pRoot : pLocPar, strLoc, strLocAbs ) );

		// Record
		CCLTRY ( pnSpc->record ( strLocAbs, bRec ) );

		// Result
		if (hr == S_OK)
			_EMT(Record,strLocAbs);
		else
			{
			_EMT(NotFound,strLocAbs);
			dbgprintf ( L"Temporal::receive:record failed:0x%x:%s\r\n", hr, (LPCWSTR)strLocAbs );
			}	// else

		}	// else if

	// State
	else if (_RCP(Root))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pRoot);
		hr = _QISAFE(unkV,IID_IDictionary,&pRoot);
		}	// else if
	else if (_RCP(Location))
		strLoc = v;
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

