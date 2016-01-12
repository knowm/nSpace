////////////////////////////////////////////////////////////////////////
//
//									STMSRC.CPP
//
//					Implementation of the stream source node
//
////////////////////////////////////////////////////////////////////////

#include "iol_.h"
#include <stdio.h>

StreamSource :: StreamSource ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pOpt			= NULL;	
	pSrc			= NULL;
	pStm			= NULL;
	pStmsIt		= NULL;
	strRefLoc	= L"Location";
	}	// StreamSource

void StreamSource :: destruct ( void )
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

HRESULT StreamSource :: onAttach ( bool bAttach )
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
	HRESULT			hr		= S_OK;
	adtIUnknown		unkV;

	// Attach
	if (bAttach)
		{
		// Default location
		pnDesc->load ( strRefLoc, strLoc );

		// Default options
		if (pnDesc->load ( adtString(L"Options"), unkV ) == S_OK)
			{
			CCLTRY(_QISAFE(unkV,IID_IDictionary,&pOpt));
			}	// if
		else
			{
			CCLTRY(COCREATE(L"Adt.Dictionary",IID_IDictionary,&pOpt));
			}	// else
		}	// if
	
	// Detach
	else
		{
		_RELEASE(pStmsIt);
		_RELEASE(pStm);
		_RELEASE(pSrc);
		_RELEASE(pOpt);
		}	// else

	return S_OK;
	}	// onAttach

HRESULT StreamSource :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

	// Open
	if (_RCP(Open))
		{
		IUnknown	*pStmOp	= NULL;

		// State check
		CCLTRYE ( strLoc.length() > 0, ERROR_INVALID_STATE );
		CCLTRYE ( pOpt != NULL, ERROR_INVALID_STATE );
		CCLTRYE ( pSrc != NULL, ERROR_INVALID_STATE );

		// Store location in options and access stream
		CCLTRY ( pOpt->store ( strRefLoc, strLoc ) );
		CCLTRY ( pSrc->open ( pOpt, &pStmOp ) );

		// Result
		if (hr == S_OK)	_EMT(Open,adtIUnknown(pStmOp) );
		else					_EMT(Error,adtInt(hr) );

		// Clean up
		_RELEASE(pStmOp);
		}	// if

	// First/next stream
	else if (_RCP(First) || _RCP(Next))
		{
		adtValue	vIt;

		// State check
		CCLTRYE ( pSrc != NULL, ERROR_INVALID_STATE );
		CCLTRYE ( strLoc.length() > 0, ERROR_INVALID_STATE );

		// First time ?
		if (_RCP(First))
			{
			// New iterate
			_RELEASE(pStmsIt);
			CCLTRY(pSrc->locations(strLoc,&pStmsIt));

			// Ensure beginning
			CCLTRY(pStmsIt->begin());
			}	// if

		// State check
		CCLTRYE ( pStmsIt != NULL, ERROR_INVALID_STATE );

		// Emit next stream info.
		if (hr == S_OK && pStmsIt->read ( vIt ) == S_OK)
			{
			// Result
			_EMT(Next,vIt);

			// Move to next stream
			pStmsIt->next();
			}	// if
		else
			_EMT(End,adtInt());
		}	// else if

	// Status
	else if (_RCP(Status))
		{
		IDictionary	*pDct	= NULL;

		// State check
		CCLTRYE ( pSrc != NULL, ERROR_INVALID_STATE );
		CCLTRYE ( strLoc.length() > 0, ERROR_INVALID_STATE );

		// Create dictionary to receive status
		CCLTRY ( COCREATE(L"Adt.Dictionary", IID_IDictionary, &pDct ) );

		// Obtain status
		CCLOK  ( dbgprintf ( L"StreamSource::receive:Status:%s\r\n", (LPCWSTR)strLoc ); )
		CCLTRY ( pSrc->status ( strLoc, pDct ) );

		// Result
		if (hr == S_OK)
			_EMT(Status,adtIUnknown(pDct));
		else
			_EMT(Error,strLoc);

		// Clean up
		_RELEASE(pDct);
		}	// else if

	// State
	else if (_RCP(Options))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pOpt);
		hr = _QI(unkV,IID_IDictionary,&pOpt);
		}	// else if
	else if (_RCP(Source))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pSrc);
		hr = _QISAFE(unkV,IID_ILocations,&pSrc);
		}	// else if
	else if (_RCP(Location))
		hr = adtValue::copy ( adtString(v), strLoc );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

