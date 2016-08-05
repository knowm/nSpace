////////////////////////////////////////////////////////////////////////
//
//									STMCOPY.CPP
//
//					Implementation of the copy StreamCopy node
//
////////////////////////////////////////////////////////////////////////

#include "iol_.h"
#include <stdio.h>

StreamCopy :: StreamCopy ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pStmSrc	= NULL;
	pStmDst	= NULL;
	iSz		= 0;
	}	// StreamCopy

void StreamCopy :: destruct ( void )
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
	_RELEASE(pStmSrc);
	_RELEASE(pStmDst);
	}	// destruct

HRESULT StreamCopy :: onAttach ( bool bAttach )
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

	// Attach
	if (bAttach)
		{
		adtValue	v;

		// Defaults
		if (pnDesc->load ( adtString(L"Size"), v ) == S_OK)
			iSz = v;
		}	// if

	// Detach
	else
		{
		_RELEASE(pStmSrc);
		_RELEASE(pStmDst);
		}	// if

	return S_OK;
	}	// onAttach

HRESULT StreamCopy :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Copy
	if (_RCP(Fire))
		{
		U64	sz;

		// State check
		CCLTRYE ( (pStmDst != NULL), ERROR_INVALID_STATE );
		CCLTRYE ( (pStmSrc != NULL), ERROR_INVALID_STATE );

		// Trigger copy
		CCLTRY ( pStmSrc->copyTo ( pStmDst, iSz, &sz ) );

		// Result
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pStmDst) );
		else
			_EMT(Error,adtInt(hr) );
		}	// if

	// State
	else if (_RCP(Source))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pStmSrc);
		hr = _QI(unkV,IID_IByteStream,&pStmSrc);
		}	// else if
	else if (_RCP(Destination))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pStmDst);
		hr = _QI(unkV,IID_IByteStream,&pStmDst);
		}	// else if
	else if (_RCP(Size))
		iSz = adtInt(v);
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive
