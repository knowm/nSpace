////////////////////////////////////////////////////////////////////////
//
//									STREAM.CPP
//
//					Implementation of the stream node
//
////////////////////////////////////////////////////////////////////////

#include "iol_.h"
#include <stdio.h>

StreamOp :: StreamOp ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pStm	= NULL;
	iOff	= 0;
	iOrg	= STREAM_SEEK_SET;
	}	// StreamOp

void StreamOp :: destruct ( void )
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
	_RELEASE(pStm);
	}	// destruct

HRESULT StreamOp :: onAttach ( bool bAttach )
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
		adtString	strOrg;

		// Defaults
		pnDesc->load ( adtString(L"Offset"), iOff );
		if (pnDesc->load ( adtString(L"Origin"), strOrg ) == S_OK)
			receive ( prOrigin, L"Value", strOrg );
		}	// if

	// Detach
	else
		{
		_RELEASE(pStm);
		}	// if

	return S_OK;
	}	// onAttach

HRESULT StreamOp :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Seek
	if (_RCP(Seek))
		{
		IByteStream	*pStmU	= pStm;
		U64			pos;
		adtIUnknown	unkV;

		// Stream to use
		_ADDREF(pStmU);
		if (hr == S_OK && pStmU == NULL)
			hr = _QISAFE((unkV=v),IID_IByteStream,&pStmU);
		
		// State check
		CCLTRYE ( pStmU != NULL, ERROR_INVALID_STATE );

		// Perform seek
//		if (hr == S_OK && iOrg == STREAM_SEEK_CUR)
//			dbgprintf ( L"Hi" );
		CCLTRY ( pStmU->seek ( iOff, iOrg, &pos ) );

		// Clean up
		_RELEASE(pStmU);

		// Result
		if (hr == S_OK)
			_EMT(Position,adtLong(pos) );
		else
			_EMT(Error,adtInt(hr) );

		// Clean up
		_RELEASE(pStmU);
		}	// if

	// Available
	else if (_RCP(Available))
		{
		IByteStream	*pStmU	= pStm;
		U64			avail		= 0;
		adtIUnknown	unkV;

		// Stream to use
		_ADDREF(pStmU);
		if (hr == S_OK && pStmU == NULL)
			hr = _QISAFE((unkV=v),IID_IByteStream,&pStmU);
		
		// State check
		CCLTRYE ( pStmU != NULL, ERROR_INVALID_STATE );

		// Available bytes
		CCLTRY ( pStmU->available ( &avail ) );

		// Result
		CCLOK ( _EMT(Available,adtLong(avail) ); )

		// Clean up
		_RELEASE(pStmU);
		}	// else if

	// Set size of stream
	else if (_RCP(Size))
		{
		IByteStream	*pStmU	= pStm;
		adtIUnknown	unkV;

		// Stream to use
		_ADDREF(pStmU);
		if (hr == S_OK && pStmU == NULL)
			hr = _QISAFE((unkV=v),IID_IByteStream,&pStmU);
		
		// State check
		CCLTRYE ( pStmU != NULL, ERROR_INVALID_STATE );

		// Set size of stream to specified offset
		CCLTRY ( pStmU->setSize ( iOff ) );

		// Result
		CCLOK ( _EMT(Size,adtIUnknown(pStmU)); )

		// Clean up
		_RELEASE(pStmU);
		}	// else if

	// State
	else if (_RCP(Stream))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pStm);
		hr = _QI(unkV,IID_IByteStream,&pStm);
		}	// else if
	else if (_RCP(Offset))
		iOff = adtInt(v);
	else if (_RCP(Origin))
		{
		adtString	strOrg(v);
		iOrg	= STREAM_SEEK_SET;
		if (strOrg.length() > 0)
			iOrg =	(!WCASECMP ( strOrg, L"Set" )) ? STREAM_SEEK_SET :
						(!WCASECMP ( strOrg, L"End" )) ? STREAM_SEEK_END : STREAM_SEEK_CUR;
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive
