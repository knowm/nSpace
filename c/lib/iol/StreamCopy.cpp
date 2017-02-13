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
	punkSrc	= NULL;
	punkDst	= NULL;
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
	_RELEASE(punkSrc);
	_RELEASE(punkDst);
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
		_RELEASE(punkSrc);
		_RELEASE(punkDst);
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
		IByteStream		*pStmSrc		= NULL;
		IByteStream		*pStmDst		= NULL;
		IMemoryMapped	*pMemSrc		= NULL;
		IMemoryMapped	*pMemDst		= NULL;
		void				*pvMemSrc	= NULL;
		void				*pvMemDst	= NULL;
		U64				sz;

		// State check
		CCLTRYE ( (punkDst != NULL), ERROR_INVALID_STATE );
		CCLTRYE ( (punkSrc != NULL), ERROR_INVALID_STATE );

		// Determine which interface each object supports
		CCLOK ( _QI(punkSrc,IID_IMemoryMapped,&pMemSrc); )
		CCLOK ( _QI(punkSrc,IID_IByteStream,&pStmSrc); )
		CCLOK ( _QI(punkDst,IID_IMemoryMapped,&pMemDst); )
		CCLOK ( _QI(punkDst,IID_IByteStream,&pStmDst); )

		// Perform copy
		if (hr == S_OK)
			{
			// Source is a stream
			if (pStmSrc != NULL)
				{
				// Stream to stream
				if (pStmDst != NULL)
					hr = pStmSrc->copyTo ( pStmDst, iSz, &sz );

				// Stream to memory mapped
				else if (pMemDst != NULL)
					hr = E_NOTIMPL;
				}	// if

			// Source it memory mapped
			else if (pMemSrc != NULL)
				{
				// Memory mapped to stream
				if (pStmDst != NULL)
					{
					U32	nCpy = iSz;
					U32	nChk;

					// Size of source data
					CCLTRY ( pMemSrc->getSize(&nChk) );

					// Adjust
					if (hr == S_OK && (nCpy == 0 || nCpy > nChk))
						nCpy = nChk;

					// Access bits
					CCLTRY ( pMemSrc->lock ( 0, 0, &pvMemSrc, NULL ) );

					// Write block to stream
					CCLTRY ( pStmDst->write ( pvMemSrc, nCpy, NULL ) );
					}	// if

				// Memory mapped to memory mapped
				else if (pMemDst != NULL)
					{
					U32	nChk,nCpy = iSz;

					// Size of source data
					CCLTRY ( pMemSrc->getSize(&nChk) );

					// Adjust
					if (hr == S_OK && (nCpy == 0 || nCpy > nChk))
						nCpy = nChk;

					// Ensure destination has enough room
					CCLTRY ( pMemDst->getSize ( &nChk ) );
					if (hr == S_OK && nChk < nCpy)
						hr = pMemDst->setSize ( nCpy );

					// Access bits
					CCLTRY ( pMemSrc->lock ( 0, 0, &pvMemSrc, NULL ) );
					CCLTRY ( pMemDst->lock ( 0, 0, &pvMemDst, NULL ) );

					// Copy data
					CCLOK ( memcpy ( pvMemDst, pvMemSrc, nCpy ); )
					}	// else if

				}	// else if
			}	// if

		// Result
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(punkDst) );
		else
			_EMT(Error,adtInt(hr) );

		// Clean up
		_RELEASE(pStmDst);
		_UNLOCK(pMemDst,pvMemDst);
		_RELEASE(pMemDst);
		_RELEASE(pStmSrc);
		_UNLOCK(pMemSrc,pvMemSrc);
		_RELEASE(pMemSrc);
		}	// if

	// State
	else if (_RCP(Source))
		{
		adtIUnknown	unkV(v);
		_RELEASE(punkSrc);
		hr = _QI(unkV,IID_IUnknown,&punkSrc);
		}	// else if
	else if (_RCP(Destination))
		{
		adtIUnknown	unkV(v);
		_RELEASE(punkDst);
		hr = _QI(unkV,IID_IUnknown,&punkDst);
		}	// else if
	else if (_RCP(Size))
		iSz = adtInt(v);
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive
