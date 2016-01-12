////////////////////////////////////////////////////////////////////////
//
//									PERSIST.CPP
//
//					Implementation of the stream persistence node
//
////////////////////////////////////////////////////////////////////////

#include "iol_.h"
#include <stdio.h>

Persist :: Persist ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pStm	= NULL;
	pPrs	= NULL;
	}	// Persist

void Persist :: destruct ( void )
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
	_RELEASE(pPrs);
	}	// destruct

HRESULT Persist :: onAttach ( bool bAttach )
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

	// Clean up
	if (!bAttach)
		{
		_RELEASE(pStm);
		_RELEASE(pPrs);
		adtValue::clear(vSave);
		}	// if

	return S_OK;
	}	// onAttach

HRESULT Persist :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

	// Save
	if (_RCP(Save))
		{
		// Value to use
		const ADTVALUE	*pUseV	= (!adtValue::empty(vSave)) ? &vSave : &v;

		// State check
		CCLTRYE ( pStm != NULL, ERROR_INVALID_STATE );
		CCLTRYE ( pPrs != NULL, ERROR_INVALID_STATE );
//		CCLTRYE ( !adtValue::empty(*pUseV), ERROR_INVALID_STATE );

		// Save value to stream
		CCLTRY ( pPrs->save ( pStm, *pUseV ) );

		// Result
		if (hr == S_OK)
			_EMT(Save,adtIUnknown(pStm) );
		else
			_EMT(Error,adtInt(hr) );
		}	// if

	// Load
	else if (_RCP(Load))
		{
		adtValue	vL;

		// State check
		CCLTRYE ( pStm != NULL, ERROR_INVALID_STATE );
		CCLTRYE ( pPrs != NULL, ERROR_INVALID_STATE );

		// Load value from stream
		CCLTRY ( pPrs->load ( pStm, vL ) );

		// Result
		if (hr == S_OK)
			_EMT(Load,vL);
		else
			_EMT(Error,adtInt(hr) );
		}	// else if

	// State
	else if (_RCP(Stream))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pStm);
		hr = _QI(unkV,IID_IByteStream,&pStm);
		}	// else if
	else if (_RCP(Parser))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pPrs);
		hr = _QISAFE(unkV,IID_IStreamPersist,&pPrs);
		}	// else if
	else if (_RCP(Value))
		hr = adtValue::copy ( v, vSave );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive


