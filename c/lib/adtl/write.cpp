////////////////////////////////////////////////////////////////////////
//
//									WRITE.CPP
//
//					Implementation of the write node
//
////////////////////////////////////////////////////////////////////////

#include "adtl_.h"

Write :: Write ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pLst	= NULL;
	}	// Write

void Write :: destruct ( void )
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

HRESULT Write :: onAttach ( bool bAttach )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the behaviour is attached to a node.
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
		pnDesc->load ( strnRefVal, vValue );
		}	// if

	// Detach
	else
		{
		_RELEASE(pLst);
		}	// else

	return S_OK;
	}	// onAttach

HRESULT Write :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

	// Write value by key in context
	if (_RCP(Fire))
		{
		// Value to use
		const ADTVALUE	*pV = (vValue.vtype != VTYPE_EMPTY) ? &vValue : &v;

		// State check
		CCLTRYE ( pLst != NULL, ERROR_INVALID_STATE );
		CCLTRYE ( pV != NULL && !adtValue::empty(*pV), ERROR_INVALID_STATE );

//if (!WCASECMP(this->strnName,L"WriteTxLst"))
//	dbgprintf ( L"Hi\r\n" );
		// Write value
		CCLTRY ( pLst->write ( *pV ) );

		// Result
		CCLOK ( _EMT(Fire,(unkV=pLst)); )
		}	// if

	// Parameters
	else if (_RCP(List))
		{
		_RELEASE(pLst);
		hr = _QISAFE((unkV=v),IID_IList,&pLst);
		}	// else if
	else if (_RCP(Value))
		hr = adtValue::copy ( v, vValue );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive
