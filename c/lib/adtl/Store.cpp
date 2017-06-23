////////////////////////////////////////////////////////////////////////
//
//									STORE.CPP
//
//					Implementation of the store node
//
////////////////////////////////////////////////////////////////////////

#include "adtl_.h"

Store :: Store ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pDct	= NULL;
	}	// Store

void Store :: destruct ( void )
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

HRESULT Store :: onAttach ( bool bAttach )
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
		pnDesc->load ( strnRefKey,	vKey );
		pnDesc->load ( strnRefVal, vValue );
		}	// if

	// Detach
	else
		{
		_RELEASE(pDct);
		}	// else

	return S_OK;
	}	// onAttach

HRESULT Store :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Store value by key in context
	if (_RCP(Fire))
		{
		// Value to use
		const ADTVALUE	*pV = (vValue.vtype != VTYPE_EMPTY) ? &vValue : &v;

		// State check
		CCLTRYE ( pDct != NULL,						ERROR_INVALID_STATE );
		CCLTRYE ( vKey.vtype != VTYPE_EMPTY,	ERROR_INVALID_STATE );

		// Debug
//		if (!WCASECMP(strnName,L"StoreValSt"))
//			dbgprintf ( L"Hi\r\n" );
/*
		// Sanity check
		#ifdef	_DEBUG
		if (hr == S_OK && pV->vtype == VTYPE_UNK && pV->punk != NULL)
			{
			// Sanity check
			IUnknown	*punkSrc = NULL;
			IUnknown	*punkDst = NULL;
			if (	_QI(pDct,IID_IUnknown,&punkSrc) == S_OK &&
					_QI(pV->punk,IID_IUnknown,&punkDst) == S_OK &&
					punkSrc == punkDst )
				{
				adtString	vName;
				hr = ERROR_INVALID_STATE;
				pnDesc->load ( adtString(STR_NSPC_NAME), vName );
				dbgprintf ( L"Store::receiveFire:Error, recursive dictionary storage denied(%s)\n",
									(LPCWSTR) vName );
				}	// if
			_RELEASE(punkSrc);
			_RELEASE(punkDst);
			}	// if
		#endif
*/
		// Perform operation
		if (hr == S_OK)
			{
			// Attempt store
			hr = pDct->store ( vKey, *pV );

			// Debug
			if (hr != S_OK)
				dbgprintf ( L"Store::receiveFire:Store failed:%s(0x%x)\r\n", (LPCWSTR)strnName, hr );
			}	// if

		// Result
		CCLOK ( _EMT(Fire,adtIUnknown(pDct) ); )
		}	// if

	// Parameters
	else if (_RCP(Dictionary))
		{
		adtIUnknown		unkV(v);
		_RELEASE(pDct);
		hr = _QISAFE(unkV,IID_IDictionary,&pDct);
		}	// else if
	else if (_RCP(Key))
		hr = adtValue::copy ( v, vKey );
	else if (_RCP(Value))
		hr = adtValue::copy ( v, vValue );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

