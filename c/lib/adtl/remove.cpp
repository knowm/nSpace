////////////////////////////////////////////////////////////////////////
//
//									REMOVE.CPP
//
//			Implementation of the remove container item node
//
////////////////////////////////////////////////////////////////////////

#include "adtl_.h"

Remove :: Remove ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pCont = NULL;
	}	// Remove

void Remove :: destruct ( void )
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

HRESULT Remove :: onAttach ( bool bAttach )
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
		pnDesc->load ( adtString(L"Key"), vKey );

	// Detach
	else
		{
		_RELEASE(pCont);
		}	// else

	return S_OK;
	}	// onAttach

HRESULT Remove :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Remove value by key, this works for both a dictionary and a container
	// NOTE: Removing an item from a container (list) can be very slow.
	if (_RCP(Fire))
		{
		// Key to use
		const ADTVALUE	*pK = (vKey.vtype != VTYPE_EMPTY) ? &vKey : &v;

		// State check
		CCLTRYE ( pCont != NULL, ERROR_INVALID_STATE );
		CCLTRYE ( !adtValue::empty(*pK), ERROR_INVALID_STATE );

		// Remove item
		if (hr == S_OK)	hr = pCont->remove ( (*pK) );

		// Result
		if (hr == S_OK)
			_EMT(Fire,(unkV = pCont));
		else
			_EMT(NotFound,(*pK));
		}	// if

	// Clear items
	else if (_RCP(Clear))
		{
		// Clear container
		if (pCont != NULL)	pCont->clear();
		else						hr = ERROR_INVALID_STATE;
		}	// else if

	// Container
	else if (_RCP(Container))
		{
		_RELEASE(pCont);
		hr = _QISAFE((unkV=v),IID_IContainer,&pCont);
		}	// else if
	else if (_RCP(Key))
		hr = adtValue::copy ( v, vKey );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive
