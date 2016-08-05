////////////////////////////////////////////////////////////////////////
//
//									DECODE.CPP
//
//					Implementation of the decoder node
//
////////////////////////////////////////////////////////////////////////

#include "miscl_.h"
#include <stdio.h>

Decode :: Decode ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pMap	= NULL;
	}	// Decode

void Decode :: destruct ( void )
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
	_RELEASE(pMap);
	}	// destruct

HRESULT Decode :: onAttach ( bool bAttach )
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
	HRESULT		hr			= S_OK;
	IIt			*pIt		= NULL;
	IList			*pLst		= NULL;
	adtIUnknown	unkV;
	adtValue		vK,vV;
	adtString	strE;

//if (!WCASECMP(strnName,L"DecodeType"))
//	dbgprintf ( L"Hi\r\n" );
	// Attach
	if (bAttach)
		{
		// Load the decode list or dictionary
		CCLTRY ( pnDesc->load ( adtString ( L"Values" ), vV ));
		CCLTRYE( (IUnknown *)(NULL) != (unkV=vV), E_UNEXPECTED );
		if (hr == S_OK && _QI(unkV,IID_IList,&pLst) != S_OK)
			hr = _QI(unkV,IID_IDictionary,&pMap);

		// Iterate keys or values
		if (hr == S_OK && pLst != NULL)
			pLst->iterate ( &pIt );
		else if (hr == S_OK && pMap != NULL)
			pMap->keys ( &pIt );
		else
			hr = E_UNEXPECTED;

		// Create an emitter for each value
		while (hr == S_OK && pIt->read ( vK ) == S_OK)
			{
			IReceptor	*pR	= NULL;

			// Obtain mapped emitter name if specified
			if (pMap != NULL && pLst == NULL)
				hr = pMap->load ( vK, vV );
			else
				adtValue::copy ( vK, vV );

			// Convert value to string for emitter name
			CCLTRY ( adtValue::toString ( vV, strE ) );

			// Add an emitter with the specified name
			CCLTRY ( strE.prepend ( L"On" ) );
			CCLTRY ( pnSpc->connection ( pnLoc, strE, L"Emitter", this, &pR ) );

			// Clean up
			pIt->next();
			}	// while

		// Clean up
		_RELEASE(pIt);
		_RELEASE(pLst);

		// No reason to fail attach
		hr = S_OK;
		}	// if

	// Detach
	else
		{
		// Clean up
		_RELEASE(pMap);
		}	// else

	return hr;
	}	// onAttach

HRESULT Decode :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

//if (!WCASECMP(strnName,L"DecodeType"))
//	dbgprintf ( L"Hi\r\n" );

	// Fire
	if (_RCP(Fire))
		{
		IReceptor	*pR	= NULL;

		// State check
		CCLTRYE ( adtValue::empty(Select) == false,	ERROR_INVALID_STATE );

		// If anything fails from now assume 'default' case
		if (hr == S_OK)
			{
			// Map value if necessary
			if (pMap != NULL)
				hr = pMap->load ( Select, vV );
			else
				hr = adtValue::copy ( Select, vV );

			// Generate emitter string
			CCLTRY ( adtValue::toString ( vV, strV ) );

			// Load the emitter itself
			CCLTRY ( strV.prepend(L"On") );
			CCLTRY ( pnLoc->load ( strV, vV ) );
			CCLTRY ( _QISAFE((unkV=vV),IID_IReceptor,&pR) );

			// Result
			if (hr == S_OK)	pR->receive ( this, prl, adtValue::empty(Value) ? v : (adtValue &) Value );
			else					_EMT(Default,adtValue::empty(Value) ? v : (adtValue &) Value );

			}	// if
		}	// if

	// Context
	else if (_RCP(Select))
		hr = adtValue::copy ( v, Select );
	else if (_RCP(Value))
		hr = adtValue::copy ( v, Value );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive
