////////////////////////////////////////////////////////////////////////
//
//									DEMUX.CPP
//
//					Implementation of the demultiplexer node
//
////////////////////////////////////////////////////////////////////////

#include "miscl_.h"
#include <stdio.h>

Demux :: Demux ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pDct		= NULL;
	pMap		= NULL;
	}	// Demux

HRESULT Demux :: onAttach ( bool bAttach )
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
	HRESULT		hr		= S_OK;
	IIt			*pIt	= NULL;
	IList			*pLst	= NULL;
	adtIUnknown	unkV;
	adtValue		vK,vV;
	adtString	strE,strV;

	// Attach
	if (bAttach)
		{
		// Key from context
		CCLTRY ( pnDesc->load ( adtString ( L"Key" ), vV ) );
		CCLOK  ( Key = vV; )

		// Load the list of desired values
		CCLTRY ( pnDesc->load ( adtString ( L"Values" ), vV ));
		CCLTRYE( (IUnknown *)(NULL) != (unkV=vV), E_UNEXPECTED );
		if (hr == S_OK && _QI(unkV,IID_IList,&pLst) != S_OK)
			hr = _QI((unkV=vV),IID_IDictionary,&pMap);

		// Iterate keys or values
		if (hr == S_OK && pLst != NULL)
			pLst->iterate ( &pIt );
		else if (hr == S_OK && pMap != NULL)
			pMap->keys ( &pIt );
		else
			hr = E_UNEXPECTED;

		// Add an emitter for each value
		while (hr == S_OK && pIt->read ( vK ) == S_OK)
			{
			IReceptor	*pR	= NULL;

			// Obtain mapped emitter name if specified
			if (pMap != NULL && pLst == NULL)
				hr = pMap->load ( vK, vV );
			else
				adtValue::copy ( vK, vV );

			// Convert value to string
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
		_RELEASE(pDct);
		_RELEASE(pMap);
		}	// else

	return hr;
	}	// onAttach

HRESULT Demux :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Fire
	if (_RCP(Fire))
		{
		IReceptor	*pR	= NULL;
		adtValue		cValue;

		// State check
		CCLTRYE ( pDct		!= NULL,	ERROR_INVALID_STATE );
		CCLTRYE ( !adtValue::empty(Key), ERROR_INVALID_STATE );

		// If anything fails from now assume 'default' case
		if (hr == S_OK)
			{
			adtValue		vV;

			// Load the value from the context
			CCLTRY ( pDct->load ( Key, cValue ) );

			// Map value if necessary
			if (pMap != NULL)
				hr = pMap->load ( cValue, vV );
			else
				hr = adtValue::copy ( cValue, vV );

			// Convert value to a string for access
			CCLTRY ( adtValue::toString ( vV, strV ) );
			CCLTRY ( strV.prepend(L"On") );

			// Load the connector
			CCLTRY ( pnLoc->load ( strV, vV ) );
			CCLTRY ( _QISAFE((unkV=vV),IID_IReceptor,&pR) );

			// Result
			if (hr == S_OK)	pR->receive ( this, prl, adtValue::empty(Value) ? v : (adtValue &) Value );
			else					_EMT(Default,adtValue::empty(Value) ? v : (adtValue &) Value );

			// Clean up
			_RELEASE(pR);
			}	// if

		}	// if

	// Dictionary
	else if (_RCP(Dictionary))
		{
		_RELEASE(pDct);
		hr = _QISAFE((unkV=v),IID_IDictionary,&pDct);
		}	// else if
	else if (_RCP(Key))
		hr = adtValue::copy ( v, Key );
	else if (_RCP(Default))
		hr = adtValue::copy ( v, Value );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

