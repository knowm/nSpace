////////////////////////////////////////////////////////////////////////
//
//									KEYS.CPP
//
//				Implementation of the key management node
//
////////////////////////////////////////////////////////////////////////

#include "adtl_.h"
#include <stdio.h>

Keys :: Keys ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pVals		= NULL;
	pDict		= NULL;
	pKeys		= NULL;
	pRcps		= NULL;
	pKeysOut	= NULL;
	bAuto		= false;
	}	// Keys

void Keys :: destruct ( void )
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
	_RELEASE(pKeys);
	_RELEASE(pDict);
	_RELEASE(pVals);
	_RELEASE(pRcps);
	_RELEASE(pKeysOut);
	}	// destruct

HRESULT Keys :: onAttach ( bool bAttach )
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
	adtIUnknown	unkV;
	adtString	strEmit;
	adtValue		v;

	// Attach
	if (bAttach)
		{
		// Defaults
		if (pnDesc->load ( adtString(L"Auto"),	v ) == S_OK)
			bAuto = adtBool(v);

		// Create a dictionary to cache values
		CCLTRY(COCREATE(L"Adt.Dictionary",IID_IDictionary,&pVals));

		// Create a dictionary to keep track of receptors and output key names
		CCLTRY(COCREATE(L"Adt.Dictionary",IID_IDictionary,&pRcps));
		CCLTRY(COCREATE(L"Adt.Dictionary",IID_IDictionary,&pKeysOut));

		// Add receptors/emitter's for each specified key.
		if (pnDesc->load ( adtString(L"Keys"), unkV ) == S_OK)
			{
			// Iterate keys, add connectors 
			CCLTRY(_QISAFE(unkV,IID_IContainer,&pKeys));
			CCLTRY(pKeys->iterate(&pIt));
			while (hr == S_OK && pIt->read ( v ) == S_OK)
				{
				adtString	strIn,strOut;
				IReceptor	*pR	= NULL;

				// String version of provided key
				CCLTRY ( adtValue::toString ( v, strIn ) );

				// Add a connectors for the specified name
				CCLTRY ( pnSpc->connection ( pnLoc, strIn, L"Receptor", this, &pR ) );

				// Associate name with key
//				CCLTRY ( pRcps->store ( adtIUnknownRef(pR), v ) );
				CCLTRY ( pRcps->store ( adtLong((U64)pR), v ) );

				// Create outgoing value for result
				CCLTRY ( adtValue::copy ( strIn, strOut ) );
				CCLTRY ( strOut.prepend ( L"On" ) );
				CCLTRY ( pnSpc->connection ( pnLoc, strOut, L"Emitter", this, &pR ) );

				// Store mapping from input to output name
				CCLTRY ( pKeysOut->store ( strIn, strOut ) );

				// Clean up
				pIt->next();
				}	// while

			// Clean up
			_RELEASE(pIt);
			}	// if

		}	// if

	// Detach
	else
		{
		_RELEASE(pRcps);
		_RELEASE(pKeysOut);
		_RELEASE(pKeys);
		}	// else

	return hr;
	}	// onAttach

HRESULT Keys :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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
	HRESULT		hr		= S_OK;

	// Load keys
	if (_RCP(Load))
		{
		// Emit all the key value's out their respective receptors
		IIt			*pIt	= NULL;
		IReceptor	*pR	= NULL;
		adtString	strKey,strEmit;
		adtValue		vK,vV,vOut;
		adtIUnknown	unkV;
		adtString	strIn;

		// State check
		CCLTRYE ( pDict != NULL, ERROR_INVALID_STATE );
		CCLTRYE ( pKeys != NULL, ERROR_INVALID_STATE );

		// Iterate keys
		CCLTRY(pKeys->iterate(&pIt));
		while (hr == S_OK && pIt->read ( vK ) == S_OK)
			{
			// String version of key
			CCLTRY ( adtValue::toString ( vK, strIn ) );

			// Load associated output name
			CCLTRY ( pKeysOut->load ( strIn, vOut ) );

			// Load connector for key
			CCLTRY ( pnLoc->load ( vOut, unkV ) );
			CCLTRY ( _QISAFE(unkV,IID_IReceptor,&pR) );

			// If value exists, emit, otherwise not found
			if (hr == S_OK && pDict->load ( vK, vV ) == S_OK)
				pR->receive ( this, prl, vV );
			else
				{
				_EMT(NotFound,vK);
				}	// else

			// Clean up
			_RELEASE(pR);
			pIt->next();
			}	// while

		// Result
		_EMT(Load,(unkV = pDict));

		// Clean up
		_RELEASE(pIt);
		}	// if

	// Store keys
	else if (_RCP(Store))
		{
		// Store received values into the current dictionary
		IIt		*pIt	= NULL;
		adtValue	vK,vV;

		// State check
		CCLTRYE ( pDict != NULL, ERROR_INVALID_STATE );

		// Iterate keys
		CCLTRY(pVals->keys(&pIt));
		while (hr == S_OK && pIt->read ( vK ) == S_OK)
			{
			// Access value for key
			if (pVals->load ( vK, vV ) == S_OK)
				hr = pDict->store ( vK, vV );
			else
				_EMT(NotFound,vK );

			// Clean up
			pIt->next();
			}	// while

		// Clean up
		_RELEASE(pIt);

		// Result
		CCLOK ( _EMT(Store,(unkV = pDict)); )
		}	// else if

	// Copy keys
	else if (_RCP(Copy))
		{
		IDictionary	*pDst = NULL;
		IIt			*pIt	= NULL;
		adtValue		vK,vV;
		adtIUnknown	unkV(v);

		// State check, destination dictionary must be specified
		CCLTRYE( pDict != NULL, ERROR_INVALID_STATE );
		CCLTRY ( _QISAFE(unkV,IID_IDictionary,&pDst) );

		// Copy key/values
		CCLTRY(pKeys->iterate(&pIt));
		while (hr == S_OK && pIt->read ( vK ) == S_OK)
			{
			// Load key from source, store in destination
			if (pDict->load ( vK, vV ) == S_OK)
				hr = pDst->store ( vK, vV );
			else
				_EMT(NotFound,vK);

			// Clean up
			pIt->next();
			}	// while

		// Clean up
		_RELEASE(pIt);
		_RELEASE(pDst);

		// Result
		CCLOK ( _EMT(Copy,v); )
		}	// else if

	// Clear cached values
	else if (_RCP(Clear))
		pVals->clear();

	// Dictionary
	else if (_RCP(Dictionary))
		{
		// Previous dictionary
		_RELEASE(pDict);

		// New dictionary
		CCLTRYE	( (IUnknown *)(NULL) != (unkV=v), E_INVALIDARG );
		CCLTRY	( _QI(unkV,IID_IDictionary,&pDict) );
		}	// else if

	// Key value
	else
		{
		// A value can be fed directly into a receptor name of the desired key.
		// This will store the value in that key.

		// State check
		CCLTRYE ( pVals != NULL, ERROR_INVALID_STATE );

		// Access key name
//		CCLTRY ( pRcps->load ( adtIUnknownRef(pr), vRecep ) );
		CCLTRY ( pRcps->load ( adtLong((U64)pr), vRecep ) );

		// Cache value at key
		CCLTRY ( pVals->store ( vRecep, v ) );

		// Execute store if enabled
		if (hr == S_OK && bAuto)
			receive ( prStore, prl, v );

		// Notify graph of store
//		CCLOK ( peS->emit ( strRecep ); )
		}	// else

	return hr;
	}	// receive
