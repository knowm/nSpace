////////////////////////////////////////////////////////////////////////
//
//										CREATE.CPP
//
//					Implementation of the object creation node
//
////////////////////////////////////////////////////////////////////////

#include "miscl_.h"

Create :: Create ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	}	// Create

HRESULT Create :: onAttach ( bool bAttach )
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
	HRESULT	hr = S_OK;

	// Attach
	if (bAttach)
		{
		adtValue		v;

		// Default Id ?
		if (pnDesc->load ( adtString(L"Id"), v ) == S_OK)
			receive ( prId, L"Value", v );
		}	// if

	return hr;
	}	// onAttach

HRESULT Create :: onReceive ( IReceptor *pr, const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	FROM	IBehaviour
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
		IUnknown	*pUnk	= NULL;

		// State check
		CCLTRYE ( strId.length() > 0, ERROR_INVALID_STATE );

//if (IsEqualGUID(clsid,CLSID_Dist))
//	dbgprintf ( L"Hi\r\n" );
		// Create object
		if (hr == S_OK)
			{
			// Assume internal string based object Id
			CCLTRY ( cclCreateObject ( strId, NULL, IID_IUnknown, (void **) &pUnk ) );
			#ifdef	_WIN32
			// For Windows, check if object Id is a COM object
			if (hr != S_OK)
				{
				CLSID	clsid;

				// Reset error status
				hr = S_OK;

				// Class Id specified directly ?
				if (hr == S_OK && strId[0] == '{')
					hr = CLSIDFromString ( strId, &clsid );

				// Assume ProgId
				else 
					{
					#ifdef	_WIN32
					// Look up the prog Id
					CCLTRY ( CLSIDFromProgID ( strId, &clsid ) );
					#else
					hr = E_NOTIMPL;
					#endif
				
					}	// else if

				// Attempt creation again
				CCLTRY ( CoCreateInstance ( clsid, NULL, CLSCTX_ALL, IID_IUnknown, (void **) &pUnk ) );
				}	// if
			#endif
			}	// if

		// Result
		if (hr == S_OK)	_EMT(Fire,(unkV=pUnk) );
		else					_EMT(Error,adtInt(hr) );

		// Clean up
		_RELEASE(pUnk);

		// Debug
		if (hr != S_OK)
			dbgprintf ( L"Miscn::Create:Unable to create object:%s:0x%x\r\n", (LPCWSTR)strId, hr );
		}	// if

	// State
	else if (_RCP(Id))
		hr = adtValue::copy ( adtString(v), strId );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive
