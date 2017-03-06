////////////////////////////////////////////////////////////////////////
//
//								SPEAK.CPP
//
//						Implementation of the 'speak' node
//
////////////////////////////////////////////////////////////////////////

#define INITGUID
#include "medial_.h"

#ifdef	_WIN32

Speak :: Speak ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for object.
	//
	//	PARAMETERS
	//		-	hInst is the application instance
	//
	////////////////////////////////////////////////////////////////////////

	// Setup
	pVoice	= NULL;
	}	// Speak

HRESULT Speak :: onAttach ( bool bAttach )
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
		adtValue vL;

		// Defaults
		pnDesc->load ( adtString(L"Value"), vSpk );

		// Object
		if (hr == S_OK)
			CoCreateInstance ( CLSID_SpVoice, NULL, CLSCTX_ALL,
										IID_ISpVoice, (void **) &pVoice );
		}	// if

	// Detach
	else
		{
		// Clean up
		_RELEASE(pVoice);
		adtValue::clear(vSpk);
		}	// else

	return hr;
	}	// onAttach

HRESULT Speak :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Speak
	if (_RCP(Fire))
		{
		const ADTVALUE *pvUse	= (!adtValue::empty(vSpk)) ? &vSpk : &v;
		adtString		strSpk;

		// State check
		CCLTRYE ( pVoice != NULL, ERROR_INVALID_STATE );

		// Convert to string version
		CCLTRY ( adtValue::toString ( *pvUse, strSpk ) );

		// Execute
		CCLOK ( pVoice->Speak ( strSpk, SPF_ASYNC, NULL ); )
		}	// if

	// State
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

#endif

