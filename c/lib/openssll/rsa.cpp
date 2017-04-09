////////////////////////////////////////////////////////////////////////
//
//									RSAImpl.CPP
//
//					Implementation of the OpenSSL RSAImpl node
//
////////////////////////////////////////////////////////////////////////

#include "openssll_.h"

// Globals
extern libSSL	libS;

RSAImpl :: RSAImpl ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	}	// RSAImpl

HRESULT RSAImpl :: onAttach ( bool bAttach )
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
	adtValue	vL;

	// Library reference
	if (bAttach)
		{
		// Defaults
		if (pnDesc->load ( adtString(L"Bits"), vL ) == S_OK)
			iBits = vL;

		// Library reference
		libS.AddRef();
		}	// if
	else
		{
		// Lbrary reference
		libS.Release();
		}	// else

	return S_OK;
	}	// onAttach

HRESULT RSAImpl :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Key generation
	if (_RCP(Generate))
		{
		IThis	*ro	= NULL;
		RSA	*r		= NULL;

		// State check
		CCLTRYE ( libS.bValid == true,	ERROR_INVALID_STATE );
		CCLTRYE ( iBits > 0,					ERROR_INVALID_STATE );

		// Generate key pair
		CCLTRYE( (r = RSA_generate_key ( iBits, RSA_F4, NULL, NULL )) != NULL,
						libS.errors(L"RSAImpl::receive:Generate") );

		// Result
		if (hr == S_OK)
			_EMT(Generate,adtIUnknown((ro = new objSSL(r))) );
		else
			_EMT(Error,adtInt(hr) );

		// Clean up
		_RELEASE(ro);
		}	// if

	// State
	else if (_RCP(Bits))
		iBits = v;
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive


