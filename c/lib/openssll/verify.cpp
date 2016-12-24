////////////////////////////////////////////////////////////////////////
//
//									VERIFY.CPP
//
//			Implementation of the OpenSSL EVP sign verify node
//
////////////////////////////////////////////////////////////////////////

#include "openssll_.h"

// Globals
extern libSSL	libS;

EVPVerify :: EVPVerify ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pctx	= NULL;
	pStm	= NULL;
	pKey	= NULL;
	}	// EVPVerify

HRESULT EVPVerify :: onAttach ( bool bAttach )
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

	// Attach
	if (bAttach)
		{
		// Defaults
		if (pnDesc->load ( adtString(L"Type"), vL ) == S_OK)
			strType = vL;

		// Library reference
		libS.AddRef();
		}	// if

	// Detach
	else
		{
		// Clean up
		if (pctx != NULL)
			{
			libS.evp_ctx_free ( pctx );
			pctx	= NULL;
			}	// if

		// Lbrary reference
		libS.Release();
		}	// else

	return S_OK;
	}	// onAttach

HRESULT EVPVerify :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Verify begin
	if (_RCP(Begin))
		{
		const EVP_MD	*pt	= NULL;

		// State check
		CCLTRYE ( libS.bValid == true,	ERROR_INVALID_STATE );

		// Previous context
		if (pctx != NULL)
			{
			libS.evp_ctx_free ( pctx );
			pctx	= NULL;
			}	// if

		// Initialize context
		CCLTRYE ( (pctx = libS.evp_ctx_new ()) != NULL, E_OUTOFMEMORY );

		// Message digest type
		CCLTRYE ( (pt = 
						(!WCASECMP(strType,L"SHA1"))	? libS.evp_sha1() : 
						(!WCASECMP(strType,L"MD5"))	? libS.evp_md5() : 
						NULL) != NULL, E_INVALIDARG );

		// Initialize signing
		CCLTRYE ( libS.evp_digestinit ( pctx, pt ) != 0, 
						libS.errors(L"EVPVerify::receive:Begin:DigestInit") );

		// Result
		if (hr == S_OK)
			_EMT(Begin,adtInt() );
		else
			_EMT(Error,adtInt(hr) );
		}	// if

	// Verify update
	else if (_RCP(Update))
		{
		U64	sz;

		// State check
		CCLTRYE ( libS.bValid == true,	ERROR_INVALID_STATE );
		CCLTRYE ( pStm != NULL,				ERROR_INVALID_STATE );
		CCLTRYE ( pctx != NULL,				ERROR_INVALID_STATE );

		// Move stream data through signing until all data is exhauseted
		while (hr == S_OK && pStm->read ( cBfr, sizeof(cBfr), &sz ) == S_OK && sz > 0)
			{
			// Process block
			CCLTRYE ( libS.evp_digestup ( pctx, cBfr, (unsigned int) sz ) != 0,
							libS.errors(L"EVPVerify::receive:Update:DigestUpdate") );
			}	// while

		// Result
		if (hr == S_OK)
			_EMT(Update,adtIUnknown(pStm) );
		else
			_EMT(Error,adtInt(hr) );
		}	// else if

	// Verify final
	else if (_RCP(Final))
		{
		unsigned char	*pcsig	= NULL;
		objSSL			*obj		= NULL;
		EVP_PKEY			*key		= NULL;
		U64				sz;

		// State check
		CCLTRYE ( libS.bValid == true,	ERROR_INVALID_STATE );
		CCLTRYE ( pStm != NULL,				ERROR_INVALID_STATE );
		CCLTRYE ( pctx != NULL,				ERROR_INVALID_STATE );
		CCLTRYE ( pKey != NULL,				ERROR_INVALID_STATE );

		// Obtain key object and assign to EVP key
		CCLTRY	( pKey->getThis ( (void **) &obj ) );
		CCLTRYE	( (key = libS.evp_pkey_new()) != NULL, E_OUTOFMEMORY );
		if (hr == S_OK)
			{
			if (obj->rsa != NULL)
				{
				CCLTRYE	( libS.evp_pkey_set_rsa ( key, obj->rsa ) != 0,
								libS.errors(L"EVPVerify::receive:Final:SetRSA") );
				}	// if
			else
				hr = E_INVALIDARG;
			}	// if

		// Size the stream from current position.
		CCLTRY ( pStm->available ( &sz ) );

		// Allocate memory for the signature
		CCLTRYE ( (pcsig = (unsigned char *) _ALLOCMEM((U32)sz)) != NULL, E_OUTOFMEMORY );

		// Read in signature
		CCLTRY ( pStm->read ( pcsig, sz, &sz ) );

		// Compute signature
		CCLTRYE ( libS.evp_verifyfinal ( pctx, pcsig, (unsigned int)sz, key ) != 0,
						libS.errors(L"EVPVerify::receive:Final:SignFinal") );

		// Result
		if (hr == S_OK)
			_EMT(Final,adtIUnknown(pStm) );
		else
			_EMT(Error,adtInt(hr) );

		// Clean up
		if (key != NULL)
			libS.evp_pkey_free(key);
		_FREEMEM(pcsig);
		if (pctx != NULL)
			{
			libS.evp_ctx_free ( pctx );
			pctx	= NULL;
			}	// if
		}	// else if

	// State
	else if (_RCP(Stream))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pStm);
		_QISAFE(unkV,IID_IByteStream,&pStm);
		}	// else if
	else if (_RCP(Key))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pKey);
		_QISAFE(unkV,IID_IThis,&pKey);
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive


