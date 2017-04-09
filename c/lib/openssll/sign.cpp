////////////////////////////////////////////////////////////////////////
//
//									SIGN.CPP
//
//					Implementation of the OpenSSL EVP sign node
//
////////////////////////////////////////////////////////////////////////

#include "openssll_.h"

// Globals
extern libSSL	libS;

EVPSign :: EVPSign ( void )
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
	}	// EVPSign

HRESULT EVPSign :: onAttach ( bool bAttach )
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
			EVP_MD_CTX_destroy ( pctx );
			pctx	= NULL;
			}	// if

		// Lbrary reference
		libS.Release();
		}	// else

	return S_OK;
	}	// onAttach

HRESULT EVPSign :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Sign begin
	if (_RCP(Begin))
		{
		const EVP_MD	*pt	= NULL;

		// State check
		CCLTRYE ( libS.bValid == true,	ERROR_INVALID_STATE );

		// Previous context
		if (pctx != NULL)
			{
			EVP_MD_CTX_destroy ( pctx );
			pctx	= NULL;
			}	// if

		// Initialize context
		CCLTRYE ( (pctx = EVP_MD_CTX_create()) != NULL, E_OUTOFMEMORY );


		// Message digest type
		CCLTRYE ( (pt = 
						(!WCASECMP(strType,L"SHA1"))	? EVP_sha1() : 
						(!WCASECMP(strType,L"MD5"))	? EVP_md5() : 
						NULL) != NULL, E_INVALIDARG );

		// Initialize signing
		CCLTRYE ( EVP_DigestInit ( pctx, pt ) != 0, 
						libS.errors(L"EVPSign::receive:Begin:DigestInit") );

		// Result
		if (hr == S_OK)
			_EMT(Begin,adtInt() );
		else
			_EMT(Error,adtInt(hr) );
		}	// if

	// Sign update
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
			CCLTRYE ( EVP_DigestUpdate ( pctx, cBfr, (unsigned int) sz ) != 0,
							libS.errors(L"EVPSign::receive:Update:DigestUpdate") );
			}	// while

		// Result
		if (hr == S_OK)
			_EMT(Update,adtIUnknown(pStm) );
		else
			_EMT(Error,adtInt(hr) );
		}	// else if

	// Sign final
	else if (_RCP(Final))
		{
		unsigned char	*pcsig	= NULL;
		objSSL			*obj		= NULL;
		EVP_PKEY			*key		= NULL;
		unsigned int	sz;

		// State check
		CCLTRYE ( libS.bValid == true,	ERROR_INVALID_STATE );
		CCLTRYE ( pStm != NULL,				ERROR_INVALID_STATE );
		CCLTRYE ( pctx != NULL,				ERROR_INVALID_STATE );

		// If a key is specified, assume signing
		if (hr == S_OK && pKey != NULL)
			{
			// Obtain key object and assign to EVP key
			CCLTRY	( pKey->getThis ( (void **) &obj ) );
			CCLTRYE	( (key = EVP_PKEY_new()) != NULL, E_OUTOFMEMORY );
			if (hr == S_OK)
				{
				if (obj->rsa != NULL)
					{
					CCLTRYE	( EVP_PKEY_set1_RSA ( key, obj->rsa ) != 0,
									libS.errors(L"EVPSign::receive:Final:SetRSA") );
					}	// if
				else
					hr = E_INVALIDARG;
				}	// if

			// Allocate memory for the signature
			CCLTRYE ( (sz = EVP_PKEY_size ( key )) > 0, E_UNEXPECTED );
			CCLTRYE ( (pcsig = (unsigned char *) _ALLOCMEM(sz)) != NULL, E_OUTOFMEMORY );

			// Compute signature
			CCLTRYE ( EVP_SignFinal ( pctx, pcsig, &sz, key ) != 0,
							libS.errors(L"EVPSign::receive:Final:SignFinal") );

			// Write the signature to the stream
			CCLTRY ( pStm->write ( pcsig, sz, NULL ) );
			}	// if

		// No key means digest/hash
		else if (hr == S_OK && pKey == NULL)
			{
			unsigned char	md_final[EVP_MAX_MD_SIZE];
			unsigned int	len;

			// Finialize hash
			CCLTRYE ( EVP_DigestFinal ( pctx, md_final, &len ) != 0,
							libS.errors(L"EVPSign::receive:Final:DigestFinal") );

			// Write the hash to the stream
			CCLTRY ( pStm->write ( md_final, len, NULL ) );
			}	// else if

		// Result
		if (hr == S_OK)
			_EMT(Final,adtIUnknown(pStm) );
		else
			_EMT(Error,adtInt(hr) );

		// Clean up
		if (key != NULL)
			EVP_PKEY_free(key);
		_FREEMEM(pcsig);
		if (pctx != NULL)
			{
			EVP_MD_CTX_destroy ( pctx );
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


