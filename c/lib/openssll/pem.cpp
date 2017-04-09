////////////////////////////////////////////////////////////////////////
//
//									PEM.CPP
//
//					Implementation of the OpenSSL PEMImpl node
//
////////////////////////////////////////////////////////////////////////

#include "openssll_.h"

// Globals
extern libSSL	libS;

PEMImpl :: PEMImpl ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pStm		= NULL;
	pObj		= NULL;
	bPub		= true;
	strType	= L"RSA";
	}	// PEMImpl

HRESULT PEMImpl :: onAttach ( bool bAttach )
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
		if (pnDesc->load ( adtString(L"Public"), vL ) == S_OK)
			bPub = vL;
		if (pnDesc->load ( adtString(L"Type"), vL ) == S_OK)
			strType = vL;

		// Library reference
		libS.AddRef();
		}	// if
	else
		{
		// Clean up
		_RELEASE(pObj);
		_RELEASE(pStm);

		// Lbrary reference
		libS.Release();
		}	// else

	return S_OK;
	}	// onAttach

HRESULT PEMImpl :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Write
	if (_RCP(Write))
		{
		BIO		*bio	= NULL;
		objSSL	*obj	= NULL;
		void		*pv	= NULL;
		long		sz;

		// State check
		CCLTRYE ( libS.bValid == true,	ERROR_INVALID_STATE );
		CCLTRYE ( pStm != NULL,				ERROR_INVALID_STATE );
		CCLTRYE ( pObj != NULL,				ERROR_INVALID_STATE );
		CCLTRY  ( pObj->getThis ( (void **) &obj ) );

		// Create a read/write BIO for output
		CCLTRYE ( (bio = BIO_new ( BIO_s_mem() )) != NULL, E_OUTOFMEMORY );

		// Write appropriate object to memory buffer
		if (hr == S_OK && obj->rsa != NULL)
			{
			// Private
			if (!WCASECMP(strType,L"RSA") && !bPub)
				{
				CCLTRYE ( PEM_write_bio_RSAPrivateKey ( bio, obj->rsa, NULL, NULL, 0, NULL, NULL ) != 0,
								libS.errors(L"PEMImpl::receive:Write:pem_wr_rsa_priv") );
				}	// if
			else if (!WCASECMP(strType,L"RSA") && bPub)
				{
				CCLTRYE ( PEM_write_bio_RSAPublicKey ( bio, obj->rsa ) != 0,
								libS.errors(L"PEMImpl::receive:Write:pem_wr_rsa_pub") );
				}	// if
			else
				hr = E_INVALIDARG;
			}	// if

		// Access memory buffer
		CCLTRYE ( (sz = BIO_ctrl( bio, BIO_CTRL_INFO, 0, (char *)&pv )) > 0,
							libS.errors(L"RSAImpl::receive:Generate:bio_ctrl") );

		// Write memory contents to provided stream
		CCLTRY ( pStm->write ( pv, sz, NULL ) );

		// Result
		if (hr == S_OK)
			_EMT(Write,adtIUnknown(pStm) );
		else
			_EMT(Error,adtInt(hr) );

		// Clean up
		if (bio != NULL)
			BIO_free(bio);
		}	// if

	// Read
	else if (_RCP(Read))
		{
		BIO		*bio	= NULL;
		void		*pv	= NULL;
		RSA		*rsa	= NULL;
		IThis		*obj	= NULL;
		U64		sz;

		// State check
		CCLTRYE ( libS.bValid == true,	ERROR_INVALID_STATE );
		CCLTRYE ( pStm != NULL,				ERROR_INVALID_STATE );

		// Size the stream
		CCLTRY ( pStm->available ( &sz ) );

		// Allocate memory for buffer and read stream
		CCLTRYE ( (pv = _ALLOCMEM ( (U32)sz )) != NULL, E_OUTOFMEMORY );
		CCLTRY  ( pStm->read ( pv, sz, &sz ) );

		// Create a read-only BIO for input
		CCLTRYE ( (bio = BIO_new_mem_buf ( pv, (int)sz )) != NULL, E_OUTOFMEMORY );

		// Read object from memory buffer
		if (hr == S_OK)
			{
			// Private
			if (!WCASECMP(strType,L"RSA") && !bPub)
				{
				CCLTRYE ( (rsa = PEM_read_bio_RSAPrivateKey ( bio, NULL, NULL, NULL )) != NULL,
								libS.errors(L"RSAImpl::receive:Read:pem_rd_rsa_priv") );
				}	// if
			else if (!WCASECMP(strType,L"RSA") && bPub)
				{
				CCLTRYE ( (rsa = PEM_read_bio_RSAPublicKey ( bio, NULL, NULL, NULL )) != NULL,
								libS.errors(L"RSAImpl::receive:Read:pem_rd_rsa_pub") );
				}	// if
			else
				hr = E_INVALIDARG;
			}	// if

		// Wrap created object
		if (rsa != NULL)
			obj = new objSSL ( rsa );

		// Result
		if (hr == S_OK)
			_EMT(Read,adtIUnknown(obj) );
		else
			_EMT(Error,adtInt(hr) );

		// Clean up
		if (bio != NULL)
			BIO_free(bio);
		_FREEMEM(pv);
		_RELEASE(obj);
		}	// else if

	// State
	else if (_RCP(Stream))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pStm);
		_QISAFE(unkV,IID_IByteStream,&pStm);
		}	// else if
	else if (_RCP(Object))
		{
		adtIUnknown unkV(v);
		_RELEASE(pObj);
		_QISAFE(unkV,IID_IThis,&pObj);
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive


