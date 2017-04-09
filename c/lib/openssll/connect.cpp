////////////////////////////////////////////////////////////////////////
//
//									CONNECT.CPP
//
//				Implementation of the OpenSSL connection node
//
////////////////////////////////////////////////////////////////////////

#include "openssll_.h"

// Globals
extern libSSL	libS;

SSLConnect :: SSLConnect ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pDsc	= NULL;
	pStm	= NULL;
	iSz	= 0;
	}	// SSLConnect

HRESULT SSLConnect :: connect ( SSL *pssl )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called to establish/continue connection negotiation.
	//
	//	PARAMETERS
	//		-	pssl is the SSL channel
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr		= S_OK;
	int		retry = 0;
	int		ret;

	// Continue connection logic
	while (hr == S_OK && (ret = SSL_connect ( pssl )) == -1 && retry++ < 100)
		{
		// Current error
		int code = SSL_get_error ( pssl, ret );

		// Reads
		// Will continue the next time a 'read' is signalled.
		if (code == SSL_ERROR_WANT_READ)
			hr = WSAEWOULDBLOCK;

		// Writes, allow a retry
		else if (code == SSL_ERROR_WANT_WRITE)
			++retry;

		// Cannot receover
		else
			{
			dbgprintf ( L"SSLConnect::connect:Error:%d\r\n", ret );
			hr = E_UNEXPECTED;
			}	// else

		}	// while

	// Error ?
	if (retry >= 100)
		hr = ERROR_TIMEOUT;

	return hr;
	}	// connect

HRESULT SSLConnect :: onAttach ( bool bAttach )
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
		// Library reference
		libS.AddRef();
		libS.sslAddRef();
		}	// if
	else
		{
		_RELEASE(pStm);
		_RELEASE(pDsc);

		// Lbrary reference
		libS.sslRelease();
		libS.Release();
		}	// else

	return S_OK;
	}	// onAttach

HRESULT SSLConnect :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Attach to descriptor
	if (_RCP(Attach))
		{
		SSL		*pssl	= NULL;
		int		fd		= -1;
		adtValue	vL;

		// State check
		CCLTRYE ( pDsc != NULL, ERROR_INVALID_STATE );

		// Supported descriptors
		if (hr == S_OK && pDsc->load ( adtString(L"Socket"), vL ) == S_OK)
			fd	= (int)adtLong(vL);

		// Create a context for the descriptor
		CCLTRYE ( (pssl = SSL_new ( libS.psslctx )) != NULL,
						E_OUTOFMEMORY );

		// Attach to descriptor
		CCLTRYE ( SSL_set_fd ( pssl, fd ) == 1, E_UNEXPECTED );

		// Store in descriptor
		CCLTRY ( pDsc->store ( adtString(L"SSL"), adtLong((U64)pssl) ) );

		// Begin the handshake.  This logic supports a non-blocking socket
		// so this call just starts the process.
		CCLTRY ( connect ( pssl ) );

		// If connection is pending, mark is as such and continue
		if (hr == WSAEWOULDBLOCK)
			pDsc->store ( adtString(L"SSLPending"), adtBool(true) );

		// Result
		if (hr == S_OK)
			_EMT(Attach,adtIUnknown(pDsc) );
		else if (hr != WSAEWOULDBLOCK)
			{
			_EMT(Error,adtInt(hr));

			// Clean up
			if (pssl != NULL)
				SSL_free ( pssl );			
			}	// else if

		}	// if

	// Detach
	else if (_RCP(Detach))
		{
		SSL		*pssl	= NULL;
		int		fd		= -1;
		adtValue	vL;

		// State check
		CCLTRYE	( pDsc != NULL, ERROR_INVALID_STATE );
		CCLTRY	( pDsc->load ( adtString(L"SSL"), vL ) );
		CCLTRYE	( (pssl = (SSL *)(U64)adtLong(vL)) != NULL, ERROR_INVALID_STATE );

		// Descriptor no longer needed
		CCLOK ( SSL_free ( pssl ); )
		if (pDsc != NULL)
			pDsc->remove ( adtString(L"SSL") );

		// Result
		if (hr == S_OK)
			_EMT(Detach,adtIUnknown(pDsc) );
		else
			_EMT(Error,adtInt(hr));
		}	// if

	// Write
	else if (_RCP(Write))
		{
		SSL		*pssl		= NULL;
		BYTE		bBfr[8192],*pb = bBfr;
		adtValue	vL;
		adtLong	lSsl;
		U64		uLeft,uLeftBlk,uToDo,uIo,uBfr;
		int		ret;

//		hr = S_FALSE;
//		DWORD	dwThen = GetTickCount();

		// State check
		CCLTRYE	( pDsc != NULL && pStm != NULL && iSz > 0, ERROR_INVALID_STATE );
		CCLTRY	( pDsc->load ( adtString(L"SSL"), vL ) );
		CCLOK		( pssl = (SSL *) (U64) (lSsl = vL); )

		// Write entire stream
		CCLOK ( uLeft = iSz; )
		while (hr == S_OK && uLeft > 0)
			{
			// Next block
			uToDo = (uLeft < sizeof(bBfr)) ? uLeft : sizeof(bBfr);
			CCLTRY ( pStm->read ( bBfr, uToDo, &uBfr ) );

			// Write entire block
			CCLOK ( uLeftBlk = uBfr; )
			CCLOK ( pb = bBfr; )
			while (hr == S_OK && uLeftBlk > 0)
				{
				// Write next block
				uToDo = (uLeftBlk < uBfr) ? uLeftBlk : uBfr;
				ret	= SSL_write ( pssl, pb, (int)uToDo );

				// Process return value
				if (ret > 0)
					{
					// Written
					uIo		= ret;
					uLeft		-= uIo;
					uLeftBlk -= uIo;
					pb			+= uIo;
					}	// if

				// Error value
				else if (ret < 0)
					hr = libS.errors ( L"SSLConnect::write" );

				// Unknown
				else
					hr = E_UNEXPECTED;
				}	// while

			}	// while

		// Result
		if (hr == S_OK)	
			_EMT(Write,adtInt(iSz));
		else					
			_EMT(Error,adtInt(hr));

		// Debug
		if (hr != S_OK)
			dbgprintf ( L"SSLConnect::write:Error:hr 0x%x\r\n", hr );
		}	// else if

	// Read
	else if (_RCP(Read))
		{
		SSL		*pssl		= NULL;
		BYTE		bBfr[8192];
		adtValue	vL;
		adtLong	lSsl;
		int		ret;

		// State check
		CCLTRYE	( pDsc != NULL && pStm != NULL, ERROR_INVALID_STATE );
		CCLTRY	( pDsc->load ( adtString(L"SSL"), vL ) );
		CCLOK		( pssl = (SSL *) (U64) (lSsl = vL); )

		// Check if connection is not yet established
		if (hr == S_OK && pDsc->load ( adtString(L"SSLPending"), vL ) == S_OK)
			{
			// Continue connection logic
			CCLTRY ( connect ( pssl ) );

			// Connection established
			if (hr == S_OK)
				{
				// No longer pending
				pDsc->remove ( adtString(L"SSLPending") );

				// Notify
				_EMT(Attach,adtIUnknown(pDsc) );
				}	// if

			// Still pending and need more data
			else if (hr == WSAEWOULDBLOCK)
				hr = S_OK;
			}	// if

		// Read normally
		else if (hr == S_OK)
			{
			// Read available data into stream.
			CCLTRYE	( (ret = SSL_read ( pssl, bBfr, sizeof(bBfr) )) > 0,
							libS.errors ( L"SSLConnect::read" ) );

			// Transfer to provided stream
			CCLTRY	( pStm->write ( bBfr, ret, NULL ) );
						
			// Result
			if (hr == S_OK)
				_EMT(Read,adtIUnknown(pStm));
			}	// else if

		// Error ?
		if (hr != S_OK)
			{
			_EMT(Error,adtInt(hr));
			dbgprintf ( L"Recv::receive:Error:hr 0x%x\r\n", hr );
			}	// if

		}	// else if


	// State
	else if (_RCP(Descriptor))
		{
		adtIUnknown unkV(v);

		// Descriptor
		_RELEASE(pDsc);
		_QISAFE(unkV,IID_IDictionary,&pDsc);
		}	// else if
	else if (_RCP(Stream))
		{
		adtIUnknown unkV(v);

		// Stream
		_RELEASE(pStm);
		_QISAFE(unkV,IID_IByteStream,&pStm);
		}	// else if
	else if (_RCP(Size))
		iSz = v;
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive


