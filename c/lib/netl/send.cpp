////////////////////////////////////////////////////////////////////////
//
//									SEND.CPP
//
//					Implementation of the send stream node
//
////////////////////////////////////////////////////////////////////////

#include "netl_.h"

// Globals
DWORD dwTickTx = 0;

Send :: Send ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pSkt		= NULL;
	pStm		= NULL;
	iSz		= 0;
	pBfr		= NULL;
	uSzBfr	= 0;
	iTo		= 2000;
	}	// Send

HRESULT Send :: onAttach ( bool bAttach )
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

	// Attach
	if (bAttach)
		NetSkt_AddRef();

	// Detach
	else
		{
		// Clean up
		_RELEASE(pSkt);
		_RELEASE(pStm);
		_FREEMEM(pBfr);
		uSzBfr = 0;
		NetSkt_Release();
		}	// else

	return S_OK;
	}	// onAttach

HRESULT Send :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Transmit
	if (_RCP(Fire))
		{
		SOCKET	skt = INVALID_SOCKET;
		adtValue	vSkt;
		adtLong	lSkt;
		U64		uWr;
		U32		uWrd;

//		DWORD	dwThen = GetTickCount();

		// State check
		CCLTRYE	( pSkt != NULL && pStm != NULL && iSz > 0, ERROR_INVALID_STATE );
		CCLTRY	( pSkt->load ( adtString(L"Socket"), vSkt ) );
		CCLOK		( skt = (SOCKET) (lSkt = vSkt); )

		// Ensure internal buffer has enough memory for outgoing stream
		if (hr == S_OK && uSzBfr < iSz)
			{
			// Re-size internal buffer
			CCLTRYE	( (pBfr = (U8 *) _REALLOCMEM ( pBfr, iSz+1 )) != NULL, E_OUTOFMEMORY );
			CCLOK		( uSzBfr = iSz; )
			}	// if

		// Access stream data
		CCLTRY ( pStm->read ( pBfr, iSz, &uWr ) );

		// Debug
//		if (hr == S_OK)
//			{
//			pBfr[uWr] = '\0';
//			dbgprintf ( L"Transmit : %S\r\n", (char *)pBfr );
//			}	// if

		// Send buffer
		CCLTRY ( NetSkt_Send ( skt, pBfr, (U32)uWr, &uWrd, iTo ) );
//		dbgprintf ( L"Send::receive:transmit:%d bytes:hr 0x%x\r\n", uWr, hr );
//		dwTickTx = GetTickCount();

		// Result
		if (hr == S_OK)	_EMT(Fire,adtInt(uWrd));
		else					_EMT(Error,adtInt(hr));

		// Debug
//		dbgprintf ( L"Send::receive:hr 0x%x:skt %d:uWrd %d:uWr %d\r\n", hr, skt, uWrd, uWr );
		if (hr != S_OK)
			dbgprintf ( L"Send::receive:Error:hr 0x%x\r\n", hr );
		}	// if

	// State
	else if (_RCP(Socket))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pSkt);
		hr = _QISAFE(unkV,IID_IDictionary,&pSkt);
		}	// else if
	else if (_RCP(Stream))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pStm);
		hr = _QISAFE(unkV,IID_IByteStream,&pStm);
		}	// else if
	else if (_RCP(Size))
		iSz = v;
	else if (_RCP(Timeout))
		iTo = v;
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

