////////////////////////////////////////////////////////////////////////
//
//									RECEIVE.CPP
//
//					Implementation of the receive stream node
//
////////////////////////////////////////////////////////////////////////

#include "netl_.h"

// Globals
extern DWORD dwTickTx;

Recv :: Recv ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pSkt	= NULL;
	pStm	= NULL;
	iTo	= 1000;
	}	// Recv

HRESULT Recv :: onAttach ( bool bAttach )
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
		_RELEASE(pSkt);
		_RELEASE(pStm);
		NetSkt_Release();
		}	// else

	return S_OK;
	}	// onAttach

HRESULT Recv :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Recv
	if (_RCP(Fire))
		{
		SOCKET	skt		= INVALID_SOCKET;
		U32		uBfrLen	= 0;
		adtValue	vSkt;
		adtLong	lSkt;

		// DEBUG
//		dbgprintf ( L"NetSend::receive:receive:From last write to now %d ms\r\n", (GetTickCount()-dwTickTx) );

		// State check
		CCLTRYE	( (pSkt != NULL && pStm != NULL), ERROR_INVALID_STATE );
		CCLTRY	( pSkt->load ( adtString(L"Socket"), vSkt ) );
		CCLOK		( skt = (SOCKET) (lSkt = vSkt); )

		// Continue reading into stream while the socket is readable
		while (hr == S_OK)
			{
			// Perform read
			CCLTRY ( NetSkt_Receive ( skt, cFrame, sizeof(cFrame), &uBfrLen, iTo ) );

			// A readable socket that returns 0 bytes means socket has been closed gracefully
			if (hr == S_OK && uBfrLen == 0)
				hr = ERROR_INVALID_STATE;

			// Write data into stream
			CCLTRY ( pStm->write ( cFrame, uBfrLen, NULL ) );

			// The amount of data read is less than requested, terminate read
			if (hr == S_OK && uBfrLen < sizeof(cFrame))
				break;
			}	// while

		// Result
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pStm));
		else
			_EMT(Error,adtInt(hr));

		// Debug
//		if (hr == 0x2746)
//			dbgprintf ( L"Recv::receive:Connection forcibly closed\r\n" );
//		else 
//		if (hr != S_OK)
//			dbgprintf ( L"Recv::receive:Error:hr 0x%x\r\n", hr );
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
	else if (_RCP(Timeout))
		iTo = v;
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

