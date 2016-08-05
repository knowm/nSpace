////////////////////////////////////////////////////////////////////////
//
//									MULTICAST.CPP
//
//					Implementation of the multicast socket node
//
////////////////////////////////////////////////////////////////////////

#include "netl_.h"

MulticastOp :: MulticastOp ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pSkt		= NULL;
	iAddrM	= INADDR_ANY;
	iAddrI	= INADDR_ANY;
	}	// Multicast

HRESULT MulticastOp :: onAttach ( bool bAttach )
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
		NetSkt_Release();
		}	// else

	return S_OK;
	}	// onAttach

HRESULT MulticastOp :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Add
	if (_RCP(Add))
		{
		SOCKET	skt = INVALID_SOCKET;
		adtValue	vSkt;
		adtLong	lSkt;
		struct	ip_mreq	mreq;

		// State check
		CCLTRYE	( (pSkt != NULL), ERROR_INVALID_STATE );
		CCLTRY	( pSkt->load ( adtString(L"Socket"), vSkt ) );
		CCLOK		( skt = (SOCKET) (lSkt = vSkt); )
		CCLTRYE	( iAddrM != INADDR_ANY,	ERROR_INVALID_STATE );

		// Add address to membership for socket
		if (hr == S_OK)
			{
			memset ( &mreq, 0, sizeof(mreq) );
			mreq.imr_multiaddr.s_addr	= htonl ( iAddrM );
			mreq.imr_interface.s_addr	= (iAddrI != INADDR_ANY) ? htonl ( iAddrI ) : INADDR_ANY;
			CCLTRYE ( setsockopt ( skt, IPPROTO_IP, IP_ADD_MEMBERSHIP,
							(char *) &mreq, sizeof(mreq) ) != -1, WSAGetLastError() );
			}	// if

		// Result
		if (hr == S_OK)
			_EMT(Add,adtIUnknown(pSkt) );
		else
			_EMT(Error,adtInt(hr) );
		}	// if

	// Remove
	else if (_RCP(Remove))
		{
		SOCKET	skt = INVALID_SOCKET;
		adtValue	vSkt;
		adtLong	lSkt;
		struct	ip_mreq	mreq;

		// State check
		CCLTRYE	( (pSkt != NULL), ERROR_INVALID_STATE );
		CCLTRY	( pSkt->load ( adtString(L"Socket"), vSkt ) );
		CCLOK		( skt = (SOCKET) (lSkt = vSkt); )
		CCLTRYE	( iAddrM != INADDR_ANY,	ERROR_INVALID_STATE );

		// Add address to membership for socket
		if (hr == S_OK)
			{
			memset ( &mreq, 0, sizeof(mreq) );
			mreq.imr_multiaddr.s_addr	= htonl ( iAddrM );
			mreq.imr_interface.s_addr	= (iAddrI != INADDR_ANY) ? htonl ( iAddrI ) : INADDR_ANY;
			CCLTRYE ( setsockopt ( skt, IPPROTO_IP, IP_DROP_MEMBERSHIP,
							(char *) &mreq, sizeof(mreq) ) != -1, WSAGetLastError() );
			}	// if

		// Result
		if (hr == S_OK)
			_EMT(Remove,adtIUnknown(pSkt) );
		else
			_EMT(Error, adtInt(hr) );
		}	// else if

	// Addresses
	else if (_RCP(Multicast))
		{
		adtInt	iPort;

		// Multi-cast address
		CCLTRY ( NetSkt_Resolve ( v, iAddrM, iPort ) );
		}	// else if
	else if (_RCP(Interface))
		{
		adtInt	iPort;

		// Interface address
		CCLTRY ( NetSkt_Resolve ( v, iAddrI, iPort ) );
		}	// else if

	// State
	else if (_RCP(Socket))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pSkt);
		hr = _QISAFE(unkV,IID_IDictionary,&pSkt);
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	// Debug
	if (hr != S_OK)
		lprintf ( LOG_ERR, L"Multicast::receive:Error:hr %d(0x%x)\r\n", hr, hr );

	return hr;
	}	// receive


