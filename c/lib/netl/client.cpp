////////////////////////////////////////////////////////////////////////
//
//									CLIENT.CPP
//
//					Implementation of the client socket node
//
////////////////////////////////////////////////////////////////////////

#include "netl_.h"

Client :: Client ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pSkt		= NULL;
	iAddr		= INADDR_ANY;
	iPort		= 0;
	iTimeout	= 2000;
	}	// Client

HRESULT Client :: onAttach ( bool bAttach )
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

HRESULT Client :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

	// Connect
	if (_RCP(Connect))
		{
		SOCKET					skt	= INVALID_SOCKET;
		struct sockaddr_in	sockaddr;
		adtValue					vSkt;
		adtLong					lSkt;

		// State check
		CCLTRYE	( pSkt != NULL, ERROR_INVALID_STATE );
		CCLTRY	( pSkt->load ( adtString(L"Socket"), vSkt ) );
		CCLOK		( skt = (SOCKET) (lSkt = vSkt); )
		CCLTRYE	( iPort != 0 && iAddr != 0, ERROR_INVALID_STATE );

		// Target address
		CCLOK ( sockaddr.sin_family		= AF_INET; )
		CCLOK ( sockaddr.sin_port			= htons ( iPort.vint ); )
		CCLOK ( sockaddr.sin_addr.s_addr = htonl ( iAddr.vint ); )

		// Connect
		if (hr == S_OK)
			{
			// It is assumed this call 'would block' so that the external environment
			// can wait for 'writability' on the socket to signal successful connection.
			CCLTRYE ( (connect ( skt, (struct sockaddr *) &sockaddr,
							sizeof(sockaddr) ) != SOCKET_ERROR), WSAGetLastError() );

			#ifdef	_WIN32
			// If call is blocked, wait for timeout.
//			if (hr == WSAEWOULDBLOCK)
//				{
//				fd_set					wfds;
//				struct timeval			to;
//				int						ret;

				// Ready descriptors for select wait
//				hr = S_OK;										// Could be ok
//				CCLOK ( FD_ZERO ( &wfds ); )
//				CCLOK ( FD_SET ( skt, &wfds ); )

				// Setup timeout.
//				to.tv_sec	= ((U32)iTimeout)/1000;
//				to.tv_usec	= ((U32)iTimeout - (to.tv_sec*1000)) * 1000;

				// Wait for connect
//				CCLTRYE	( (ret = select ( 1, NULL, &wfds, NULL, &to )) != SOCKET_ERROR, WSAGetLastError() );
//				CCLTRYE	( (ret != 0), ERROR_TIMEOUT );
//				}	// if
			#endif

			}	// if

		// Result
		dbgprintf ( L"Client::receive:Connect:skt %d Addr 0x%x:Port %d:0x%x\r\n", 
						skt, iAddr.vint, iPort.vint, hr );

		// Connection completed immediately
		if (hr == S_OK)
			_EMT(Connect,adtIUnknown(pSkt) );

		// Connection is pending
		#ifdef	_WIN32
		else if (hr == WSAEWOULDBLOCK)
		#elif		__unix__ || __APPLE__
		else if (hr == EINPROGRESS)
		#endif
			_EMT(Pending,adtIUnknown(pSkt) );

		// Other error
		else
			_EMT(Error,adtInt(hr) );
		}	// if

	// Connected
	else if (_RCP(Connected))
		{
		// Nothing to do, originally added for Java.
		}	// else if

	// Accept client connection
	else if (_RCP(Accept))
		{
		SOCKET		sktSrvr		= INVALID_SOCKET;
		SOCKET		skt			= INVALID_SOCKET;
		IDictionary	*pSktSrvr	= NULL;
		adtIUnknown	unkV(v);
		adtValue		vSkt;
		adtLong		lSkt;

		// State check
		CCLTRYE	( pSkt != NULL, ERROR_INVALID_STATE );
		CCLTRY	( _QISAFE(unkV,IID_IDictionary,&pSktSrvr) );
		CCLTRY	( pSktSrvr->load ( adtString(L"Socket"), vSkt ) );
		CCLOK		( sktSrvr = (SOCKET) (lSkt = vSkt); )

		// Accept connection
		CCLTRYE	( (skt = accept ( sktSrvr, NULL, NULL )) != INVALID_SOCKET, WSAGetLastError() );

		// Always a non-blocking socket
		#ifdef	_WIN32
		if (hr == S_OK)
			{
			u_long	cmd;
			CCLOK ( cmd = 1; )
			CCLOK	( ioctlsocket ( skt, FIONBIO, &cmd ); )
			}	// if
		#endif

		// Store socket in dictionary
		CCLTRY ( pSkt->store ( adtString(L"Socket"), adtLong(skt) ) );

		// Result
		dbgprintf ( L"Client::receive:Accept:sktSrvr %d skt %d\r\n", 
						sktSrvr, skt );
		if (hr == S_OK)
			_EMT(Accept,adtIUnknown(pSkt) );
		else
			_EMT(Error,adtInt(hr) );

		// Clean up
		_RELEASE(pSktSrvr);
		}	// else if

	// State
	else if (_RCP(Socket))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pSkt);
		hr = _QISAFE(unkV,IID_IDictionary,&pSkt);
		}	// else if
	else if (_RCP(Address))
		hr = NetSkt_Resolve ( v, iAddr, iPort );
	else if (_RCP(Port))
		iPort = v;
	else if (_RCP(Timeout))
		hr = adtValue::copy ( adtInt(v), iTimeout );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive
