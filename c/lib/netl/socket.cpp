////////////////////////////////////////////////////////////////////////
//
//									SOCKET.CPP
//
//					Implementation of the socket node
//
////////////////////////////////////////////////////////////////////////

#include "netl_.h"

SocketOp :: SocketOp ( void )
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
	}	// Socket

HRESULT SocketOp :: onAttach ( bool bAttach )
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

HRESULT SocketOp :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

	// Debug
//	if (!WCASECMP(strnName,L"SocketTx"))
//		dbgprintf ( L"Hi\r\n" );

	// Open
	if (_RCP(Open))
		{
		SOCKET		skt			= INVALID_SOCKET;
		BOOL			bDatagram	= FALSE;
		BOOL			bBroadcast	= FALSE;
		BOOL			bListen		= FALSE;
		struct 
		sockaddr_in	sockaddr;
		socklen_t	alen;
		adtValue		vSkt,vL;

		// State check
		CCLTRYE	( (pSkt != NULL), ERROR_INVALID_STATE );
		CCLTRYE	( pSkt->load ( adtString(L"Socket"), vSkt ) != S_OK, ERROR_INVALID_STATE );

		// Socket options
		if (hr == S_OK)
			{
			// Datagram / stream
			if (	pSkt->load ( adtString ( L"Datagram" ), vL ) == S_OK &&
					vL.vtype == VTYPE_BOOL && vL.vbool == TRUE )
				bDatagram = TRUE;

			// Broadcast
			if (	pSkt->load ( adtString ( L"Broadcast" ), vL ) == S_OK &&
					vL.vtype == VTYPE_BOOL && vL.vbool == TRUE )
				bBroadcast = TRUE;

			// Bind port
			if ( pSkt->load ( adtString ( L"Port" ), vL ) == S_OK )
				adtValue::copy ( iPort, vL );

			// Bind address
			if (	pSkt->load ( adtString ( L"Address" ), vL ) == S_OK )
				NetSkt_Resolve ( vL, iAddr, iPort );

			// Listen (server)
			if (	pSkt->load ( adtString ( L"Listen" ), vL ) == S_OK &&
					vL.vtype == VTYPE_BOOL && vL.vbool == TRUE )
				bListen = TRUE;
			}	// if

		// Create a socket of the specified type
		CCLTRYE ( (skt = socket ( AF_INET, (bDatagram) ? SOCK_DGRAM : SOCK_STREAM, 0 )) != INVALID_SOCKET,
						WSAGetLastError() );

		// Allow re-use/multiple servers on same port. (multicast support)
		// When processes crash (like on Linux) it keeps ports as 'used' 
		// for a while.
		if (hr == S_OK)
			{
			int	reuse	= 1;
			CCLTRYE	( setsockopt ( skt, SOL_SOCKET, SO_REUSEADDR, (char *) &reuse,
							sizeof(reuse) ) != -1, WSAGetLastError() );
			}	// if

		// Bind to optionally specified address / port
		if (hr == S_OK)
			{
			memset ( &sockaddr, 0, sizeof(sockaddr) );
			sockaddr.sin_family			= AF_INET;
			sockaddr.sin_port				= htons ( iPort.vint );
			sockaddr.sin_addr.s_addr	= (iAddr != 0) ? htonl ( iAddr.vint ) : INADDR_ANY;
			hr = (bind ( skt, (struct sockaddr *) &sockaddr,
							sizeof(sockaddr) ) == 0) ? S_OK : WSAGetLastError();
			}	// if

		// Make socket a non-blocking socket
		#ifdef	_WIN32
		u_long					cmd;
		CCLOK ( cmd = 1; )
		CCLOK	( ioctlsocket ( skt, FIONBIO, &cmd ); )
		#endif

		// Broadcast ?
		if (hr == S_OK && bBroadcast)
			{
			U32	brd = TRUE;

			// Set
			CCLTRYE ( setsockopt ( skt, SOL_SOCKET, SO_BROADCAST, (char *) &brd,
						sizeof(brd) ) != SOCKET_ERROR, WSAGetLastError() );
			}	// if

		// Listen ?
		if (hr == S_OK && bListen)
			{
			CCLTRYE ( (listen ( skt, SOMAXCONN ) == 0), WSAGetLastError() );
			}	// if

		// Debug
		if (hr == S_OK)
			{
			// Obtain/emit socket information
			CCLOK		( alen = sizeof(sockaddr); )
			CCLTRYE	( getsockname ( skt, (struct sockaddr *) &sockaddr, &alen )
												!= SOCKET_ERROR, WSAGetLastError() );
			}	// if
//		dbgprintf ( L"Socket::receive:open:skt %d:Addr 0x%x:%d:%s:0x%x\r\n", 
//						skt, ntohl ( sockaddr.sin_addr.s_addr ), ntohs ( sockaddr.sin_port ), 
//						(LPCWSTR)this->strnName, hr );

		// Store socket in context
		if (hr == S_OK)
			hr = pSkt->store ( adtString(L"Socket"), adtLong(skt) );
		else if (skt != INVALID_SOCKET)
			closesocket ( skt );

		// Result
		if (hr == S_OK)
			_EMT(Open,adtIUnknown(pSkt) );
		else
			_EMT(Error,adtInt(hr) );


/*
// Debug
if (true)
	{
	int		ret;
	SOCKET	sd;
	sd = socket ( AF_INET, SOCK_DGRAM, 0 );
	int		reuse = 1;
	ret = setsockopt ( sd, SOL_SOCKET, SO_REUSEADDR, (char *)&reuse, sizeof(reuse) );
	struct sockaddr_in	sa;
	memset ( &sa, 0, sizeof(sa) );
	sa.sin_family = AF_INET;
	sa.sin_port   = htons(16368);
	sa.sin_addr.s_addr = INADDR_ANY;
	ret = bind ( sd, (struct sockaddr *)&sa, sizeof(sa) );
	struct ip_mreq group;
	group.imr_multiaddr.s_addr = inet_addr("239.255.255.222");
	group.imr_interface.s_addr = inet_addr("192.168.1.20");
	ret = setsockopt ( sd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&group, sizeof(group) );
	if (sd != NULL)
		closesocket(sd);
	}
*/
		}	// if

	// Close
	else if (_RCP(Close))
		{
		adtValue	vSkt;
		adtLong	lSkt;
		SOCKET	skt = INVALID_SOCKET;

		// State check
		CCLTRYE	( (pSkt != NULL), ERROR_INVALID_STATE );
		CCLTRY	( pSkt->load ( adtString(L"Socket"), vSkt ) );
		CCLOK		( skt = (SOCKET) (lSkt = vSkt); )

		// Socket closing
		CCLOK ( _EMT(Close,adtIUnknown(pSkt) ); )

		// Socket
		if (hr == S_OK && skt != INVALID_SOCKET)
			{
			// Close
			shutdown ( skt, 2 );
			closesocket ( skt );
			}	// if

		// Clean up
		if (pSkt != NULL)
			pSkt->remove ( adtString(L"Socket") );
		}	// else if

	// Query
	else if (_RCP(Query))
		{
		adtValue					vSkt;
		adtLong					lSkt;
		SOCKET					skt = INVALID_SOCKET;
		struct sockaddr_in	sockaddr;
		socklen_t				alen;

		// State check
		CCLTRYE	( (pSkt != NULL), ERROR_INVALID_STATE );
		CCLTRY	( pSkt->load ( adtString(L"Socket"), vSkt ) );
		CCLOK		( skt = (SOCKET) (lSkt = vSkt); )

		// Obtain/emit socket information
		CCLOK		( alen = sizeof(sockaddr); )
		CCLTRYE	( getsockname ( skt, (struct sockaddr *) &sockaddr, &alen )
											!= SOCKET_ERROR, WSAGetLastError() );

		// Result
//		CCLOK ( dbgprintf ( L"Socket::receive:Port %d(%d)\r\n", sockaddr.sin_port, ntohs ( sockaddr.sin_port ) ); )
		CCLOK	( _EMT(Address,adtInt ( ntohl ( sockaddr.sin_addr.s_addr ) ) ); )
		CCLOK	( _EMT(Port,adtInt ( ntohs ( sockaddr.sin_port ) ) ); )

		// Peer information
		CCLOK		( alen = sizeof(sockaddr); )
		if (hr == S_OK && getpeername ( skt, (struct sockaddr *) &sockaddr, &alen )
													!= SOCKET_ERROR )
			{
			// Result
//			dbgprintf ( L"Socket::receive:Peer Port %d(%d)\r\n", sockaddr.sin_port, ntohs ( sockaddr.sin_port ) );
			CCLOK	( _EMT(PeerAddress,adtInt ( ntohl ( sockaddr.sin_addr.s_addr ) ) ); )
			CCLOK	( _EMT(PeerPort,adtInt ( ntohs ( sockaddr.sin_port ) ) ); )
			}	// if

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
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

