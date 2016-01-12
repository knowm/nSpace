////////////////////////////////////////////////////////////////////////
//
//									NETL.CPP
//
//						General network utilities
//
////////////////////////////////////////////////////////////////////////

#include "netl_.h"

// Globals
static	U32		NetSktRefCnt	= 0;
static	U32		uSizeRx			= 0;
static	U32		uSizeTx			= 0;
#ifdef	_WIN32
static	WSADATA	wsaData;
#endif

HRESULT NetSkt_AddRef ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Increments socket use for this DLL.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Already initialized ?
	if (++NetSktRefCnt > 1) return S_OK;

	#ifdef	_WIN32
	// Initialize sockets
	CCLTRYE ( (WSAStartup ( MAKEWORD(2,0), &wsaData ) == 0), WSAGetLastError() );
	#endif

	// Statistics
	uSizeRx = 0;
	uSizeTx = 0;

	// Clean up
	if (hr != S_OK) NetSktRefCnt = 0;

	return hr;
	}	// NetSkt_AddRef

HRESULT NetSkt_Release ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Decrements socket use for this DLL.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////

	// Last one ?
	--NetSktRefCnt;
	#ifdef	_WIN32
	if (!NetSktRefCnt) WSACleanup();
	#endif

	// DEBUG
	#if	defined(_DEBUG) || defined(DEBUG)
	if (!NetSktRefCnt)
		{
		dbgprintf ( L"NetSkt_Release:Rx %d bytes, Tx %d bytes\n", uSizeRx, uSizeTx );
		}	// if
	#endif

	return S_OK;
	}	// NetSkt_Release

HRESULT NetSkt_Resolve ( const ADTVALUE &v, adtInt &iAddr, adtInt &iPort )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Attempts to resolve the specified value into a network byte
	//			order address and optionally a port
	//
	//	PARAMETERS
	//		-	v contains the value to use
	//		-	iAddr will receive the network byte order address
	//		-	iPort will receive the network byte order port if specified
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	char			*ascName	= NULL;
	char			*colon	= NULL;
	adtString	strRslv;

	// Be flexible, if a string is received try and convert it to a 'real' address
	// A port can be specified by adding the appropriate ":<port>" to the string

	// String ?
	if (adtValue::type(v) == VTYPE_STR && (strRslv = v).length() > 0)
		{
		// Routine use ASCII strings
		CCLTRY	( strRslv.toAscii ( &ascName ); )

		// Specifying the port is also allowed so check for that first.
		if (hr == S_OK && ((colon = strchr ( ascName, ':' )) != NULL) )
			*colon = '\0';

		// Newer inet_pton, etc not available on XP
		#if _WIN32 && WINVER >= _WIN32_WINNT_VISTA

		// Failed ?  Try resolving it as a hostname.
		int	addr = -1;
		if (inet_pton ( AF_INET, ascName, &addr ) != 1)
			{
			struct addrinfo *n, *ai = NULL;
			if (getaddrinfo ( ascName, NULL, NULL, &ai ) == 0)
				{
				// Looking for IPv4 address
				for (n = ai; n != NULL; n = n->ai_next)
					if (n->ai_family == AF_INET)
						break;

				// Found
				if (n != NULL)
					addr = ((struct sockaddr_in *)(n->ai_addr))->sin_addr.S_un.S_addr;
				}	// if

			// Clean up
			if (ai != NULL)
				freeaddrinfo ( ai );
			}	// if

		// Success ?
		CCLTRYE ( ((S32)(iAddr = addr) != -1), E_UNEXPECTED );

		#else

		// Try 'dot' notation first
		if (hr == S_OK)
			{
			// This works if address is "ww.xx.yy.zz"
			iAddr = inet_addr ( ascName );

			// Failed ?  Try resolving it as a hostname.
			if ((S32)iAddr == -1)
				{
				struct hostent *phost = gethostbyname ( ascName );
				if (phost != NULL)
					iAddr = ((struct in_addr *)phost->h_addr)->s_addr;
				}	// if
			}	// if
			
		// Success ?
		CCLTRYE ( ((S32)iAddr != -1), E_UNEXPECTED );

		#endif // WINVER >= _WIN32_WINNT_VISTA

		// Addresses stored locally in host order
		CCLOK ( iAddr = ntohl ( iAddr ); )

		// Convert the port # if specified, otherwise use default.
		if (hr == S_OK && colon != NULL)
			iPort = atoi ( colon+1 );

		// Clean up
		_FREEMEM(ascName);
		}	// if

	// Integer ?
	else
		iAddr = (U32)adtInt(v);

	return hr;
	}	// NetSkt_Resolve

HRESULT NetSkt_Receive ( SOCKET skt, void *pvBfr, U32 nio, U32 *pnio, U32 toms )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Reads the specified # of bytes from the socket.
	//
	//	PARAMETERS
	//		-	skt is the socket to transfer
	//		-	pvBfr will receive the data
	//		-	nio is the # of bytes to transfer
	//		-	pnio is the # of bytes transferred
	//		-	toms is the timeout in milliseconds for the read
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr		= S_OK;
	struct timeval	to		= { 0, 0 };
	int				len	= 0;
	int				ret	= 0;
	fd_set			rfds,efds;

	// Setup
	CCLTRYE	( (skt != INVALID_SOCKET), E_UNEXPECTED );

	// Initialize descriptors
	CCLOK ( FD_ZERO ( &rfds ); )
	CCLOK ( FD_SET ( skt, &rfds ); )
	CCLOK ( FD_ZERO ( &efds ); )
	CCLOK ( FD_SET ( skt, &efds ); )

	// Timeout
	CCLOK ( to.tv_sec		= (toms/1000); )
	CCLOK ( to.tv_usec	= (toms % 1000) * 1000; )

	// Wait for readability.  This makes sure the recv does not hang forever.
	CCLTRYE	( (ret = select ( (int)skt+1, &rfds, NULL, &efds, &to )) != SOCKET_ERROR, WSAGetLastError() );
	CCLTRYE	( (ret != 0), ERROR_TIMEOUT );
	CCLTRYE	( FD_ISSET ( skt, &rfds ), ERROR_TIMEOUT );

	// Read data
	CCLTRYE	( (len = recv ( skt, (char *) pvBfr, nio, 0 )) >= 0, WSAGetLastError() );

	// A readable socket that returns 0 bytes means socket has been closed gracefully
	if (hr == S_OK && len == 0)
		hr = WSAECONNRESET;

	// Debug
//	dbgprintf ( L"CommSktLoad::read:pvBfr %p nio %d len %d\r\n", pvBfr, nio, len );

	// Result
	if (pnio != NULL)		*pnio = len;
	if (hr == S_OK)		uSizeRx += len;

	// Debug
//	dbgprintf ( L"Net_Receive:skt 0x%x hr 0x%x len %d nio %d\n", skt, hr, len, nio );
	#if	defined(_DEBUG) || defined(DEBUG)
	if (hr != S_OK)
		{
		dbgprintf ( L"Net_Receive:skt 0x%x hr 0x%x len %d nio %d\n", skt, hr, len, nio );
		}	// if
	#endif

	// Debug
//	CommSkt_Log ( skt, L"Net_Receive", pvBfr, nio, len, hr );

	return hr;
	}	// NetSkt_Receive

HRESULT NetSkt_Send ( SOCKET skt, void const *pcvBfr, U32 nio, 
								U32 *pnio, U32 toms )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Writes the specified # of bytes to the stream.
	//
	//	PARAMETERS
	//		-	skt is the socket
	//		-	pcvBfr contains the data
	//		-	nio is the # of bytes to transfer
	//		-	pnio is the # of bytes transferred
	//		-	toms is the timeout in milliseconds for the write
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr		= S_OK;
	struct timeval	to		= { 0, 0 };
	int				len	= 0;
	int				ret	= 0;
	fd_set			wfds,efds;

	// Setup
	CCLTRYE	( (skt != INVALID_SOCKET), E_UNEXPECTED );
	if (pnio != NULL) *pnio = 0;

	// Timeout
	CCLOK ( to.tv_sec		= (toms/1000); )
	CCLOK ( to.tv_usec	= (toms % 1000) * 1000; )

	// Initialize descriptors
	CCLOK ( FD_ZERO ( &wfds ); )
	CCLOK ( FD_SET ( skt, &wfds ); )
	CCLOK ( FD_ZERO ( &efds ); )
	CCLOK ( FD_SET ( skt, &efds ); )

	// Wait for writeability.  This is how the timeout is used.
	if (hr == S_OK)
		{
		CCLTRYE	( (ret = select ( (int)skt+1, NULL, &wfds, &efds, &to )) != SOCKET_ERROR, WSAGetLastError() );

		// If error on select, socket is bad
		if (hr != S_OK) skt = INVALID_SOCKET;
		}	// if
	CCLTRYE	( (ret != 0), ERROR_TIMEOUT );
	CCLTRYE	( FD_ISSET ( skt, &wfds ), ERROR_TIMEOUT );

	// Write data
	if (hr == S_OK)
		{
		// Attempt write
		CCLTRYE	( (len = send ( skt, (const char *) pcvBfr, (int)nio, 0 )) >= 0, WSAGetLastError() );
		}	// if

	// Result
	if (pnio != NULL && hr == S_OK)	*pnio = len;
	if (hr == S_OK)						uSizeTx += len;

	// If the write gets a timeout, do best to detect a reset connection to avoid continual retry
	if (hr == ERROR_TIMEOUT)
		{
		fd_set	rfds;

		// Initialize descriptors
		FD_ZERO ( &rfds );
		FD_SET ( skt, &rfds );
		memset ( &to, 0, sizeof(to) );

		// See if data is available on socket, a 'reset' connection will have data waiting
		ret = select ( (int)skt+1, &rfds, NULL, NULL, &to );

		// Socket marked ?
		if (ret > 0 && FD_ISSET ( skt, &rfds ))
			{
			char	byte;

			// Peek at incoming data just to see if 'recv' returns an error
			ret = recv ( skt, &byte, 1, MSG_PEEK );

			// If connection is bad, read returns immediately with 0 bytes read
			if (ret == 0)
				{
				// Close socket ourselves, this will cause future access to error out
				shutdown ( skt, 2 );
				closesocket ( skt );
				hr = E_UNEXPECTED;
				}	// if
			}	// if
		}	// if

	// Debug
//	dbgprintf ( L"NetSkt_Send:hr 0x%x skt 0x%x len %d nio %d\n", hr, skt, len, nio );
	#if	defined(_DEBUG) || defined(DEBUG)
	if (hr != S_OK && skt != INVALID_SOCKET)
		{
		dbgprintf ( L"NetSkt_Send:hr 0x%x skt 0x%x len %d nio %d\n", hr, skt, len, nio );
		}	// if
	#endif

	// Debug
//	Net_Log ( skt, L"NetSkt_Send", pcvBfr, nio, len, hr );

	return hr;
	}	// NetSkt_Send
