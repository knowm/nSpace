////////////////////////////////////////////////////////////////////////
//
//									DGRAM.CPP
//
//					Implementation of the datagram operations node
//
////////////////////////////////////////////////////////////////////////

#include "netl_.h"

DatagramOp :: DatagramOp ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pSkt		= NULL;
	pStm		= NULL;
	iAddr		= INADDR_ANY;
	iPort		= 0;
	iSz		= 0;
	}	// DatagramOp

HRESULT DatagramOp :: onAttach ( bool bAttach )
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

HRESULT DatagramOp :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

	// Receive
	if (_RCP(Receive))
		{
		SOCKET					skt		= INVALID_SOCKET;
		U32						uBfrLen	= 0;
		adtValue					vSkt;
		adtLong					lSkt;
		struct sockaddr_in	from;
		#ifdef					_WIN32
		int						flen;
		#else
		socklen_t				flen;
		#endif

		// State check
		CCLTRYE	( (pSkt != NULL && pStm != NULL), ERROR_INVALID_STATE );
		CCLTRY	( pSkt->load ( adtString(L"Socket"), vSkt ) );
		CCLOK		( skt = (SOCKET) (lSkt = vSkt); )

		// Read datagram into buffer
		CCLOK		( flen = sizeof(sockaddr); )
		CCLTRYE	( (uBfrLen = recvfrom ( skt, (char *) cFrame, SIZE_FRAME_ETHERNET, 0,
							(struct sockaddr *) &from, &flen )) >= 0, WSAGetLastError() );
		CCLTRYE	( (uBfrLen > 0), ERROR_TIMEOUT );

		// Write datagram into stream
		CCLTRY	( pStm->write ( cFrame, uBfrLen, NULL ) );

		// Result
		if (hr == S_OK)
			{
			_EMT(Address,adtInt(ntohl(from.sin_addr.s_addr)));
			_EMT(Port,adtInt(ntohs(from.sin_port)));
			_EMT(Receive,adtIUnknown(pStm));
			}	// if
		else
			_EMT(Error,adtInt(hr));
		}	// if

	// Transmit
	else if (_RCP(Transmit))
		{
		SOCKET	skt = INVALID_SOCKET;
		U64		uBfrLen	= 0;
		adtValue	vSkt;
		adtLong	lSkt;

		// State check
		CCLTRYE	( (pSkt != NULL && pStm != NULL && iPort != 0 && iSz != 0), ERROR_INVALID_STATE );
		CCLTRY	( pSkt->load ( adtString(L"Socket"), vSkt ) );
		CCLOK		( skt = (SOCKET) (lSkt = vSkt); )

		// Wait for writeability.
		if (hr == S_OK)
			{
			struct timeval	to;
			int				ret;
			fd_set			wfds,efds;

			// Timeout
			to.tv_sec	= 5;
			to.tv_usec	= 0;

			// Initialize descriptors
			FD_ZERO ( &wfds );
			FD_SET ( skt, &wfds );
			FD_ZERO ( &efds );
			FD_SET ( skt, &efds );

			// Wait
			CCLTRYE	( (ret = select ( (int)skt+1, NULL, &wfds, &efds, &to )) != SOCKET_ERROR, WSAGetLastError() );
			CCLTRYE	( (ret != 0), ERROR_TIMEOUT );
			CCLTRYE	( FD_ISSET ( skt, &wfds ), ERROR_TIMEOUT );
			}	// if

		// Read in data to transmit
		CCLTRY ( pStm->read ( cFrame, (iSz.vint < SIZE_FRAME_ETHERNET) ? iSz.vint : SIZE_FRAME_ETHERNET, &uBfrLen ) );

		// Transmit packet.
		if (hr == S_OK)
			{
			struct sockaddr_in	sockaddr;

			// Fill destination information
			memset ( &sockaddr, 0, sizeof(sockaddr) );
			sockaddr.sin_family			= AF_INET;
			sockaddr.sin_addr.s_addr	= htonl ( iAddr );
			sockaddr.sin_port				= htons ( iPort );

			// Send
			CCLTRYE	( (uBfrLen = sendto ( skt, (char *) cFrame, (U32)uBfrLen, 0, (struct sockaddr *) &sockaddr,
							sizeof(sockaddr) )) != (U32)SOCKET_ERROR, WSAGetLastError() );
			}	// if

		// Debug
//		dbgprintf ( L"%s:DatagramOp::receive:Transmit:iAddr 0x%x:iPort %d:0x%x\r\n", (LPCWSTR)strnName, (int)iAddr, (int)iPort, hr );
		#if	defined(_DEBUG)
		if (hr != S_OK)
			dbgprintf ( L"%s:DatagramOp::receive:Transmit:Error!!!:0x%x\r\n", (LPCWSTR)strnName, hr );
		#endif

		// Result
		if (hr == S_OK)	_EMT(Transmit,adtIUnknown(pStm));
		else					_EMT(Error,adtInt(hr));
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
	else if (_RCP(Address))
		hr = NetSkt_Resolve ( v, iAddr, iPort );
	else if (_RCP(Port))
		iPort = v;
	else if (_RCP(Size))
		iSz = v;
	else
		hr = ERROR_NO_MATCH;

	// Debug
	if (hr != S_OK)
		dbgprintf ( L"CommSktMulticast::receive:Error:hr 0x%x\r\n", hr );

	return hr;
	}	// receive

