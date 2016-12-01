////////////////////////////////////////////////////////////////////////
//
//									Avail.CPP
//
//					Implementation of the socket Avail node
//
////////////////////////////////////////////////////////////////////////

#include "netl_.h"
#include <stdio.h>

Avail :: Avail ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the availability node
	//
	////////////////////////////////////////////////////////////////////////
	pThrd		= NULL;
	pAvails	= NULL;
	pInSkt	= NULL;
	bAvail	= false;
	pSkt		= NULL;
	bRead		= true;
	bWrite	= false;
	iTo		= 1000;
	}	// Avail

HRESULT Avail :: onAttach ( bool bAttach )
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
	HRESULT	hr = S_OK;

	// Attach
	if (bAttach)
		{
		adtValue	v;

		// Initialize sockets
		CCLTRY ( NetSkt_AddRef() );

		// Objects
		CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pAvails ) );
		CCLTRY ( pAvails->keys ( &pInSkt ));

		// Attributes
		if (hr == S_OK && pnDesc->load ( adtString(L"Timeout"), v ) == S_OK)
			iTo = v;
		if (hr == S_OK && pnDesc->load ( adtString(L"Read"), v ) == S_OK)
			bRead = v;
		if (hr == S_OK && pnDesc->load ( adtString(L"Write"), v ) == S_OK)
			bWrite = v;
		}	// if

	// Detach
	else
		{
		// Shutdown thread
		if (pThrd != NULL)
			{
			pThrd->threadStop(5000);
			pThrd->Release();
			pThrd = NULL;
			}	// if

		// Clean up
		_RELEASE(pInSkt);
		_RELEASE(pSkt);
		_RELEASE(pAvails);

		// Free sockets
		CCLTRY ( NetSkt_Release() );
		}	// if

	return hr;
	}	// onAttach

HRESULT Avail :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Start
	if (_RCP(Start))
		{
		// State check
		CCLTRYE ( pThrd == NULL, ERROR_INVALID_STATE );
//		dbgprintf ( L"%s:Avail::receive:Start\r\n", (LPCWSTR)strnName );

		// Start server thread
		CCLTRY(COCREATE(L"Sys.Thread", IID_IThread, &pThrd ));
		CCLOK (bAvail = true;)
		CCLTRY(pThrd->threadStart ( this, 5000 ));
		}	// if

	// Stop
	else if (_RCP(Stop))
		{
		// State check
		CCLTRYE ( pThrd != NULL, ERROR_INVALID_STATE );
//		dbgprintf ( L"%s:Avail::receive:Stop\r\n", (LPCWSTR)strnName );

		// Shutdown worker thread
		if (hr == S_OK)
			pThrd->threadStop(5000);
		_RELEASE(pThrd);
		}	// if

	// Add
	else if (_RCP(Add))
		{
		adtValue	vSkt;
		adtLong	lSkt;

		// State check
		CCLTRYE	( pSkt != NULL, ERROR_INVALID_STATE );
		CCLTRY	( pSkt->load ( adtString(L"Socket"), vSkt ) );

		// Store in dictionary with socket as key and dictionary as value
//		dbgprintf ( L"%s:Avail::receive:Add:%d\r\n", (LPCWSTR)strnName, vSkt.vlong );
		CCLTRY	( pAvails->store ( (lSkt = vSkt), adtIUnknown(pSkt) ) );
		}	// else if

	// Remove
	else if (_RCP(Remove))
		{
		adtValue	vSkt;
		adtLong	lSkt;

		// State check
		CCLTRYE	( pSkt != NULL, ERROR_INVALID_STATE );
		CCLTRY	( pSkt->load ( adtString(L"Socket"), vSkt ) );

		// Remove from list
//		dbgprintf ( L"%s:Avail::receive:Remove:%d\r\n", (LPCWSTR)strnName, vSkt.vlong );
		CCLTRY	( pAvails->remove ( (lSkt = vSkt) ) );
		}	// else if

	// State
	else if (_RCP(Socket))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pSkt);
		hr = _QISAFE(unkV,IID_IDictionary,&pSkt);
		}	// else if
	else if (_RCP(Timeout))
		iTo = v;
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

HRESULT Avail :: tickAbort ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITickable
	//
	//	PURPOSE
	//		-	Notifies the object that 'ticking' should abort.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////

	// Turn off Availing
	bAvail = false;

	return S_OK;
	}	// tickAbort

HRESULT Avail :: tick ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITickable
	//
	//	PURPOSE
	//		-	Perform one 'tick's worth of work.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr			= S_OK;
	int				s			= 0;
	SOCKET			sktCli	= INVALID_SOCKET;
	SOCKET			sktMax	= INVALID_SOCKET;
	fd_set			rfds,efds,wfds;
	adtLong			lSkt;
	struct timeval	to;

	// Debug
//	dbgprintf ( L"Avail::tick { %s\r\n", (LPCWSTR)strnName );

	// Sanity check
	CCLTRYE ( (bAvail == true), S_FALSE );

	// Wait for activity on sockets
	if (hr == S_OK)
		{
		// Initialize file descriptor set
		CCLOK ( FD_ZERO ( &rfds ); )
		CCLOK ( FD_ZERO ( &wfds ); )
		CCLOK ( FD_ZERO ( &efds ); )

		// Run down list setting sockets.  Keep track of largest one.
//		dbgprintf ( L"%s:Avail::tick:1\r\n", (LPCWSTR)strnName );
		CCLOK	( sktMax = (SOCKET) INVALID_SOCKET; )
		CCLTRY( pInSkt->begin() );
		while (hr == S_OK && pInSkt->read ( lSkt ) == S_OK)
			{
			// Set next one
			CCLOK ( sktCli = (SOCKET)(U64)lSkt; )
			if (hr == S_OK && bRead == true)
				FD_SET ( sktCli, &rfds );
			if (hr == S_OK && bWrite == true)
				FD_SET ( sktCli, &wfds );
//			CCLOK ( FD_SET ( sktCli, &efds ); )
//			dbgprintf ( L"%s:Avail::tick:Socket:%d\r\n", (LPCWSTR)strnName, sktCli );

			// Highest one ?
			if (	sktMax == INVALID_SOCKET ||
					sktCli > sktMax )
				sktMax = sktCli;

			// Next
			pInSkt->next();
			}	// while

		// Setup timeout.  Timeout the select every so often to catch
		// 'stop' requests.
		CCLOK	( to.tv_sec		= (iTo/1000); )
		CCLOK ( to.tv_usec	= (iTo % 1000) * 1000; )

		// Wait for activity or just sleep if nothing to do
		CCLOK	( s = 0; )
		if (hr == S_OK)
			{
			// Execte
			if (sktMax != INVALID_SOCKET)
				{
				hr = ((s = select ( (int)sktMax+1, 
											(bRead == true) ? &rfds : NULL, 
//											(bWrite == true) ? &wfds : NULL, &efds, &to )) >= 0) ?
											(bWrite == true) ? &wfds : NULL, NULL, &to )) >= 0) ?
											S_OK : WSAGetLastError();
				// If a socket has closed recently this function will return WSAENOTSOCK.
				// Still notify the outside world of this, do NOT terminate this thread
				// because of it
				#ifdef	_WIN32
				if (hr == WSAENOTSOCK)
				#elif		__unix__ || __APPLE__
				if (hr == EBADF)
				#else
				if (0)
				#endif
					{
					dbgprintf ( L"Avail::tick:WSAENOTSOCK Error!\n" );
					}	// if
				}	// if
			else
				{
				#ifdef	_WIN32
				Sleep ( (DWORD)(to.tv_sec*1000.0 + to.tv_usec/1000.0) );
				#else
				usleep(to.tv_sec*1000000+to.tv_usec);
				#endif
				}	// else
			}	// if
//      dbgprintf ( L"Avail::tick:hr 0x%x\r\n", hr );
		}	// if

	// Still Availing ?
	CCLTRYE ( (bAvail == true), E_UNEXPECTED );

	// Loop through which sockets have data available if no timeout.
	// NOTE: Sockets that are disconnected appear 'readable'.  Unfortunately there
	// is no way to see if that is the case without doing a 'recv'.  Up to external
	// logic (usually the sktLoad/sktRead nodes) to handle disconnected peers.
	if (hr == S_OK && s > 0)
		{
		adtValue	vSkt;
//		dbgprintf ( L"%s:Avail::tick:2\r\n", (LPCWSTR)strnName );
		CCLOK ( pInSkt->begin(); )
		while (hr == S_OK && pInSkt->read ( lSkt ) == S_OK)
			{
			// This socket set ?
			CCLOK ( sktCli = (SOCKET)(U64)lSkt; )
			if (bRead == true && FD_ISSET ( sktCli, &rfds ))
				{
				// Socket handled
				FD_CLR ( sktCli, &rfds );

				// Emit socket that is readable...
				if (hr == S_OK && pAvails->load ( lSkt, vSkt ) == S_OK)
					{
//					dbgprintf ( L"%s:Avail::tick:Read:%d\r\n", (LPCWSTR)strnName, lSkt.vlong );
					_EMT(Read,vSkt);
					}	// if
				}	// if
			if (bWrite == true && FD_ISSET ( sktCli, &wfds ))
				{
				// Socket handled
				FD_CLR ( sktCli, &wfds );

				// Emit socket that is writeable...
				if (hr == S_OK && pAvails->load ( lSkt, vSkt ) == S_OK)
					{
//					dbgprintf ( L"%s:Avail::tick:Write:%d\r\n", (LPCWSTR)strnName, lSkt.vlong );
					_EMT(Write,vSkt );
					}	// if
				}	// if
			if (FD_ISSET ( sktCli, &efds ))
				{
				dbgprintf ( L"%s:Avail::tick:Exception!:%d\r\n", (LPCWSTR)strnName, lSkt.vlong );
				}	// if

			// Next
			pInSkt->next();
			}	// while
		}	// if

	// If there was an error during select, scan for invalid descriptors.
	#ifdef	_WIN32
	if (hr == WSAENOTSOCK || s < 0)
	#endif
	#if	__unix__ || __APPLE__
	if (hr == EBADF || s < 0)
	#endif
		{
		adtValue	vSkt;
		// There is no 'IsValidSocket' call so test for validity by doing a zero timeout
		// select on each socket
		hr = S_OK;											// No reason to exit thread
		CCLOK ( pInSkt->begin(); )
//		dbgprintf ( L"%s:Avail::tick:3\r\n", (LPCWSTR)strnName );
		while (hr == S_OK && pInSkt->read ( lSkt ) == S_OK)
			{
			// Valid socket ?
			CCLOK ( sktCli = (SOCKET)(U64)lSkt; )
			CCLOK ( to.tv_sec = to.tv_usec = 0; )
			CCLOK ( FD_ZERO ( &rfds ); )
			CCLOK ( FD_SET ( sktCli, &rfds ); )
			if ( hr == S_OK && (select ( (int)sktCli+1, &rfds, NULL, NULL, &to ) < 0))
				{
				// Emit 'availability'.  Futher processing will cause error.
				if (hr == S_OK && pAvails->load ( lSkt, vSkt ) == S_OK)
					{
//					dbgprintf ( L"%s:Avail::tick:Read:Error:%d\r\n", (LPCWSTR)strnName, lSkt.vlong );
					_EMT(Read,vSkt);
					}	// if
				}	// if

			// Next
			pInSkt->next();
			}	// while

		}	// else if

	// Debug
	#ifdef	_WIN32
	if (hr != S_OK && bAvail == true)
		{
		dbgprintf ( L"Avail::tick:Returning error! 0x%x\n", hr );
		}	// if
	#endif
//	dbgprintf ( L"} Avail::tick\n" );

	return hr;
	}	// tick

HRESULT Avail :: tickBegin ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITickable
	//
	//	PURPOSE
	//		-	Notifies the object that it should get ready to 'tick'.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	dbgprintf ( L"Avail::tickBegin:Read:%s:Write:%s {}\n",
						(bRead == true) ? L"true" : L"false",
						(bWrite == true)? L"true" : L"false" );
	return S_OK;
	}	// tickBegin

HRESULT Avail :: tickEnd ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITickable
	//
	//	PURPOSE
	//		-	Notifies the object that 'ticking' is to stop.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	dbgprintf ( L"%s:Avail::tickEnd\r\n", (LPCWSTR)strnName );
	return S_OK;
	}	// tickEnd

