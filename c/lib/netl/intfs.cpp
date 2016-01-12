////////////////////////////////////////////////////////////////////////
//
//									INTFS.CPP
//
//			Implementation of the network interfaces node
//
////////////////////////////////////////////////////////////////////////

#include "netl_.h"

Interfaces :: Interfaces ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pThrd			= NULL;
	bRun			= false;
	hNotify		= NULL;
	hevNotify	= NULL;
	#ifdef		_WIN32
	pAddrs		= NULL;
	#elif			__unix__ || __APPLE__
	ifap			= NULL;
	ifnxt			= NULL;
	#endif
	}	// Interfaces

HRESULT Interfaces :: onAttach ( bool bAttach )
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
		// Verify thread shutdown
		if (pThrd != NULL)
			pThrd->threadStop(5000);
		_RELEASE(pThrd);

		// Clean up
		#ifdef	_WIN32
		_FREEMEM(pAddrs);
		#elif		__unix__ || __APPLE__
		if (ifap != NULL)
			freeifaddrs(ifap);
		#endif
		NetSkt_Release();
		}	// if

	return S_OK;
	}	// onAttach

HRESULT Interfaces :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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
	
	// First
	if (_RCP(First))
		{
		#ifdef	_WIN32
		DWORD	dwRet;
		ULONG	ulLen = 0;

		// Clean up
		_FREEMEM(pAddrs);

		// Default allocation
		CCLTRYE ( (pAddrs = (PIP_ADAPTER_ADDRESSES) _ALLOCMEM(sizeof(IP_ADAPTER_ADDRESSES)))
						!= NULL, E_OUTOFMEMORY );

		// Currently defaults to IPv4
		CCLTRYE ( (dwRet = GetAdaptersAddresses ( AF_INET, 0, NULL, pAddrs, &ulLen )) 
						== ERROR_SUCCESS, dwRet );
	
		// Need more space ?					
		if (hr == ERROR_BUFFER_OVERFLOW)
			{
			// Allocate required amount of memory for full information
			hr	= S_OK;
			_FREEMEM ( pAddrs );
			CCLTRYE	( ulLen != 0, ERROR_INVALID_STATE );
			CCLTRYE ( (pAddrs = (PIP_ADAPTER_ADDRESSES) _ALLOCMEM(ulLen)) != NULL, E_OUTOFMEMORY );

			// Obtain all information
			CCLTRYE ( (dwRet = GetAdaptersAddresses ( AF_INET, 0, NULL, pAddrs, &ulLen )) 
							== ERROR_SUCCESS, dwRet );
			}	// if

		#elif	__unix__ || __APPLE__
		if (hr == S_OK)
			{
			// Previous interface list
			if (ifap != NULL)
				{
				freeifaddrs(ifap);
				ifap 	= NULL;
				ifnxt	= NULL;
				}	// if

			// Obtain new list
			CCLTRYE ( getifaddrs ( &ifap ) == 0, errno );
			CCLOK   ( ifnxt = ifap; )
			}	// if
		#endif

		// Proceed to 'next' adapter
		CCLTRY ( receive ( prNext, pl, v ) );
		}	// if

	// Next
	else if (_RCP(Next))
		{
		IDictionary	*pDct	= NULL;

		#ifdef	_WIN32
		// State check
		CCLTRYE ( pAddrs != NULL, ERROR_INVALID_STATE );

		// Current adapter
		if (hr == S_OK)
			{
			IList			*pAddrList	= NULL;
			IDictionary	*pAddrDct	= NULL;
			adtString	strName;

			// Proceed to next adapter
			PIP_ADAPTER_ADDRESSES
				pAddr	= pAddrs;
			pAddrs	= pAddrs->Next;

			// Create a context to receive the results
			CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDct ) );

			// Results.  More properties can be added over time as necessary.
//			CCLOK ( dbgprintf ( L"Friendly name : %s\r\n", pAddr->FriendlyName ); )
			CCLOK  ( strName = pAddr->AdapterName; )
			CCLTRY ( pDct->store ( adtString(L"Name"), strName ) );
			CCLTRY ( pDct->store ( adtString(L"FriendlyName"), adtString(pAddr->FriendlyName) ) );
			CCLTRY ( pDct->store ( adtString(L"Description"), adtString(pAddr->Description) ) );
			CCLTRY ( pDct->store ( adtString(L"Status"), 
							adtString	(	(pAddr->OperStatus == IfOperStatusDown)	? L"Down" :
												(pAddr->OperStatus == IfOperStatusUp)		? L"Up" : L"Unknown" ) ) );

			// Create a list to receive the addresses
			CCLTRY ( COCREATE ( L"Adt.List", IID_IList, &pAddrList ) );
			CCLTRY ( pDct->store ( adtString ( L"AddressList" ), adtIUnknown(pAddrList) ) );

			// Addresses
			for (PIP_ADAPTER_UNICAST_ADDRESS	pU	= pAddr->FirstUnicastAddress;
					hr == S_OK && pU != NULL;pU = pU->Next )
				{
				char	hostname[255];

				// Create context for address information
				CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pAddrDct ) );

				// Fill
				CCLTRY ( pAddrDct->store ( adtString(L"LeaseLifetime"), adtInt(pU->LeaseLifetime) ) );

				// Obtain string version of address
				CCLTRYE ( getnameinfo ( pU->Address.lpSockaddr, pU->Address.iSockaddrLength,	
												hostname, sizeof(hostname), NULL, 0, NI_NUMERICHOST ) == 0, GetLastError() );
//				CCLOK ( dbgprintf ( L"Address :       %S\r\n", hostname ); )
				CCLOK ( strName = hostname; )
				CCLTRY( pAddrDct->store ( adtString ( L"Address" ), strName ) );

				// Add to list
				CCLTRY ( pAddrList->write ( adtIUnknown(pAddrDct) ) );

				// Clean up
				_RELEASE(pAddrDct);
				}	// for

			// Clean up
			_RELEASE(pAddrList);
			}	// if

		#elif	__unix__ || __APPLE__

		// State check
		CCLTRYE ( ifnxt != NULL, ERROR_INVALID_STATE );

		// Next interface
		if (hr == S_OK)
			{
			IList						*pAddrList	= NULL;
			IDictionary				*pAddrDct	= NULL;
			adtString				str;
			struct sockaddr_in	*pin	= (struct sockaddr_in *) (ifnxt->ifa_addr);

			// Information context
			CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDct ) );

			// Currently only emitted IPV4 addresses
			while (hr == S_OK && ifnxt != NULL)
				{
				// Address for this adapter
				pin	= (struct sockaddr_in *) (ifnxt->ifa_addr);
				if (pin->sin_family == AF_INET)
					break;
				
				// Next interface
				ifnxt = ifnxt->ifa_next;
				}	// while

			// Valid interface ?
			CCLTRYE ( ifnxt != NULL, ERROR_INVALID_STATE );
			
			// Interface name
			CCLOK  ( str = ifnxt->ifa_name; )
//			CCLOK  ( dbgprintf ( L"CommNetInterfaces::Name:%s\r\n", (PCWSTR)str ); )
			CCLTRY ( pDct->store ( adtString(L"Name"), str ) );
			CCLTRY ( pDct->store ( adtString(L"FriendlyName"), str ) );

			// Up flag for unix
			CCLTRY ( pDct->store ( adtString(L"Status"),
							adtString	( (ifnxt->ifa_flags & IFF_UP) ? L"Up" : L"Down" ) ) );
//			if (hr == S_OK)
//				{
//				dbgprintf ( L"Interfaces::Next:ifa_name %S\r\n", ifnxt->ifa_name );
//				dbgprintf ( L"Interfaces::Next:ifa_flags 0x%x\r\n", ifnxt->ifa_flags );
//				dbgprintf ( L"Interfaces::Next:ifa_addr 0x%x\r\n", pin->sin_addr );
//				dbgprintf ( L"Interfaces::Next:ifa_family 0x%x\r\n", pin->sin_family );
//				dbgprintf ( L"Interfaces::Next:ifa_port 0x%x\r\n", pin->sin_port );
//				dbgprintf ( L"Interfaces::Next:ifa_next 0x%x\r\n", (int)ifnxt->ifa_next );
//				}	// if
			
			// Create a list to receive the addresses
			CCLTRY ( COCREATE ( L"Adt.List", IID_IList, &pAddrList ) );
			CCLTRY ( pDct->store ( adtString ( L"AddressList" ), adtIUnknown(pAddrList) ) );

			// Currently just one address ?

			// Create context for address information
			CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pAddrDct ) );

			// Address
			CCLOK		( str = inet_ntoa ( pin->sin_addr ); )
			CCLTRY	( pAddrDct->store ( adtString ( L"Address" ), str ) );

			// Add to list
			CCLTRY ( pAddrList->write ( adtIUnknown(pAddrDct) ) );

			// Next interface
			CCLOK   ( ifnxt = ifnxt->ifa_next; )

			// Clean up
			_RELEASE(pAddrDct);
			_RELEASE(pAddrList);
			}	// if
		#endif

		// Result
		if (hr == S_OK)
			_EMT(Next,adtIUnknown ( pDct ) );
		else
			_EMT(Last,adtInt(hr) );

		// Clean up
		_RELEASE(pDct);
		}	// if

	// Start monitoring for changes
	else if (_RCP(Start))
		{
		// State check
		CCLTRYE ( pThrd == NULL, ERROR_INVALID_STATE );

		// Create monitoring thread
		CCLOK (bRun = true;)
		CCLTRY(COCREATE(L"Sys.Thread", IID_IThread, &pThrd ));
		CCLTRY(pThrd->threadStart ( this, 0 ));
		}	// else if

	// Stop monitoring for changes
	else if (_RCP(Stop))
		{
		// State check
		CCLTRYE ( pThrd != NULL, ERROR_INVALID_STATE );

		// Shutdown worker thread
		CCLOK ( pThrd->threadStop(5000); )

		// Clean up
		_RELEASE(pThrd);
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

HRESULT Interfaces :: tickAbort ( void )
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

	// Signal stop
	bRun = false;
	#ifdef	_WIN32
	if (hevNotify != NULL)
		SetEvent ( hevNotify );
	#endif

	return S_OK;
	}	// tickAbort

HRESULT Interfaces :: tick ( void )
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
	HRESULT		hr	= S_OK;

	// Debug
	dbgprintf ( L"Interfaces::tick {\n" );

	// Request for notification has to happen every time
	while (hr == S_OK && bRun == true)
		{
		#ifdef	_WIN32
		OVERLAPPED	ov;

		// Request notification of address changes (this needs to be done every time)
		CCLOK ( memset ( &ov, 0, sizeof(ov) ); )
		CCLOK ( ov.hEvent = hevNotify; )
		CCLTRYE ( NotifyAddrChange ( &hNotify, &ov ) == ERROR_IO_PENDING,
						GetLastError() );

		// Wait for event
		CCLTRYE ( WaitForSingleObject ( hevNotify, INFINITE ) == WAIT_OBJECT_0,
						GetLastError() );
		CCLTRYE ( (bRun == true), S_FALSE );

		// Signal change
		CCLOK ( dbgprintf ( L"Interfaces::tick:Change\r\n" ); )
		CCLOK ( _EMT(Change,adtInt(0) ); )
		#elif	__unix__ || __APPLE__
		// TODO
		usleep ( 1000000 );
		#endif
		}	// while

	// Debug
	dbgprintf ( L"} Interfaces::tick (0x%x)\n", hr );

	return hr;
	}	// tick

HRESULT Interfaces :: tickBegin ( void )
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
	HRESULT		hr = S_OK;

	// Debug
	dbgprintf ( L"Interfaces::tickBegin {\n" );

	// Create an event to be signaled for address changes
	#if	defined(_WIN32)
	CCLTRYE ( (hevNotify = CreateEvent ( NULL, FALSE, FALSE, NULL )) != NULL,
					GetLastError() );
	#endif

	// Debug
	dbgprintf ( L"} Interfaces::tickBegin (0x%x)\n", hr );

	return hr;
	}	// tickBegin

HRESULT Interfaces :: tickEnd ( void )
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
	dbgprintf ( L"Interfaces::tickEnd {}\n" );

	// Clean up
	#if	defined(_WIN32)
	if (hevNotify != NULL)
		{
		CloseHandle ( hevNotify );
		hevNotify = NULL;
		}	// if
	#endif

	return S_OK;
	}	// tickEnd

