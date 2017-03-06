////////////////////////////////////////////////////////////////////////
//
//									MAINSVC.CPP
//
//					"main" for an nSpace Windows service.
//
////////////////////////////////////////////////////////////////////////

#include "nshl_.h"

#ifdef	_WIN32

// Prototypes
DWORD	WINAPI	svcHandler	( DWORD, DWORD, LPVOID, LPVOID );
HRESULT			svcReg		( IDictionary * );

void WINAPI svcEntry ( DWORD dwArgc, LPWSTR *ppArgv )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Entry point into service.
	//
	//	PARAMETERS
	//		-	dwArgc is the # of arguments in the command line
	//		-	ppArgv are the command line arguments
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT						hr				= S_OK;
	SERVICE_STATUS_HANDLE	hSvc			= NULL;
	HANDLE						hevSvc		= NULL;
	IDictionary					*pDctCmd		= NULL;
	IThread						*pThrd		= NULL;
	Shell							*pShell		= NULL;
	adtString					*pstrName	= NULL;
	bool							bCoInitd		= false;
	SERVICE_STATUS				sS;

	// Create a notification event
	CCLTRYE ( (hevSvc = CreateEvent ( NULL, TRUE, FALSE, NULL ))
					!= NULL, GetLastError() );

	// Initialize COM
	if (hr == S_OK)
		{
		// Make attempt
		hr = CoInitializeEx ( NULL, COINIT_MULTITHREADED );

		// Initialized here
		bCoInitd = (hr == S_OK);

		// Ok (debug)
		if (hr == S_FALSE) hr = S_OK;
		}	// if

	// Obtain service name from parsed command line
	if (hr == S_OK)
		{
		WCHAR			*pwQ	= NULL;
		adtString	strCmdLine(GetCommandLine());
		adtString	strName;
		adtValue		vL;
		adtIUnknown	unkV;

		// Command line.  The name of the EXE is included in the command line so
		// skip over that for remaining command line parameters.
		CCLTRYE ( strCmdLine[0] == '\"', E_UNEXPECTED );
		CCLTRYE ( (pwQ = wcschr ( &(strCmdLine.at(1)), '\"' )) != NULL, E_UNEXPECTED );
		CCLTRYE ( pwQ[1] == ' ', E_UNEXPECTED );

		// Command line dictionary
		CCLTRY ( strToVal ( adtString(pwQ+2), vL ) );
		CCLTRY ( _QISAFE((unkV=vL),IID_IDictionary,&pDctCmd) );

		// Service name
		CCLTRY  ( pDctCmd->load ( adtString(L"Name"), vL ) );
		CCLTRYE ( (pstrName = new adtString(vL)) != NULL, E_OUTOFMEMORY );
		CCLTRYE ( pstrName->length() > 0, E_UNEXPECTED );
		}	// if

	// Register service
	if (hr == S_OK)
		{
		// Initial status is pending
		memset ( &sS, 0, sizeof(sS) );
		sS.dwServiceType					= SERVICE_WIN32;
		sS.dwCurrentState					= SERVICE_START_PENDING;
		sS.dwControlsAccepted			= SERVICE_ACCEPT_STOP;
		CCLTRYE ( (hSvc = RegisterServiceCtrlHandlerEx ( (const WCHAR *) (*pstrName), 
						svcHandler, &hevSvc )) != NULL, GetLastError() );
		}	// if

	// Start shell on own thread
	CCLTRYE ( (pShell = new Shell(pDctCmd)) != NULL, E_OUTOFMEMORY );
	CCLOK   ( pShell->AddRef(); )
	CCLTRY  ( COCREATE(L"Sys.Thread",IID_IThread,&pThrd) );
	CCLTRY  ( pThrd->threadStart(pShell,30000) );

	// Service now running
	if (hr == S_OK)
		{
		sS.dwCurrentState					= SERVICE_RUNNING;
		CCLTRYE ( SetServiceStatus ( hSvc, &sS ), GetLastError() );
		}	// if

	// Wait for signal to shutdown
	CCLOK ( WaitForSingleObject ( hevSvc, INFINITE ); )

	// Signal service is shutting down
	if (hSvc != NULL)
		{
		sS.dwCurrentState	= SERVICE_STOP_PENDING;
		SetServiceStatus ( hSvc, &sS );
		}	// if

	// Clean up
	if (pThrd != NULL)
		{
		pThrd->threadStop(30000);
		pThrd->threadJoin(30000);
		_RELEASE(pThrd);
		}	// if
	_RELEASE(pShell);
	_RELEASE(pDctCmd);
	if (pstrName != NULL)	delete pstrName;
	if (bCoInitd)				CoUninitialize();
	if (hevSvc != NULL)		CloseHandle ( hevSvc );

	// Signal service stopped
	if (hSvc != NULL)
		{
		sS.dwCurrentState	= SERVICE_STOPPED;
		SetServiceStatus ( hSvc, &sS );
		}	// if

	}	// svcEntry

DWORD WINAPI svcHandler (	DWORD dwControl, DWORD dwEventType,
									LPVOID lpEventData, LPVOID lpCtx )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called back to handle service notifications.
	//
	//	PARAMETERS
	//		-	dwControl is the control code
	//		-	dwEventType,lpEventData is event information
	//		-	lpCtx is application specific
	//
	//	RETURN VALUE
	//		NO_ERROR if successful
	//
	////////////////////////////////////////////////////////////////////////
	DWORD				ret = NO_ERROR;

	// Handle code
	switch (dwControl)
		{
		case SERVICE_CONTROL_STOP :
			// Signal shutdown
			if (lpCtx != NULL) SetEvent ( *((HANDLE *)lpCtx) );
			break;
		default :
			ret = ERROR_CALL_NOT_IMPLEMENTED;
		}	// switch

	return ret;
	}	// svcHandler

HRESULT WINAPI svcMain ( IDictionary *pDctCmd )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Entry point into program.
	//
	//	PARAMETERS
	//		-	pDctCmd contains command line information
	//
	//	RETURN VALUE	
	//		0 on failure
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr				= S_OK;
	adtValue		vL;
	adtString	strExec;

	// Execute command ?
	if (hr == S_OK && pDctCmd->load ( adtString(L"Execute"), vL ) == S_OK)
		strExec = vL;

	// Register / unreigster ?
	if (hr == S_OK && (!WCASECMP(strExec,L"Register") || !WCASECMP(strExec,L"Unregister")))
		hr = svcReg ( pDctCmd );

	// Run service
	else if (hr == S_OK)
		{
		// Single service
		SERVICE_TABLE_ENTRY	ste[] =
			{
				{ L"",	svcEntry },
				{ NULL,	NULL }
			};

		// Start service (returns after service stopped)
		CCLTRYE ( StartServiceCtrlDispatcher ( ste ), GetLastError() );

		// For debug, call directly
//		entrySvc(0,NULL);
		}	// else if

	// Clean up
	dbgprintf ( L"mainSvc:Exit:hr 0x%x\r\n", hr );

	return hr;
	}	// svcMain

HRESULT svcReg ( IDictionary *pDctCmd )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Register/unregister service.
	//
	//	PARAMETERS
	//		-	pDctCmd contains command line information
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	SC_HANDLE	hSvcMgr	= NULL;
	SC_HANDLE	hSvc		= NULL;
	bool			bReg		= false;
	bool			bUnreg	= false;
	adtValue		vL;
	adtString	strExec,strName;

	// Execute command ?
	if (hr == S_OK && pDctCmd->load ( adtString(L"Execute"), vL ) == S_OK)
		strExec = vL;

	// Register/unregister
	CCLOK ( bReg	= !WCASECMP(strExec,L"Register"); )
	CCLOK ( bUnreg= !WCASECMP(strExec,L"Unregister"); )

	// Access service manager
	CCLTRYE ( (hSvcMgr = OpenSCManager ( NULL, NULL, SC_MANAGER_ALL_ACCESS ))
					!= NULL, GetLastError() );

	// Service name
	CCLTRY ( pDctCmd->load ( adtString(L"Name"), vL ) );
	CCLTRYE( (strName = vL).length() > 0, E_UNEXPECTED );

	// Register
	if (hr == S_OK && bReg)
		{
		WCHAR			szPath[MAX_PATH];
		adtString	strPath;
		adtString	strCmd;

		// Obtain path to executable
		CCLTRYE ( GetModuleFileName ( NULL, szPath, MAX_PATH ) > 0, GetLastError() );

		// Remove the 'register' command so remaining dictionary can be used
		// as the command line for the service.
		CCLOK ( pDctCmd->remove ( adtString(L"Execute") ); )
		CCLOK ( pDctCmd->remove ( adtString(L"Instance") ); )

		// Save command line dictionary
		CCLTRY ( valToStr ( adtIUnknown(pDctCmd), strCmd ) );

		// Generate command line for service.  Put EXE name in quote in case of spaces.
		CCLOK  ( strPath = L"\""; )
		CCLTRY ( strPath.append ( szPath ) );
		CCLTRY ( strPath.append ( L"\" " ) );
		CCLTRY ( strPath.append ( strCmd ) );

		// Create the service
		CCLTRYE ( (hSvc = CreateService ( hSvcMgr, strName, strName, SERVICE_ALL_ACCESS,
						SERVICE_WIN32_OWN_PROCESS, SERVICE_DEMAND_START,
						SERVICE_ERROR_NORMAL, strPath, NULL, NULL, NULL, NULL, NULL ))
						!= NULL, GetLastError() );
		}	// if

	// Unregister
	else if (hr == S_OK && !bReg)
		{
		// Access the service
		CCLTRYE ( (hSvc = OpenService ( hSvcMgr, strName, DELETE )) != NULL,
						GetLastError() );

		// Delete service
		CCLTRYE ( DeleteService ( hSvc ), GetLastError() );
		}	// else if

	// Clean up
	if (hSvc != NULL)		CloseServiceHandle ( hSvc );
	if (hSvcMgr != NULL)	CloseServiceHandle ( hSvcMgr );

	return hr;
	}	// svcReg

#endif

