////////////////////////////////////////////////////////////////////////
//
//									MAINX.CPP
//
//					"main" for external control of nSpace.
//
////////////////////////////////////////////////////////////////////////

#include "nshl_.h"

#ifdef 	_WIN32

// Globals
HANDLE		hevTermX	= NULL;

HRESULT WINAPI xMain ( IDictionary *pDctCmd )
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
	HRESULT		hr			= S_OK;
	ShellFactX	*pFact	= NULL;
	IShellX		*pSh		= NULL;
	bool			bRegCF	= false;
	bool			bReg		= false;
	DWORD			dwRegCF	= 0;
	DWORD			dwReg		= 0;
	adtString	strExec;
	adtValue		vL;

	// Some system components require this
	#if	defined(_WIN32) && !defined(UNDER_CE)
	CCLTRY ( CoInitializeSecurity ( NULL, -1, NULL, NULL,
					RPC_C_AUTHN_LEVEL_DEFAULT, RPC_C_IMP_LEVEL_IMPERSONATE,
					NULL, EOAC_SECURE_REFS, NULL ) );
	#endif

	// Execute command
	if (hr == S_OK && pDctCmd->load ( adtString(L"Execute"), vL ) == S_OK)
		strExec = vL;

	// Registration
	if (!WCASECMP(strExec,L"Register") || !WCASECMP(strExec,L"Unregister"))
		{
		// Perform registration
		ShellFactX::reg ( !WCASECMP(strExec,L"Register") );

		return hr;
		}	// if

	// Create an event that will be signaled when it is time to quit
	CCLTRYE ( (hevTermX = CreateEvent ( NULL, TRUE, FALSE, NULL )) != NULL, 
					GetLastError() );

	// Create a class factory for the nSpace application object.
	CCLTRYE ( (pFact = new ShellFactX()) != NULL, E_OUTOFMEMORY );
	CCLOK   ( pFact->AddRef(); )

	// Register the class factory for the object
	CCLTRY ( CoRegisterClassObject ( CLSID_ShellX, (IClassFactory *) pFact, 
					CLSCTX_LOCAL_SERVER, REGCLS_MULTIPLEUSE, &dwRegCF ) );
	CCLOK  ( bRegCF = true; )

	// Create application object
	CCLTRY	( pFact->CreateInstance ( NULL, IID_IShellX, (void **) &pSh ) );

	// Register object in global table
	CCLTRY ( RegisterActiveObject ( pSh, CLSID_ShellX, ACTIVEOBJECT_STRONG, &dwReg ) );
	CCLOK  ( bReg	= true; )

	// Run until stopped by external shell object
	CCLOK ( dbgprintf ( L"mainX::Waiting for shutdown:sizeof(ADTVALUE) %d:sizeof(adtString) %d\r\n", 
				sizeof(ADTVALUE), sizeof(adtString) ); )
	CCLOK ( WaitForSingleObject ( hevTermX, INFINITE ); )

	// Clean up
	if (bReg)		RevokeActiveObject(dwReg,NULL);
	_RELEASE(pSh);
	if (bRegCF)		CoRevokeClassObject(dwRegCF);
	_RELEASE(pFact);
	if (hevTermX != NULL) CloseHandle ( hevTermX );
	dbgprintf ( L"mainX:Exit:hr 0x%x\r\n", hr );

	return hr;
	}	// xMain

#endif

