////////////////////////////////////////////////////////////////////////
//
//									SHFCTX.CPP
//
//			Implementation of the singleton nSpace external shell factory.
//
////////////////////////////////////////////////////////////////////////

#include "nshl_.h"
#include <stdio.h>

ShellFactX :: ShellFactX ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pX	= NULL;
	}	// ShellFactX

HRESULT ShellFactX :: CreateInstance ( IUnknown *punkOuter, REFIID iid,
													void **ppv )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Create an uninitialized object.
	//
	//	PARAMETERS
	//		-	punkOuter is the outer unknown.
	//		-	iid is the requested interface.
	//		-	ppv will receive the object.
	//
	//	RETURN VALUE	
	//		S_OK on success
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	dbgprintf ( L"ShellFactX::CreateInstance:0x%x\r\n", iid.Data1 );
	
	// Setup
	(*ppv) = NULL;

	// Singleton object
	if (pX == NULL)
		{
		// New application object
		CCLTRYE ( (pX	= new ShellX()) != NULL, E_OUTOFMEMORY );
		CCLOK   ( pX->AddRef(); )
		CCLTRY  ( pX->construct() );

		// Failure
		if (hr != S_OK)
			{
			_RELEASE(pX);
			}	// if
		}	// if

	// Assign object
	if (hr == S_OK && pX != NULL)
		hr = pX->QueryInterface ( iid, ppv );

	return hr;
	}	// CreateInstance

void ShellFactX :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		CCLObject
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed
	//
	////////////////////////////////////////////////////////////////////////
	_RELEASE(pX);
	}	// destruct

HRESULT ShellFactX :: LockServer ( BOOL bLock )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Keep server open in memory.
	//
	//	PARAMETERS
	//		-	bLock is TRUE to increment the count, FALSE to decrement the count.
	//
	//	RETURN VALUE	
	//		S_OK on success
	//
	////////////////////////////////////////////////////////////////////////
	dbgprintf ( L"ShellFactX::LockServer:%d\r\n", bLock );
	return S_OK;
	}	// LockServer

HRESULT ShellFactX :: reg ( BOOL bReg )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Register or unregister the object with/from the system.
	//
	//	PARAMETERS
	//		-	bReg is TRUE to register, FALSE to unregister
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr					= S_OK;
	HKEY			hKeyTop			= NULL;
	HKEY			hKeyCur			= NULL;
	WCHAR			szIdSh[]			= L"{2534d087-8628-11d2-868c-006008addfed}";
	WCHAR			szIntfSh[]		= L"Interface\\{2534d086-8628-11d2-868c-006008addfed}";
	WCHAR			szIntfSpc[]		= L"Interface\\{2534d085-8628-11d2-868c-006008addfed}";
	WCHAR			szIntfLstn[]	= L"Interface\\{2534d084-8628-11d2-868c-006008addfed}";
	WCHAR			szTypeLib[]		= L"TypeLib\\{2534d083-8628-11d2-868c-006008addfed}";
	LONG			ret;
	adtString	strBfr;
	WCHAR			szExe[1024],szBfr[1024],szKey[255];
	WCHAR			*pw;

	// The following items must be in the registry :
	// - Type library (nshl.tlb)
	//	- Interfaces
	// - Embedded local server (this exe)

	// Current EXE location
	CCLTRYE ( (GetModuleFileName ( NULL, szExe, sizeof(szExe)/sizeof(szExe[0]) ) > 0),
					GetLastError() );

	// Convert to path
	CCLTRYE ( (pw		= wcsrchr ( szExe, WCHAR('\\') )) != NULL, E_UNEXPECTED );
	CCLOK	  ( pw[1]	= WCHAR('\0'); )

	////////////////
	// Type library
	////////////////

	// Does the type library exist ?
	if (hr == S_OK && bReg)
		{
		// Expected type library
		swprintf_s ( szBfr, L"%snshl.tlb", szExe );
		if (GetFileAttributes ( szBfr ) == INVALID_FILE_ATTRIBUTES)
			{
			MessageBox ( NULL, szBfr, L"Unable to register type library", MB_OK );
			return E_FAIL;
			}	// if
		}	// if

	// Type library.
	if (hr == S_OK)
		{
		// Register.  Register type library
		if (bReg)
			{
			// Key location
			swprintf_s ( szKey, L"%s\\1.0\\0\\win32", szTypeLib );

			// Create/access key
			CCLTRYE ( (ret = RegCreateKeyExW ( HKEY_CLASSES_ROOT, szKey, 0, NULL,
											REG_OPTION_NON_VOLATILE, KEY_ALL_ACCESS,
											NULL, &hKeyTop, NULL )) == ERROR_SUCCESS, ret );

			// Set location of type library
			swprintf_s ( szBfr, L"%snshl.tlb", szExe );
			CCLTRYE ( (ret = RegSetValueExW ( hKeyTop, NULL, 0, REG_SZ,
								(CONST BYTE *) szBfr,
								(DWORD) (wcslen(szBfr)+1)*sizeof(WCHAR) )) == ERROR_SUCCESS, ret );

			// Clean up
			if (hKeyTop != NULL)
				{
				RegCloseKey ( hKeyTop );
				hKeyTop = NULL;
				}	// if

			}	// if

		// Unregister.  Delete all the keys.
		else
			{
			// 1.0\\0\\win32
			swprintf_s ( szKey, L"%s\\1.0\\0", szTypeLib );
			if (RegOpenKeyEx ( HKEY_CLASSES_ROOT, szKey, 0, KEY_ALL_ACCESS, &hKeyTop ) == ERROR_SUCCESS)
				{
				RegDeleteKey ( hKeyTop, L"win32" );
				RegCloseKey ( hKeyTop );
				hKeyTop = NULL;
				}	// if

			// 1.0\\0
			swprintf_s ( szKey, L"%s\\1.0", szTypeLib );
			if (RegOpenKeyEx ( HKEY_CLASSES_ROOT, szKey, 0, KEY_ALL_ACCESS, &hKeyTop ) == ERROR_SUCCESS)
				{
				RegDeleteKey ( hKeyTop, L"0" );
				RegCloseKey ( hKeyTop );
				hKeyTop = NULL;
				}	// if

			// 1.0
			swprintf_s ( szKey, L"%s", szTypeLib );
			if (RegOpenKeyEx ( HKEY_CLASSES_ROOT, szKey, 0, KEY_ALL_ACCESS, &hKeyTop ) == ERROR_SUCCESS)
				{
				RegDeleteKey ( hKeyTop, L"1.0" );
				RegCloseKey ( hKeyTop );
				hKeyTop = NULL;
				}	// if

			// Top level key
			if (RegOpenKeyEx ( HKEY_CLASSES_ROOT, L"TypeLib", 0, KEY_ALL_ACCESS, &hKeyTop ) == ERROR_SUCCESS)
				{
				RegDeleteKey ( hKeyTop, L"{2534d083-8628-11d2-868c-006008addfed}" );
				RegCloseKey ( hKeyTop );
				hKeyTop = NULL;
				}	// if

			}	// else
		}	// if

	//////////////
	// Interfaces
	//////////////

	// All interfaces
	for (int i = 0;i < 3 && hr == S_OK;++i)
		{
		// Current interface
		WCHAR	*pwIntf =	(i == 0) ? szIntfSh :
								(i == 1) ? szIntfSpc : szIntfLstn;

		// Register
		if (bReg)
			{
			// Create/access interface key
			CCLTRYE ( (ret = RegCreateKeyExW ( HKEY_CLASSES_ROOT, pwIntf, 0, NULL,
											REG_OPTION_NON_VOLATILE, KEY_ALL_ACCESS,
											NULL, &hKeyTop, NULL )) == ERROR_SUCCESS, ret );

			// Interface name
			wcscpy_s ( szBfr,	(i == 0) ? L"IShellX" :
									(i == 1) ? L"INamespaceX" : L"IListenX" );
			CCLTRYE ( (ret = RegSetValueExW ( hKeyTop, NULL, 0, REG_SZ,
								(CONST BYTE *) szBfr,
								(DWORD) (wcslen(szBfr)+1)*sizeof(WCHAR) )) == ERROR_SUCCESS, ret );

			// Number of methods
			wcscpy_s ( szBfr,	(i == 1) ? L"4" : L"1" );
			CCLTRYE ( (ret = RegCreateKeyExW ( hKeyTop, L"NumMethods", 0, NULL,
											REG_OPTION_NON_VOLATILE, KEY_ALL_ACCESS,
											NULL, &hKeyCur, NULL )) == ERROR_SUCCESS, ret );
			CCLTRYE ( (ret = RegSetValueExW ( hKeyCur, NULL, 0, REG_SZ,
								(CONST BYTE *) szBfr, (DWORD) (wcslen(szBfr)+1)*sizeof(WCHAR) )) 
								== ERROR_SUCCESS, ret );
			if (hKeyCur != NULL)
				{
				RegCloseKey ( hKeyCur );
				hKeyCur = NULL;
				}	// if

			// Stub CLSID (default stub)
			wcscpy_s ( szBfr,	L"{00020424-0000-0000-C000-000000000046}" );
			CCLTRYE ( (ret = RegCreateKeyExW ( hKeyTop, L"ProxyStubClsid32", 0, NULL,
											REG_OPTION_NON_VOLATILE, KEY_ALL_ACCESS,
											NULL, &hKeyCur, NULL )) == ERROR_SUCCESS, ret );
			CCLTRYE ( (ret = RegSetValueExW ( hKeyCur, NULL, 0, REG_SZ,
								(CONST BYTE *) szBfr, (DWORD) (wcslen(szBfr)+1)*sizeof(WCHAR) )) 
								== ERROR_SUCCESS, ret );
			if (hKeyCur != NULL)
				{
				RegCloseKey ( hKeyCur );
				hKeyCur = NULL;
				}	// if

			// Type library
			wcscpy_s ( szBfr,	L"{2534d083-8628-11d2-868c-006008addfed}" );
			CCLTRYE ( (ret = RegCreateKeyExW ( hKeyTop, L"TypeLib", 0, NULL,
											REG_OPTION_NON_VOLATILE, KEY_ALL_ACCESS,
											NULL, &hKeyCur, NULL )) == ERROR_SUCCESS, ret );
			CCLTRYE ( (ret = RegSetValueExW ( hKeyCur, NULL, 0, REG_SZ,
								(CONST BYTE *) szBfr, (DWORD) (wcslen(szBfr)+1)*sizeof(WCHAR) )) 
								== ERROR_SUCCESS, ret );
			if (hKeyCur != NULL)
				{
				RegCloseKey ( hKeyCur );
				hKeyCur = NULL;
				}	// if

			// Clean up
			if (hKeyTop != NULL)
				{
				RegCloseKey ( hKeyTop );
				hKeyTop = NULL;
				}	// if

			}	// if

		// Unregister
		else if (RegOpenKeyEx ( HKEY_CLASSES_ROOT, pwIntf, 0, KEY_ALL_ACCESS, &hKeyTop ) == ERROR_SUCCESS)
			{
			// Delete subkeys
			RegDeleteKey ( hKeyTop, L"TypeLib" );
			RegDeleteKey ( hKeyTop, L"ProxyStubClsid32" );
			RegDeleteKey ( hKeyTop, L"NumMethods" );

			// Done with top level key
			RegCloseKey ( hKeyTop );
			hKeyTop = NULL;

			// Access interface key
			if (RegOpenKeyEx ( HKEY_CLASSES_ROOT, L"Interface", 0, KEY_ALL_ACCESS, &hKeyTop ) == ERROR_SUCCESS)
				{
				// Find just the CLSID of the interface
				WCHAR	*pwId = wcsstr ( pwIntf, L"\\" );
				if (pwId != NULL)
					RegDeleteKey ( hKeyTop, pwId+1 );

				// Clean up
				RegCloseKey ( hKeyTop );
				hKeyTop = NULL;
				}	// if

			}	// else if

		}	// for

	///////////////
	// Application
	///////////////

	//
	// ProgId
	//

	// Register
	if (hr == S_OK && bReg)
		{
		// Create/access prog Id
		wcscpy_s ( szBfr,	L"nSpace.ShellX" );
		CCLTRYE ( (ret = RegCreateKeyExW ( HKEY_CLASSES_ROOT, L"nSpace.ShellX", 0, NULL,
										REG_OPTION_NON_VOLATILE, KEY_ALL_ACCESS,
										NULL, &hKeyTop, NULL )) == ERROR_SUCCESS, ret );
		CCLTRYE ( (ret = RegSetValueExW ( hKeyTop, NULL, 0, REG_SZ,
							(CONST BYTE *) szBfr, (DWORD) (wcslen(szBfr)+1)*sizeof(WCHAR) )) 
							== ERROR_SUCCESS, ret );

		// CLSID
		wcscpy_s ( szBfr,	L"{2534d087-8628-11d2-868c-006008addfed}" );
		CCLTRYE ( (ret = RegCreateKeyExW ( hKeyTop, L"CLSID", 0, NULL,
										REG_OPTION_NON_VOLATILE, KEY_ALL_ACCESS,
										NULL, &hKeyCur, NULL )) == ERROR_SUCCESS, ret );
		CCLTRYE ( (ret = RegSetValueExW ( hKeyCur, NULL, 0, REG_SZ,
							(CONST BYTE *) szBfr, (DWORD) (wcslen(szBfr)+1)*sizeof(WCHAR) )) 
							== ERROR_SUCCESS, ret );
		if (hKeyCur != NULL)
			{
			RegCloseKey ( hKeyCur );
			hKeyCur = NULL;
			}	// if

		// CurVer
		wcscpy_s ( szBfr,	L"1" );
		CCLTRYE ( (ret = RegCreateKeyExW ( hKeyTop, L"CurVer", 0, NULL,
										REG_OPTION_NON_VOLATILE, KEY_ALL_ACCESS,
										NULL, &hKeyCur, NULL )) == ERROR_SUCCESS, ret );
		CCLTRYE ( (ret = RegSetValueExW ( hKeyCur, NULL, 0, REG_SZ,
							(CONST BYTE *) szBfr, (DWORD) (wcslen(szBfr)+1)*sizeof(WCHAR) )) 
							== ERROR_SUCCESS, ret );
		if (hKeyCur != NULL)
			{
			RegCloseKey ( hKeyCur );
			hKeyCur = NULL;
			}	// if
		}	// if

	// Unregister
	else if (	hr == S_OK &&
					RegOpenKeyEx ( HKEY_CLASSES_ROOT, L"nSpace.Application", 0, KEY_ALL_ACCESS, &hKeyTop ) == ERROR_SUCCESS)
		{
		// Delete subkeys
		RegDeleteKey ( hKeyTop, L"CLSID" );
		RegDeleteKey ( hKeyTop, L"CurVer" );

		// Done with top level key
		RegCloseKey ( hKeyTop );
		hKeyTop = NULL;

		// Access class key
		if (RegOpenKeyEx ( HKEY_CLASSES_ROOT, NULL, 0, KEY_ALL_ACCESS, &hKeyTop ) == ERROR_SUCCESS)
			{
			// Delete prog Id
			RegDeleteKey ( hKeyTop, L"nSpace.Application" );

			// Clean up
			RegCloseKey ( hKeyTop );
			hKeyTop = NULL;
			}	// if

		}	// else if

	// CLSID

	// Register
	if (hr == S_OK && bReg)
		{
		// Create/access prog Id
		wcscpy_s ( szBfr,	L"nSpace.ShellX" );
		CCLTRYE ( (ret = RegCreateKeyExW ( HKEY_CLASSES_ROOT, L"CLSID\\{2534d087-8628-11d2-868c-006008addfed}", 
										0, NULL, REG_OPTION_NON_VOLATILE, KEY_ALL_ACCESS,
										NULL, &hKeyTop, NULL )) == ERROR_SUCCESS, ret );
		CCLTRYE ( (ret = RegSetValueExW ( hKeyTop, NULL, 0, REG_SZ,
							(CONST BYTE *) szBfr, (DWORD) (wcslen(szBfr)+1)*sizeof(WCHAR) )) 
							== ERROR_SUCCESS, ret );

		// Current EXE location
		CCLTRYE ( (GetModuleFileName ( NULL, szBfr, sizeof(szBfr)/sizeof(szBfr[0]) ) > 0),
						GetLastError() );

		// LocalServer32
		CCLTRYE ( (ret = RegCreateKeyExW ( hKeyTop, L"LocalServer32", 0, NULL,
										REG_OPTION_NON_VOLATILE, KEY_ALL_ACCESS,
										NULL, &hKeyCur, NULL )) == ERROR_SUCCESS, ret );
		CCLTRYE ( (ret = RegSetValueExW ( hKeyCur, NULL, 0, REG_SZ,
							(CONST BYTE *) szBfr, (DWORD) (wcslen(szBfr)+1)*sizeof(WCHAR) )) 
							== ERROR_SUCCESS, ret );
		if (hKeyCur != NULL)
			{
			RegCloseKey ( hKeyCur );
			hKeyCur = NULL;
			}	// if

		// ProgId
		wcscpy_s ( szBfr,	L"nSpace.Application" );
		CCLTRYE ( (ret = RegCreateKeyExW ( hKeyTop, L"ProgId", 0, NULL,
										REG_OPTION_NON_VOLATILE, KEY_ALL_ACCESS,
										NULL, &hKeyCur, NULL )) == ERROR_SUCCESS, ret );
		CCLTRYE ( (ret = RegSetValueExW ( hKeyCur, NULL, 0, REG_SZ,
							(CONST BYTE *) szBfr, (DWORD) (wcslen(szBfr)+1)*sizeof(WCHAR) )) 
							== ERROR_SUCCESS, ret );
		if (hKeyCur != NULL)
			{
			RegCloseKey ( hKeyCur );
			hKeyCur = NULL;
			}	// if
		}	// if

	// Unregister
	else if (hr == S_OK &&
				RegOpenKeyEx ( HKEY_CLASSES_ROOT, L"CLSID\\{2534d087-8628-11d2-868c-006008addfed}", 0, KEY_ALL_ACCESS, &hKeyTop ) == ERROR_SUCCESS)
		{
		// Delete subkeys
		RegDeleteKey ( hKeyTop, L"LocalServer32" );
		RegDeleteKey ( hKeyTop, L"ProgId" );

		// Done with top level key
		RegCloseKey ( hKeyTop );
		hKeyTop = NULL;

		// Access class key
		if (RegOpenKeyEx ( HKEY_CLASSES_ROOT, L"CLSID", 0, KEY_ALL_ACCESS, &hKeyTop ) == ERROR_SUCCESS)
			{
			// Delete class ic
			RegDeleteKey ( hKeyTop, L"{2534d087-8628-11d2-868c-006008addfed}" );

			// Clean up
			RegCloseKey ( hKeyTop );
			hKeyTop = NULL;
			}	// if

		}	// else if

	// Debug
	dbgprintf ( L"ShellFactX::reg:%d:0x%x\r\n", bReg, hr );

	return hr;
	}	// reg
