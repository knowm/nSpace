////////////////////////////////////////////////////////////////////////
//
//								REGISTER.CPP
//
//				Routines to handle registration of objects
//
////////////////////////////////////////////////////////////////////////

#include "ccl.h"
#include "../adtl/adtl.h"
#include <stdio.h>

// Globals
extern HINSTANCE	ccl_hInst;

#ifdef	_WIN32
void cclDeleteKey ( void *hTop, PCWSTR lpStr )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Recursively deletes a registry key.
	//
	//	PARAMETERS
	//		-	hTop is a handle to the top level key
	//		-	lpStr is the name of the key to delete
	//
	////////////////////////////////////////////////////////////////////////
	HKEY	hKeyCur = NULL;
	WCHAR	keyname[MAX_PATH];

	// Access key
	if (RegOpenKeyExW ( (HKEY) hTop, lpStr, 0, KEY_ALL_ACCESS, &hKeyCur ) ==
			ERROR_SUCCESS)
		{
		// Make sure subkeys are deleted
		DWORD		keysize	= sizeof(keyname)/sizeof(WCHAR);
		FILETIME	ft;
		while (RegEnumKeyExW ( hKeyCur, 0, keyname, &keysize, NULL, NULL,
					NULL, &ft ) == ERROR_SUCCESS)
			{
			cclDeleteKey ( hKeyCur, keyname );
			keysize	= sizeof(keyname);
			}	// while
		RegCloseKey ( hKeyCur );
		}	// if

	// Delete key
	RegDeleteKeyW ( (HKEY) hTop, lpStr );
	}	// cclDeleteKey
#endif

STDAPI cclRegister ( CLSID clsid, LPCWSTR progid, bool bReg )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Registers/unregisters an object.
	//
	//	PARAMETERS
	//		-	clsid specifies the class ID
	//		-	progid is the ProgID of the class
	//		-	bReg is TRUE of register FALSE to unregister
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr				= S_OK;
	#ifdef		_WIN32
	HKEY			hKeyTop		= NULL;
	HKEY			hKeyCur		= NULL;
	WCHAR			*pclsidstr	= NULL;
	WCHAR			pathbufr[MAX_PATH];
	int			ipb			= sizeof(pathbufr)/sizeof(WCHAR);
	LONG			ret;

	// Register
	if (bReg)
		{
		// Convert class ID to string
		if (hr == S_OK) hr = StringFromCLSID ( clsid, &pclsidstr );

		///////////////////////
		// CLSID\\{XXXXX...XX}
		///////////////////////
		// Top-level key
		if (hr == S_OK)
			{
			swprintf ( pathbufr, ipb, L"CLSID\\%ls", pclsidstr );
			ret = RegCreateKeyExW ( HKEY_CLASSES_ROOT, pathbufr, 0, NULL,
											REG_OPTION_NON_VOLATILE, KEY_ALL_ACCESS,
											NULL, &hKeyTop, NULL );
			if (ret != ERROR_SUCCESS) hr = ret;
			}	// if

		// Default value
		if (hr == S_OK)
			{
			ret = RegSetValueExW ( hKeyTop, NULL, 0, REG_SZ,
											(CONST BYTE *) progid,
											(DWORD) (wcslen(progid)+1)*sizeof(WCHAR) );
			if (ret != ERROR_SUCCESS) hr = ret;
			}	// if

		//////////////////
		// InProcServer32
		//////////////////
		if (hr == S_OK)
			{
			ret = RegCreateKeyExW ( hKeyTop, L"InprocServer32", 0, NULL,
											REG_OPTION_NON_VOLATILE, KEY_ALL_ACCESS,
											NULL, &hKeyCur, NULL );
			if (ret != ERROR_SUCCESS) hr = ret;
			}	// if
		if (hr == S_OK)
			{
			GetModuleFileNameW ( ccl_hInst, pathbufr, sizeof(pathbufr)/sizeof(WCHAR) );
			ret = RegSetValueExW ( hKeyCur, NULL, 0, REG_SZ,
											(CONST BYTE *) pathbufr,
											(DWORD) (wcslen(pathbufr)+1)*sizeof(WCHAR) );
			if (ret != ERROR_SUCCESS) hr = ret;
			}	// if
		if (hr == S_OK)
			{
			ret = RegSetValueExW ( hKeyCur, L"ThreadingModel", 0, REG_SZ,
											(CONST BYTE *) L"Both", 10 );
//											(CONST BYTE *) L"Free", 10 );
			if (ret != ERROR_SUCCESS) hr = ret;
			}	// if

		// Clean up
		if (hKeyCur != NULL)
			{
			RegCloseKey ( hKeyCur );
			hKeyCur = NULL;
			}	// if

		//////////////////
		// ProgId
		//////////////////
		if (hr == S_OK)
			{
			ret = RegCreateKeyExW ( hKeyTop, L"ProgID", 0, NULL,
											REG_OPTION_NON_VOLATILE, KEY_ALL_ACCESS,
											NULL, &hKeyCur, NULL );
			if (ret != ERROR_SUCCESS) hr = ret;
			}	// if
		if (hr == S_OK)
			{
			ret = RegSetValueExW ( hKeyCur, NULL, 0, REG_SZ,
											(CONST BYTE *) progid,
											(DWORD) (wcslen(progid)+1)*sizeof(WCHAR) );
			if (ret != ERROR_SUCCESS) hr = ret;
			}	// if

		// Clean up
		if (hKeyCur != NULL)
			{
			RegCloseKey ( hKeyCur );
			hKeyCur = NULL;
			}	// if

		// Top level prog ID
		if (hr == S_OK)
			{
			swprintf ( pathbufr, ipb, L"%ls\\CLSID", progid );
			ret = RegCreateKeyExW ( HKEY_CLASSES_ROOT, pathbufr, 0, NULL,
											REG_OPTION_NON_VOLATILE, KEY_ALL_ACCESS,
											NULL, &hKeyCur, NULL );
			if (ret != ERROR_SUCCESS) hr = ret;
			}	// if

		// Default value
		if (hr == S_OK)
			{
			swprintf ( pathbufr, ipb, L"%ls", pclsidstr );
			ret = RegSetValueExW ( hKeyCur, NULL, 0, REG_SZ,
											(CONST BYTE *) pathbufr,
											(DWORD) (wcslen(pathbufr)+1)*sizeof(WCHAR) );
			if (ret != ERROR_SUCCESS) hr = ret;
			}	// if

		// Clean up
		if (hKeyCur != NULL)
			{
			RegCloseKey ( hKeyCur );
			hKeyCur = NULL;
			}	// if

		}	// if

	// Unregister
	else
		{
		// Convert class ID to string
		if (StringFromCLSID ( clsid, &pclsidstr ) == S_OK)
			{
			swprintf ( pathbufr, ipb, L"CLSID\\%ls", pclsidstr );
			cclDeleteKey ( HKEY_CLASSES_ROOT, pathbufr );
			cclDeleteKey ( HKEY_CLASSES_ROOT, progid );
			}	// if

		}	// else

	// Clean up
	if (hKeyCur != NULL) RegCloseKey ( hKeyCur );
	if (hKeyTop != NULL) RegCloseKey ( hKeyTop );
	if (pclsidstr != NULL) CoTaskMemFree ( pclsidstr );
	#endif

	return hr;
	}	// cclRegister

