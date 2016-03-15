////////////////////////////////////////////////////////////////////////
//
//									ENUMDEVS.CPP
//
//					Implementation of the device enumeration node
//
////////////////////////////////////////////////////////////////////////

#include "iol_.h"
#include <stdio.h>

EnumDevices :: EnumDevices ( void ) : dl ( L"SETUPAPI.DLL" )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	guidClass	= GUID_NULL;
	hEnum			= INVALID_HANDLE_VALUE;
	idx			= 0;
	sdddil		= NULL;
	}	// EnumDevices

HRESULT EnumDevices :: onAttach ( bool bAttach )
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
	HRESULT			hr		= S_OK;

	// Attach
	if (bAttach)
		{
		adtValue	vL;

		// Setup library
		dl.AddRef();

		// Valid ?
		if ((HINSTANCE)NULL != dl)
			{
			// Functions ptr.
			CCLTRYE	( (sdedi = (BOOL (WINAPI *)( HDEVINFO, PSP_DEVINFO_DATA, const GUID *,
											DWORD, PSP_DEVICE_INTERFACE_DATA ))
							GetProcAddress ( dl, "SetupDiEnumDeviceInterfaces" ))
							!= NULL, GetLastError() );
			CCLTRYE	( (sdgcd = (HDEVINFO (WINAPI *)( const GUID *, PCTSTR, HWND, DWORD ))
							GetProcAddress ( dl, "SetupDiGetClassDevsW" ))
							!= NULL, GetLastError() );
			CCLTRYE	( (sdgdid = (BOOL (WINAPI *)( HDEVINFO, PSP_DEVICE_INTERFACE_DATA,
											PSP_DEVICE_INTERFACE_DETAIL_DATA, DWORD, DWORD *,
											PSP_DEVINFO_DATA ))
							GetProcAddress ( dl, "SetupDiGetDeviceInterfaceDetailW" ))
							!= NULL, GetLastError() );
			CCLTRYE	( (sdddil = (BOOL (WINAPI *)( HDEVINFO ))
							GetProcAddress ( dl, "SetupDiDestroyDeviceInfoList" ))
							!= NULL, GetLastError() );
			CCLTRYE	( (sdodrk = (HKEY (WINAPI *)( HDEVINFO, PSP_DEVINFO_DATA, DWORD,
											DWORD, DWORD, REGSAM ))
							GetProcAddress ( dl, "SetupDiOpenDevRegKey" ))
							!= NULL, GetLastError() );
			}	// if

		// Defaults
		if (	pnDesc->load ( adtString(L"Class"), vL ) == S_OK )
			{
			adtString	strClass(vL);
			if (strClass.length() > 0)
				CLSIDFromString ( strClass, &guidClass );
			}	// if
		}	// if

	// Detach
	else
		{
		// Clean up
		if (hEnum != INVALID_HANDLE_VALUE)
			sdddil ( hEnum );
		dl.Release();
		sdddil = NULL;
		}	// else

	return hr;
	}	// onAttach

HRESULT EnumDevices :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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
		adtString	sClass;

		// Previous state
		#if	!defined(UNDER_CE)
		if (hEnum != INVALID_HANDLE_VALUE)
			{
			sdddil ( hEnum );
			hEnum = INVALID_HANDLE_VALUE;
			}	// if
		#endif
		idx	= 0;

		// State check
		CCLTRYE ( guidClass != GUID_NULL, ERROR_INVALID_STATE );

		// Enumeration handle.  A device class or an enumerator string
		// can be specified.
		#if	!defined(UNDER_CE)
		CCLTRYE ( (hEnum = sdgcd ( &guidClass, NULL, NULL,
							DIGCF_PRESENT | DIGCF_DEVICEINTERFACE )) != 
							INVALID_HANDLE_VALUE, GetLastError() );
		#endif

		// Next item
		CCLTRY ( receive(prNext,pl,v) );
		}	// if

	// Next
	else if (_RCP(Next))
		{
		#if	!defined(UNDER_CE)
		SP_INTERFACE_DEVICE_DETAIL_DATA	*pDetail	= NULL;
		HKEY										hKey		= NULL;
		SP_DEVICE_INTERFACE_DATA			ifdata;
		SP_DEVINFO_DATA						didata;
		DWORD										sz;

		// State check
		CCLTRYE ( hEnum != INVALID_HANDLE_VALUE, ERROR_INVALID_STATE );

		// Enumerate next device
		if (hr == S_OK)
			{
			memset ( &ifdata, 0, sizeof(ifdata) );
			ifdata.cbSize	= sizeof(ifdata);
			hr = (sdedi ( hEnum, NULL, &guidClass, idx,
							&ifdata) == TRUE) ? S_OK : GetLastError();
			}	// if

		// Size of the detail structure
		CCLTRYE (	(sdgdid ( hEnum, &ifdata, NULL,
								0, &sz, NULL ) == TRUE) ||
						(GetLastError() == ERROR_INSUFFICIENT_BUFFER),
						GetLastError() );

		//// DevicePath ////

		// Allocate memory for the detail information
		CCLTRYE ( (pDetail = (SP_INTERFACE_DEVICE_DETAIL_DATA *)
						_ALLOCMEM ( sz )) != NULL, E_OUTOFMEMORY );

		// Retrieve details
		if (hr == S_OK)
			{
			pDetail->cbSize	= sizeof(SP_INTERFACE_DEVICE_DETAIL_DATA);
			memset ( &didata, 0, sizeof(didata) );
			didata.cbSize		= sizeof(didata);
			didata.cbSize		= sizeof(didata);
			hr = (sdgdid ( hEnum, &ifdata, pDetail,
								sz, NULL, &didata) == TRUE) ? S_OK : GetLastError();
			}	// if

		// Result
		if (hr == S_OK)
			{
			++idx;												// Device enumerated
			_EMT(Next,adtString(pDetail->DevicePath) );
			}	// if
		else
			{
			// No more in enumeration
			_EMT(End,adtInt() );

			// Take this opportunity to clean up
			if (hEnum != INVALID_HANDLE_VALUE)
				{
				sdddil ( hEnum );
				hEnum = INVALID_HANDLE_VALUE;
				idx	= 0;
				}	// if

			}	// else

		//// Friendly Name ////

		// Some devices like COM ports have a 'friendly' name that users are
		// used to seeing.  See if one of those exists for the current device.

		// Access device registry key
		if (hr == S_OK && (hKey = sdodrk ( hEnum, &didata, DICS_FLAG_GLOBAL, 0,
														DIREG_DEV, KEY_QUERY_VALUE )) != NULL)
			{
			WCHAR	wName[255];

			// The friendly name is GUID specific.
			if (IsEqualGUID ( guidClass, GUID_DEVINTERFACE_COMPORT ))
				{
				DWORD	dwSz = sizeof(wName);

				// Read port name
				CCLTRYE ( RegQueryValueEx ( hKey, L"PortName", NULL, NULL,
							(LPBYTE) wName, &dwSz ) == ERROR_SUCCESS, S_FALSE );
				}	// if
			else
				hr = S_FALSE;

			// Emit result
			CCLOK ( _EMT(Name,adtString ( wName ) ); )

			// Clean up
			RegCloseKey ( hKey );

			// Do not fail out of whole enumeration if friend name not found
			hr = S_OK;
			}	// if

		// Clean up
		_FREEMEM(pDetail);
		#endif

		#if	defined(UNDER_CE)

		// No device enumeration under Windows CE so just use known GUIDs to return
		// device names/locations.
		WCHAR	wPrefix[5];

		// Prefix to check
		if (hr == S_OK)
			{
			// CasaWorks ZPC
			if (IsEqualGUID ( guidClass, GUID_ZPCDevice ))
				wcscpy ( wPrefix, L"ZPC" );
			// Matrix Orbital VFD device
			else if (IsEqualGUID ( guidClass, GUID_MatrixOrb ))
				wcscpy ( wPrefix, L"FTD" );
			// ACT
			else if (IsEqualGUID ( guidClass, GUID_ZACTDevice ))
				wcscpy ( wPrefix, L"ZAT" );
			// COM
			else if (IsEqualGUID ( guidClass, GUID_ComPort ))
				wcscpy ( wPrefix, L"COM" );
			else
				hr = S_FALSE;
			}	// if

		// Presence check
		if (hr == S_OK)
			{
			HANDLE	hDev;
			WCHAR		wLoc[21];

			// Check for prescence on next index.  Enumerate to reasonable maximum limit.
			while (idx < 10)
				{
				swprintf ( wLoc, L"%ls%d:", wPrefix, ++idx );
				if ( (hDev = CreateFile ( wLoc, 0, 0, NULL, OPEN_EXISTING, 
							0, NULL )) != INVALID_HANDLE_VALUE )
					{
					// Clean up
					CloseHandle ( hDev );

				
					// Result
					peFire->emit ( adtString(wLoc) );

					// 'Friendly' name is without the colon
					wLoc[wcslen(wLoc)-1] = WCHAR('\0');
					peName->emit ( adtString ( wLoc ) );

					// Done
					break;
					}	// if
				}	// while

			// Done ?
			if (idx == 10) hr = S_FALSE;
			}	// if

		// Result
		if (hr != S_OK) peEnd->emit(adtInt());
		#endif
		}	// else if

	// Class
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

