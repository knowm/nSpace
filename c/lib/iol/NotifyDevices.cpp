////////////////////////////////////////////////////////////////////////
//
//									NOTIFYDEVS.CPP
//
//				Implementation of the device notification node
//
////////////////////////////////////////////////////////////////////////

#include "iol_.h"
#ifdef 	_WIN32
#include <Dbt.h>
//#include <usbiodef.h>
#include <stdio.h>

// Globals
extern HINSTANCE ccl_hInst;

NotifyDevices :: NotifyDevices ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pThrd		= NULL;
	hWndDev	= NULL;
	hDev		= NULL;
	pDctN		= NULL;
	}	// NotifyDevices

HRESULT NotifyDevices :: onAttach ( bool bAttach )
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

		// Defaults
		if (	pnDesc->load ( adtString(L"Class"), vL ) == S_OK )
			{
			adtString	strClass(vL);
			if (strClass.length() > 0)
				CLSIDFromString ( strClass, &guidClass );
			}	// if

		// Notification dictionary
		CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDctN ) );
		}	// if

	// Detach
	else
		{
		// Clean up
		receive ( prStop, L"Value", adtInt() );
		_RELEASE(pDctN);
		}	// else

	return hr;
	}	// onAttach

HRESULT NotifyDevices :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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
		// Create notification thread
		CCLTRY(COCREATE(L"Sys.Thread",IID_IThread,&pThrd));
		CCLTRY(pThrd->threadStart(this,5000));
		}	// if

	// Stop
	else if (_RCP(Stop))
		{
		// Shutdown thread/window
		if (pThrd != NULL)
			{
			pThrd->threadStop(5000);
			_RELEASE(pThrd);
			}	// if
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

HRESULT NotifyDevices :: tickAbort ( void )
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

	// Close window handle
	if (hWndDev != NULL)
		PostMessage(hWndDev,WM_QUIT,0,0);

	return S_OK;
	}	// tickAbort

HRESULT NotifyDevices :: tick ( void )
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
	HRESULT	hr			= S_OK;
	MSG		msg;

	// Run message loop until quit message is received
	memset ( &msg, 0, sizeof(msg) );
	while (GetMessage ( &msg, (HWND) NULL, 0, 0 ) != 0)
		{
		TranslateMessage(&msg);
		DispatchMessage(&msg);
		}	// while

	// Invalidate object ptr.
	SetWindowLongPtr ( hWndDev, GWLP_USERDATA, NULL );

	// WM_QUIT received
	return  S_FALSE;
	}	// tick

HRESULT NotifyDevices :: tickBegin ( void )
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
	HRESULT								hr = S_OK;
	WNDCLASS								wndClass;
	DEV_BROADCAST_DEVICEINTERFACE	devIntf;

	// Register a class for the notification window.  Ok if it fails since
	// there be other notification windows already running.
	memset ( &wndClass, 0, sizeof(wndClass) );
	wndClass.lpfnWndProc		= windowProc;
	wndClass.hInstance		= ccl_hInst;
	wndClass.lpszClassName	= L"nspc_notifydevices";
	RegisterClass ( &wndClass );

	// Create a hidden window for Windows to send messsges too
	CCLTRYE ( (hWndDev = CreateWindow ( L"nspc_notifydevices", 
					L"nspc_notifydevices", WS_POPUP, 0, 0, 10, 10, 
					NULL, NULL, ccl_hInst, this ) ) != NULL, GetLastError() );

	// Registrations
	if (hr == S_OK)
		{
		// TODO: Support list of classes ?
		memset ( &devIntf, 0, sizeof(devIntf) );
		devIntf.dbcc_size			= sizeof(devIntf);
		devIntf.dbcc_devicetype	= DBT_DEVTYP_DEVICEINTERFACE;
		devIntf.dbcc_classguid	= guidClass;
		CCLTRYE ( (hDev = RegisterDeviceNotification ( hWndDev, &devIntf,
						DEVICE_NOTIFY_WINDOW_HANDLE )) != NULL, GetLastError() );		
		}	// if

	return hr;
	}	// tickBegin

HRESULT NotifyDevices :: tickEnd ( void )
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

	// Clean up
	if (hDev != NULL)
		UnregisterDeviceNotification ( hDev );
	if (hWndDev != NULL)
		{
		DestroyWindow(hWndDev);
		hWndDev = NULL;
		}	// if

	return S_OK;
	}	// tickEnd

LRESULT CALLBACK NotifyDevices ::
	windowProc ( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Window procedure.
	//
	//	PARAMETERS
	//		-	hWnd is a handle to the window
	//		-	uMsg is the message
	//		-	wParam,lParam are the parameters for the message
	//
	//	RETURN VALUE
	//		Message dependent
	//
	////////////////////////////////////////////////////////////////////////
	NotifyDevices	*pThis	= (NotifyDevices *) GetWindowLongPtr(hWnd,GWLP_USERDATA);
	LRESULT			ret		= 0;

	// Handle messages
	switch (uMsg)
		{
		case WM_CREATE :
			{
			LPCREATESTRUCT lpcs = (LPCREATESTRUCT) lParam;

			// Store owner object ptr. in window data
			SetWindowLongPtr ( hWnd, GWLP_USERDATA, (LONG_PTR)lpcs->lpCreateParams );
			}	// WM_CREATE
			break;

		case WM_DEVICECHANGE :
//			lprintf ( LOG_DBG, L"WM_DEVICECHANGE:0x%x:0x%x", wParam, lParam );
			switch (wParam)
				{
				// Arrival/removal
				case DBT_DEVICEARRIVAL :
				case DBT_DEVICEREMOVECOMPLETE :
					{
					HRESULT				hr		= S_OK;
					DEV_BROADCAST_HDR *hdr	= (DEV_BROADCAST_HDR *) lParam;

					// Type
					CCLTRY ( pThis->pDctN->clear() );
					CCLTRY ( pThis->pDctN->store ( adtString(L"Event"), 
								adtString ( (wParam == DBT_DEVICEARRIVAL) ? L"Arrive" : L"Depart" ) ) );

					// Device type
					switch (hdr->dbch_devicetype)
						{
						// Device class
						case DBT_DEVTYP_DEVICEINTERFACE :
							{
							DEV_BROADCAST_DEVICEINTERFACE	*brd		= (DEV_BROADCAST_DEVICEINTERFACE *) lParam;
							LPOLESTR								pstrVal	= NULL;

							// Debug
							lprintf ( LOG_DBG, L"DBT_DEVTYP_DEVICEINTERFACE:%s", brd->dbcc_name );

							// Type
							CCLTRY ( pThis->pDctN->store ( adtString(L"Type"), adtString(L"Device") ) );

							// Class GUID
							CCLTRY ( StringFromCLSID ( brd->dbcc_classguid, &pstrVal ) );
							CCLTRY ( pThis->pDctN->store ( adtString(L"GUID"), adtString(pstrVal) ) );

							// Device path
							CCLTRY ( pThis->pDctN->store ( adtString(L"Name"), adtString(brd->dbcc_name) ) );

							// Result
							CCLOK ( pThis->notify(); )

							// Clean up
							if (pstrVal != NULL)
								CoTaskMemFree(pstrVal);
							}	// DBT_DEVTYP_DEVICEINTERFACE
							break;

						// Port device
						case DBT_DEVTYP_PORT :
							{
							DEV_BROADCAST_PORT	*prt = (DEV_BROADCAST_PORT *) lParam;

							// Debug
							lprintf ( LOG_DBG, L"DBT_DEVTYP_PORT:%s", prt->dbcp_name );

							// Type
							CCLTRY ( pThis->pDctN->store ( adtString(L"Type"), adtString(L"Port") ) );

							// Device path
							CCLTRY ( pThis->pDctN->store ( adtString(L"Name"), adtString(prt->dbcp_name) ) );

							// Result
							CCLOK ( pThis->notify(); )
							}	// DBT_DEVTYP_PORT
							break;

						// Logical volume
						case DBT_DEVTYP_VOLUME :
							{
							DEV_BROADCAST_VOLUME	*pVol = (DEV_BROADCAST_VOLUME *) lParam;
							int						i;
							adtString				strVol(L"C:/");

							// Debug
							lprintf ( LOG_DBG, L"DBT_DEVTYP_VOLUME:0x%x", pVol->dbcv_unitmask );

							// Extract drive letter of media
							for (i = 0;i < 26;++i)
								{
								// Bit set ?
								if (pVol->dbcv_unitmask & (1<<i))
									break;
								}	// for

							// Path to volume
							CCLTRYE	( (i < 26), ERROR_NOT_FOUND );
							CCLOK		( strVol.at(0) = (WCHAR('A')+i); )
							CCLTRY	( pThis->pDctN->store ( adtString(L"Name"), strVol ) );

							// Media type
							CCLTRY	( pThis->pDctN->store ( adtString(L"Type"), adtString (
											(pVol->dbcv_flags == DBTF_NET) ? L"Network Volume" : L"Media Volume" ) ) );

							// Result
							CCLOK   ( pThis->notify(); )
							}	// DBT_DEVTYPE_VOLUME
							break;
						}	// switch

					}	// DBT_DEVICEXXX

				}	// switch(wParam)
			break;

		case WM_DESTROY :
			// Object ptr now invalid
			SetWindowLongPtr ( hWnd, GWLP_USERDATA, 0 );
			break;

		}	// switch

	// Default beahviour
	return DefWindowProc ( hWnd, uMsg, wParam, lParam );
	}	// windowProc

#endif

