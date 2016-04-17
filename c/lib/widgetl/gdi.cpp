////////////////////////////////////////////////////////////////////////
//
//									GDI.CPP
//
//					GDI base class implementations
//
////////////////////////////////////////////////////////////////////////

#include "widgetl_.h"

// Globals
extern HINSTANCE	ccl_hInst;

gdiWindow :: gdiWindow ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for object.
	//
	////////////////////////////////////////////////////////////////////////
	ui_hInst	= ccl_hInst;
	pCtls		= NULL;
	ui_hWnd	= NULL;
	}	// gdiWindow

gdiWindow :: ~gdiWindow ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Destructor for object.
	//
	////////////////////////////////////////////////////////////////////////

	// In case window is still up
	if (ui_hWnd != NULL)
		{
		// Unassign object ptr. to avoid future messages
		SetWindowLongPtr ( *this, GWLP_USERDATA, NULL );

		// Close the window
		DestroyWindow ( ui_hWnd );
		}	// if

	// Clean up
	_RELEASE(pCtls);
	}	// ~gdiWindow

HRESULT gdiWindow :: bind ( IDictionary *pCtl, U32 id )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Bind a child window to provided dictionary.
	//
	//	PARAMETERS
	//		-	pCtl is a ptr. to a dictionary for the widget
	//		-	id is the child window Id
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr		= S_OK;
	HWND		hWndC	= NULL;
	adtValue	vL;

	// Obtain child handle
	CCLTRYE ( (hWndC = GetWindow ( *this, GW_CHILD )) != NULL,
					E_INVALIDARG );

	// Verify window is not already bound
	CCLTRYE ( pCtls->load ( adtLong((U64)hWndC), vL ) != S_OK,
					E_UNEXPECTED );

	// Associate dictionary with window handle
	CCLTRY ( pCtls->store ( adtLong((U64)hWndC), adtIUnknown(pCtl) ) );

	// Store the child window inside its dictionary
	CCLTRY ( pCtl->store ( adtString(L"Window"), adtLong((U64)hWndC) ) );

	return hr;
	}	// bind

HRESULT gdiWindow :: classInfo ( WNDCLASSEX *pC )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Fill in window class information.
	//
	//	PARAMETERS
	//		-	pC will receive the info.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Defaults
	pC->lpfnWndProc	= windowProc;
	pC->hInstance		= ui_hInst;
	pC->hIcon			= LoadIcon ((HINSTANCE)NULL,IDI_APPLICATION);
	pC->hCursor			= LoadCursor((HINSTANCE)NULL,IDC_ARROW);
	pC->hbrBackground	= (HBRUSH)(COLOR_WINDOW+1);
	pC->lpszClassName	= L"nSpaceWindow";

	return hr;
	}	// classInfo

HRESULT gdiWindow :: create ( HWND hParent, LPCWSTR pwTitle, DWORD dwStyle )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Creates the window.
	//
	//	PARAMETERS
	//		-	hParent is the parent window
	//		-	pwTitle is the title for the window
	//		-	dwStyle is the window style to use
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;
	WNDCLASSEX	clsWnd;

	// Window class registration
	if (hr == S_OK)
		{
		// Obtain class information
		memset ( &clsWnd, 0, sizeof(clsWnd) );
		clsWnd.cbSize = sizeof(clsWnd);
		CCLTRY ( classInfo(&clsWnd) );

		// Register the class
		CCLOK ( RegisterClassEx ( &clsWnd ); )
		}	// if

	// Create the window
	if (hr == S_OK)
		{
		// Create
		ui_hWnd = CreateWindow ( clsWnd.lpszClassName, pwTitle,
				WS_CHILD|WS_CLIPCHILDREN|WS_CLIPSIBLINGS,
				CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT,
				hParent, NULL, *this, this );
		hr = (ui_hWnd != NULL) ? S_OK  : GetLastError();
		}	// if

	// Create a dictionary to keep track of bound widgets
	CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pCtls ) );

	return hr;
	}	// create

LRESULT gdiWindow :: onMessage ( UINT uMsg, WPARAM wParam, LPARAM lParam )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Process a window message.
	//
	//	PARAMETERS
	//		-	uMsg is the message
	//		-	wParam,lParam are message parameters
	//
	//	RETURN VALUE
	//		WindowProc return value
	//
	////////////////////////////////////////////////////////////////////////

	// Process message
	switch (uMsg)
		{
		// Command message
		case WM_COMMAND :
			{
			HRESULT		hr			= S_OK;
			HWND			hWndC		= (HWND)lParam;
			IDictionary *pCtl		= NULL;
			IReceptor	*pRcpF	= NULL;
			IReceptor	*pRcpT	= NULL;
			adtValue		vL;
			adtIUnknown	unkV;

			// Is there a dictionary associated with 
			// Is there a receptor associated with
			CCLTRY ( pCtls->load ( adtLong((U64)hWndC), vL ) );
			CCLTRY ( _QISAFE((unkV=vL),IID_IDictionary,&pCtl) );
			CCLTRY ( _QISAFE(unkV,IID_IReceptor,&pRcpF) );
			switch ( HIWORD(wParam) )
				{
				case BN_CLICKED :
					// Attempt to load the proper emitter
					CCLTRY ( pCtl->load ( adtString(L"OnClick"), vL ) );
					CCLTRY ( _QISAFE((unkV=vL),IID_IReceptor,&pRcpT) );
					CCLOK  ( pRcpT->receive ( pRcpF, L"Value", 
								adtBool ( ( SendMessage ( hWndC, 
								BM_GETCHECK, 0, 0 ) == BST_CHECKED ) ) ); )
					break;
				}	// switch

			// Clean up
			_RELEASE(pRcpT);
			_RELEASE(pRcpF);
			_RELEASE(pCtl);
			}	// WM_COMMAND
			break;
		}	// switch

	return DefWindowProc ( *this, uMsg, wParam, lParam );
	}	// onMessage

LRESULT CALLBACK gdiWindow :: windowProc (	HWND hWnd, UINT uMsg, 
															WPARAM wParam, LPARAM lParam )
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
	LRESULT		ret = 0;
	gdiWindow	*pThis;

	// Extract object pointer
	pThis = (gdiWindow *) GetWindowLongPtr(hWnd,GWLP_USERDATA);

	// Process message
	switch(uMsg)
		{
		// Creation
		case WM_CREATE :
			{
			// Object is at lParam
			pThis = (gdiWindow *) lParam;

			// Assign object to window
			SetWindowLongPtr ( hWnd, GWLP_USERDATA, (LONG_PTR)pThis );

			// Assign window handle
			pThis->ui_hWnd = hWnd;

			// Process
			ret = pThis->onMessage ( uMsg, wParam,  lParam );
			}	// WM_CREATE
			break;

		// Destruction
		case WM_DESTROY :
			// Valid object ?
			if (pThis != NULL)
				{
				// No more bound controls
				if (pThis->pCtls != NULL)
					pThis->pCtls->clear();

				// Unassign object ptr. to avoid future messages
				SetWindowLongPtr ( hWnd, GWLP_USERDATA, NULL );

				// Allow normal processing
				ret = pThis->onMessage ( uMsg, wParam, lParam );

				// Window handle value
				pThis->ui_hWnd = NULL;
				}	// if	
			break;

		// Object or default processing
		default :
			{
			// Process
			if (pThis != NULL)
				ret = pThis->onMessage ( uMsg, wParam, lParam );
			else
				ret = DefWindowProc ( hWnd, uMsg, wParam, lParam );
			}	// default

		}	// switch

	return ret;
	}	// windowProc
