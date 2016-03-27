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
	ui_hInst		= ccl_hInst;
	ui_hWnd		= NULL;
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

	}	// gdiWindow

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
			pThis->onMessage ( uMsg, wParam,  lParam );
			}	// WM_CREATE
			break;

		// Destruction
		case WM_DESTROY :
			// Valid object ?
			if (pThis != NULL)
				{
				// Unassign object ptr. to avoid future messages
				SetWindowLongPtr ( hWnd, GWLP_USERDATA, NULL );

				// Allow normal processing
				pThis->onMessage ( uMsg, wParam, lParam );

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
