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
		}	// if

	return hr;
	}	// create
