////////////////////////////////////////////////////////////////////////
//
//								LABEL.CPP
//
//					Label class implementation
//
////////////////////////////////////////////////////////////////////////

#include "widgetl_.h"
#include <stdio.h>

// Globals
extern HINSTANCE	ccl_hInst;

//
// Node
//

Label :: Label ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pDct	= NULL;
	pOwn	= NULL;
	}	// Label

HRESULT Label :: onAttach ( bool bAttach )
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

	// Detach
	if (!bAttach)
		{
		_RELEASE(pDct);
		_RELEASE(pOwn);
		}	// if

	return S_OK;
	}	// onAttach

HRESULT Label :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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
	HRESULT	hr		= S_OK;
	HWND		hWnd	= NULL;
	adtLong	lWnd;

	// Threading
	if (gdiControl::receive ( pOwn, this, pr, pl, v ) != S_OK)
		return S_OK;

	// Open
	if (_RCP(Open))
		{
		HWND			hWndP			= NULL;
		adtValue		vL;

		// State check
		CCLTRYE ( pDct != NULL, ERROR_INVALID_STATE );

		// Already open ?
		CCLTRYE ( pDct->load ( adtString(L"Window"), vL ) != S_OK, ERROR_INVALID_STATE );

		// Owner required for child controls
		CCLTRYE ( (pOwn != NULL), ERROR_INVALID_STATE );
		CCLTRY  ( pOwn->load ( adtString(L"Window"), lWnd ) );
		CCLTRYE ( (hWndP = (HWND)(U64)lWnd) != NULL, ERROR_INVALID_STATE );

		// Create control
		CCLTRYE ( (hWnd = CreateWindowEx ( WS_EX_TRANSPARENT, L"Static", L"Static", 
						WS_CHILD|SS_CENTER|SS_CENTERIMAGE|SS_WORDELLIPSIS,
						0, 0, 10, 10,
						hWndP, (HMENU)0, ccl_hInst, (gdiControl *) this )) 
						!= NULL, GetLastError() );

		// Assign the control to the dictionary
		CCLTRY ( assign ( hWnd, pDct ) );

		// This is for style purposes.
		#if	!defined(UNDER_CE)
		CCLOK  ( SendMessage ( hWnd, WM_SETFONT,
						(WPARAM) GetStockObject ( DEFAULT_GUI_FONT ), TRUE ); )
		#endif

		// Result
		CCLOK ( _EMT(Open,adtIUnknown(pDct)); )

		// Clean up
		if (hr != S_OK && hWnd != NULL)
			DestroyWindow ( hWnd );
		}	// if

	// Close
	else if (_RCP(Close))
		{
		// State check
		CCLTRYE ( pDct != NULL, ERROR_INVALID_STATE );

		// Widget
		CCLOK ( pDct->load ( adtString(L"Window"), lWnd ); )
		CCLOK ( unassign ( pDct ); )

		// Close window
		if ( (hWnd = (HWND)(U64)lWnd) != 0 )
			{
			SetWindowText ( hWnd, L"DestroyMe" );
			if (!DestroyWindow ( hWnd ))
				dbgprintf ( L"Label::receive:Close:Destroy failed:%d\r\n", GetLastError() );
			}	// if

		// Result
		CCLOK ( _EMT(Close,adtIUnknown(pDct)); )
		}	// if

	// Matrix
	else if (_RCP(Matrix))
		{
		adtLong		lWnd;
		adtIUnknown	unkV(v);
		RECT			rct;

		// State check
		CCLTRYE	( pDct != NULL, ERROR_INVALID_STATE );
		CCLOK		( pDct->load ( adtString(L"Window"), lWnd ); )
		CCLTRYE	( IsWindow((hWnd = (HWND)(U64)lWnd)), ERROR_INVALID_STATE );

		// Dimensions
		CCLTRY ( gdiControl::transform ( GetParent(hWnd), unkV, &rct ) );

		// Update size/position
		CCLOK ( MoveWindow ( hWnd, rct.left, rct.top, 
									(rct.right-rct.left), (rct.bottom-rct.top), TRUE ); )

		// Do this here ?
		CCLOK ( ShowWindow ( hWnd, SW_SHOWNORMAL ); )
		}	// else if

	// Enable
	else if (_RCP(Enable))
		{
		adtLong	lWnd;
		adtBool	bEn(v);

		// State check
		CCLTRYE	( pDct != NULL, ERROR_INVALID_STATE );
		CCLOK		( pDct->load ( adtString(L"Window"), lWnd ); )
		CCLTRYE	( IsWindow((hWnd = (HWND)(U64)lWnd)), ERROR_INVALID_STATE );

		// Update dictionary and window
		if (pDct != NULL)
			pDct->store ( adtString(L"Enable"), bEn );
		if (hr == S_OK)
			EnableWindow ( hWnd, (bEn == true) );
		}	// else if

	//
	// Label specific
	//

	// Set label
	else if (_RCP(Text))
		{
		adtString	strV;

		// State check
		CCLTRYE	( pDct != NULL, ERROR_INVALID_STATE );
		CCLOK		( pDct->load ( adtString(L"Window"), lWnd ); )
		CCLTRYE	( IsWindow((hWnd = (HWND)(U64)lWnd)), ERROR_INVALID_STATE );

		// Obtain string version of value
		CCLTRY ( adtValue::toString ( v, strV ) );

		// Update dictionary and window
		if (pDct != NULL)
			pDct->store ( adtString(L"Text"), strV );
		if (hr == S_OK)
			SetWindowText ( hWnd, strV );
		}	// else if

	// State
	else if (_RCP(Dictionary))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pDct);
		_QISAFE(unkV,IID_IDictionary,&pDct);
		}	// if
	else if (_RCP(Owner))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pOwn);
		_QISAFE(unkV,IID_IDictionary,&pOwn);
		}	// if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive


