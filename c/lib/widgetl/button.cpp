////////////////////////////////////////////////////////////////////////
//
//								BUTTON.CPP
//
//					Button class implementation
//
////////////////////////////////////////////////////////////////////////

#define  INITGUID
#include "widgetl_.h"
#include <stdio.h>

// Globals
extern HINSTANCE	ccl_hInst;

Button :: Button ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pDct	= NULL;
	pOwn	= NULL;
	}	// Button

HRESULT Button :: onAttach ( bool bAttach )
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
/*
LRESULT Button :: onClicked ( IDictionary *pCtl )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the button is 'clicked'.
	//
	//	PARAMETERS
	//		-	pCtl is the assigned control dictionary
	//
	//	RETURN VALUE
	//		Zero if message is processed.
	//
	////////////////////////////////////////////////////////////////////////
	adtIUnknown	unkV(pCtl);

	// Notify
//	dbgprintf ( L"Button::onClicked:%p\r\n", pCtl );
	_EMT(Dictionary,unkV);

	// Emit 'checked' state along with click notification.
	_EMT(Click,
		adtBool ( 
			( SendMessage ( *this, BM_GETCHECK, 0, 0 ) == BST_CHECKED ) ) );

	return 1;
	}	// onClicked
*/

HRESULT Button :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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
/*
	// Open
	if (_RCP(Open))
		{
		HWND			hWndP		= NULL;
		DWORD			dwStyle	= WS_CHILD|WS_VISIBLE|BS_MULTILINE;
		adtInt		iX			= 0;
		adtInt		iY			= 0;
		adtInt		iW			= 20;
		adtInt		iH			= 20;
		adtString	strText(L""),strType(L"");
		adtValue		vL;
		adtLong		lWnd;

		// State check
		CCLTRYE ( pDct != NULL, ERROR_INVALID_STATE );

		// Owner required for child controls
		CCLTRYE ( (pOwn != NULL), ERROR_INVALID_STATE );
		CCLTRY  ( pOwn->load ( adtString(L"Window"), lWnd ) );
		CCLTRYE ( (hWndP = (HWND)(U64)lWnd) != NULL, ERROR_INVALID_STATE );

		// Possible button parameters
		if (hr == S_OK)
			{
			if (pDct->load ( adtString(L"Xpos"), vL ) == S_OK)
				iX = vL;
			if (pDct->load ( adtString(L"Ypos"), vL ) == S_OK)
				iY = vL;
			if (pDct->load ( adtString(L"Width"), vL ) == S_OK)
				iW = vL;
			if (pDct->load ( adtString(L"Height"), vL ) == S_OK)
				iH = vL;
			if (pDct->load ( adtString(L"Text"), vL ) == S_OK)
				strText = vL;
			if (pDct->load ( adtString(L"Type"), vL ) == S_OK)
				strType = vL;
			}	// if

		// Button style
		if (hr == S_OK && !WCASECMP(strType,L"Check"))
			dwStyle |= BS_CHECKBOX;
		else if (hr == S_OK && !WCASECMP(strType,L"AutoCheck"))
			dwStyle |= BS_AUTOCHECKBOX;
		else if (hr == S_OK && !WCASECMP(strType,L"Radio"))
			dwStyle |= BS_RADIOBUTTON;
		else if (hr == S_OK && !WCASECMP(strType,L"AutoRadio"))
			dwStyle |= BS_AUTORADIOBUTTON;
		else if (hr == S_OK && !WCASECMP(strType,L"GroupBox"))
			dwStyle |= BS_GROUPBOX;
		else if (hr == S_OK && !WCASECMP(strType,L"Push"))
			dwStyle |= BS_PUSHLIKE;

		// Enabled ?
		if (	hr == S_OK && 
				pDct->load ( adtString(L"Enable"), vL ) == S_OK &&
				adtBool(vL) == false)
			dwStyle |= (WS_DISABLED);

		// Already 'open' ?
		if (hr == S_OK && pDct->load ( adtString(L"Window"), vL )  != S_OK)
			{
			// Create control
			CCLTRYE ( (hWnd = CreateWindow ( L"Button", strText, dwStyle,
							iX, iY, iW, iH, hWndP, (HMENU)0, 
							ccl_hInst, (gdiControl *) this )) != NULL, GetLastError() );

			// Assign the control to the dictionary
			CCLTRY ( assign ( hWnd, pDct ) );

			// This is for style purposes.
			#if	!defined(UNDER_CE)
			CCLOK  ( SendMessage ( hWnd, WM_SETFONT,
							(WPARAM) GetStockObject ( DEFAULT_GUI_FONT ), TRUE ); )
			#endif

			// Result
			CCLOK ( _EMT(Open,adtIUnknown(pDct)); )
			}	// if

		// Otherwise just update the state of the control
		else if (hr == S_OK)
			{
			// Control window handle
			CCLTRYE	( IsWindow(hWnd), ERROR_INVALID_STATE );

			// Update size/position
			CCLOK ( MoveWindow ( hWnd, iX, iY, iW, iH, TRUE ); )
			}	// else if

		// Update other state
		CCLOK ( SetWindowText ( hWnd, strText ); )

		// Enable/clicked
		if (hr == S_OK && pDct->load ( adtString(L"Enable"), vL ) == S_OK)
			receive ( prEnable, pl, vL );
		if (hr == S_OK && pDct->load ( adtString(L"Clicked"), vL ) == S_OK)
			receive ( prClick, pl, vL );
		if (hr == S_OK && pDct->load ( adtString(L"Highlight"), vL ) == S_OK)
			receive ( prHighlight, pl, vL );

		// Clean up
		if (hr != S_OK && hWnd != NULL)
			DestroyWindow ( hWnd );
		}	// if

	// Close
	else if (_RCP(Close))
		{
		adtLong	lWnd;

		// State check
		CCLTRYE ( pDct != NULL, ERROR_INVALID_STATE );

		// Widget
		CCLOK ( unassign ( pDct ); )

		// Close window
		if (hWnd != NULL)
			{
			DestroyWindow ( hWnd );
			hWnd = NULL;
			}	// if

		// Result
		CCLOK ( _EMT(Close,adtIUnknown(pDct)); )
		}	// if

	// Clicked
	else if (_RCP(Click))
		{
		adtLong	lWnd;
		adtBool	bClk(v);

		// State check
		CCLTRYE	( pDct != NULL, ERROR_INVALID_STATE );
		CCLOK		( pDct->load ( adtString(L"Window"), lWnd ); )
		CCLTRYE	( IsWindow((hWnd = (HWND)(U64)lWnd)), ERROR_INVALID_STATE );

		// Update dictionary and window
		if (pDct != NULL)
			pDct->store ( adtString(L"Clicked"), bClk );
		if (hr == S_OK)
			SendMessage ( hWnd, BM_SETCHECK, (bClk == true) ? BST_CHECKED : BST_UNCHECKED, 0l );
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

	// Highlight
	else if (_RCP(Highlight))
		{
		adtLong	lWnd;
		adtBool	bHigh(v);

		// State check
		CCLTRYE	( pDct != NULL, ERROR_INVALID_STATE );
		CCLOK		( pDct->load ( adtString(L"Window"), lWnd ); )
		CCLTRYE	( IsWindow((hWnd = (HWND)(U64)lWnd)), ERROR_INVALID_STATE );

		// Update dictionary and window
		if (pDct != NULL)
			pDct->store ( adtString(L"Highlight"), bHigh );
		if (hr == S_OK)
			SendMessage ( hWnd, BM_SETSTATE, (bHigh == true) ? TRUE : FALSE, 0l );
		}	// else if

	// State
	else if (_RCP(Dictionary))
		{
		adtLong		lWnd;
		adtIUnknown	unkV(v);
		_RELEASE(pDct);
		_QISAFE(unkV,IID_IDictionary,&pDct);

		// Widget handle
		if (pDct != NULL && pDct->load ( adtString(L"Window"), lWnd ) == S_OK)
			hWnd = ((HWND)(U64)lWnd);
		else
			hWnd = NULL;
		}	// if
	else if (_RCP(Owner))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pOwn);
		_QISAFE(unkV,IID_IDictionary,&pOwn);
		}	// if
	else
*/		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

/*
//
// Base class
//

gdiButton :: gdiButton ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for object.
	//
	////////////////////////////////////////////////////////////////////////
	}	// gdiButton

LRESULT gdiButton :: onClicked ( IDictionary *pCtl )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the button is 'clicked'.
	//
	//	PARAMETERS
	//		-	pCtl is the assigned control dictionary
	//
	//	RETURN VALUE
	//		Zero if message is processed.
	//
	////////////////////////////////////////////////////////////////////////
	return 1;
	}	// onClicked

LRESULT gdiButton :: onMessage ( IDictionary *pCtl, UINT uMsg, 
												WPARAM wParam, LPARAM lParam )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Process a window message.
	//
	//	PARAMETERS
	//		-	pCtl is the assigned control dictionary
	//		-	uMsg is the message
	//		-	wParam,lParam are the parameters for the message
	//
	//	RETURN VALUE
	//		WindowProc return value
	//
	////////////////////////////////////////////////////////////////////////
	LRESULT	ret = 0;

	// Widget specific messages
	switch (uMsg)
		{
		case WM_COMMAND :
			switch (HIWORD(wParam))
				{
				case BN_CLICKED :
					onClicked(pCtl);
					break;
				}	// switch
			break;
		}	// switch

	return gdiControl::onMessage ( pCtl, uMsg, wParam, lParam );
	}	// onMessage

*/