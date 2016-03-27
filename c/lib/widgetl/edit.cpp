////////////////////////////////////////////////////////////////////////
//
//									EDIT.CPP
//
//						Edit class implementation
//
////////////////////////////////////////////////////////////////////////

#include "widgetl_.h"
#include <stdio.h>

// Globals
extern HINSTANCE	ccl_hInst;

//
// Node
//

Edit :: Edit ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pDct	= NULL;
	pOwn	= NULL;
	}	// Edit

HRESULT Edit :: onAttach ( bool bAttach )
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

LRESULT Edit :: onEnter ( IDictionary *pCtl )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the user hits the 'enter' key in the edit box.
	//
	//	PARAMETERS
	//		-	pCtl is the assigned control dictionary
	//
	//	RETURN VALUE
	//		Zero if message is processed.
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;
	adtString	str;

	// Obtain current text
	CCLTRY ( getText ( str ) );

	// If different, emit
	if (hr == S_OK && WCASECMP(str,strText))
		{
		// New text
		strText = str;

		// Emit result
		CCLOK ( _EMT(Dictionary,adtIUnknown(pCtl) ); )
		CCLOK ( _EMT(Text,strText ); )
		}	// if

	return gdiEdit::onEnter(pCtl);
	}	// onEnter

LRESULT Edit :: onFocus ( IDictionary *pCtl, bool bSet )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the focus changes for the control.
	//
	//	PARAMETERS
	//		-	pCtl is the assigned control dictionary
	//		-	bSet to set focus, false to kill focus
	//
	//	RETURN VALUE
	//		Zero if message is processed.
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;
	adtString	str;

	// Obtain current text
	CCLTRY ( getText ( str ) );
//	if (!bSet)
//		dbgprintf ( L"Edit::onFocus::Current:%s:Last:%s\r\n", (LPCWSTR) str, (LPCWSTR) strText );

	// When focus set, update current text
	if (hr == S_OK && bSet)
		strText = str;

	// When focus is lost, update if different
	else if (hr == S_OK && !bSet && WCASECMP(str,strText))
		{
		// New text
		strText = str;

		// Emit result
		CCLOK ( _EMT(Dictionary,adtIUnknown(pCtl) ); )
		CCLOK ( _EMT(Text,strText ); )
		}	// if

	return gdiEdit::onFocus(pCtl,bSet);
	}	// onFocus

HRESULT Edit :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

	// Open
	if (_RCP(Open))
		{
		HWND			hWndP		= NULL;
		DWORD			dwStyle	= WS_CHILD|WS_VISIBLE;
		adtInt		iX			= 0;
		adtInt		iY			= 0;
		adtInt		iW			= 20;
		adtInt		iH			= 20;
		adtString	strStyle(L"");
		adtValue		vL;
		adtLong		lWnd;

		// State check
		CCLTRYE ( pDct != NULL, ERROR_INVALID_STATE );

		// Owner required for child controls
		CCLTRYE ( (pOwn != NULL), ERROR_INVALID_STATE );
		CCLTRY  ( pOwn->load ( adtString(L"Window"), lWnd ) );
		CCLTRYE ( (hWndP = (HWND)(U64)lWnd) != NULL, ERROR_INVALID_STATE );

		// Possible Edit parameters
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
			}	// if

		// Default styles
		CCLOK ( dwStyle |= (ES_LEFT|ES_AUTOHSCROLL|WS_BORDER|WS_TABSTOP); )

		// Enabled ?
		if (	hr == S_OK && 
				pDct->load ( adtString(L"Enable"), vL ) == S_OK &&
				adtBool(vL) == false)
			dwStyle |= (WS_DISABLED);

		// Already 'open' ?
		if (hr == S_OK && pDct->load ( adtString(L"Window"), vL )  != S_OK)
			{
			// Create control
//			strText = L"Debug";
			CCLTRYE ( (hWnd = CreateWindowEx ( WS_EX_CLIENTEDGE, L"Edit", strText, dwStyle,
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
			CCLTRYE	( IsWindow((hWnd = (HWND)(U64)(lWnd=vL))), ERROR_INVALID_STATE );

			// Update size/position
			CCLOK ( MoveWindow ( hWnd, iX, iY, iW, iH, TRUE ); )
			}	// else if

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
		if ((HWND)0 != (HWND)(U64)lWnd)
			DestroyWindow ( (HWND)(U64)lWnd );

		// Result
		CCLOK ( _EMT(Close,adtIUnknown(pDct)); )
		}	// if

	//
	// Edit specific
	//

	// Set text
	else if (_RCP(Text))
		{
		adtString	strV,strE;

		// Obtain string version of value
		CCLTRY ( adtValue::toString ( v, strV ) );

		// State check
		CCLTRYE	( pDct != NULL, ERROR_INVALID_STATE );
		CCLOK		( pDct->load ( adtString(L"Window"), lWnd ); )
		CCLTRYE	( IsWindow((hWnd = (HWND)(U64)lWnd)), ERROR_INVALID_STATE );

		// Set text if different
		if (pDct != NULL)
			pDct->store ( adtString(L"Text"), strV );
		if (hr == S_OK && getText ( strE ) == S_OK && WCASECMP(strE,strV))
			SetWindowText ( hWnd, strV );
		}	// else if

	// Enable
	else if (_RCP(Enable))
		{
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

//
// Base class
//

gdiEdit :: gdiEdit ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for object.
	//
	////////////////////////////////////////////////////////////////////////
	}	// gdiEdit

HRESULT gdiEdit :: getText ( adtString &str )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Retrieve the current text.
	//
	//	PARAMETERS
	//		-	str will receive the text.
	//
	//	RETURN VALUE
	//		Zero if message is processed.
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;
	int		len;

	// Length of text
	CCLTRYE ( (len = GetWindowTextLength ( *this )) >= 0, GetLastError() );

	// Allocate room for text
	CCLTRY ( str.allocate ( len ) );

	// Obtain text
	CCLTRYE ( GetWindowText ( *this, &str.at(), len+1 ) >= 0, GetLastError() );

	return hr;
	}	// onEnter

LRESULT gdiEdit :: onEnter ( IDictionary *pCtl )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the user hits the 'enter' key in the edit box.
	//
	//	PARAMETERS
	//		-	pCtl is the assigned control dictionary
	//
	//	RETURN VALUE
	//		Zero if message is processed.
	//
	////////////////////////////////////////////////////////////////////////
	return 1;
	}	// onEnter

LRESULT gdiEdit :: onFocus ( IDictionary *pCtl, bool bSet )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the focus changes for the control.
	//
	//	PARAMETERS
	//		-	pCtl is the assigned control dictionary
	//		-	bSet to set focus, false to kill focus
	//
	//	RETURN VALUE
	//		Zero if message is processed.
	//
	////////////////////////////////////////////////////////////////////////
	return 1;
	}	// onFocus

LRESULT gdiEdit :: onMessage ( IDictionary *pCtl, UINT uMsg, 
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
				case EN_KILLFOCUS :
				case EN_SETFOCUS :
					return onFocus(pCtl, (HIWORD(wParam)) == EN_SETFOCUS);
					break;
				}	// switch
			break;
		case WM_CHAR :
			// Enter ?
			if (wParam == VK_RETURN)
				return onEnter ( pCtl );
			break;
		}	// switch

	return gdiControl::onMessage ( pCtl, uMsg, wParam, lParam );
	}	// onMessage
