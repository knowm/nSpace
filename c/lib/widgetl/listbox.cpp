////////////////////////////////////////////////////////////////////////
//
//								LISTBOX.CPP
//
//					Listbox class implementation
//
////////////////////////////////////////////////////////////////////////

#include "widgetl_.h"
#include <stdio.h>

// Globals
extern HINSTANCE	ccl_hInst;

//
// Node
//

Listbox :: Listbox ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pDct	= NULL;
	pOwn	= NULL;
	}	// Listbox

HRESULT Listbox :: onAttach ( bool bAttach )
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

LRESULT Listbox :: onSelChange ( IDictionary *pCtl, LRESULT lIdx )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the listbox selection changes.
	//
	//	PARAMETERS
	//		-	pCtl is the assigned control dictionary
	//		-	lIdx is the selected index
	//
	//	RETURN VALUE
	//		Zero if message is processed.
	//
	////////////////////////////////////////////////////////////////////////
	int			iSel;

	// Selection is specified by 1-based array index
	iSel = (int)SendMessage ( *this, LB_GETCURSEL, 0, 0 );
	iSel = (iSel == LB_ERR) ? 0 : (iSel+1);

	// Emit result
	_EMT(Dictionary,adtIUnknown(pCtl) );
	_EMT(Select,adtInt(iSel));

	// Notify
//	dbgprintf ( L"Listbox::onSelChange:%p,%d\r\n", pCtl, iSel );

	return 0;
	}	// onSelChange

HRESULT Listbox :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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
		DWORD			dwStyle	= WS_CHILD|WS_VISIBLE|LBS_HASSTRINGS|LBS_STANDARD|LBS_NOTIFY|LBS_NOINTEGRALHEIGHT;
		adtInt		iX			= 0;
		adtInt		iY			= 0;
		adtInt		iW			= 20;
		adtInt		iH			= 20;
		adtValue		vL;
		adtLong		lWnd;

		// State check
		CCLTRYE ( pDct != NULL, ERROR_INVALID_STATE );

		// Owner required for child controls
		CCLTRYE ( (pOwn != NULL), ERROR_INVALID_STATE );
		CCLTRY  ( pOwn->load ( adtString(L"Window"), lWnd ) );
		CCLTRYE ( (hWndP = (HWND)(U64)lWnd) != NULL, ERROR_INVALID_STATE );

		// Possible Listbox parameters
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
			}	// if

		// Default is to not sort, external environment maintains proper order
		dwStyle &= ~(LBS_SORT);

		// Enabled ?
		if (	hr == S_OK && 
				pDct->load ( adtString(L"Enable"), vL ) == S_OK &&
				adtBool(vL) == false)
			dwStyle |= (WS_DISABLED);

		// Already 'open' ?
		if (hr == S_OK && pDct->load ( adtString(L"Window"), vL )  != S_OK)
			{
			// Create control
			CCLTRYE ( (hWnd = CreateWindowEx ( WS_EX_CLIENTEDGE, L"Listbox", NULL, dwStyle,
//			CCLTRYE ( (hWnd = CreateWindow ( L"Listbox", L"", dwStyle,
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
	// Listbox specific work
	//

	// Add value to list
	else if (_RCP(Add))
		{
		adtString	strV;

		// State check
		CCLTRYE	( pDct != NULL, ERROR_INVALID_STATE );
		CCLOK		( pDct->load ( adtString(L"Window"), lWnd ); )
		CCLTRYE	( IsWindow((hWnd = (HWND)(U64)lWnd)), ERROR_INVALID_STATE );

		// Obtain string version of value
		CCLTRY ( adtValue::toString ( v, strV ) );

		// Add to list box
		CCLOK ( SendMessage ( hWnd, LB_ADDSTRING, 0, (LPARAM)(LPCWSTR)strV ); )
		}	// else if

	// Reset list
	else if (_RCP(Reset))
		{
		// State check
		CCLTRYE	( pDct != NULL, ERROR_INVALID_STATE );
		CCLOK		( pDct->load ( adtString(L"Window"), lWnd ); )
		CCLTRYE	( IsWindow((hWnd = (HWND)(U64)lWnd)), ERROR_INVALID_STATE );

		// Clear existing list
		CCLOK ( SendMessage ( hWnd, LB_RESETCONTENT, 0, 0 ); )
		}	// else if

	// Select (by index or string)
	else if (_RCP(Select))
		{
		// State check
		CCLTRYE	( pDct != NULL, ERROR_INVALID_STATE );
		CCLOK		( pDct->load ( adtString(L"Window"), lWnd ); )
		CCLTRYE	( IsWindow((hWnd = (HWND)(U64)lWnd)), ERROR_INVALID_STATE );

		// Make selection
		if (hr == S_OK)
			{
			// By index
			if (v.vtype == VTYPE_I4 || v.vtype == VTYPE_I8)
				SendMessage ( hWnd, LB_SETCURSEL, (WPARAM)adtInt(v)-1, 0 );

			// By string
			else if (adtValue::type(v) == VTYPE_STR && v.pstr != NULL)
				SendMessage ( hWnd, LB_SELECTSTRING, 0, (LPARAM)(v.pstr) );
			}	// if
		}	// else if

	// List.
	else if (_RCP(List))
		{
		IContainer	*pCnt	= NULL;
		IIt			*pIt	= NULL;
		adtIUnknown	unkV(v);
		adtValue		vL;

		// State check
		CCLTRYE	( pDct != NULL, ERROR_INVALID_STATE );
		CCLOK		( pDct->load ( adtString(L"Window"), lWnd ); )
		CCLTRYE	( IsWindow((hWnd = (HWND)(U64)lWnd)), ERROR_INVALID_STATE );

		// Entire contents of listbox
		CCLTRY ( _QISAFE(unkV,IID_IContainer,&pCnt) );
		CCLTRY ( pCnt->iterate ( &pIt ) );

		// Clear existing contents
		CCLOK  ( receive ( prReset, pl, v ); )

		// Add items to list
		while (hr == S_OK && pIt->read ( vL ) == S_OK)
			{
			// Add to list
			receive ( prAdd, pl, vL );

			// Next item
			pIt->next();
			}	// while

		// Clean up
		_RELEASE(pIt);
		_RELEASE(pCnt);
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

gdiListbox :: gdiListbox ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for object.
	//
	////////////////////////////////////////////////////////////////////////
	}	// gdiListbox

LRESULT gdiListbox :: onSelChange ( IDictionary *pCtl, LRESULT lRes )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the listbox selection changes.
	//
	//	PARAMETERS
	//		-	pCtl is the assigned control dictionary
	//		-	lIdx is the selected index
	//
	//	RETURN VALUE
	//		Zero if message is processed.
	//
	////////////////////////////////////////////////////////////////////////
	return 1;
	}	// onSelChange

LRESULT gdiListbox :: onMessage ( IDictionary *pCtl, UINT uMsg, 
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
				case LBN_SELCHANGE :
					onSelChange(pCtl,SendMessage((HWND)lParam,LB_GETCURSEL,0,0));
					break;
				}	// switch
			break;
		}	// switch

	return gdiControl::onMessage ( pCtl, uMsg, wParam, lParam );
	}	// onMessage

