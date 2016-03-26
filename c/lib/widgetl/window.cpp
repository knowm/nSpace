////////////////////////////////////////////////////////////////////////
//
//									WINDOW.CPP
//
//					Window base class implementation
//
////////////////////////////////////////////////////////////////////////

#include "widgetl_.h"

// Globals
extern HINSTANCE	ccl_hInst;
/*
Window :: Window ( void ) :
	// String references
	strX(L"X"),strY(L"Y"),strEv(L"Event"),strUp(L"Up"),strDn(L"Down"),
	strMv(L"Move"),strBt(L"Button"),strBt1(L"Button1"),strBt2(L"Button2"),strBt3(L"Button3")
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pDct			= NULL;
	pOwn			= NULL;
	hWndModal	= NULL;
	pThrd			= NULL;
	pDctInM		= NULL;
	pDctInK		= NULL;
	pIn			= NULL;
	pOut			= NULL;
	}	// Window

HRESULT Window :: onAttach ( bool bAttach )
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
	HRESULT	hr = S_OK;

	// Attach
	if (bAttach)
		{
		WNDCLASS	wc;
		adtValue	vL;

		// Window class for default windows
		if (hr == S_OK && !GetClassInfo ( ccl_hInst, L"WIDGETN:Window", &wc ))
			{
			// 'Generic' Windows.
			memset ( &wc, 0, sizeof(wc) );
			wc.lpfnWndProc		= gdiControl::controlProcS;
			wc.hInstance		= ccl_hInst;
			wc.hCursor			= LoadCursor ( (HINSTANCE)NULL, IDC_ARROW );
			wc.hbrBackground	= GetSysColorBrush ( COLOR_3DFACE );
			wc.lpszClassName	= L"WIDGETN:Window";
			RegisterClass ( &wc );
			}	// if

		// Custom values
		if (hr == S_OK && pnDesc->load ( adtString(L"Values"), vL ) == S_OK)
			{
			IContainer	*pRs	= NULL;
			IIt			*pIt	= NULL;
			adtIUnknown	unkV(vL);

			// Create a dictionary to map connectors to name
			CCLTRY(COCREATE(L"Adt.Dictionary",IID_IDictionary,&pIn));
			CCLTRY(COCREATE(L"Adt.Dictionary",IID_IDictionary,&pOut));

			// Iterate receptor names
			CCLTRY ( _QISAFE(unkV,IID_IContainer,&pRs) );
			CCLTRY ( pRs->iterate ( &pIt ) );
			while (hr == S_OK && pIt->read ( vL ) == S_OK)
				{
				IReceptor	*pR	= NULL;
				adtString	strIn,strOut;

				// String version of provided key
				CCLTRY ( adtValue::toString ( vL, strIn ) );

				// Add a connectors for the specified name
				CCLTRY ( pnSpc->connection ( pnLoc, strIn, L"Receptor", this, &pR ) );

				// Associate name with key
				CCLTRY ( pIn->store ( adtIUnknownRef(pR), strIn ) );

				// Create outgoing value for result
				CCLTRY ( adtValue::copy ( strIn, strOut ) );
				CCLTRY ( strOut.prepend ( L"On" ) );
				CCLTRY ( pnSpc->connection ( pnLoc, strOut, L"Emitter", this, &pR ) );

				// Store mapping from input name to connection
				CCLTRY ( pOut->store ( strIn, adtIUnknownRef(pR) ) );

				// Clean up
				pIt->next();
				}	// while

			// Clean up
			_RELEASE(pIt);
			_RELEASE(pRs);

			// No need to fail attachment for invalid receptors
			hr = S_OK;
			}	// if

		// Run-time data
		CCLTRY(COCREATE(L"Adt.Dictionary",IID_IDictionary,&pDctInM));
		CCLTRY(COCREATE(L"Adt.Dictionary",IID_IDictionary,&pDctInK));
		}	// if

	// Detach
	else
		{
		_RELEASE(pThrd);
		_RELEASE(pDctInM);
		_RELEASE(pDctInK);
		_RELEASE(pDct);
		_RELEASE(pOwn);
		_RELEASE(pIn);
		_RELEASE(pOut);
		}	// if

	return hr;
	}	// onAttach

LRESULT Window :: onMessage ( IDictionary *pCtl, UINT uMsg, 
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

	// Handle relevant messages
	switch (uMsg)
		{
		case WM_COMMAND :
			// Controls handle their own messages so reflect control messages.
			if (lParam != NULL)
				ret = SendMessage ( (HWND)lParam, uMsg, wParam, lParam );
			break;
		case WM_NOTIFY :
			// Controls handle their own messages so reflect control messages.
			if (lParam != NULL)
				ret = SendMessage ( ((NMHDR *)lParam)->hwndFrom, uMsg, wParam, lParam );
			break;

		// Custom receptors
		case WIDGET_USER_EMIT :
			{
			// Output value
			if (prOut != NULL)
				prOut->receive ( this, L"Value", vOut );
			}	// GDIN_USER_EMIT
			break;

		// Close
		case WM_CLOSE :
			ret = gdiControl::onMessage(pCtl,uMsg,wParam,lParam);
			break;

		// Which input window wants
		case WM_GETDLGCODE :
			// Ensures 'IsDialogMessage' in message loop will still forward keyboard input.
			ret = DLGC_WANTALLKEYS;
			break;

		// Scroll (for trackbars, etc)
		case WM_HSCROLL :
		case WM_VSCROLL :
			ret = SendMessage ( (HWND)lParam, uMsg, wParam, lParam );
			break;

		// Input
		case WM_LBUTTONDOWN		:
		case WM_LBUTTONUP			:
		case WM_MBUTTONDOWN		:
		case WM_MBUTTONUP			:
		case WM_RBUTTONDOWN		:
		case WM_RBUTTONUP			:
		case WM_MOUSEMOVE			:
			{
			HRESULT	hr = S_OK;
			RECT		cr;
			double	x,y;

			// Debug
//			dbgprintf ( L"Window::onMessage:%d,%d\r\n", (S32)((S16)LOWORD(lParam)), (S32)((S16)HIWORD(lParam)) );

			// Size of window area
			CCLOK ( GetClientRect ( *this, &cr ); )

			// Calculate normalized coordinates, Y inverted in GDI
			CCLOK ( x = (LOWORD(lParam)-(cr.right/2))/((double)cr.right); )
			CCLOK ( y = ((cr.bottom-HIWORD(lParam))-(cr.bottom/2))/((double)cr.bottom); )

			// Event position
			CCLTRY ( pDctInM->store ( strX, adtDouble(x) ) );
			CCLTRY ( pDctInM->store ( strY, adtDouble(y) ) );

			// Event name
			CCLTRY ( pDctInM->store ( strEv,
							(uMsg == WM_LBUTTONDOWN || uMsg == WM_RBUTTONDOWN || uMsg == WM_MBUTTONDOWN)	?	
								strDn	:
							(uMsg == WM_LBUTTONUP || uMsg == WM_RBUTTONUP || uMsg == WM_MBUTTONUP)	?	
								strUp : 
								strMv ) );

			// Button
			if (uMsg != WM_MOUSEMOVE)
				{
				CCLTRY ( pDctInM->store ( strBt,
								(uMsg == WM_RBUTTONDOWN || uMsg == WM_RBUTTONUP)	?	strBt2		:
								(uMsg == WM_MBUTTONDOWN || uMsg == WM_MBUTTONUP)	?	strBt3	:
																										strBt1 ) );
				}	// if
			else
				pDctInM->remove ( strBt );
	
			// Result
			CCLOK ( _EMT(Dictionary,adtIUnknown(pCtl) ); )
			CCLOK ( _EMT(Mouse,adtIUnknown(pDctInM) ); )
			}	// WM_XXX
			break;

		// Mouse wheel
		case WM_MOUSEWHEEL :
			{
			HRESULT	hr = S_OK;
			int		iAmt,i,j;

			// Event position
			CCLTRY ( pDctInM->store ( adtString(L"X"), adtInt((S32)((S16)LOWORD(lParam))) ) );
			CCLTRY ( pDctInM->store ( adtString(L"Y"), adtInt((S32)((S16)HIWORD(lParam))) ) );

			// Button
			CCLTRY ( pDctInM->store ( adtString(L"Button"), adtString(L"Wheel") ) );

			// Wheel amount
			CCLOK ( iAmt = GET_WHEEL_DELTA_WPARAM(wParam)/WHEEL_DELTA; )
//			dbgprintf ( L"Window::onMessage:%d\r\n", iAmt );

			// Events
			CCLOK ( j = (iAmt > 0) ? iAmt : -iAmt; )
			for (i = 0;hr == S_OK && i < j;++i)
				{
				// Event name
				CCLTRY ( pDctInM->store ( adtString(L"Event"),
								(iAmt < 0) ? adtString(L"Back") : adtString(L"Forward") ) );

				// Result
				CCLOK ( _EMT(Dictionary,adtIUnknown(pCtl) ); )
				CCLOK ( _EMT(Mouse,adtIUnknown(pDctInM) ); )
				}	// for

			}	// case WM_MOUSEWHEEL
			break;

		// Keyboard input
		case WM_KEYDOWN :
		case WM_KEYUP :
			{
			HRESULT	hr = S_OK;
			WCHAR		wKey[21];

			// Get the name of the key
			CCLTRYE ( GetKeyNameText ( (LONG)lParam, wKey, sizeof(wKey)/sizeof(WCHAR) ) > 0, GetLastError() );
//			dbgprintf ( L"Window::WM_KEYXXX:0x%x 0x%x:%s\r\n", wParam, lParam, wKey );

			// Event name
			CCLTRY ( pDctInK->store ( strEv, (uMsg == WM_KEYDOWN) ? strDn : strUp ) );

 			// Key/button name
			CCLTRY ( pDctInK->store ( strBt, adtString(wKey) ) );

			// Result
			CCLOK ( _EMT(Dictionary,adtIUnknown(pCtl) ); )
			CCLOK ( _EMT(Key,adtIUnknown(pDctInK) ); )
			}	// WM_KEYXXX
			break;

//	PostMessage ( hWnd, WIDGET_USER_DROPTEST, 0, (LPARAM) pDctDrop );

		// Child destruction
		case WIDGET_USER_DESTROY :
			DestroyWindow ( (HWND) wParam );
//			CloseWindow ( (HWND) wParam );
			break;

		// Destruction
		case WM_DESTROY :

			// In case it was registered..
			RevokeDragDrop(*this);
		
			// Shutdown
			dbgprintf ( L"Window::message:WM_DESTROY {\r\n" );
			if ( (GetWindowLong ( *this, GWL_STYLE ) & WS_OVERLAPPED) == WS_OVERLAPPED)
				PostQuitMessage(0);
			dbgprintf ( L"} Window::message:WM_DESTROY\r\n" );
			break;

		default :
			ret = gdiControl::onMessage ( pCtl, uMsg, wParam, lParam );
		}	// switch

	return ret;
	}	// onMessage

HRESULT Window :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

	// Close - Can be executed from any thread, in fact not same thread
	// as window to allow shutdown.
	if (_RCP(Close))
		{
		// Shutdown thread
		if (pThrd != NULL)
			pThrd->threadStop(10000);
		_RELEASE (pThrd);
		return S_OK;
		}	// if

	// Threading
	else if (gdiControl::receive ( pDct, this, pr, pl, v ) != S_OK)
		return S_OK;

	// Open
	else if (_RCP(Open))
		{
		// Fix this
		_RELEASE(pThrd);

		// State check
		CCLTRYE ( pThrd == NULL,	ERROR_INVALID_STATE );
		CCLTRYE ( pDct != NULL,		ERROR_INVALID_STATE );

		// Create window thread
		CCLTRY(COCREATE(L"Sys.Thread",IID_IThread,&pThrd));
		CCLTRY(pThrd->threadStart(this,5000));
		}	// if

	// Modal/modeless
	else if (_RCP(Modal))
		{
		adtBool	vModal(v);

		// Current parent window
		HWND
		hWndParent = GetParent ( *this );

		// Modal in this context means to disable/enable parent window

		// Enable if necessary
		if (vModal == true && hWndModal == NULL && IsWindowEnabled ( hWndParent ))
			{
			hWndModal = hWndParent;
			EnableWindow ( hWndModal, FALSE );
			}	// if

		// Disable if necessary
		else if (vModal == false && hWndModal != NULL)
			{
			EnableWindow ( hWndModal, TRUE );
			hWndModal = NULL;
			}	// else if

		}	// else if

	// Matrix
	else if (_RCP(Matrix))
		{
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

	// Window type
	else if (_RCP(Type))
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
			pDct->store ( adtString(L"Type"), strV );
		}	// else if

	// Enable/disable
	else if (_RCP(Enable))
		{
		adtBool	bEn(v);

		// Enable/disable control
		CCLOK ( EnableWindow ( *this, (bEn == true) ); )
		}	// if

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

	// Custom value
	else
		{
		adtIUnknown	unkV;
		adtValue		vL;

		// State check
		CCLTRYE ( pIn != NULL && pOut != NULL, ERROR_INVALID_STATE );

		// Load name for input connector
		CCLTRY ( pIn->load ( adtIUnknown(pr), vL ) );

		// Load output connector for name
		CCLTRY ( pOut->load ( vL, vL ) );

		// Output connector and value
		CCLTRY ( _QISAFE((unkV=vL),IID_IReceptor,&prOut) );
		CCLTRY ( adtValue::copy ( v, vOut ) );

		// Output
		CCLOK ( SendMessage ( *this, WIDGET_USER_EMIT, 0, 0l ); )

		// Clean up
		adtValue::clear ( vOut );
		_RELEASE(prOut);
		}	// else if

	return hr;
	}	// receive

U32 Window :: stylesToInt ( IUnknown *punkStyles, U32 *pEx )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Utility to convert a list of string based styles into
	//				an integer mask.
	//
	//	PARAMETERS
	//		-	punkStyles is the list
	//		-	pEx will receive optional extended styles
	//
	//	RETURN VALUE
	//		Mask of styles
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	IContainer	*pCont	= NULL;
	IIt			*pIt		= NULL;
	U32			mask		= 0;
	adtString	strStyle;
	adtValue		v;

	// Iterate strings
	CCLTRY(_QISAFE(punkStyles,IID_IContainer,&pCont));
	CCLTRY(pCont->iterate(&pIt));
	while (hr == S_OK && pIt->read ( v ) == S_OK)
		{
		// Style string
		CCLTRY ( adtValue::copy ( v, strStyle ) );
		dbgprintf ( L"WIDGETN::stylesToInt:strStyle %s\r\n", (LPCWSTR)strStyle );
		if (strStyle.length() < 1)
			continue;

		// Possible styles

		// Dialog styles
		if (!WCASECMP(strStyle,L"DS_SETFONT"))
			mask |= DS_SETFONT;
		else if (!WCASECMP(strStyle,L"DS_MODALFRAME"))
			mask |= DS_MODALFRAME;
		else if (!WCASECMP(strStyle,L"DS_FIXEDSYS"))
			mask |= DS_FIXEDSYS;

		// Window styles
		else if (!WCASECMP(strStyle,L"WS_POPUP"))
			mask |= WS_POPUP;
		else if (!WCASECMP(strStyle,L"WS_CAPTION"))
			mask |= WS_CAPTION;
		else if (!WCASECMP(strStyle,L"WS_SYSMENU"))
			mask |= WS_SYSMENU;
		else if (!WCASECMP(strStyle,L"WS_VSCROLL"))
			mask |= WS_VSCROLL;
		else if (!WCASECMP(strStyle,L"WS_TABSTOP"))
			mask |= WS_TABSTOP;

		// Control styles
		else if (!WCASECMP(strStyle,L"BS_AUTOCHECKBOX"))
			mask |= BS_AUTOCHECKBOX;

		else if (!WCASECMP(strStyle,L"ES_AUTOHSCROLL"))
			mask |= ES_AUTOHSCROLL;

		else if (!WCASECMP(strStyle,L"CBS_DROPDOWN"))
			mask |= CBS_DROPDOWN;
		else if (!WCASECMP(strStyle,L"CBS_SORT"))
			mask |= CBS_SORT;

		else if (!WCASECMP(strStyle,L"LBS_SORT"))
			mask |= LBS_SORT;
		else if (!WCASECMP(strStyle,L"LBS_NOINTEGRALHEIGHT"))
			mask |= LBS_NOINTEGRALHEIGHT;

		// Extended styles
		if (pEx != NULL)
			{
			if (!WCASECMP(strStyle,L"WS_EX_TOPMOST"))
				(*pEx) |= WS_EX_TOPMOST;
			else if (!WCASECMP(strStyle,L"WS_EX_DLGMODALFRAME"))
				(*pEx) |= WS_EX_DLGMODALFRAME;
			}	// if

		// Proceed to next style
		pIt->next();
		}	// while

	// Clean up
	_RELEASE(pIt);
	_RELEASE(pCont);

	return mask;
	}	// stylesToInt

HRESULT Window :: tick ( void )
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
	bool		bShift	= false;
	MSG		msg;

	// Message loop
	dbgprintf ( L"Window::tick {\r\n" );
	while (GetMessage ( &msg, (HWND) NULL, 0, 0 ) != 0)
		{
		// Filter 'default' behaviour that does apply in current environment.

		// Stop the 'Hit return or escape to post a WM_COMMAND message' behaviour
		if (	(msg.wParam  == VK_RETURN	|| msg.wParam == VK_ESCAPE) ||
				!IsDialogMessage ( *this, &msg ) )
			{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
			}	// if

		}	// while
	dbgprintf ( L"} Window::tick\r\n" );

	// Was modal ?
	if (hWndModal != NULL)
		{
		EnableWindow ( hWndModal, TRUE );
		hWndModal = NULL;
		}	// if	

	return S_FALSE;
	}	// tick

HRESULT Window :: tickAbort ( void )
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

	// Time to close window/thread
	dbgprintf ( L"Window::tickAbort:0x%x:Thread %d {\r\n", *this, GetCurrentThreadId() );
//	if (hWndCont != NULL)
//		PostMessage ( hWndCont, WM_USER_DESTROY, 0, 0 );
	if (dwThrdId != 0)
		PostThreadMessage ( dwThrdId, WM_QUIT, 0, 0l );
	dbgprintf ( L"} Window::tickAbort\r\n" );

	return S_OK;
	}	// tickAbort

HRESULT Window :: tickBegin ( void )
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
	HRESULT		hr				= S_OK;
	HWND			hWnd			= NULL;
	HWND			hWndP			= NULL;
	bool			bOver			= false;
	U32			dwStyle		= 0;
	U32			dwStyleEx	= 0;
	adtString	strText(L"");
	adtValue		vL;
	adtBool		bVisible(true);

	// Initialize COM for thread
	CCLTRYE ( CoInitializeEx ( NULL, COINIT_MULTITHREADED ) == S_OK,
					GetLastError() );

	// Store thread ID for later notification
	CCLOK ( dwThrdId = GetCurrentThreadId(); )

	// State check
	CCLTRYE ( pDct != NULL, ERROR_INVALID_STATE );
	CCLTRYE ( pDct->load ( adtString(L"Window"), vL ) != S_OK, ERROR_INVALID_STATE );

	// Options
	if (hr == S_OK)
		{
		adtValue::copy ( adtString(L"Overlapped"), vL );
		if (	//pDct->load ( adtString(L"Type"), vL ) == S_OK &&
				adtValue::type(vL) == VTYPE_STR &&
				vL.pstr != NULL )
			{
			// Adjust style based on type of window
			if (!WCASECMP(vL.pstr,L"Overlapped"))
				{
				dwStyle &= ~(WS_CHILD);
				dwStyle |= (WS_OVERLAPPEDWINDOW);
				bOver	= true;
				}	// if
			else if (!WCASECMP(vL.pstr,L"Popup"))
				{
				dwStyle &= ~(WS_CHILD);
				dwStyle |= (WS_POPUP);
				}	// if
			}	// if
		if (	pDct->load ( adtString(L"Text"), vL ) == S_OK || 
				pDct->load ( adtString(L"Caption"), vL ) == S_OK)
			strText = vL;
		if (pDct->load ( adtString(L"Visible"), vL ) == S_OK)
			bVisible = vL;
		}	// if

	// Styles
	if (strText.length() > 0)
		dwStyle |= (WS_CAPTION);

	// Additional styles
	if (hr == S_OK && pDct->load ( adtString(L"Styles"), vL ) == S_OK)
		dwStyle |= stylesToInt ( adtIUnknown(vL), &dwStyleEx );

	// Sanity check
	if (	!bOver && !(dwStyle & (WS_CHILD|WS_POPUP)))
		dwStyle |= WS_CHILD;

	// Owner check
 	if (hr == S_OK && (dwStyle & WS_CHILD))
		{
		adtLong	lWnd;

		// Retrieve owner information/handle
		CCLTRYE ( (pOwn != NULL), ERROR_INVALID_STATE );
		CCLTRY  ( pOwn->load ( adtString(L"Window"), lWnd ) );
		CCLTRYE ( (hWndP = (HWND)(U64)lWnd) != NULL, ERROR_INVALID_STATE );
		}	// if

	// If no owner or popup, default to cursor position
//	if (!bOver && (hWndP == NULL || (dwStyle & WS_POPUP)))
//		{
		// Is this the best default ?
//		POINT	pt;
//		GetCursorPos ( &pt );
//		rct.left	= pt.x;
//		rct.top	= pt.y;
//		}	// if

	// If the window is captioned, increase size of window to account for 'decoration'
//	if (dwStyle & WS_CAPTION)
//		{
//		iW.vint += 2 * GetSystemMetrics ( SM_CXFIXEDFRAME );
//		iH.vint += 2 * GetSystemMetrics ( SM_CYFIXEDFRAME ) + GetSystemMetrics ( SM_CYCAPTION );
//		}	// if

	// Opening "background" window
//	if (hWndP == NULL)
//		dwStyle &= ~(WS_VISIBLE);

	// Window rectangle.  Ensure client size is available.
//	wr.left	= iX;
//	wr.right = iX+iW;
//	wr.top	= iY;
//	wr.bottom= iY+iH;
//	CCLOK ( AdjustWindowRect ( &rct, dwStyle, FALSE ); )

	// Adjust creation size
//	iW	= (wr.right-wr.left);
//	iH = (wr.bottom-wr.top);	

	// Default
	dwStyle |= WS_SYSMENU;

	// Visible ?
	if (hr == S_OK && bVisible == true)
		dwStyle |= WS_VISIBLE;

	// Create control
	CCLTRYE ( (hWnd = CreateWindowEx ( dwStyleEx, L"WIDGETN:Window", strText, dwStyle,
					0, 0, 800, 600,
//					0, 0, 320, 240,
//					0, 0, 10, 10,
					hWndP, NULL, ccl_hInst, (gdiControl *) this )) 
					!= NULL, GetLastError() );

	// Assign the window to the dictionary
	CCLTRY ( assign ( hWnd, pDct, false ) );

	// This is for style purposes.
	#if	!defined(UNDER_CE)
	CCLOK  ( SendMessage ( hWnd, WM_SETFONT,
					(WPARAM) GetStockObject ( DEFAULT_GUI_FONT ), TRUE ); )
	#endif

	// Debug
//	CCLOK ( ShowWindow ( hWnd, SW_SHOWNORMAL ); )

	// Result
	CCLOK ( _EMT(Open,adtIUnknown(pDct)); )

	// Enable drag and drop if available
	if (	hr == S_OK &&
			pDct->load ( strRefDragDrop, vL ) == S_OK)
		{
		IDropTarget	*pTarget	= NULL;
		adtIUnknown	unkV(vL);

		// Register own window
		if (_QI(unkV,IID_IDropTarget,&pTarget) == S_OK)
			RegisterDragDrop ( hWnd, pTarget );

		// Clean up
		_RELEASE(pTarget);
		}	// if

	// Clean up
	if (hr != S_OK && hWnd != NULL)
		DestroyWindow ( hWnd );

	return hr;
	}	// tickBegin

HRESULT Window :: tickEnd ( void )
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
	HRESULT	hr			= S_OK;
	bool		bOv		= false;
	bool		bPopup	= false;
	adtLong	lWnd;

	// State check
	CCLTRYE ( pDct != NULL, ERROR_INVALID_STATE );

	// Notify shutdown
	CCLOK ( _EMT(Close,adtIUnknown(pDct)); )

	// Widget
	CCLOK ( pDct->load ( adtString(L"Window"), lWnd ); )
	CCLOK ( unassign ( pDct ); )

	// Clean up
	CoUninitialize();

	return S_OK;
	}	// tickEnd

//
// Base class
//

gdiWindow :: gdiWindow ( HINSTANCE hInst )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for object.
	//
	//	PARAMETERS
	//		-	hInst is the application instance
	//
	////////////////////////////////////////////////////////////////////////
	ui_hInst		= hInst;
	ui_hWnd		= NULL;
	bClassReg	= false;
	}	// gdiWindow

gdiWindow :: ~gdiWindow ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Destructor for object.
	//
	////////////////////////////////////////////////////////////////////////
	if (bClassReg && !UnregisterClass ( ui_wndclass.lpszClassName, ui_hInst ))
		dbgprintf ( L"gdiWindow::~gdiWindow:Info:Failure in unregistering class\n" );
	}	// ~gdiWindow

HRESULT gdiWindow :: construct ( HWND hParent )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Construct/create frame class.
	//
	//	PARAMETERS
	//		-	hParent is the parent window
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;
	WNDCLASS	wctmp;

	// Fill class information
	getClassInfo ( &ui_wndclass );

	// Class already exist ?
	if (hr == S_OK && !GetClassInfo ( *this, ui_wndclass.lpszClassName, &wctmp ))
		hr = (RegisterClass ( &ui_wndclass ) != 0) ? S_OK : GetLastError();

	// Create the window
	if (hr == S_OK)
		hr = ((ui_hWnd = createWindow ( hParent, ui_wndclass.lpszClassName ))
				!= NULL) ? S_OK : GetLastError();

	return hr;
	}	// construct

HWND gdiWindow :: createWindow ( HWND hParent, LPCWSTR lpszClassName )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Creates the window.
	//
	//	PARAMETERS
	//		-	hParent is the parent window
	//		-	lpszClassName is the name to use for the window class
	//
	//	RETURN VALUE
	//		Handle to window
	//
	////////////////////////////////////////////////////////////////////////
	return CreateWindow ( lpszClassName, L"",
				WS_CHILD|WS_CLIPCHILDREN|WS_CLIPSIBLINGS,
				CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT,
				hParent, NULL, *this, this );
	}	// createWindow

void gdiWindow :: getClassInfo ( WNDCLASSW *pWndClass )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Fill in window class information.
	//
	//	PARAMETERS
	//		-	pWndClass will receive the info.
	//
	////////////////////////////////////////////////////////////////////////

	// Defaults
	memset ( pWndClass, 0, sizeof(WNDCLASS) );
	pWndClass->lpfnWndProc	= gdiWindowProc;
	pWndClass->hInstance		= ui_hInst;
	pWndClass->hCursor		= LoadCursor((HINSTANCE)NULL,IDC_ARROW);
	pWndClass->hbrBackground= (HBRUSH)(COLOR_APPWORKSPACE+1);
	pWndClass->lpszClassName= L"gdiWindowClass";
	}	// getClassInfo

LRESULT gdiWindow :: onMsg ( UINT uMsg, WPARAM wParam, LPARAM lParam )
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

	// Clean up on destruction
	if (uMsg == WM_DESTROY)
		ui_hWnd = NULL;

	return DefWindowProc ( *this, uMsg, wParam, lParam );
	}	// onMsg

LRESULT CALLBACK gdiWindow ::
	gdiWindowProc ( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam )
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
	LRESULT	ret = 0;

	// Extract object pointer
	#if	!defined(UNDER_CE)
	gdiWindow *pThis = (gdiWindow *) GetWindowLongPtr(hWnd,GWLP_USERDATA);
	#else
	gdiWindow *pThis = (gdiWindow *) GetWindowLong(hWnd,GWL_USERDATA);
	#endif

	// Process message
	if (pThis != NULL)
		ret = pThis->onMsg ( uMsg, wParam, lParam );

	// Invalid object, creation ?
	else if (uMsg == WM_CREATE)
		{
		// Depending on the window type, the creation structure may be different
		LONG gwl = GetWindowLong ( hWnd, GWL_EXSTYLE );

		// MDI Child ?
		#ifdef	WS_EX_MDICHILD
		if (gwl & WS_EX_MDICHILD)
			pThis = (gdiWindow *) ((LPMDICREATESTRUCT)
											(((LPCREATESTRUCT)(lParam))->lpCreateParams))->lParam;

		// 'Normal' window
		else
		#endif
			pThis = (gdiWindow *) ((LPCREATESTRUCT)(lParam))->lpCreateParams;

		// Setup object pointer.
		pThis->ui_hWnd			= hWnd;					// Window handle
		#if	!defined(UNDER_CE)
		SetWindowLongPtr ( hWnd, GWLP_USERDATA, (LONG_PTR)pThis );
		#else
		SetWindowLong ( hWnd, GWL_USERDATA, (LONG)pThis );
		#endif
		ret = pThis->onMsg ( uMsg, wParam, lParam );
		}	// if
	else
		ret = DefWindowProc ( hWnd, uMsg, wParam, lParam );

	return ret;
	}	// gdiWindowProc

*/
