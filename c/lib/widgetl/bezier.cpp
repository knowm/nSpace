////////////////////////////////////////////////////////////////////////
//
//									BEZIER.CPP
//
//						Bezier curve rendering node
//
////////////////////////////////////////////////////////////////////////

#include "widgetl_.h"
//#include	<GdiPlusGraphics.h>

// Globals
extern HINSTANCE	ccl_hInst;


Bezier :: Bezier ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pDct			= NULL;
	pOwn			= NULL;
	hWnd			= NULL;
	}	// Bezier

HRESULT Bezier :: onAttach ( bool bAttach )
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

		// Window class.
		if (hr == S_OK && !GetClassInfo ( ccl_hInst, L"WIDGETN:Bezier", &wc ))
			{
			// 'Generic' Beziers.
			memset ( &wc, 0, sizeof(wc) );
			wc.lpfnWndProc		= gdiControl::controlProcS;
			wc.hInstance		= ccl_hInst;
			wc.hCursor			= LoadCursor ( (HINSTANCE)NULL, IDC_ARROW );
//			wc.hbrBackground	= GetSysColorBrush ( COLOR_3DFACE );
			wc.hbrBackground	= NULL;
			wc.lpszClassName	= L"WIDGETN:Bezier";
			RegisterClass ( &wc );
			}	// if

		// GDI Plus
		gdiControl::gdiAddRef();
		}	// if

	// Detach
	else
		{
		_RELEASE(pDct);
		_RELEASE(pOwn);

		// GDI Plus
		gdiControl::gdiRelease();
		}	// if

	return hr;
	}	// onAttach

LRESULT Bezier :: onMessage ( IDictionary *pCtl, UINT uMsg, 
										WPARAM wParam, LPARAM lParam )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Process a Bezier message.
	//
	//	PARAMETERS
	//		-	pCtl is the assigned control dictionary
	//		-	uMsg is the message
	//		-	wParam,lParam are the parameters for the message
	//
	//	RETURN VALUE
	//		BezierProc return value
	//
	////////////////////////////////////////////////////////////////////////
	LRESULT	ret = 0;

	// Handle relevant messages
	switch (uMsg)
		{
		// Paint
		case WM_PAINT :
			{
			HRESULT		hr			= S_OK;
			HDC			hDC		= NULL;
			int			*pP		= NULL;
			U32			sz			= 0;
			adtValue		vL;
			PAINTSTRUCT	ps;

			// Setup
			CCLTRYE ( (hDC = BeginPaint ( hWnd, &ps )) != NULL, GetLastError() );

//			RECT	cr;
//			CCLOK ( GetClientRect ( *this, &cr ); )
//			CCLOK ( SelectObject ( hDC, GetStockObject ( WHITE_BRUSH ) ); )
//			CCLOK ( Rectangle ( hDC, 0, 0, cr.right, cr.bottom ); )

			// Vertex control points specified ?
			if (hr == S_OK && vertices ( &pP, &sz ) == S_OK)
				{
				Gdiplus::Graphics		gr(hDC);
				Gdiplus::Pen			pen(Gdiplus::Color(255,255,0,0));
				RECT						wr,wrn;

				// Currently only supports 4-control point pairs
				CCLTRYE ( sz == 8, E_UNEXPECTED );

				// Window will be the span of the coordinates
				if (hr == S_OK)
					{
					// Initialize rect
					wr.left		= pP[0];
					wr.top		= pP[1];
					wr.right		= wr.left + 1;
					wr.bottom	= wr.top + 1;

					// Expand rectangle to encompass curve
					for (int i = 1;i < 4;++i)
						{
						int	x = pP[2*i+0];
						int	y = pP[2*i+1];

						// Expand as necessary
						if (x < wr.left)
							wr.left = x;
						if (x > wr.right)
							wr.right = x;
						if (y < wr.top)
							wr.top = y;
						if (y > wr.bottom)
							wr.bottom = y;
						}	// for

					// Get current window rect of this window
					GetWindowRect ( hWnd, &wrn );
					ScreenToClient ( GetParent(hWnd), (LPPOINT) &wrn.left );
					ScreenToClient ( GetParent(hWnd), (LPPOINT) &wrn.right );

					// Did window size change ?
/*					if (!EqualRect ( &wr, &wrn ))
						{
						MoveWindow ( hWnd, wr.left, wr.top, wr.right-wr.left, wr.bottom-wr.top, FALSE );
						}	// if
					else
						dbgprintf ( L"Match!\r\n" );
*/
					}	// if

				// DEBUG
//				if (hr == S_OK)
//					{
//					pP[0] = 100;
//					pP[1] = 100;
//					pP[2] = 200;
//					pP[3] = 10;
//					pP[4] = 350;
//					pP[5] = 50;
//					pP[6] = 500;
//					pP[7] = 100;
//					}	// if

				// Draw curve
				CCLOK ( GetWindowRect ( hWnd, &wrn ); )
				CCLOK ( ScreenToClient ( GetParent(hWnd), (LPPOINT) &wrn.left ); )
				CCLTRYE ( (gr.DrawBezier ( &pen, pP[0], pP[1], pP[2], pP[3],
								pP[4], pP[5], pP[6], pP[7] ) == Gdiplus::Ok),
								E_UNEXPECTED );

				// Debug
				if (hr != S_OK)
					dbgprintf ( L"Bezier::onMessage:WM_PAINT:Unable to draw Bezier curve\r\n" );
				}	// if

			// Debug
			else
				{
				RECT	cr;
				CCLOK ( GetClientRect ( *this, &cr ); )
				CCLOK ( SelectObject ( hDC, GetStockObject ( BLACK_BRUSH ) ); )
				CCLOK ( Rectangle ( hDC, 0, 0, cr.right, cr.bottom ); )
				}	// else

			// Clean up
			_FREEMEM(pP);
			if (hDC != NULL)
				EndPaint ( hWnd, &ps );
			}	// WM_PAINT
			break;

		case WM_ERASEBKGND :
			ret = 0;
			break;

		default :
			ret = gdiControl::onMessage ( pCtl, uMsg, wParam, lParam );
		}	// switch

	return ret;
	}	// onMessage

HRESULT Bezier :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

	// Open
	if (_RCP(Open))
		{
		HWND			hWndP			= NULL;
		DWORD			dwStyle		= WS_CHILD|WS_VISIBLE;
		DWORD			dwStyleEx	= WS_EX_TRANSPARENT;
		adtInt		iX				= 0;
		adtInt		iY				= 0;
		adtInt		iW				= 20;
		adtInt		iH				= 20;
		adtValue		vL;
		adtLong		lWnd;

		// State check
		CCLTRYE ( pDct != NULL, ERROR_INVALID_STATE );

		// Owner required for child controls
		CCLTRYE ( (pOwn != NULL), ERROR_INVALID_STATE );
		CCLTRY  ( pOwn->load ( adtString(L"Window"), vL ) );
		CCLTRYE ( (hWndP = (HWND)(U64)(lWnd=vL)) != NULL, ERROR_INVALID_STATE );

		// Possible parameters
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

		// Need to create window ?
		if (hr == S_OK && pDct->load ( adtString(L"Window"), vL )  != S_OK)
			{
			RECT	cr;

			// Debug, same size as owner ?
			CCLOK ( GetClientRect ( hWndP, &cr ); )

			// Create child window
			CCLTRYE ( (hWnd = CreateWindowEx ( dwStyleEx, L"WIDGETN:Bezier", L"", dwStyle,
							0, 0, cr.right, cr.bottom, hWndP, NULL, ccl_hInst, (gdiControl *) this )) 
//							0, 0, 10, 10, hWndP, NULL, ccl_hInst, (gdiControl *) this )) 
							!= NULL, GetLastError() );

			// Assign the control to the dictionary
			CCLTRY ( assign ( hWnd, pDct, false ) );

			// Result
			CCLOK ( _EMT(Open,adtIUnknown(pDct)); )
			}	// if

		// Update the state
		if (hr == S_OK && pDct->load ( adtString(L"Vertex"), vL ) == S_OK)
			receive ( prVertex, pl, vL );
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

	// Vertices
	else if (_RCP(Vertex))
		{
		adtValue		vL;
		adtLong		lWnd;

		// State check
		CCLTRYE	( pDct != NULL, ERROR_INVALID_STATE );
		CCLOK		( pDct->load ( adtString(L"Window"), vL ); )
		CCLTRYE	( IsWindow((hWnd = (HWND)(U64)(lWnd=vL))), ERROR_INVALID_STATE );

		// Update dictionary and window
		if (pDct != NULL)
			pDct->store ( adtString(L"Vertex"), v );

		// Repaint control
		if (hr == S_OK)
			InvalidateRect ( hWnd, NULL, TRUE );
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

	return hr;
	}	// receive

HRESULT Bezier :: vertices ( int **ppV, U32 *pc )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called to retrieve the vertex positions for the contorl points.
	//
	//	PARAMETERS
	//		-	ppV will receive the vertrex array
	//		-	pc will receive the point count
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	IDictionary	*pVert	= NULL;
	IIt			*pIt		= NULL;
	U32			iCnt		= 0;
	adtValue		vL;
	adtIUnknown	unkV;

	// Setup
	(*pc) = 0;

	// State check
	CCLTRYE ( pDct != NULL, ERROR_INVALID_STATE );

	// Vertex array
	CCLTRY ( pDct->load ( adtString(L"Vertex"), vL ) );
	CCLTRY ( _QISAFE((unkV=vL),IID_IDictionary,&pVert) );

	// Vertex dictionary is a list of vertex dictionaries where
	// each vertex key is the order in the list.
	CCLTRY ( pVert->size( &iCnt ) );
	CCLTRYE( iCnt > 0, E_UNEXPECTED );

	// Allocate memory for internal structures to contain vertex information
	CCLTRYE ( ((*ppV) = (int *) _ALLOCMEM ( 2*iCnt*sizeof(int) )) 
					!= NULL, E_OUTOFMEMORY );
	CCLOK   ( memset ( (*ppV), 0, 2*iCnt*sizeof(int) ); )

	// Iterate through vertices and store applicable information
	CCLTRY ( pVert->keys ( &pIt ) );
	while (hr == S_OK && pIt->read ( vL ) == S_OK)
		{
		IDictionary	*pDctV	= NULL;
		adtInt		iIdx(vL);

		// Access dictionary
		CCLTRY(pVert->load ( iIdx, vL ) );
		CCLTRY(_QISAFE((unkV=vL),IID_IDictionary,&pDctV) );

		// Position information ?
		if (hr == S_OK && pDctV->load ( adtString(L"Position"), vL ) == S_OK)
			{
			IDictionary	*pDctP	= NULL;

			// Access position information
			CCLTRY(_QISAFE((unkV=vL),IID_IDictionary,&pDctP) );

			// Coordinates
			CCLOK ( (*ppV)[(*pc)++] = (pDctP->load ( adtString(L"X"), vL ) == S_OK) ? adtInt(vL) : 0; )
			CCLOK ( (*ppV)[(*pc)++] = (pDctP->load ( adtString(L"Y"), vL ) == S_OK) ? adtInt(vL) : 0; )

			// Debug
//			dbgprintf ( L"Widget::Bezier:Pos:(%d,%d)\r\n", (*ppV)[(*pc)-2], (*ppV)[(*pc)-1] );

			// Clean up
			_RELEASE(pDctP);
			}	// if

		// Clean up
		_RELEASE(pDct);
		pIt->next();
		}	// while

	// Clean up
	_RELEASE(pIt);
	_RELEASE(pVert);

	return hr;
	}	// vertices

