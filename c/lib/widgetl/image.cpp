////////////////////////////////////////////////////////////////////////
//
//									IMAGE.CPP
//
//							Image rendering node
//
////////////////////////////////////////////////////////////////////////

#include "widgetl_.h"
#include <math.h>

// Globals
extern HINSTANCE	ccl_hInst;

// Macros
#define	ALIGNROW(a)	( ((a) % 4) ? ( (a) + (4 - ((a) % 4)) ) : (a) )

Image :: Image ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pDct			= NULL;
	pOwn			= NULL;
	}	// Image

HRESULT Image :: image ( IDictionary *pImg )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Set the active image for the window.
	//
	//	PARAMETERS
	//		-	pImg contains the image information
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT					hr			= S_OK;
	IMemoryMapped			*pBits	= NULL;
	Gdiplus::Bitmap		*pBmp		= NULL;
	Gdiplus::PixelFormat	fmt		= 0;
	U8							*pcBits	= NULL;
	adtValue					vL;
	adtIUnknown				unkV;
	adtString				strFmt;
	adtStringSt				strBMP(L"GDIBMP");
	int						w,h,bpp;

	// State check
	CCLTRYE ( pDct != NULL, ERROR_INVALID_STATE );
	w = h = bpp = 0;

	// Previous bitmap
	if (hr == S_OK && pDct->load ( strBMP, vL ) == S_OK)
		{
		pDct->remove ( adtStringSt(strBMP) );
		delete (Gdiplus::Bitmap *)((U64)adtLong(vL));
		}	// if

	// Image properties
	CCLTRY ( pImg->load ( adtStringSt(L"Width"), vL ) );
	CCLTRYE( (w = adtInt(vL)) > 0, E_UNEXPECTED );
	CCLTRY ( pImg->load ( adtStringSt(L"Height"), vL ) );
	CCLTRYE( (h = adtInt(vL)) > 0, E_UNEXPECTED );
	CCLTRY ( pImg->load ( adtString(L"Format"), vL ) );
	CCLTRYE( (strFmt = vL).length() > 0, E_UNEXPECTED );

	// Image bits
	CCLTRY ( pImg->load ( adtString(L"Bits"), vL ) );
	CCLTRYE( (IUnknown *)NULL != (unkV = vL), E_UNEXPECTED );
	CCLTRY ( _QI(unkV,IID_IMemoryMapped,&pBits) );
	CCLTRY ( pBits->lock ( 0, 0, (void **) &pcBits, NULL ) );

	// Format
	if (hr == S_OK && !WCASECMP(strFmt,L"B8G8R8A8"))
		{
		fmt = PixelFormat32bppARGB;
		bpp = 32;
		}	// if
//	else if (hr == S_OK && !WCASECMP(strFmt,L"R8G8B8A8"))
//		bpp = 32;
	else if (hr == S_OK && !WCASECMP(strFmt,L"B8G8R8"))
		{
		fmt	= PixelFormat24bppRGB;
		bpp	= 24;
		}	// else if
	else if (hr == S_OK)
		hr = E_NOTIMPL;

	// Bitmap
	CCLTRYE ( (pBmp = new Gdiplus::Bitmap ( w, h, (w*bpp)/8, 
					fmt, pcBits )) != NULL, E_UNEXPECTED );
	CCLTRYE ( (pBmp->GetLastStatus() == Gdiplus::Ok), E_UNEXPECTED );

	// Result
	CCLTRY ( pDct->store ( strBMP, adtLong((U64)pBmp) ) );

	// Clean up
	_UNLOCK(pBits,pcBits);
	_RELEASE(pBits);

	return hr;
	}	// image

HRESULT Image :: onAttach ( bool bAttach )
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
	adtValue	vL;

	// Attach
	if (bAttach)
		{
		WNDCLASS	wc;

		// Window class.
		if (hr == S_OK && !GetClassInfo ( ccl_hInst, L"WIDGETN:Image", &wc ))
			{
			// 'Generic' Images.
			memset ( &wc, 0, sizeof(wc) );
			wc.lpfnWndProc		= gdiControl::controlProcS;
			wc.hInstance		= ccl_hInst;
			wc.hCursor			= LoadCursor ( (HINSTANCE)NULL, IDC_ARROW );
//			wc.hbrBackground	= GetSysColorBrush ( COLOR_WINDOW );
//			wc.hbrBackground	= (HBRUSH)GetStockObject ( BLACK_BRUSH );
			wc.hbrBackground	= NULL;
			wc.lpszClassName	= L"WIDGETN:Image";
			RegisterClass ( &wc );
			}	// if

		// GDI Plus
		gdiControl::gdiAddRef();
		}	// if

	// Detach
	else
		{
		// Remove GDI+ objects
		if (pDct != NULL && pDct->load ( adtString(L"GDIMAP"), vL ) == S_OK)
			{
			pDct->remove ( adtString(L"GDIMAP") );
			delete (Gdiplus::Point *)((U64)adtLong(vL));
			}	// if
		if (hr == S_OK && pDct != NULL && pDct->load ( adtStringSt(L"GDIBMP"), vL ) == S_OK)
			{
			pDct->remove ( adtString(L"GDIBMP") );
			delete (Gdiplus::Bitmap *)((U64)adtLong(vL));
			}	// if

		// Clean up
		_RELEASE(pDct);
		_RELEASE(pOwn);

		// GDI Plus
		gdiControl::gdiRelease();
		}	// if

	return hr;
	}	// onAttach

LRESULT Image :: onMessage ( IDictionary *pCtl, UINT uMsg, 
										WPARAM wParam, LPARAM lParam )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Process a Image message.
	//
	//	PARAMETERS
	//		-	pCtl is the assigned control dictionary
	//		-	uMsg is the message
	//		-	wParam,lParam are the parameters for the message
	//
	//	RETURN VALUE
	//		ImageProc return value
	//
	////////////////////////////////////////////////////////////////////////
	LRESULT	ret = 0;

	// Handle relevant messages
	switch (uMsg)
		{
		// Paint
		case WM_PAINT :
			{
			HRESULT				hr		= S_OK;
			HDC					hDC	= NULL;
			Gdiplus::Bitmap	*pBmp	= NULL;
			int					*pP	= NULL;
			U32					sz		= 0;
			adtValue				vL;
			PAINTSTRUCT			ps;

			// Setup
			CCLTRYE ( (hDC = BeginPaint ( *this, &ps )) != NULL, GetLastError() );
//			CCLOK ( dbgprintf ( L"Image::onMessage:WM_PAINT\r\n" ); )

			// State check
			CCLTRYE ( pCtl != NULL, ERROR_INVALID_STATE );
			CCLTRY  ( pCtl->load ( adtStringSt(L"GDIBMP"), vL ) );
			CCLTRYE ( (pBmp = (Gdiplus::Bitmap *)(U64)adtLong(vL)) != NULL, ERROR_INVALID_STATE );

			// Draw DIB
			if (hr == S_OK)
				{
				Gdiplus::Graphics	gr(hDC);
				Gdiplus::Point		pts[3];
				RECT					cr;

				// Entire window will be used
				CCLOK		( GetClientRect ( *this, &cr ); )

				// Mapping points specified ?
				if (hr == S_OK && pCtl->load ( adtString(L"GDIMAP"), vL ) == S_OK)
					{
					Gdiplus::Point	*map	= (Gdiplus::Point *)((U64)adtLong(vL));

					// Transfer to destination
					pts[0] = map[0];
					pts[1] = map[1];
					pts[2] = map[2];
					}	// if
				else
					{
					// Default is just corners of window
					pts[0].X = 0;
					pts[0].Y = 0;
					pts[1].X = cr.right;
					pts[1].Y = 0;
					pts[2].X = 0;
					pts[2].Y = cr.bottom;
					}	// else

				// Draw image to destination
//					dbgprintf ( L"Image::DrawImage:0x%x:pBmp %p:(%d,%d) (%d,%d) (%d,%d)\r\n", 
//										(HWND)*this, pBmp, pts[0].X, pts[0].Y, pts[1].X, pts[1].Y, pts[2].X, pts[2].Y );
				CCLOK		( gr.DrawImage ( pBmp, pts, 3 ); )
				}	// if

			// Debug
			else if (hDC != NULL)
				{
				RECT	cr;
				CCLOK ( GetClientRect ( *this, &cr ); )
				CCLOK ( SelectObject ( hDC, GetStockObject ( BLACK_BRUSH ) ); )
				CCLOK ( Rectangle ( hDC, 0, 0, cr.right, cr.bottom ); )
				}	// else if

			// Clean up
			_FREEMEM(pP);
			if (hDC != NULL)
				EndPaint ( *this, &ps );
			}	// WM_PAINT
			break;

//		case WM_ERASEBKGND :
//			ret = 0;
//			break;

		// Forward input to parent
		case WM_LBUTTONDOWN		:
		case WM_LBUTTONUP			:
		case WM_MBUTTONDOWN		:
		case WM_MBUTTONUP			:
		case WM_RBUTTONDOWN		:
		case WM_RBUTTONUP			:
		case WM_MOUSEMOVE			:
			ret = gdiControl::forwardInput ( pCtl, uMsg, wParam, lParam );
			break;

		default :
			ret = gdiControl::onMessage ( pCtl, uMsg, wParam, lParam );
		}	// switch

	return ret;
	}	// onMessage

HRESULT Image :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

	// Threading
	if (gdiControl::receive ( pOwn, this, pr, pl, v ) != S_OK)
		return S_OK;

	// Open
	if (_RCP(Open))
		{
		HWND				hWndP			= NULL;
		DWORD				dwStyle		= WS_CHILD|WS_VISIBLE;
		DWORD				dwStyleEx	= WS_EX_TRANSPARENT;
		adtValue			vL;
		adtLong			lWnd;

		// State check
		CCLTRYE ( pDct != NULL, ERROR_INVALID_STATE );

		// Already open ?
		CCLTRYE ( pDct->load ( adtString(L"Window"), vL ) != S_OK, ERROR_INVALID_STATE );

		// Owner required for child controls
		CCLTRYE ( (pOwn != NULL), ERROR_INVALID_STATE );
		CCLTRY  ( pOwn->load ( adtString(L"Window"), vL ) );
		CCLTRYE ( (hWndP = (HWND)(U64)(lWnd=vL)) != NULL, ERROR_INVALID_STATE );

		// Create child window
		CCLTRYE ( (hWnd = CreateWindowEx ( dwStyleEx, L"WIDGETN:Image", L"", dwStyle,
						0, 0, 10, 10, hWndP, (HMENU)0, ccl_hInst, (gdiControl *) this )) 
						!= NULL, GetLastError() );

		// Assign the control to the dictionary
		CCLTRY ( assign ( hWnd, pDct, false ) );

		// Drag and drop support if available
		if (	hr == S_OK &&
				pOwn->load ( strRefDragDrop, vL ) == S_OK)
			{
			IDropTarget	*pTarget	= NULL;
			adtIUnknown	unkV(vL);

			// Register own window
			if (_QI(unkV,IID_IDropTarget,&pTarget) == S_OK)
				RegisterDragDrop ( hWnd, pTarget );

			// Clean up
			_RELEASE(pTarget);
			}	// if

		// Result
		CCLOK ( _EMT(Open,adtIUnknown(pDct)); )
		}	// if

	// Close
	else if (_RCP(Close))
		{
		adtLong	lV;
		adtValue	vL;

		// State check
		CCLTRYE ( pDct != NULL, ERROR_INVALID_STATE );

		// Widget
		CCLOK ( unassign ( pDct ); )

		// Remove GDI+ objects
		if (pDct != NULL && pDct->load ( adtString(L"GDIMAP"), vL ) == S_OK)
			{
			pDct->remove ( adtString(L"GDIMAP") );
			delete (Gdiplus::Point *)((U64)(lV=vL));
			}	// if
		if (pDct != NULL && pDct->load ( adtStringSt(L"GDIBMP"), vL ) == S_OK)
			{
			pDct->remove ( adtString(L"GDIBMP") );
			delete (Gdiplus::Bitmap *)((U64)adtLong(vL));
			}	// if

		// Close window
		if (hWnd != NULL)
			{
			RevokeDragDrop(hWnd);
//HWND hWndP = GetParent(hWnd);
			DestroyWindow ( hWnd );
			hWnd = NULL;
//InvalidateRect ( hWndP, NULL, TRUE );
			}	// if

		// Result
		CCLOK ( _EMT(Close,adtIUnknown(pDct)); )
		}	// if

	// Matrix
	else if (_RCP(Matrix))
		{
		Gdiplus::Point	*map			= NULL;
		adtLong			lWnd;
		adtIUnknown		unkV(v);
		adtValue			vL;
		RECT				rct;
		POINT				pts[4];

		// State check
		CCLTRYE	( pDct != NULL, ERROR_INVALID_STATE );
		CCLOK		( pDct->load ( adtString(L"Window"), lWnd ); )
		CCLTRYE	( IsWindow((hWnd = (HWND)(U64)lWnd)), ERROR_INVALID_STATE );

		// Dimensions
		CCLTRY ( gdiControl::transform ( GetParent(hWnd), unkV, &rct, pts ) );

		// Create/access array of mapping points for parallelogram mapping.
		if (hr == S_OK && pDct->load ( adtString(L"GDIMAP"), vL ) != S_OK)
			{
			// Create a new array if points to map
			CCLTRYE ( ((map = new Gdiplus::Point[3]) != NULL), E_OUTOFMEMORY );

			// Associate points with widget
			CCLTRY ( pDct->store ( adtString(L"GDIMAP"), adtLong((U64)map) ) );
			}	// if
		else
			map = (Gdiplus::Point *)((U64)adtLong(vL));

		// Assign computed mapped points
		for (int i = 0;hr == S_OK && i < 3;++i)
			{
			map[i].X = pts[i].x;
			map[i].Y = pts[i].y;
			}	// for

		// Update size/position
		CCLOK ( MoveWindow ( hWnd, rct.left, rct.top, 
									(rct.right-rct.left), (rct.bottom-rct.top), TRUE ); )

		// Do this here ?
		CCLOK ( ShowWindow ( hWnd, SW_SHOWNORMAL ); )
		}	// else if

	// Image
	else if (_RCP(Image))
		{
		IDictionary	*pImg	= NULL;
		adtValue		vL;
		adtLong		lWnd;
		adtIUnknown	unkV(v);

		// State check
		CCLTRYE	( pDct != NULL, ERROR_INVALID_STATE );
		CCLOK		( pDct->load ( adtString(L"Window"), vL ); )
		CCLTRYE	( IsWindow((hWnd = (HWND)(U64)(lWnd=vL))), ERROR_INVALID_STATE );
		CCLTRY	( _QISAFE(unkV,IID_IDictionary,&pImg) );

		// Update dictionary 
		if (pDct != NULL)
			pDct->store ( adtString(L"Image"), v );

		// Set new image
		CCLTRY ( image ( pImg ) );

		// Update window
		if (IsWindow(hWnd))
			InvalidateRect ( hWnd, NULL, TRUE );

		// Clean up
		_RELEASE(pImg);
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

