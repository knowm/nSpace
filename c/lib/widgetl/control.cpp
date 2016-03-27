////////////////////////////////////////////////////////////////////////
//
//								CONTROL.CPP
//
//						GDI control implementation
//
////////////////////////////////////////////////////////////////////////

#include "widgetl_.h"
#include <stdio.h>

// GDI+ reference counting.
static ULONG		lGDICnt	= 0;
static ULONG_PTR	lGDITok	= 0;

// String references
static adtString	strRefSub ( L"Subclass" );

gdiControl :: gdiControl ( void )
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
	pCtls		= NULL;
	hWndCtl	= NULL;
	}	// gdiControl

gdiControl :: ~gdiControl ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Destructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	_RELEASE(pCtls);
	}	// ~gdiControl

HRESULT gdiControl :: assign ( HWND hWnd, IDictionary *pCtl, bool bSub )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Assign a window handle and a dictionary to a control.
	//
	//	PARAMETERS
	//		-	hWnd is the window handle of the control.
	//		-	pCtl is the dictionary associated with the control.
	//		-	bSub is true to subclass the provided window
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;

	// Associated window handle with context
	if (hr == S_OK && pCtls == NULL)
		hr = COCREATE(L"Adt.Dictionary",IID_IDictionary,&pCtls);

	// Subclass ?
	if (hr == S_OK && bSub == true)
		{
		WNDPROC	pProc	= NULL;

		// Subclass the window
		CCLTRYE	( (pProc = (WNDPROC) (LONG_PTR) SetWindowLongPtr ( hWnd, GWLP_WNDPROC,
						(LONG_PTR)controlProcS )) != NULL, GetLastError() );

		// Update context
		CCLTRY	( pCtl->store	( strRefSub, adtLong((U64)pProc) ) );
		}	// if

	// Update context
	CCLTRY	( pCtl->store	( adtString(L"Window"),	adtLong((U64)hWnd) ) );
	CCLTRY	( pCtls->store ( adtLong((U64)hWnd),	adtIUnknown(pCtl) ) );

	// Associate this object with window
	CCLOK		( SetWindowLongPtr ( hWnd, GWLP_USERDATA, (LONG_PTR)this ); )

	return hr;
	}	// assign

LRESULT gdiControl :: controlProc ( HWND hWnd, UINT uMsg, WPARAM wParam, 
												LPARAM lParam )
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
	HRESULT		hr		= S_OK;
	IDictionary	*pCtl	= NULL;
	LRESULT		ret	= 0;
	adtIUnknown	unkV;

	// Active window
	hWndCtl = hWnd;

	// Attempt to load assigned context
	CCLTRYE	( pCtls != NULL, E_UNEXPECTED );
	CCLTRY	( pCtls->load ( adtLong ((U64)hWnd), unkV ) );
	CCLTRY	( _QISAFE(unkV,IID_IDictionary,&pCtl) );

	// Process message
	CCLOK		( (ret = onMessage ( pCtl, uMsg, wParam, lParam )); )

	// Do NOT set hWndCtl to NULL when complete to handle re-entrant messages
//	hWndCtl = NULL;

	// Clean up
	_RELEASE(pCtl);

	return ret;
	}	// controlProc

LRESULT CALLBACK gdiControl ::
	controlProcS ( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam )
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
	gdiControl *pThis = (gdiControl *) GetWindowLongPtr(hWnd,GWLP_USERDATA);
	#else
	gdiControl *pThis = (gdiControl *) GetWindowLong(hWnd,GWL_USERDATA);
	#endif

	// Process message
	if (pThis != NULL)
		ret = pThis->controlProc ( hWnd, uMsg, wParam, lParam );
	else
		ret = DefWindowProc ( hWnd, uMsg, wParam, lParam );

	return ret;
	}	// controlProcS

LRESULT gdiControl :: forwardInput ( IDictionary *pCtl, UINT uMsg, 
													WPARAM wParam, LPARAM lParam )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Forward input messages to parent window.
	//
	//	PARAMETERS
	//		-	pCtl is the assigned control dictionary
	//		-	uMsg is the message
	//		-	wParam,lParam are the parameters for the message
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	LRESULT	ret = 1;

	// Handle relevant messages
	switch (uMsg)
		{
		// Forward input to parent
		case WM_LBUTTONDOWN		:
		case WM_LBUTTONUP			:
		case WM_MBUTTONDOWN		:
		case WM_MBUTTONUP			:
		case WM_RBUTTONDOWN		:
		case WM_RBUTTONUP			:
		case WM_MOUSEMOVE			:
			{
			POINT	pt = { LOWORD(lParam), HIWORD(lParam) };

			// Convert input to parent coordinates
			ClientToScreen ( *this, &pt );
			ScreenToClient ( GetParent ( *this ), &pt );

			// Forward input
			PostMessage ( GetParent ( *this ), uMsg, wParam, MAKELONG(pt.x,pt.y) );

			// Processed
			ret = 0;
			}
			break;
		}	// switch

	return ret;
	}	// forwardInput

ULONG gdiControl :: gdiAddRef ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Increase reference count on GDI+ usage.
	//
	//	RETURN VALUE
	//		Reference count
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;

	// Reference count
	if (++lGDICnt == 1)
		{
		Gdiplus::GdiplusStartupInput	gsi;
		Gdiplus::GdiplusStartup ( &lGDITok, &gsi, NULL );
		}	// if

	return lGDICnt;
	}	// gdiAddRef

ULONG gdiControl :: gdiRelease ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Decrease reference count on GDI+ usage.
	//
	//	RETURN VALUE
	//		Reference count
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;

	// Reference count
	if (lGDICnt > 0 && --lGDICnt == 0)
		{
		Gdiplus::GdiplusShutdown ( lGDITok );
		lGDITok = 0;
		}	// if

	return lGDICnt;
	}	// gdiRelease

LRESULT gdiControl :: onMessage ( IDictionary *pCtl, UINT uMsg, 
												WPARAM wParam, LPARAM lParam )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Process control message.
	//
	//	PARAMETERS
	//		-	pCtl is the assigned control dictionary
	//		-	uMsg is the message
	//		-	wParam,lParam are the parameters for the message
	//
	//	RETURN VALUE
	//		Message dependent
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr		= S_OK;
	WNDPROC	pProc	= NULL;
	adtLong	lInt;

	// If window is subclassed, the original window procedure is called,
	// otherwise a default handler is used.
	CCLTRYE ( pCtl != NULL, E_UNEXPECTED );
	if (hr == S_OK && pCtl->load ( strRefSub, lInt ) == S_OK)
		pProc = (WNDPROC)(U64)lInt;

	// Default procssing
	switch (uMsg)
		{
		case WIDGET_USER_RECEIVE :
			{
			CTLRX	*p = (CTLRX *)lParam;

			// Receive the value again on this correct thread
			if (p->prRxT != NULL)
				p->prRxT->receive ( p->prRxF, p->plRx, *p->pvRx );
			}	// WIDGET_USER_RECEIVE
			break;
		}	// switch

	// Default
	return (pProc != NULL) ?	CallWindowProc ( pProc, hWndCtl, uMsg, wParam, lParam ) :
										DefWindowProc ( hWndCtl, uMsg, wParam, lParam );
	}	// onMessage

HRESULT gdiControl :: receive ( IDictionary *pCtl, IReceptor *prT,
											IReceptor *prF,  const WCHAR *pl, 
											const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Receive value into the same thread in which the window was 
	//			created.
	//
	//	PARAMETERS
	//		-	pCtl is the control dictionary
	//		-	prT is the receptor to which to receive the value.
	//		-	prF is the receptor from which the value was received.
	//		-	pl is the receive location
	//		-	v is the value
	//
	//	RETURN VALUE
	//		S_OK if receive should be processed as normal
	//		S_FALSE is receive was processed on window thread
	//
	////////////////////////////////////////////////////////////////////////
	HWND		hWnd	= NULL;
	adtLong	lWnd;
	adtValue	vL;

	// Determine if current thread is ok.
	if	(	pCtl == NULL												||
			pCtl->load ( strRefWindow, vL ) != S_OK			||
			!IsWindow ( (hWnd = (HWND)(U64)(lWnd = vL)) )	||
			GetWindowThreadProcessId ( hWnd, NULL ) ==
			GetCurrentThreadId() )
		return S_OK;

	// Window is valid but reception was on another thread.
	// Process receive via message.
	DWORD_PTR	dwRes;
	CTLRX			rx;

	// Store parameters
	rx.prRxT	=	prT;
	rx.prRxF	=	prF;
	rx.plRx	=	pl;
	rx.pvRx	=	&v;

	// Send the message and avoid infinite blocking
	SendMessageTimeout ( hWnd, WIDGET_USER_RECEIVE, 0, (LPARAM) &rx, 
								SMTO_NORMAL, 5000, &dwRes );			

	return S_FALSE;
	}	// receive

HRESULT gdiControl :: transform ( HWND hParent, IUnknown *punk, 
												RECT *rct, POINT pts[4] )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Perform GDI calculations using provded 3D transform.
	//
	//	PARAMETERS
	//		-	hParent is the parent window to use
	//		-	punk is the tranformation matrix
	//		-	rct will receve the bounding rectangle
	//		-	pts (optional) will receive the unit square mapping
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	IDictionary	*pMat	= NULL;
	IIt			*pIt	= NULL;
	U32			sz		= 0;
	double		dMat[16],dSq[5][4],dXmin,dXmax,dYmin,dYmax;
	adtValue		vL;
	int			i;
	RECT			pr;

	// Remove previous

	// Compute bounding rectangle of window required for matrix.
	CCLTRY ( _QISAFE(punk,IID_IDictionary,&pMat) );

	// Read in the matrix components
	CCLTRY ( pMat->iterate ( &pIt ) );
	while (hr == S_OK && pIt->read ( vL ) == S_OK && sz < 16)
		{
		// Assign value
		dMat[sz++] = adtDouble(vL);

		// Clean up
		pIt->next();
		}	// while

	// Expecting 4x4 matrix
	CCLTRYE ( sz == 16, E_UNEXPECTED );

	// The unit square is mapped to the extent of the view (window)
	// Combine the matrix with the parent window area.
	if (hr == S_OK)
		{
		double	dMatW[16],dMatRes[16];
		U32		nr;

		// Curent size of target/client area
		GetClientRect ( (hParent != NULL) ? hParent : GetDesktopWindow(), &pr );

		// Window matrix
		memset ( dMatW, 0, sizeof(dMatW) );
		dMatW[0] = (pr.right-pr.left);				// Scaling
		dMatW[5] = (pr.bottom-pr.top);				// Scaling
		dMatW[12] = dMatW[0]/2;							// GDI center is upper left (0,0)
		dMatW[13] = dMatW[5]/2;

		// Combine window matrix with provided matrix
		mathMult ( dMat, 16, dMatW, 16, dMatRes, &nr );

		// Use result as control matrix
		memcpy ( dMat, dMatRes, sizeof(dMat) );
		}	// if

	// Transform the unit square through the matrix and compute bounding rectangle
	for (i = 0;hr == S_OK && i < 4;++i)
		{
		// Unit square corners
		dSq[4][0] = (i == 0 || i == 1) ? -0.5 : +0.5;
		dSq[4][1] = (i == 0 || i == 2) ? -0.5 : +0.5;
		dSq[4][2] = 0;
		dSq[4][3] = 1;

		// Transform
		mathMult ( dSq[4], 4, dMat, 16, dSq[i], &sz );
//		dbgprintf ( L"(%g,%g) -> (%g,%g)\r\n", 
//						dSq[4][0], dSq[4][1], dSq[i][0], dSq[i][1] );

		// The unit square is mapped to the extent of the view (window)
		// Scale unit square before application of matrix.
//		dSq[4][0] *= (pr.right-pr.left);
//		dSq[4][1] *= (pr.bottom-pr.top);

		// Limits
		if (i == 0)
			{
			dXmin	= dXmax = dSq[0][0];
			dYmin	= dYmax = dSq[0][1];
			}	// if
		else
			{
			if (dXmin > dSq[i][0])	dXmin = dSq[i][0];
			if (dXmax < dSq[i][0])	dXmax = dSq[i][0];
			if (dYmin > dSq[i][1])	dYmin = dSq[i][1];
			if (dYmax < dSq[i][1])	dYmax = dSq[i][1];
			}	// else
		}	// for

	// Compute bounding rectangle, GDI Y coordinates inverted from 3D
	if (hr == S_OK)
		{
		rct->left	= (long)dXmin;
		rct->top		= (long)(pr.bottom - (dYmin + (dYmax-dYmin)));
		rct->right	= (long)dXmax;
		rct->bottom	= (long)(pr.bottom - (dYmax - (dYmax-dYmin)));
//		dbgprintf ( L"(%d,%d,%d,%d)\r\n", rct->left, rct->top, rct->right, rct->bottom );
		}	// if

	// If caller is requesting the mapping of the unit square..
	if (hr == S_OK && pts != NULL)
		{
		// Upper-left
		pts[0].x = (int)dSq[1][0];
		pts[0].y = (int)dSq[1][1];

		// Upper-right
		pts[1].x = (int)dSq[3][0];
		pts[1].y = (int)dSq[3][1];

		// Lower-left
		pts[2].x = (int)dSq[0][0];
		pts[2].y = (int)dSq[0][1];

		// Lower-right
		pts[3].x = (int)dSq[2][0];
		pts[3].y = (int)dSq[2][1];

		// Window will be offset by bounding rectangle so remove that from mapping.
		// Y coordinates are inverted.
		for (i = 0;i < 4;++i)
			{
			// Offset by bounding rectangle and invert Y in relation to container
//			dbgprintf ( L"%d) %d %d\r\n", i, pts[i].x, pts[i].y );
			pts[i].x -= (long)dXmin;
			pts[i].y -= (long)dYmin;

			// Invert Y inside container
			pts[i].y =	((rct->bottom-rct->top) - pts[i].y);
//			dbgprintf ( L"%d) %d %d\r\n", i, pts[i].x, pts[i].y );
			}	// for

		}	// if

	// Clean up
	_RELEASE(pIt);
	_RELEASE(pMat);

	return hr;
	}	// transform

HRESULT gdiControl :: unassign ( IDictionary *pCtl )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Unassign a window handle from a context.
	//
	//	PARAMETERS
	//		-	pCtl is the dictionary associated with the widget.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr		= S_OK;
	HWND		hWnd	= NULL;
	WNDPROC	pProc	= NULL;
	adtLong	lWnd,lInt;
	U32		sz;

	// Valid ?
	CCLTRYE ( (pCtl != NULL), E_UNEXPECTED );
	CCLTRY  ( pCtl->load ( adtString(L"Window"), lWnd ) );
	CCLTRYE ( (hWnd = (HWND)(U64)lWnd) != NULL, E_UNEXPECTED );

	// Subclassed ?
	if (hr == S_OK && pCtl->load ( strRefSub, lInt ) == S_OK)
		{
		// Previous window procedure
		CCLTRYE ( (pProc = (WNDPROC)(U64)lInt) != NULL, E_UNEXPECTED );

		// Restore original window procedure
		if (hr == S_OK && hWnd != NULL && pProc != NULL)
			SetWindowLongPtr(hWnd, GWLP_WNDPROC, (LONG_PTR)pProc);
		}	// if

	// Clear object ptr.
	if (hr == S_OK && hWnd != NULL)
		SetWindowLongPtr(hWnd, GWLP_USERDATA, (LONG_PTR)NULL);

	// Clean up contexts
	if (pCtl != NULL)
		{
		pCtl->remove	( strRefSub );
		pCtl->remove	( adtString(L"Window") );
		}	// if
	if (pCtls != NULL && hWnd != NULL)
		pCtls->remove	( adtLong((U64)hWnd) );

	// If assignments are empty, release
	if (pCtls != NULL && pCtls->size ( &sz ) == S_OK && sz == 0)
		{
		_RELEASE(pCtls);
		}	// if

	return hr;
	}	// unassign

