////////////////////////////////////////////////////////////////////////
//
//								DROPT.CPP
//
//						Drop target implementation
//
////////////////////////////////////////////////////////////////////////

#include "widgetl_.h"
#include <stdio.h>

// Globals
extern HINSTANCE	ccl_hInst;

DropTarget :: DropTarget ( HWND _hWnd ) :
	// String references
	strX(L"X"),strY(L"Y"),strType(L"Type")
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	//	PARAMETERS
	//		-	_hWnd is the window to associate with the object.
	//		-	_prTst is the drop test receptor
	//		-	_prDrop is the drop receptor
	//
	////////////////////////////////////////////////////////////////////////
	hWnd		= _hWnd;
	pThrd		= NULL;
	dwThrdId	= 0;
	pDctDrop	= NULL;
	fmtData	= 0;
	uNspcFmt	= 0;

	// So creator does not have too...
	AddRef();
	}	// DropTarget

HRESULT DropTarget :: construct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		CCLObject
	//
	//	PURPOSE
	//		-	Called when the object is being created.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr	= S_OK;

	// Run-time data
	CCLTRY(COCREATE(L"Adt.Dictionary",IID_IDictionary,&pDctDrop));

	// Create worker thread
	CCLTRY(COCREATE(L"Sys.Thread",IID_IThread,&pThrd));
	CCLTRY(pThrd->threadStart(this,0));

	return hr;
	}	// construct

void DropTarget :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		CCLObject
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed
	//
	////////////////////////////////////////////////////////////////////////

	// Shutdown thread
	if (pThrd != NULL)
		{
		pThrd->threadStop(5000);
		_RELEASE(pThrd);
		}	// if
	_RELEASE(pDctDrop);
	}	// destruct

HRESULT DropTarget :: DragEnter ( IDataObject *pObj, DWORD dwState,
												POINTL pt, DWORD *pdwEff )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IDropTarget
	//
	//	PURPOSE
	//		-	Indiciates whether a drop can be accepted.
	//
	//	PARMAETERS
	//		-	pObj is the source data
	//		-	dwState is keyboard modifier state
	//		-	pt are the cursor coordinates
	//		-	pdwEff is the operation effect
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr			= S_OK;
	IEnumFORMATETC	*pEnum	= NULL;
	FORMATETC		fmt;
	DWORD				dw;

	// To client coordinates
	ScreenToClient ( hWnd, (POINT *) &pt );

	// Debug
	dbgprintf ( L"Window::DragEnter:%p,0x%x,(%d,%d)\r\n",
					pObj, dwState, pt.x, pt.y );

	// Access nSpace format
	if (hr == S_OK && uNspcFmt == 0)
		uNspcFmt = RegisterClipboardFormat ( L"nSpace.Value" );

	// Enumerate the data formats
	CCLTRY ( pObj->EnumFormatEtc ( DATADIR_GET, &pEnum ) );
	while (hr == S_OK && fmtData == 0 && pEnum->Next ( 1, &fmt, &dw ) == S_OK)
		{
		// Supported format ?

		// File drop
		if (fmt.cfFormat == CF_HDROP)
			{
			// Coordinates
			CCLTRY ( pDctDrop->store ( strX, adtInt(pt.x) ) );
			CCLTRY ( pDctDrop->store ( strY, adtInt(pt.y) ) );

			// Type
			CCLTRY ( pDctDrop->store ( strType, adtString(L"Files") ) );

			//
			// Unpack the files into an nSpace value.
			//


			// Done
			fmtData = CF_HDROP;
			}	// if

		// nSpace value
		else if (fmt.cfFormat == uNspcFmt)
			{
			dbgprintf ( L"DropTarget::DragEnter:nSpace value\r\n" );
			}	// else if

		}	// while

	// Clean up
	_RELEASE(pEnum);

	// Send message to host window.  Message required since
	// this IDropTarget executes in its own apartment thread.
	if (fmtData)
		PostMessage ( hWnd, WIDGET_USER_DROPTEST, 0, (LPARAM) pDctDrop );

	return hr;
	}	// DragEnter

HRESULT DropTarget :: DragLeave ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IDropTarget
	//
	//	PURPOSE
	//		-	Removes target feedback and releases data object
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Data now invalid
	fmtData = 0;

	// Need to notify window ?
//	dbgprintf ( L"DropTarget::DragLeave\r\n" );

	return hr;
	}	// DragLeave

HRESULT DropTarget :: DragOver ( DWORD dwState, POINTL pt, DWORD *pdwEff )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IDropTarget
	//
	//	PURPOSE
	//		-	Communicates drop effect.
	//
	//	PARMAETERS
	//		-	dwState is keyboard modifier state
	//		-	pt are the cursor coordinates
	//		-	pdwEff is the operation effect
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Update message
	if (fmtData)
		{
		// To client coordinates
		ScreenToClient ( hWnd, (POINT *) &pt );

		// Coordinates
		CCLTRY ( pDctDrop->store ( strX, adtInt(pt.x) ) );
		CCLTRY ( pDctDrop->store ( strY, adtInt(pt.y) ) );

		// Send message to host window.  Message required since
		// this IDropTarget executes in its own apartment thread.
		PostMessage ( hWnd, WIDGET_USER_DROPTEST, 0, (LPARAM) pDctDrop );
		}	// if

//	dbgprintf ( L"DropTarget::DragOver:0x%x,(%d,%d)\r\n",
//					dwState, pt.x, pt.y );

	return hr;
	}	// DragOver

HRESULT DropTarget :: Drop ( IDataObject *pObj, DWORD dwState,
										POINTL pt, DWORD *pdwEff )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IDropTarget
	//
	//	PURPOSE
	//		-	Incorporates the source data into the target window.
	//
	//	PARMAETERS
	//		-	pObj is the source data
	//		-	dwState is keyboard modifier state
	//		-	pt are the cursor coordinates
	//		-	pdwEff is the operation effect
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Data format
	dbgprintf ( L"DropTarget::Drop:%p,0x%x,(%d,%d),0x%x\r\n",
					pObj, dwState, pt.x, pt.y, fmtData );

	// Files
	if (fmtData == CF_HDROP)
		{
		}	// if

	// nSpace value
	else if (fmtData == uNspcFmt)
		{
		}	// else if

	// 
	// Notify
/*	if (pParent != NULL)
		{
		// Coordinates are given in screen coordinates, convert to client
		ScreenToClient ( *pParent, (POINT *) &pt );

		// Notify
		pParent->drop ( pObj, pt );
		}	// if
*/

	return hr;
	}	// Drop

HRESULT DropTarget :: tick ( void )
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
	MSG		msg;

	// Message loop
	while (GetMessage ( &msg, (HWND) NULL, 0, 0 ) != 0)
		{
		TranslateMessage(&msg);
		DispatchMessage(&msg);
		}	// while

	return S_FALSE;
	}	// tick

HRESULT DropTarget :: tickAbort ( void )
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

	// Close thread
	if (dwThrdId != 0)
		PostThreadMessage ( dwThrdId, WM_QUIT, 0, 0 );

	return S_OK;
	}	// tickAbort

HRESULT DropTarget :: tickBegin ( void )
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
	HRESULT	hr = S_OK;

	// Store thread ID for later notification
	CCLOK ( dwThrdId = GetCurrentThreadId(); )

	// Initialize OLE, this is required for Shell drag and drop operations
	// and thusly the reason for a seperate thread since by default all
	// threads are multi-threaded.
	CCLTRY( OleInitialize(NULL) );

	// Register as a drag and drop target
	CCLTRY( RegisterDragDrop(hWnd,this) );

	// Debug
	if (hr != S_OK)
		dbgprintf ( L"DropTarget::tickBegin:Error 0x%x\r\n", hr );

	return hr;
	}	// tickBegin

HRESULT DropTarget :: tickEnd ( void )
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

	// Unregister
	RevokeDragDrop(hWnd);

	// Clean up
	dwThrdId = 0;
	OleUninitialize();

	return S_OK;
	}	// tickEnd

