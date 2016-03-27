////////////////////////////////////////////////////////////////////////
//
//								Tree.CPP
//
//					Tree class implementation
//
////////////////////////////////////////////////////////////////////////

#include "widgetl_.h"
#include <stdio.h>

// Globals
extern HINSTANCE	ccl_hInst;

Tree :: Tree ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pDct		= NULL;
	pOwn		= NULL;
	hWnd		= NULL;
	pLstPath	= NULL;
//	pLstSel	= NULL;
//	pLstSelR	= NULL;
	}	// Tree

HRESULT Tree :: find	( HTREEITEM hParent, const WCHAR *wTxt, HTREEITEM *ph )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Find the child item that has the specified text.
	//
	//	PARAMETERS
	//		-	hParent is the parent item.
	//		-	wTxt is the text to match
	//		-	ph will receive the selected item, or NULL if not found
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;
	HTREEITEM	hChild;
	TVITEM		tvi;

	// Find child item that matches key text
	*ph = NULL;
	for (	hChild = TreeView_GetChild ( *this, hParent );
			hChild != NULL && *ph == NULL;
			hChild = TreeView_GetNextSibling ( *this, hChild ) )
		{
		// Obtain text of item
		memset ( &tvi, 0, sizeof(tvi) );
		tvi.hItem		= hChild;
		tvi.mask			= TVIF_TEXT;
		tvi.pszText		= wBfr;
		tvi.cchTextMax	= sizeof(wBfr)/sizeof(WCHAR);
		if (	TreeView_GetItem ( *this, &tvi ) &&
				!WCASECMP(wBfr,wTxt) )
			*ph = hChild;
		}	// for

	return (hr == S_OK && *ph != NULL) ? S_OK : ERROR_NOT_FOUND;
	}	// find

HRESULT Tree :: onAttach ( bool bAttach )
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
		// List of path names
		CCLTRY(COCREATE(L"Adt.List",IID_IList,&pLstPath));

		// Lists for selections
//		CCLTRY(COCREATE(L"Adt.List",IID_IList,&pLstSel));
//		CCLTRY(COCREATE(L"Adt.List",IID_IList,&pLstSelR));
		}	// if

	// Detach
	if (!bAttach)
		{
//		_RELEASE(pLstSelR);
//		_RELEASE(pLstSel);
//		_RELEASE(pPath);
		_RELEASE(pLstPath);
		_RELEASE(pDct);
		_RELEASE(pOwn);
		}	// if

	return hr;
	}	// onAttach

LRESULT Tree :: onMessage ( IDictionary *pCtl, UINT uMsg, 
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

	// Handle messages
	switch (uMsg)
		{
		case WM_NOTIFY :
			{
			NMHDR	*hdr = (NMHDR *)lParam;
			switch (hdr->code)
				{
				case TVN_SELCHANGED :
				case NM_DBLCLK :
					{
					HRESULT		hr		= S_OK;
					NMTREEVIEW	*tv	= (NMTREEVIEW *)lParam;
					HTREEITEM	hAt	= NULL;
					adtString	strPath;
					WCHAR			wTxt[256];

					// Setup
					CCLOK ( strPath = L""; )

					// Generate path up to root from selected value
					if (hdr->code == TVN_SELCHANGED)
						hAt = tv->itemNew.hItem;
					else
						hAt = TreeView_GetSelection ( *this );
					while (hr == S_OK && hAt != NULL)
						{
						TVITEM	tvi;

						// Get text for current value
						memset ( &tvi, 0, sizeof(tvi) );
						tvi.hItem		= hAt;
						tvi.mask			= TVIF_TEXT;
						tvi.pszText		= wTxt;
						tvi.cchTextMax	= sizeof(wTxt)/sizeof(WCHAR);
						if (TreeView_GetItem ( *this, &tvi ))
							{
							// Prepend next location
							CCLTRY ( strPath.prepend ( L"/" ) );
							CCLTRY ( strPath.prepend ( tvi.pszText ) );
							}	// if

						// Next level up
						hAt = TreeView_GetParent ( *this, hAt );
						}	// while

					// Final root at top
					CCLTRY ( strPath.prepend ( L"/" ) );

					// Remove trailing slash
					CCLOK ( strPath.at(strPath.length()-1) = '\0'; )

					// Notify
					dbgprintf ( L"TreeView::onSelect:%p:%s\r\n", pCtl, (LPCWSTR)strPath );
					_EMT(Dictionary,adtIUnknown(pCtl) );
					if (hdr->code == TVN_SELCHANGED)
						_EMT(Select,strPath);
					else
						_EMT(Activate,strPath);					
					}	// TVN_SELCHANGED
					break;

				// Drag
				case TVN_BEGINDRAG :
					{
					HRESULT			hr		= S_OK;
					NMTREEVIEW		*ptv	= (NMTREEVIEW *) lParam;
					dbgprintf ( L"UITree::message:TVN_BEGINDRAG!!\r\n" );

					// Control context
//					CCLOK ( peCtx->emit ( adtIUnknown(pCtx) ); )

					// Update position context
//					CCLTRY ( pCtxAt->store ( strRefXpos, adtInt(ptv->ptDrag.x) ) );
//					CCLTRY ( pCtxAt->store ( strRefYpos, adtInt(ptv->ptDrag.y) ) );
//					CCLOK  ( peAt->emit ( adtIUnknown(pCtxAt) ); )

					// Item Id for drag
//					CCLOK ( peDrag->emit ( adtLong((U64)ptv->itemNew.hItem) ); )
					}	// TVN_BEGINDRAG
					break;

				default :
					ret = gdiControl::onMessage ( pCtl, uMsg, wParam, lParam );
				}	// switch
			}	// WM_NOTIFY
			break;

		// Forward input to parent
		case WM_LBUTTONDOWN	:
		case WM_LBUTTONUP		:
		case WM_MBUTTONDOWN	:
		case WM_MBUTTONUP		:
		case WM_RBUTTONDOWN	:
		case WM_RBUTTONUP		:
		case WM_MOUSEMOVE		:
			// Forward input
			gdiControl::forwardInput ( pCtl, uMsg, wParam, lParam );

			// Default handling
			ret = gdiControl::onMessage ( pCtl, uMsg, wParam, lParam );
			break;

		default :
			ret = gdiControl::onMessage ( pCtl, uMsg, wParam, lParam );
		}	// switch

	return ret;
	}	// onMessage

HRESULT Tree :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

	// Open
	if (_RCP(Open))
		{
		HWND			hWndP		= NULL;
		adtValue		vL;
		adtLong		lWnd;

		// State check
		CCLTRYE ( pDct != NULL, ERROR_INVALID_STATE );

		// Already open ?
		CCLTRYE ( pDct->load ( adtString(L"Window"), vL ) != S_OK, ERROR_INVALID_STATE );

		// Owner required for child controls
		CCLTRYE ( (pOwn != NULL), ERROR_INVALID_STATE );
		CCLTRY  ( pOwn->load ( adtString(L"Window"), lWnd ) );
		CCLTRYE ( (hWndP = (HWND)(U64)lWnd) != NULL, ERROR_INVALID_STATE );

		// Create control
		CCLTRYE ( (hWnd = CreateWindowEx ( WS_EX_CLIENTEDGE, L"SysTreeView32", NULL, 
						WS_CHILD|WS_VISIBLE|TVS_HASLINES|TVS_LINESATROOT|TVS_HASBUTTONS|
						TVS_SHOWSELALWAYS|TVS_DISABLEDRAGDROP,
						0, 0, 10, 10,
						hWndP, (HMENU)0, ccl_hInst, (gdiControl *) this )) != NULL, GetLastError() );

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
		adtLong	lWnd;

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

	//
	// Tree specific work
	//

	// Store
	else if (_RCP(Store))
		{
		IIt				*pIt		= NULL;
		HTREEITEM		hItem		= NULL;
		HTREEITEM		hFound	= NULL;
		TVINSERTSTRUCT	tvis;
		adtLong			lWnd;
		adtIUnknown		unkV(v);
		adtValue			vL;

		// State check
		CCLTRYE	( pDct != NULL, ERROR_INVALID_STATE );
		CCLOK		( pDct->load ( adtString(L"Window"), lWnd ); )
		CCLTRYE	( IsWindow((hWnd = (HWND)(U64)lWnd)), ERROR_INVALID_STATE );
		CCLTRYE	( pLstPath->isEmpty() != S_OK, ERROR_INVALID_STATE );

		// Ensure each token exists in tree
		CCLTRY ( pLstPath->iterate ( &pIt ) );
		while (hr == S_OK && pIt->read ( vL ) == S_OK)
			{
			adtString	strKey(vL);

			// Find child item that matches key text
			find ( hItem, strKey, &hFound );

			// If entry was not found, time to add it
			if (hFound == NULL)
				{
				// Insert new item
				memset ( &tvis, 0, sizeof(tvis) );
				tvis.hParent			= (HTREEITEM) hItem;
				tvis.hInsertAfter		= TVI_LAST;
				tvis.item.mask			= TVIF_TEXT;
				tvis.item.pszText		= &strKey.at();
				hFound = TreeView_InsertItem ( hWnd, &tvis );

				// Default is to display added items
				if (hFound != NULL)
					TreeView_EnsureVisible ( hWnd, hFound );
				}	// if

			// Clean up
			hItem = hFound;
			pIt->next();
			}	// while

		// Clean up
		_RELEASE(pIt);
		}	// else if

	// Reset
	else if (_RCP(Reset))
		{
		adtLong		lWnd;
		adtIUnknown	unkV(v);

		// State check
		CCLTRYE	( pDct != NULL, ERROR_INVALID_STATE );
		CCLOK		( pDct->load ( adtString(L"Window"), lWnd ); )
		CCLTRYE	( IsWindow((hWnd = (HWND)(U64)lWnd)), ERROR_INVALID_STATE );

		// Clear contents
		CCLOK ( TreeView_DeleteAllItems ( hWnd ); )
		}	// else if

	// Select (by path)
	else if (_RCP(Select))
		{
		HTREEITEM	hAt	= NULL;
		IIt			*pIt	= NULL;
		adtLong		lWnd;
		adtString	strPath(v);
		adtValue		vL;

		// State check
		CCLTRYE	( pDct != NULL, ERROR_INVALID_STATE );
		CCLOK		( pDct->load ( adtString(L"Window"), lWnd ); )
		CCLTRYE	( IsWindow((hWnd = (HWND)(U64)lWnd)), ERROR_INVALID_STATE );
		CCLTRYE	( pLstPath->isEmpty() != S_OK, ERROR_INVALID_STATE );

		// Update dictionary
//		if (pDct != NULL)
//			pDct->store ( adtString(L"Select"), v );

		// Select item at the end of the path.
		// To support recursive selection messages, do not select item if it is already selected.

		// March down the hierarchy
		CCLTRY ( pLstPath->iterate ( &pIt ) );
		while (hr == S_OK && pIt->read ( vL ) == S_OK)
			{
			adtString	strKey(vL);

			// Find selected text
			if (find ( hAt, strKey, &hAt ) != S_OK)
				break;

			// Clean up
			pIt->next();
			}	// while

		// Item found ?
		if (hr == S_OK && hAt != NULL)
			{
			// Select item if not selected
			if (TreeView_GetSelection ( *this ) != hAt)
				TreeView_SelectItem ( *this, hAt );
			}	// if
		else if (hWnd != NULL && hAt == NULL)
			dbgprintf ( L"Tree::receive:Select error:%s\r\n", (LPCWSTR)strPath );

		// Clean up
		_RELEASE(pIt);
		}	// else if

/*
	// Tree
	else if (_RCP(Tree))
		{
		adtLong		lWnd;
		adtIUnknown	unkV(v);

		// State check
		CCLTRYE	( pDct != NULL, ERROR_INVALID_STATE );
		CCLOK		( pDct->load ( adtString(L"Window"), lWnd ); )
		CCLTRYE	( IsWindow((hWnd = (HWND)(U64)lWnd)), ERROR_INVALID_STATE );

		// Update dictionary
		if (pDct != NULL)
			pDct->store ( adtString(L"Tree"), v );

		// Valid widget ?
		if (hr == S_OK)
			{
			IDictionary	*pTree	= NULL;

			// Interface to tree
			CCLTRY ( _QISAFE(unkV,IID_IDictionary,&pTree) );

			// Synchronize control to tree
			CCLOK ( sync ( NULL, pTree ); )

			// Clean up
			_RELEASE(pTree);
			}	// if

		}	// else if

*/
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
	else if (_RCP(Path))
		{
		// Generate list of tokens in path
		CCLTRY ( pLstPath->clear() );
		CCLTRY ( nspcTokens ( adtString(v), L"/", pLstPath ) );
		}	// if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

HRESULT Tree :: sync ( HTREEITEM hItem, IDictionary *pDct )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Synchronize the tree items at the specified level with
	//			the items in the provided dictionary.
	//
	//	PARAMETERS
	//		-	hItem is the current tree item.
	//		-	pDct contains the reference state for the tree.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr			= S_OK;
	IIt				*pKeys	= NULL;
	HTREEITEM		hChild	= NULL;
	HTREEITEM		hFound	= NULL;
	adtValue			vK,vV;
	adtIUnknown		unkV;
	TVINSERTSTRUCT	tvis;

	// Add items in the dictionary that are not under the item.
	CCLTRY ( pDct->keys ( &pKeys ) );
	while (hr == S_OK && pKeys->read ( vK ) == S_OK)
		{
		IDictionary	*pChild	= NULL;
		adtString	strKey(vK);

		// Dictionary ?
		if (	hr == S_OK &&
				pDct->load ( vK, vV ) == S_OK &&
				(IUnknown *)(NULL) != (unkV=vV) &&
				_QI(unkV,IID_IDictionary,&pChild) == S_OK)
			{ 
			// Find child item that matches key text
			find ( hItem, strKey, &hFound );

			// If entry was not found, time to add it
			if (hFound == NULL)
				{
				// Insert new item
				memset ( &tvis, 0, sizeof(tvis) );
				tvis.hParent			= (HTREEITEM) hItem;
				tvis.hInsertAfter		= TVI_LAST;
				tvis.item.mask			= TVIF_TEXT;
				tvis.item.pszText		= &strKey.at();
				hFound = TreeView_InsertItem ( hWnd, &tvis );

				// Default is to display added items
				if (hFound != NULL)
					TreeView_EnsureVisible ( hWnd, hFound );
				}	// if

			// Synchronize this dictionary to child item
			if (hFound != NULL)
				hr = sync ( hFound, pChild );
			}	// if

		// Clean up
		_RELEASE(pChild);
		pKeys->next();
		}	// while

	// Clean up
	_RELEASE(pKeys);

	// Remove items under item that are not in the dictionary


	return hr;
	}	// sync
