////////////////////////////////////////////////////////////////////////
//
//									DRAGDROP.CPP
//
//						Drag and drop node and helper objects.
//
////////////////////////////////////////////////////////////////////////

#include "widgetl_.h"
#include <shlobj.h>

// Globals
extern HINSTANCE	ccl_hInst;

DragDrop :: DragDrop ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	hWnd			= NULL;
	pOwn			= NULL;
	pdSrc			= NULL;
	pdDst			= NULL;
	pdDstA		= NULL;
	hevDst		= NULL;
	}	// DragDrop

HRESULT DragDrop :: onAttach ( bool bAttach )
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
		// Make sure the environment format is registered
		RegisterClipboardFormat ( L"nSpace Value" );
		}	// if

	// Detach
	else
		{
		// Clean up
		if (pdDst != NULL) pdDst->deny();
		_RELEASE(pdDst);
		_RELEASE(pdDstA);
		_RELEASE(pdSrc);
		_RELEASE(pOwn);
		}	// if

	return hr;
	}	// onAttach

HRESULT DragDrop :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

	// Start
	if (_RCP(Start))
		{
		adtValue		lWnd;

		// State check
		CCLTRYE ( pOwn != NULL, ERROR_INVALID_STATE );

		// Obtain owner window handle
		CCLTRY ( pOwn->load ( adtString ( L"Window" ), lWnd ) );
		CCLTRYE ( (hWnd = (HWND)(U64)adtLong ( lWnd )) != NULL, ERROR_INVALID_STATE );

		// If owner window handle is valid, start drag/drop operation 
		if (hr == S_OK && hWnd != NULL)
			hr = start ();
		}	// if

	// Stop
	else if (_RCP(Stop))
		{
		// Terminate previous drag and drop activity
		stop ();
		}	// if

	// Specify the drag type
	else if (_RCP(Type))
		{
		adtString	sType(v);

		// WIN32
		// Update drag effect.
		dbgprintf ( L"DragDrop::receive:Type:%s\r\n", (LPCWSTR) sType );
		if (pdDst != NULL)
			{
			// Set the drag effect directly
			pdDst->bAcceptf = false;
			if (!WCASECMP(sType,L"Copy"))
				{
				pdDst->dwDragEff = DROPEFFECT_COPY;
				pdDst->bAcceptf = true;
				}	// if
			else if (!WCASECMP(sType,L"Move"))
				{
				pdDst->dwDragEff = DROPEFFECT_MOVE;
				pdDst->bAcceptf = true;
				}	// if
			else if (!WCASECMP(sType,L"Link"))
				{
				pdDst->dwDragEff = DROPEFFECT_LINK;
				pdDst->bAcceptf = true;
				}	// if
			else if (!WCASECMP(sType,L"None"))
				pdDst->dwDragEff = DROPEFFECT_NONE;
			else
				pdDst->dwDragEff = DROPEFFECT_NONE;
			}	// if

		}	// else if

	// Drag
	else if (_RCP(Drag))
		{
//		dbgprintf ( L"DragDrop::receive:DragDrop\r\n" );

		// State check
		CCLTRYE ( (pdSrc == NULL) || (pdSrc->bDrag == FALSE), ERROR_INVALID_STATE );
		if (hr != S_OK)
			dbgprintf ( L"DragDrop::DragDrop:Double drag request detected\r\n" );

		// Drag and drop is performed in helper object because the system
		// call to perform the drag blocks until complete.

		// Previous object
		_RELEASE(pdSrc);

		// New object
		CCLTRYE	( (pdSrc = new DragDropSrc ( this, v )) != NULL, E_OUTOFMEMORY );
		CCLOK		( pdSrc->AddRef(); )
		CCLTRY	( pdSrc->construct(); )
		}	// if

	// State
	else if (_RCP(Owner))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pOwn);
		CCLTRY(_QISAFE(unkV,IID_IDictionary,&pOwn));
		}	// if

	return hr;
	}	// receive

HRESULT DragDrop :: start ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Start drag and drop operations.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;
	IDropTarget	*pdt;

	// State check
	CCLTRYE ( (pOwn != NULL), ERROR_INVALID_STATE );
	CCLTRYE ( (hWnd  != NULL), ERROR_INVALID_STATE );
	CCLTRYE ( (pdDst == NULL), S_FALSE );

	// Create an event for the two destination helpers to synchronize
	CCLTRYE	( (hevDst = CreateEvent ( NULL, FALSE, FALSE, NULL )) != NULL,
					GetLastError() );

	// Graph environment interaction helper object
	CCLTRYE	( (pdDstA = new DragDropDstA ( this )) != NULL, E_OUTOFMEMORY );
	CCLOK		( pdDstA->AddRef(); )
	CCLTRY	( pdDstA->start() );

	// Win32 interaction helper object
	CCLTRYE	( (pdDst = new DragDropDst ( this )) != NULL, E_OUTOFMEMORY );
	CCLOK		( pdDst->AddRef(); )
	CCLTRY	( pdDst->accept() );

	// Store interface to drop target in instance
	CCLTRY	( pOwn->store ( strRefDragDrop, adtIUnknown ( (pdt = pdDst) ) ) );

	return hr;
	}	// start

HRESULT DragDrop :: stop ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Stop drag and drop operations.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;

	// Win32 interaction helper object
	if (pdDst != NULL) pdDst->deny();
	_RELEASE(pdDst);

	// Graph environment interaction helper object
	if (pdDstA != NULL) pdDstA->stop();
	_RELEASE(pdDstA);

	// Remove target and deny drops
	if (pOwn != NULL) pOwn->remove ( strRefDragDrop );

	// Clean up
	if (hevDst != NULL)
		{
		hevDst = NULL;
		CloseHandle ( hevDst );
		}	// if

	return hr;
	}	// stop

/////////////////////////////////////////////////////////////////////////////////
//
// DragDropSrc
//	-	Helper object to 'source' a drag and drop operation.  The Win32 call to
//		initiate an OLE drag and drop operation blocks.  This object will initiate
//		the operation in an apartment thread.
//
/////////////////////////////////////////////////////////////////////////////////

DragDropSrc :: DragDropSrc ( DragDrop *_pParent, const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	//	PARAMETERS
	//		-	_pParent is the parent object
	//		-	v is the value to drag and drop
	//
	////////////////////////////////////////////////////////////////////////
	pThrd		= NULL;
	bDrag		= FALSE;
	pParent	= _pParent;
	adtValue::copy ( v, vDrag );
	}	// DragDropSrc

HRESULT DragDropSrc :: construct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called to construct the object.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr				= S_OK;
//	IDictionary		*pCtxDrag	= NULL;
	HGLOBAL			hGlb			= NULL;
	IByteStream		*pStm			= NULL;
	IStreamPersist	*pParse		= NULL;
	void				*pvGlb		= NULL;
	U64				sz;
	adtIUnknown		unkV(vDrag);
	adtString		sFilename;
//	adtValue			vValue;

	// Allow different types of data to be sourced.  The default
	// is 'nSpace value' unless another type is specified.
//	CCLTRY ( _QISAFE(unkV,IID_IDictionary,&pCtxDrag) );
//	CCLTRY ( pCtxDrag->load ( adtString(L"Value"), vValue ) );

	// Create a memory stream to receive the data
	CCLTRY	(COCREATE(L"Io.StmMemory",IID_IByteStream,&pStm));

	// Is a type specified ?
//	if (	hr == S_OK &&
//			pCtxDrag->load ( adtString(L"TypeValue"), sDragType ) != S_OK)
		sDragType = L"nSpace Value";

	// If nSpace value, stream value to memory directly
	if (!WCASECMP(sDragType,L"nSpace Value"))
		{
		// Stream value using the binary parser
		CCLTRY	(COCREATE(L"Io.StmPrsBin",IID_IStreamPersist,&pParse));
		CCLTRY	(pParse->save ( pStm, vDrag ));
		}	// if
/*
	// 'File Contents' is a stream of data that can be associated with a 'file'
	else if (!WCASECMP(sDragType,L"File Contents"))
		{
		// Windows supports a 'FILEGROUPDESCRIPTOR for 'virtual' files but almost
		// all other programs cannot support it.  Convert file contents to
		// a 'CF_HDROP' to maximize compatbility.  Write file contents to a
		// local temporary file.
		IResource		*pFile	= NULL;
		IDictionary		*pOpts	= NULL;
		IByteStream		*pStmSrc	= NULL;
		IByteStream		*pStmDst	= NULL;
		DROPFILES		df;
		adtIUnknown		unkVV(vValue);
		WCHAR				wTempPath[MAX_PATH];
		DWORD				len;

		// Specified filename
		if (pCtxDrag->load ( adtString ( L"Filename" ), sFilename ) != S_OK)
			sFilename = L"File.Dat";

		// Obtain path to temporary location
		CCLTRYE ( (len = GetTempPath ( sizeof(wTempPath)/sizeof(wTempPath[0]), 
						wTempPath )) > 0, GetLastError() );

		// Generate a location to a temporary file.  The filename has to
		// match the specified source so its the same after the drag operation.
		CCLOK		( sTmpLoc = wTempPath; )
		CCLTRY	( sTmpLoc.append ( sFilename ) );
		CCLOK		( dbgprintf ( L"DragDropSrc::construct:From:%s\r\n", (LPCWSTR)sTmpLoc ); )


		// Create an set options for file
		CCLTRY	( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pOpts ) );
		CCLTRY	( pOpts->store ( adtString(L"Location"),	sTmpLoc ) );
		CCLTRY	( pOpts->store ( adtString(L"ReadOnly"),	adtBool(false) ) );
		CCLTRY	( pOpts->store ( adtString(L"Create"),		adtBool(true) ) );
		CCLTRY	( pOpts->store ( adtString(L"Truncate"),	adtBool(true) ) );

		// Create a file resource and open
		CCLTRY	( COCREATE ( L"Io.SysStmFile", IID_IResource, &pFile ) );
		CCLTRY	( pFile->open ( pOpts ) );

		// Copy source byte stream to destination byte stream
		CCLTRY	( _QI(pFile,IID_IByteStream,&pStmDst) );
		CCLTRY	( _QISAFE(unkVV,IID_IByteStream,&pStmSrc) );
		CCLTRY	( pStmSrc->copyTo ( pStmDst, 0, NULL ) );

		// Done with temp. file
		if (pFile != NULL) pFile->close();

		// For CF_HDROP, the format is a global memory object containing a DROPFILES structure
		// Write structure and path to temp. file to memory stream
		if (hr == S_OK)
			{
			// DROPFILES
			memset ( &df, 0, sizeof(df) );
			df.pFiles	= sizeof(df);					// Right after structure
			df.fWide		= TRUE;							// Unicode
			CCLTRY(pStm->write ( &df, sizeof(df), NULL ));
			}	// if
		if (hr == S_OK)
			{
			WCHAR	n = WCHAR('\0');

			// Write unicode path
			CCLTRY(pStm->write ( (LPCWSTR) sTmpLoc, sTmpLoc.length()*sizeof(WCHAR), NULL ));

			// Structure requires double termination
			CCLTRY(pStm->write ( &n, sizeof(n), NULL ));
			CCLTRY(pStm->write ( &n, sizeof(n), NULL ));
			}	// if

		// Clean up
		_RELEASE(pStmSrc);
		_RELEASE(pStmDst);
		_RELEASE(pFile);
		_RELEASE(pOpts);
		}	// else if
*/
	// Unhanlded type
	else hr = E_NOTIMPL;

	// Obtain size of streamed data
	CCLTRY	(pStm->available ( &sz ) );
	CCLTRYE	( sz > 0, E_UNEXPECTED );

	// Allocate a global memory object to use as the storage medium
	CCLTRYE	( (hGlb = GlobalAlloc ( GHND, (SIZE_T)sz )) != NULL, E_OUTOFMEMORY );
	CCLTRYE	( (pvGlb = GlobalLock ( hGlb )) != NULL, GetLastError() );
	CCLTRY	( pStm->read ( pvGlb, sz, NULL ) );

	// Initialize storage medium
	if (hr == S_OK)
		{
		memset ( &stgDrag, 0, sizeof(stgDrag) );
		stgDrag.tymed		= TYMED_HGLOBAL;
		stgDrag.hGlobal	= hGlb;
		}	// if

	// Clean up
	if (pvGlb != NULL) GlobalUnlock ( hGlb );
	_RELEASE(pParse);
	_RELEASE(pStm);
//	_RELEASE(pCtxDrag);

	// Create thread for drag and drop
	CCLTRY(COCREATE(L"Sys.Thread",IID_IThread,&pThrd));
	CCLTRY(pThrd->threadStart(this,1000));

	return hr;
	}	// construct

void DragDropSrc :: destruct ( void )
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

	// Shutdown worker thread
	if (pThrd != NULL)
		{
		// Shutdown thread
		pThrd->threadStop(5000);
		pThrd->Release();
		pThrd = NULL;
		}	// if

	// Clean up temp. file is used
	if (sTmpLoc.length())
		{
		dbgprintf ( L"DragDropSrc::destruct:Deleting temporary file:%s\r\n",
						(LPCWSTR)sTmpLoc );
		DeleteFile ( sTmpLoc );
		}	// if

	}	// destruct

HRESULT DragDropSrc :: EnumFormatEtc ( DWORD dwDir, IEnumFORMATETC **ppE )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IDataObject
	//
	//	PURPOSE
	//		-	Creates an object for enumerating formats.
	//
	//	PARAMETERS
	//		-	dwDir is the direction of the eventual request
	//		-	ppE will receive the enumerator
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// New object
//	dbgprintf ( L"DragDropSrc::EnumFormatEtc:%s\r\n", (LPCWSTR)sDragType );
	CCLTRYE	( ((*ppE) = new DragDropEnum(sDragType)) != NULL, E_OUTOFMEMORY );

	return hr;
	}	// EnumFormatEtc

HRESULT DragDropSrc :: GetData ( FORMATETC *pFmt, STGMEDIUM *pStg )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IDataObject
	//
	//	PURPOSE
	//		-	Called to obtain data from the source data object.
	//
	//	PARAMETERS
	//		-	pFmt is the format to use
	//		-	pStg receives the data
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	HGLOBAL		hGlb		= NULL;
	void			*pvSrc	= NULL;
	void			*pvDst	= NULL;
	adtString	sType;

	// Copy the streamed 'nSpace Value'
//	dbgprintf ( L"DragDropSrc::GetData:cfFormat 0x%x (0x%x,0x%x) tymed 0x%x ptd %p dwAspect %d lindex %d\r\n",
//						pFmt->cfFormat, RegisterClipboardFormat ( L"nSpace Value" ),
//						RegisterClipboardFormat ( CFSTR_FILECONTENTS ), pFmt->tymed, pFmt->ptd, pFmt->dwAspect, pFmt->lindex );
	if (	(	pFmt->cfFormat == RegisterClipboardFormat ( L"nSpace Value" ) ||
				pFmt->cfFormat == CF_HDROP ) &&
			(	pFmt->tymed & TYMED_HGLOBAL ) )
		{
		SIZE_T	sz;

		// Callers get a copy of data value
		CCLTRYE ( (stgDrag.hGlobal != NULL), E_UNEXPECTED );
		CCLOK   ( memset ( pStg, 0, sizeof(STGMEDIUM) ); )

		// Copy memory
		CCLTRYE	( (sz = GlobalSize ( stgDrag.hGlobal )) != 0, GetLastError() );
		CCLTRYE	( (hGlb = GlobalAlloc ( GHND, sz )) != NULL, GetLastError() );
		CCLTRYE	( (pvDst = GlobalLock ( hGlb )) != NULL, GetLastError() );
		CCLTRYE	( (pvSrc = GlobalLock ( stgDrag.hGlobal )) != NULL, GetLastError() );
		CCLOK		( memcpy ( pvDst, pvSrc, sz ); )

		// Fill storage structure
		if (hr == S_OK)
			{
			pStg->tymed		= TYMED_HGLOBAL;
			pStg->hGlobal	= hGlb;
			}	// if
		}	// if

	// Asking for drop effect during CF_HDROP ?
	else if (	(	pFmt->cfFormat == RegisterClipboardFormat ( CFSTR_PREFERREDDROPEFFECT ) ) &&
					(	pFmt->tymed & TYMED_HGLOBAL ) )
		{
		// File contents ?  Copy from temporary location
		if (!WCASECMP ( sDragType, L"File Contents" ))
			{
			// For drop effect, storage is an HGLOBAL to a DWORD
			CCLTRYE	( (hGlb = GlobalAlloc ( GHND, sizeof(DWORD) )) != NULL, GetLastError() );
			CCLTRYE	( (pvDst = GlobalLock ( hGlb )) != NULL, GetLastError() );
			CCLOK		( *((U32 *)pvDst) = DROPEFFECT_COPY; )
			CCLOK		( pStg->tymed		= TYMED_HGLOBAL; )
			CCLOK		( pStg->hGlobal	= hGlb; )
			}	// if
		else
			hr = DV_E_FORMATETC;
		}	// else if

	else
		hr = DV_E_FORMATETC;


	// Clean up
	if (pvSrc != NULL) GlobalUnlock ( stgDrag.hGlobal );
	if (pvDst != NULL) GlobalUnlock ( hGlb );
	if (hr != S_OK && hGlb != NULL)
		GlobalFree ( hGlb );
//	dbgprintf ( L"DragDropSrc::GetData:0x%x (0x%x)\r\n", hGlb, hr );

	return hr;
	}	// GetData

HRESULT DragDropSrc :: GetDataHere ( FORMATETC *pFmt, STGMEDIUM *pStg )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IDataObject
	//
	//	PURPOSE
	//		-	Called to obtain data from the source data object with
	//			pre-allocated storage.
	//
	//	PARAMETERS
	//		-	pFmt is the format to use
	//		-	pStg receives the data
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	dbgprintf ( L"DragDropSrc::GetDataHere\r\n" );
	return E_NOTIMPL;
	}	// GetDataHere

HRESULT DragDropSrc :: QueryGetData ( FORMATETC *pFmt )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IDataObject
	//
	//	PURPOSE
	//		-	Determines if object can 'render' the data according to the
	//			specified format.
	//
	//	PARAMETERS
	//		-	pFmt is the format to use
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;
	adtString	sType;

	// Supported formats
//	dbgprintf ( L"DragDropSrc::QueryGetData:cfFormat 0x%x (0x%x,0x%x) tymed 0x%x\r\n",
//						pFmt->cfFormat, RegisterClipboardFormat ( L"nSpace Value" ),
//						RegisterClipboardFormat ( CFSTR_FILECONTENTS ), pFmt->tymed );

	// Default is unsupported
	hr = DV_E_FORMATETC;

	// nSpace value to HGLOBAL
	hr = (	(	pFmt->cfFormat == RegisterClipboardFormat ( L"nSpace Value" ) ||
					pFmt->cfFormat == CF_HDROP ) &&
				(	pFmt->tymed & TYMED_HGLOBAL ) ) ? S_OK : DV_E_FORMATETC;

//	dbgprintf ( L"DragDropSrc::QueryGetData:hr 0x%x\r\n", hr );

	return hr;
	}	// QueryGetData

HRESULT DragDropSrc :: GiveFeedback ( DWORD dwEff )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IDropSource
	//
	//	PURPOSE
	//		-	Gives visual feedback to user during drag and drop.
	//
	//	PARAMETERS
	//		-	dwEff is the drop effect
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	// Use the default OLE feedback
//	dbgprintf ( L"DragDropSrc::GiveFeedback\r\n" );
	return DRAGDROP_S_USEDEFAULTCURSORS;
	}	// GiveFeedback

HRESULT DragDropSrc :: QueryContinueDrag ( BOOL bEscape, DWORD dwState )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IDropSource
	//
	//	PURPOSE
	//		-	Determines whether a drag-and-drop operation should be
	//			continued.
	//
	//	PARAMETERS
	//		-	bEscape is TRUE if escape key has been pressed.
	//		-	dwState is the keyboard modifier state
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Finish, cancel, continue drag based on flags
//	dbgprintf ( L"DragDropSrc::QueryContinueDrag:(0x%x),(0x%x)\r\n", bEscape, dwState );
	hr =	(bEscape == TRUE)													? DRAGDROP_S_CANCEL :
			((dwState & (MK_LBUTTON|MK_MBUTTON|MK_RBUTTON)) == 0) ? DRAGDROP_S_DROP
																					: S_OK;

	return hr;
	}	// QueryContinueDrag

HRESULT DragDropSrc :: tickAbort ( void )
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
	// Thread terminates automatically
	return S_OK;
	}	// tickAbort

HRESULT DragDropSrc :: tick ( void )
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
	HRESULT	hr		= S_OK;
	DWORD		dwEff;

	// Peform operation
	dbgprintf ( L"DragDropSrc::tick:DoDragDrop {\r\n" );
	hr		= DoDragDrop ( this, this,	DROPEFFECT_NONE|DROPEFFECT_COPY|
												DROPEFFECT_MOVE|DROPEFFECT_LINK, &dwEff );
	dbgprintf ( L"} DragDropSrc::tick:DoDragDrop, 0x%x\r\n", hr );

	return S_FALSE;
	}	// tick

HRESULT DragDropSrc :: tickBegin ( void )
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
	HRESULT		hr			= S_OK;
	IOleCache	*pCache	= NULL;

	// Drag/Drop not supported under CE  (No 'AttachThreadInput') ?
	// Check this if it becomes necessary.
//	dbgprintf ( L"DragDropSrc::tickBegin {\r\n" );
	#if	!defined(UNDER_CE)

	// Dragging
	CCLOK ( bDrag = TRUE; )

	// Store thread ID for later notification
	CCLOK ( dwThrdId = GetCurrentThreadId(); )

	// Initialize OLE, this is required for Shell drag and drop operations.
	CCLTRY( OleInitialize(NULL) );

	// This is a hacky requirement of Win32.  For DragDrop to work properly
	// in another thread, the input queues from the instance thread has
	// to be attached to this thread.  (Not supported under CE).

	// Thread Id of instance window
	CCLTRYE ( (dwThrdInstId = GetWindowThreadProcessId ( pParent->hWnd, NULL ))
					!= 0, GetLastError() );

	// Attach input queues
	CCLTRYE ( AttachThreadInput ( dwThrdInstId, dwThrdId, TRUE ), GetLastError() );

	// Attached ?
	bAttached = (hr == S_OK);

	#else
	hr = E_NOTIMPL;
	#endif
//	dbgprintf ( L"} DragDropSrc::tickBegin 0x%x\r\n", hr );

	return hr;
	}	// tickBegin

HRESULT DragDropSrc :: tickEnd ( void )
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
//	dbgprintf ( L"DragDropSrc::tickEnd {\r\n" );

	// Detach input queues
	#if	!defined(UNDER_CE)
	if (bAttached == TRUE)
		AttachThreadInput ( dwThrdInstId, dwThrdId, FALSE );
	#endif

	// Clean up
	dwThrdId = 0;
	OleUninitialize();

	// Not dragging
	bDrag = FALSE;

//	dbgprintf ( L"} DragDropSrc::tickEnd\r\n" );
	return S_OK;
	}	// tickEnd

//
// Unsupported/unimplemented methods
//

HRESULT DragDropSrc :: DAdvise ( FORMATETC *, DWORD, IAdviseSink *, DWORD * )
	{
	dbgprintf ( L"DragDropSrc::DAdvise\r\n" );
	return OLE_E_ADVISENOTSUPPORTED;
	}	// DAdvise

HRESULT DragDropSrc :: DUnadvise ( DWORD )
	{
	dbgprintf ( L"DragDropSrc::DUnadvise\r\n" );
	return OLE_E_ADVISENOTSUPPORTED;
	}	// DUnadvise

HRESULT DragDropSrc :: EnumDAdvise ( IEnumSTATDATA ** )
	{
	dbgprintf ( L"DragDropSrc::EnumDAdvise\r\n" );
	return OLE_E_ADVISENOTSUPPORTED;
	}	// EnumDAdvise

HRESULT DragDropSrc :: GetCanonicalFormatEtc ( FORMATETC *pIn,
																	FORMATETC *pOut )
	{
	dbgprintf ( L"DragDropSrc::GetCanonicalFormatEtc\r\n" );
	return DATA_S_SAMEFORMATETC;
	}	// GetCanonicalFormatEtc

HRESULT DragDropSrc :: SetData ( FORMATETC *f, STGMEDIUM *s, BOOL b )
	{
//	dbgprintf ( L"DragDropSrc::SetData\r\n" );
	return E_NOTIMPL;
	}	// SetData

/////////////////////////////////////////////////////////////////////////////////
//
// DragDropDst
//	-	Destination drop target helper object.  This goal of the DragDrop node
//		in general is to 'externalize' system drag and drop activity.  Each
//		appropriate child in this UI node library is 'drag and drop aware'.
//		If a drop target is present in the running instance each child will
//		register/revoke itself as a drop target so that drag logic can be
//		centralized and communicated through a single interface to the graph.
//
/////////////////////////////////////////////////////////////////////////////////

DragDropDst :: DragDropDst ( DragDrop *_pParent )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	//	PARAMETERS
	//		-	_pParent is the parent object
	//
	////////////////////////////////////////////////////////////////////////
	pParent	= _pParent;
	pThrd		= NULL;
	dwThrdId	= NULL;
	}	// DragDropDst

HRESULT DragDropDst :: accept ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Accept drag and drop targets.
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// OLE interaction must be done through an apartment thread with a
	// message queue.
	CCLTRY(COCREATE(L"Sys.Thread",IID_IThread,&pThrd));
	CCLTRY(pThrd->threadStart(this,1000));

	return hr;
	}	// accept

HRESULT DragDropDst :: deny ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Deny drag and drop processing.
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Shutdown worker thread
	if (pThrd != NULL)
		{
		// Shutdown thread
		pThrd->threadStop(5000);
		pThrd->Release();
		pThrd = NULL;
		}	// if

	return hr;
	}	// deny

HRESULT DragDropDst :: DragEnter ( IDataObject *pObj, DWORD dwState,
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
	DWORD				dwThrdIdDst,nf;

	// Debug
//	dbgprintf ( L"DragDropDst::DragEnter:%p,0x%x,(%d,%d)\r\n",
//					pObj, dwState, pt.x, pt.y );

	// State check
	CCLTRYE ( pObj					!= NULL, E_INVALIDARG );
	CCLTRYE ( pParent->hWnd		!= NULL, E_UNEXPECTED );
	CCLTRYE ( pParent->pdDstA	!= NULL, E_UNEXPECTED );
	CCLOK	  ( dwThrdIdDst		= pParent->pdDstA->dwThrdId; )
	CCLTRYE ( dwThrdIdDst		!= 0,		E_UNEXPECTED );

	// Defaults
	(*pdwEff)	= DROPEFFECT_NONE;
	dwDragState	= dwState;
	dwDragEff	= (*pdwEff);

	// Coordinates are always in relation to the instance window
	if (hr == S_OK)
		{
		ptDrag.x = pt.x; ptDrag.y = pt.y;
		ScreenToClient ( pParent->hWnd, &ptDrag );
		}	// if

	// This apartment thread is the only one allowed to access the
	// data object.  Enumerate formats on behalf of the node to let
	// the graph decide whether or not accept the incoming type.
	CCLOK  ( bAcceptf = false; )
	CCLTRY ( pObj->EnumFormatEtc ( DATADIR_GET, &pEnum ) );
	while (	hr == S_OK && bAcceptf == false && 
				pEnum->Next ( 1, &fmtDrag, &nf ) == S_OK && nf == 1)
		{
		// Obtain data information in case data type is accepted.
		CCLOK  ( memset ( &stgDrag, 0, sizeof(stgDrag) ); )
		CCLTRY ( pObj->GetData ( &fmtDrag, &stgDrag ) );

		// Ensure data is in a format that is accessible by another thread.
		if (hr == S_OK)
			{
			switch (stgDrag.tymed)
				{
				// Global memory handle, accessible from any thread, no processing required
				case TYMED_HGLOBAL :
					break;

				// IStream.  Interface on a apartment thread.  Convert format
				case TYMED_ISTREAM :
					hr = S_FALSE;
					break;

				// Unsupported format at this type
				default :
					hr = S_FALSE;
				}	// switch
			}	// if

		// Query destination helper thread to see if it wants to deal with this data type.
//		dbgprintf ( L"DragDropDst::DragEnter:Format 0x%x, ptd %p Aspect 0x%x, lindex %d, tymed 0x%x, hr 0x%x\r\n",
//						fmtDrag.cfFormat, fmtDrag.ptd, fmtDrag.dwAspect, fmtDrag.lindex, fmtDrag.tymed, hr );
		CCLTRYE ( PostThreadMessage ( dwThrdIdDst, WIDGET_USER_DRAGENTER, 0, 0l ),
						GetLastError() );
		CCLTRYE ( WaitForSingleObject ( pParent->hevDst, 10000 ) == WAIT_OBJECT_0,
						ERROR_TIMEOUT );

		// Clean up
		ReleaseStgMedium ( &stgDrag );

		// Allow 'soft' failures
		if (hr == S_FALSE) hr = S_OK;
		}	// while

	// Update drag effect
	if (hr == S_OK)	(*pdwEff) = dwDragEff;
	else					(*pdwEff) = DROPEFFECT_NONE;

	// Clean up
	_RELEASE(pEnum);

	return hr;
	}	// DragEnter

HRESULT DragDropDst :: DragLeave ( void )
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
	DWORD		dwThrdIdDst;

	// State check
	CCLTRYE ( pParent->hWnd		!= NULL, E_UNEXPECTED );
	CCLTRYE ( pParent->pdDstA	!= NULL, E_UNEXPECTED );
	CCLOK	  ( dwThrdIdDst		= pParent->pdDstA->dwThrdId; )
	CCLTRYE ( dwThrdIdDst		!= 0,		E_UNEXPECTED );

	// Notify parent and wait for processing
	dbgprintf ( L"DragDropDst::DragLeave\r\n" );
	CCLTRYE ( PostThreadMessage ( dwThrdIdDst, WIDGET_USER_DRAGLEAVE, 0, 0l ),
					GetLastError() );
	CCLTRYE ( WaitForSingleObject ( pParent->hevDst, 10000 ) == WAIT_OBJECT_0,
					ERROR_TIMEOUT );

	return hr;
	}	// DragLeave

HRESULT DragDropDst :: DragOver ( DWORD dwState, POINTL pt, DWORD *pdwEff )
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
	DWORD		dwThrdIdDst;

	// State check
	CCLTRYE ( pParent->hWnd		!= NULL, E_UNEXPECTED );
	CCLTRYE ( pParent->pdDstA	!= NULL, E_UNEXPECTED );
	CCLOK	  ( dwThrdIdDst		= pParent->pdDstA->dwThrdId; )
	CCLTRYE ( dwThrdIdDst		!= 0,		E_UNEXPECTED );

	// Coordinates are always in relation to the instance window
//	dbgprintf ( L"DragDropDst::DragOver:%d,%d\r\n", pt.x, pt.y );
	if (hr == S_OK)
		{
		ptDrag.x = pt.x; ptDrag.y = pt.y;
		ScreenToClient ( pParent->hWnd, &ptDrag );
		}	// if

	// Notify parent and wait for processing
	CCLTRYE ( PostThreadMessage ( dwThrdIdDst, WIDGET_USER_DRAGOVER, 0, 0l ),
					GetLastError() );
	CCLTRYE ( WaitForSingleObject ( pParent->hevDst, 10000 ) == WAIT_OBJECT_0,
					ERROR_TIMEOUT );

	// Result
	if (bAcceptf)
		(*pdwEff) = dwDragEff;
	else
		(*pdwEff) = DROPEFFECT_NONE;

	return hr;
	}	// DragOver

HRESULT DragDropDst :: Drop ( IDataObject *pObj, DWORD dwState,
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
	DWORD		dwThrdIdDst;

	// State check
	CCLTRYE ( pParent->hWnd		!= NULL, E_UNEXPECTED );
	CCLTRYE ( pParent->pdDstA	!= NULL, E_UNEXPECTED );
	CCLOK	  ( dwThrdIdDst		= pParent->pdDstA->dwThrdId; )
	CCLTRYE ( dwThrdIdDst		!= 0,		E_UNEXPECTED );

	dbgprintf ( L"DragDropDst::Drop:%p,0x%x,(%d,%d)\r\n",
					pObj, dwState, pt.x, pt.y );

	// Coordinates are always in relation to the instance window
	if (hr == S_OK)
		{
		ptDrag.x = pt.x; ptDrag.y = pt.y;
		ScreenToClient ( pParent->hWnd, &ptDrag );
		}	// if

	// Notify parent and wait for processing
	CCLTRYE ( PostThreadMessage ( dwThrdIdDst, WIDGET_USER_DRAGDROP, 0, 0l ),
					GetLastError() );
	CCLTRYE ( WaitForSingleObject ( pParent->hevDst, 10000 ) == WAIT_OBJECT_0,
					ERROR_TIMEOUT );

	return hr;
	}	// Drop

HRESULT DragDropDst :: tickAbort ( void )
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

HRESULT DragDropDst :: tick ( void )
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
	HRESULT	hr		= S_OK;
	MSG		msg;

	// Message loop
	while (GetMessage ( &msg, (HWND) NULL, 0, 0 ) != 0)
		{
		TranslateMessage(&msg);
		DispatchMessage(&msg);
		}	// while

	return S_FALSE;
	}	// tick

HRESULT DragDropDst :: tickBegin ( void )
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

	// Register main instance window has a drop target
	CCLOK ( RegisterDragDrop ( pParent->hWnd, this ); )

	// Debug
	if (hr != S_OK)
		dbgprintf ( L"DragDropDst::tickBegin:Error 0x%x\r\n", hr );
//	dbgprintf ( L"DragDropDst::tickBegin:0x%x\r\n", hr );

	return hr;
	}	// tickBegin

HRESULT DragDropDst :: tickEnd ( void )
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

	// Parent window no longer drag and drop
	if (pParent->hWnd != NULL)
		RevokeDragDrop ( pParent->hWnd );

	// Clean up
	dwThrdId = 0;
	OleUninitialize();
//	dbgprintf ( L"DragDropDst::tickEnd\r\n" );

	return S_OK;
	}	// tickEnd

/////////////////////////////////////////////////////////////////////////////////
//
// DragDropDstA
//	-	Asynchronous destination drop target helper object.
//		This node communicates with the outside graph on behalf of the apartment
//		threaded drop target object.  This is necessary so that emissions into
//		the graph are inside free threaded COM context.
//
/////////////////////////////////////////////////////////////////////////////////

DragDropDstA :: DragDropDstA ( DragDrop *_pParent )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	//	PARAMETERS
	//		-	_pParent is the parent object
	//
	////////////////////////////////////////////////////////////////////////
	pParent		= _pParent;
	pThrd			= NULL;
	dwThrdId		= NULL;
	pCtxNotify	= NULL;
	}	// DragDropDstA

HRESULT DragDropDstA :: dragEnter ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Process the 'DragEnter' event.
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	DragDropDst	*pHelper = (pParent->pdDst);
	adtString	sType;
	double		x,y;
	RECT			cr;

	// Debug
//	dbgprintf ( L"DragDropDstA::dragEnter\r\n" );

	// State check
	CCLTRYE ( pHelper != NULL, ERROR_INVALID_STATE );

	// Notify outside world of format that is trying to be dragged
	// into the context.  Attempt OS-independence by mapping known
	// format strings.  If the outside graph wants to accept the
	// format, it will signal acceptance via node receptor.

	// Size of parent window
	CCLOK ( GetClientRect ( pParent->hWnd, &cr ); )

	// Calculate normalized coordinates, Y inverted in GDI
	CCLOK ( x = (pParent->pdDst->ptDrag.x-(cr.right/2))/((double)cr.right); )
	CCLOK ( y = ((cr.bottom-pParent->pdDst->ptDrag.y)-(cr.bottom/2))/((double)cr.bottom); )

	// Store coordinates for query
	CCLTRY ( pCtxNotify->store ( strRefX, adtDouble(x) ) );
	CCLTRY ( pCtxNotify->store ( strRefY, adtDouble(y) ) );

	// Predefined non-string types
	if (hr == S_OK && pHelper->fmtDrag.cfFormat == CF_HDROP)
		{
		// Locations of a group of existing files
		sType = L"Files";
		}	// if

	// Predefined string types
	else if (hr == S_OK && pHelper->fmtDrag.cfFormat >= 0xc000 && pHelper->fmtDrag.cfFormat <= 0xffff)
		{
		// Attempt conversion to string registered by another application
		CCLTRY ( sType.allocate ( 255 ) );
		CCLTRYE ( GetClipboardFormatName ( pHelper->fmtDrag.cfFormat, &sType.at(), 255 ) > 0, GetLastError() );

		// Try and filter OS dependent types 'leaking' into the environment.  OS dependent types will
		// still leak through.  Graph will have to perform some kind of mapping of all types.
		if (hr == S_OK)
			{
			// Files ?
			if (!WCASECMP(sType,L"FileName"))
				{
				// Always in a UNICODE environment, ignore this format and wait for 'FileNameW'
				hr = S_FALSE;
				}	// if
			else if (!WCASECMP(sType,L"FileNameW"))
				{
				// Filename
				sType = L"FileName";
				}	// else if
			}	// if
		}	// if

	// Unknown
	else
		hr = S_FALSE;

	// Drag type
//	dbgprintf ( L"DragDropDstA::dragEnter:Type %s\r\n", sType.pwstr );
	CCLTRY	( pCtxNotify->store ( strRefType, sType ) );

	// Package data in an environment friendly format.
	if (hr == S_OK)
		{
		// Process based on type.  This will have to be expanded to support new data types
		// to keep OS specific formats from entering the environment.

		// File list ?
		if (!WCASECMP(sType,L"Files"))
			{
			IList			*pFiles	= NULL;
			IIt			*pIt		= NULL;
			DROPFILES	*pdf		= NULL;
			WCHAR			*pwf		= NULL;

			// Memory for a file drop is a DROPFILES structure
			// Extract and format locations.

			// Access DROPFILES memory
			CCLTRYE ( (pdf = (DROPFILES *) GlobalLock ( pHelper->stgDrag.hGlobal ))
							!= NULL, GetLastError() );

			// Create a list to receive filenames
			CCLTRY ( COCREATE ( L"Adt.List", IID_IList, &pFiles ) );
			CCLTRY ( pFiles->iterate ( &pIt ) );
			CCLTRY ( pCtxNotify->store ( adtString(L"Value"), adtIUnknown(pFiles) ) );

			// 'pFiles' contains the offset from the beginning of 'DROPFILES'
			// of the double null terminated file list.
			CCLTRYE ( (pdf->pFiles != 0), E_UNEXPECTED );
			CCLOK   ( pwf = (WCHAR *) ((U8 *)(pdf) + pdf->pFiles); )
			while (hr == S_OK && (*pwf) != WCHAR('\0'))
				{
				// Add file to list
				CCLTRY(pFiles->write(adtString(pwf)));

				// Next file
				pwf += (wcslen(pwf)+1);
				}	// while

			// Clean up
			_RELEASE(pIt);
			_RELEASE(pFiles);
			if (pdf != NULL) GlobalUnlock ( pHelper->stgDrag.hGlobal );
			}	// if

		// nSpace value ?
		else if (!WCASECMP(sType,L"nSpace Value"))
			{
			IByteStream		*pStm		= NULL;
			IStreamPersist	*pParse	= NULL;
			void				*pvGlb	= NULL;
			SIZE_T			sz;
			adtValue			vLoad;

			// Access data
			CCLTRYE	( (pHelper->stgDrag.hGlobal != NULL), E_UNEXPECTED );
			CCLTRYE	( (sz = GlobalSize ( pHelper->stgDrag.hGlobal )) != 0, GetLastError() );
			CCLTRYE	( (pvGlb = GlobalLock ( pHelper->stgDrag.hGlobal )) != NULL, GetLastError() );

			// Copy the data into a stream for loading
			CCLTRY	(COCREATE(L"Io.StmMemory",IID_IByteStream,&pStm));
			CCLTRY	(pStm->write ( pvGlb, (U32)sz, NULL ));
			CCLTRY	(pStm->seek ( 0, STREAM_SEEK_SET, NULL ));

			// Stream value using the binary parser
			CCLTRY	(COCREATE(L"Io.StmPrsBin",IID_IStreamPersist,&pParse));
			CCLTRY	(pParse->load ( pStm, vLoad ) );

			// Value
			CCLTRY ( pCtxNotify->store ( adtString(L"Value"), vLoad ) );

			// Clean up
			_RELEASE(pParse);
			_RELEASE(pStm);
			if (pvGlb != NULL) GlobalUnlock ( pHelper->stgDrag.hGlobal );
			}	// else if

		// Unhanlded type
		else hr = E_NOTIMPL;
		}	// if

	// Notify, environment will analyze type
	CCLOK		( pParent->peOnEnter->receive ( NULL, L"Value", adtIUnknown(pCtxNotify) ); )

	return hr;
	}	// dragEnter

HRESULT DragDropDstA :: start ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Start processing messages.
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Create worker thread for asynchronous emissions.
	CCLTRY(COCREATE(L"Sys.Thread",IID_IThread,&pThrd));
	CCLTRY(pThrd->threadStart(this,1000));

	return hr;
	}	// start

HRESULT DragDropDstA :: stop ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Stop processing messages.
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Shutdown worker thread
	if (pThrd != NULL)
		{
		// Shutdown thread
		pThrd->threadStop(5000);
		pThrd->Release();
		pThrd = NULL;
		}	// if

	return hr;
	}	// stop

HRESULT DragDropDstA :: tickAbort ( void )
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

HRESULT DragDropDstA :: tick ( void )
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
	HRESULT	hr		= S_OK;
	MSG		msg;

	// Message loop
	while (GetMessage ( &msg, (HWND) NULL, 0, 0 ) != 0)
		{
		// Process custom messages
		switch (msg.message)
			{
			// Sent for every format during a drag enter event
			case WIDGET_USER_DRAGENTER :
				// Process
				dragEnter();

				// Synchronize
				SetEvent ( pParent->hevDst );
				break;

			// Sent as the drag source is moved around
			case WIDGET_USER_DRAGOVER :
				{
				double	x,y;
				RECT		cr;

				// Size of parent window
				GetClientRect ( pParent->hWnd, &cr );

				// Calculate normalized coordinates, Y inverted in GDI
				x = (pParent->pdDst->ptDrag.x-(cr.right/2))/((double)cr.right);
				y = ((cr.bottom-pParent->pdDst->ptDrag.y)-(cr.bottom/2))/((double)cr.bottom);

				// Update coordinates for query
				pCtxNotify->store ( strRefX, adtDouble(x) );
				pCtxNotify->store ( strRefY, adtDouble(y) );

				// Notify
				pParent->peOnOver->receive ( NULL, L"Value", adtIUnknown(pCtxNotify) );

				// Synchronize
				SetEvent ( pParent->hevDst );
				}
				break;

			// Sent when dragging is complete
			case WIDGET_USER_DRAGLEAVE :
				{
				double	x,y;
				RECT		cr;

				// Size of parent window
				GetClientRect ( pParent->hWnd, &cr );

				// Calculate normalized coordinates, Y inverted in GDI
				x = (pParent->pdDst->ptDrag.x-(cr.right/2))/((double)cr.right);
				y = ((cr.bottom-pParent->pdDst->ptDrag.y)-(cr.bottom/2))/((double)cr.bottom);

				// Update coordinates for query
				pCtxNotify->store ( strRefX, adtDouble(x) );
				pCtxNotify->store ( strRefY, adtDouble(y) );

				// Notify
				pParent->peOnLeave->receive ( NULL, L"Value", adtIUnknown(pCtxNotify) );

				// Synchronize
				SetEvent ( pParent->hevDst );
				}
				break;

			// Sent when an item is dropped
			case WIDGET_USER_DRAGDROP :
				{
				double	x,y;
				RECT		cr;

				// Size of parent window
				GetClientRect ( pParent->hWnd, &cr );

				// Calculate normalized coordinates, Y inverted in GDI
				x = (pParent->pdDst->ptDrag.x-(cr.right/2))/((double)cr.right);
				y = ((cr.bottom-pParent->pdDst->ptDrag.y)-(cr.bottom/2))/((double)cr.bottom);

				// Update coordinates for query
				pCtxNotify->store ( strRefX, adtDouble(x) );
				pCtxNotify->store ( strRefY, adtDouble(y) );

				// Notify
				pParent->peOnDrop->receive ( NULL, L"Value", adtIUnknown(pCtxNotify) );

				// Synchronize
				SetEvent ( pParent->hevDst );
				}
				break;

			default :
				// Make sense for a dedicated thread ?
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}	// switch

		}	// while

	return S_FALSE;
	}	// tick

HRESULT DragDropDstA :: tickBegin ( void )
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

	// This thread uses environment compatible initialization
	CCLTRY ( CoInitializeEx ( NULL, COINIT_MULTITHREADED ) );

	// Context to use for notifications
	CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pCtxNotify ) );

	// Debug
	if (hr != S_OK)
		dbgprintf ( L"DragDropDstA::tickBegin:Error 0x%x\r\n", hr );
//	dbgprintf ( L"DragDropDstA::tickBegin:0x%x\r\n", hr );

	return hr;
	}	// tickBegin

HRESULT DragDropDstA :: tickEnd ( void )
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

	// Clean up
	dwThrdId = 0;
	_RELEASE(pCtxNotify);
	CoUninitialize();

//	dbgprintf ( L"DragDropDstA::tickEnd\r\n" );
	return S_OK;
	}	// tickEnd

/////////////////////////////////////////////////////////////////////////////////
//
// DragDropEnum
//	-	Enumerator for the supported data formats.
//
/////////////////////////////////////////////////////////////////////////////////

DragDropEnum :: DragDropEnum ( const adtString &sType )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	//	PARAMETERS
	//		-	sType specifies the internal drag type
	//
	////////////////////////////////////////////////////////////////////////
	idx = 0;
	adtValue::copy ( sType, sDragType );
	AddRef();
	}	// DragDropEnum

HRESULT DragDropEnum :: Clone ( IEnumFORMATETC **ppE )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IEnumFORMATETC
	//
	//	PURPOSE
	//		-	Creates another enumerator with the same state.
	//
	//	PARAMETERS
	//		-	ppE will receive the new enumerator
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr			= S_OK;
	DragDropEnum	*pClone	= NULL;

	// Clone
//	dbgprintf ( L"DragDropEnum::Clone\r\n" );
	CCLTRYE	( (pClone = new DragDropEnum(sDragType)) != NULL, E_OUTOFMEMORY );
	CCLOK		( pClone->idx = idx; )
	CCLOK		( (*ppE) = pClone; )

	return hr;
	}	// Clone

HRESULT DragDropEnum :: Next ( DWORD nf, FORMATETC *pFmt, DWORD *pnf )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IEnumFORMATETC
	//
	//	PURPOSE
	//		-	Returns the next format in the list
	//
	//	PARAMETERS
	//		-	nf is the number of items to fetch
	//		-	pFmt will receive the format
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr	= S_OK;

	// Setup
//	dbgprintf ( L"DragDropEnum::Next:nf %d pFmt %p pnf %p {\r\n", nf, pFmt, pnf );
	memset  ( pFmt, 0, sizeof(FORMATETC) );
	if (pnf != NULL) (*pnf) = 0;

	// Only supports one format at this time
	if (idx == 0)
		{
		// Fill structure
		pFmt->cfFormat = (!WCASECMP(sDragType,L"File Contents")) ? CF_HDROP :
									RegisterClipboardFormat ( L"nSpace Value" );
		pFmt->dwAspect	= DVASPECT_CONTENT;
		pFmt->lindex	= 0;
		pFmt->tymed		= TYMED_HGLOBAL;
		if (pnf != NULL) (*pnf) = 1;
		++idx;
		}	// if
	else
		hr = S_FALSE;
//	dbgprintf ( L"} DragDropEnum::Next:Format 0x%x, hr 0x%x\r\n", pFmt->cfFormat, hr );

	return hr;
	}	// Next

HRESULT DragDropEnum :: Reset ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IEnumFORMATETC
	//
	//	PURPOSE
	//		-	Reset enumeration state.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
//	dbgprintf ( L"DragDropEnum::Reset\r\n" );
	idx = 0;
	return S_OK;
	}	// Reset

HRESULT DragDropEnum :: Skip ( DWORD ns )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IEnumFORMATETC
	//
	//	PURPOSE
	//		-	Skips over the specified number of elements.
	//
	//	PARAMETERS
	//		-	ns is the number to skip
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
//	dbgprintf ( L"DragDropEnum::Skip\r\n" );
	if (idx == 0 && ns == 1)
		{
		idx = 1;
		return S_OK;
		}	// if
	return S_FALSE;
	}	// Skip
