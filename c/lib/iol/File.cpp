////////////////////////////////////////////////////////////////////////
//
//									FILE.CPP
//
//				Implementation of the file I/O node.
//
////////////////////////////////////////////////////////////////////////

#include "iol_.h"
#include <stdio.h>

// Globals
#define	SIZE_FILE_BFR		8192

File :: File ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	#ifdef		_WIN32
	hFile		= INVALID_HANDLE_VALUE;
	hevWr		= NULL;
	hevRd		= NULL;
	hevStop	= NULL;
	#endif
	pThrd		= NULL;
	pStmIo	= NULL;
	iSzIo		= 0;
	bAsync	= false;
	pcBfr		= NULL;
	iSzBfr	= SIZE_FILE_BFR;
	}	// File

HRESULT File :: construct ( void )
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
	HRESULT			hr			= S_OK;

	// Buffer memory
	CCLTRYE ( (pcBfr = (U8 *) _ALLOCMEM ( iSzBfr )) != NULL, E_OUTOFMEMORY );

	return hr;
	}	// construct

void File :: destruct ( void )
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
	_FREEMEM(pcBfr);
	}	// destruct

HRESULT File :: fileIo  ( BOOL bWr, DWORD uIo, DWORD uTo, DWORD *puIo )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Unified asynchronous I/O for packet buffer.
	//
	//	PARAMETERS
	//		-	bWr is TRUE to write, FALSE to read.
	//		-	uIo is the size of the requested transfer
	//		-	uTo is the timeout in milliseconds
	//		-	puIo will receive the actual amount transferred.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;
	#ifdef		_WIN32
	HANDLE		hevs[2]	= { (bWr) ? hevWr : hevRd, hevStop };
	OVERLAPPED	ov;
	DWORD			dwRet,uXfer;

	// Debug
//	dbgprintf ( L"File::fileIo:bWr %d:uIo %d\r\n", bWr, uIo );

	// I/O is always overlapped since that it the way USB devices are opened.
	memset ( &ov, 0, sizeof(ov) );
	ov.hEvent = hevs[0];
	if (hr == S_OK && bWr)
		{
		// Begin a write
		CCLTRYE ( WriteFile ( hFile, pcBfr, uIo, NULL, &ov ) == TRUE, GetLastError() );
		}	// if
	else if (hr == S_OK)
		{
		// Begin a read
		CCLTRYE ( ReadFile ( hFile, pcBfr, uIo, NULL, &ov ) == TRUE, GetLastError() );
		}	// else if

	// I/O is still pending, must wait
	if (hr == ERROR_IO_PENDING)
		{
		// Wait for completion or signal to stop
		hr		= S_OK;
		dwRet = WaitForMultipleObjects ( 2, hevs, FALSE, uTo );

		// Success ?
		if (dwRet != WAIT_OBJECT_0)
			{
			// Stop event detected
			if (dwRet == WAIT_OBJECT_0+1)
				hr = S_FALSE;

			// Timeout
			else if (dwRet == WAIT_TIMEOUT)
				hr = ERROR_TIMEOUT;

			// ??
			else
				hr = GetLastError();
			}	// if

		}	// if

	// Amount transfered
	CCLTRYE ( GetOverlappedResult ( hFile, &ov, &uXfer, TRUE ) == TRUE,
					GetLastError() );

	// Result
	if (hr == S_OK && puIo != NULL)
		*puIo = uXfer;

	// Debug
//	dbgprintf ( L"File::fileIo:bWr %d:uXfer %d:hr 0x%x\r\n", bWr, uXfer, hr );
	#else
	hr = E_NOTIMPL;
	#endif
	
	return hr;
	}	// fileIo

HRESULT File :: onAttach ( bool bAttach )
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
		adtValue		vL;

		// Defaults
		if (pnDesc->load ( adtString(L"AsyncRead"), vL ) == S_OK)
			bAsync = vL;
		if (pnDesc->load ( adtString(L"Size"), vL ) == S_OK)
			iSzIo = adtInt(vL);

		// I/O events
		#ifdef		_WIN32
		CCLTRYE ( (hevWr = CreateEvent ( NULL, FALSE, FALSE, NULL )) != NULL,
						GetLastError() );
		CCLTRYE ( (hevRd = CreateEvent ( NULL, FALSE, FALSE, NULL )) != NULL,
						GetLastError() );
		CCLTRYE ( (hevStop = CreateEvent ( NULL, TRUE, FALSE, NULL )) != NULL,
						GetLastError() );
		#endif
		}	// if

	// Detach
	else
		{
		// Shutdown worker thread if necessary
		if (pThrd != NULL)
			{
			pThrd->threadStop(5000);
			_RELEASE(pThrd);
			}	// if

		// Clean up
		#ifdef		_WIN32
		if (hevWr != NULL)
			{
			CloseHandle ( hevWr );
			hevWr = NULL;
			}	// if
		if (hevRd != NULL)
			{
			CloseHandle ( hevRd );
			hevRd = NULL;
			}	// if
		if (hevStop != NULL)
			{
			CloseHandle ( hevStop );
			hevStop = NULL;
			}	// if
		#endif
		_RELEASE(pStmIo);
		}	// else

	return hr;
	}	// onAttach


HRESULT File :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Write
	if (_RCP(Write))
		{
		U32			uLeft	= 0;
		U64			uXferd;
		ULONG			uXfer;

		// State check
		#ifdef		_WIN32
		CCLTRYE ( hFile != INVALID_HANDLE_VALUE,	ERROR_INVALID_STATE );
		#endif
		CCLTRYE ( pStmIo != NULL,						ERROR_INVALID_STATE );

		// Size of transfer, size of 0 means entire stream
		if (hr == S_OK && iSzIo == 0)
			{
			U64	uAv = 0;
			CCLTRY ( pStmIo->available ( &uAv ) );
			CCLOK  ( uLeft = (U32) uAv; )
			}	// if
		else if (hr == S_OK)
			uLeft = iSzIo;

		// Write packet size data from stream
		while (hr == S_OK && uLeft > 0)
			{
			// Amount to write on next transaction
			uXfer = (uLeft < iSzBfr) ? uLeft : (ULONG)iSzBfr;

			// Read from source stream
			CCLTRY ( pStmIo->read ( pcBfr, uXfer, &uXferd ) );

			// Write out File
			CCLTRY ( fileIo ( TRUE, (DWORD)uXferd, 5000, &uXfer ) );

			// Next block
			CCLOK ( uLeft -= uXfer; )
			}	// while

		// Result
		if (hr != S_OK)
			dbgprintf ( L"File::Write:hr 0x%x, I/O %d/%d\r\n", hr, uLeft, (U32)iSzIo );
		if (hr == S_OK)
			_EMT(Write,adtIUnknown(pStmIo));
		else
			_EMT(Error,adtInt(hr));
		}	// if

	// Read
	else if (_RCP(Read))
		{
		U32			uLeft	= 0;
		ULONG			uXfer;

		// State check
		#ifdef		_WIN32
		CCLTRYE ( hFile != INVALID_HANDLE_VALUE,	ERROR_INVALID_STATE );
		#endif
		CCLTRYE ( pStmIo != NULL && iSzIo > 0,		ERROR_INVALID_STATE );

		// If asynchronous reads are enabled, reads happen in thread
		CCLTRYE ( bAsync == false, ERROR_INVALID_STATE );

		// Read packet size data from File
		CCLOK ( uLeft = iSzIo; )
		while (hr == S_OK && uLeft > 0)
			{
			// Amount to read on next transaction
			uXfer = (uLeft < iSzBfr) ? uLeft : (ULONG)iSzBfr;

			// Read from File.
			CCLTRY ( fileIo ( FALSE, uXfer, 5000, &uXfer ) );

			// Write to destination stream
			CCLTRY ( pStmIo->write ( pcBfr, uXfer, NULL ) );

			// Next block
			CCLOK ( uLeft -= uXfer; )

			// A short packet means end of transfer
			if (hr == S_OK && uXfer < iSzBfr)
				break;
			}	// while

		// Result
		if (hr != S_OK)
			dbgprintf ( L"File::Read:hr 0x%x, I/O %d/%d\r\n", hr, uLeft, (U32)iSzIo );
		if (hr == S_OK)
			_EMT(Read,adtIUnknown(pStmIo));
		else
			_EMT(Error,adtInt(hr));
		}	// if

	// State
	else if (_RCP(File))
		{
		IResource		*pRes = NULL;
		adtIUnknown		unkV(v);
		adtValue			vId;

		// Shutdown current state
		if (pThrd != NULL)
			{
			pThrd->threadStop(5000);
			_RELEASE(pThrd);
			}	// if
		#ifdef	_WIN32
		hFile = INVALID_HANDLE_VALUE;
		#endif
		
		// New resource/file handle
		CCLTRY ( _QISAFE(unkV,IID_IResource,&pRes) );
		CCLTRY ( pRes->getResId ( vId ) );
		#ifdef	_WIN32
		CCLOK  ( hFile = (HANDLE)(U64)adtLong(vId); )
		#endif
		_RELEASE(pRes);
		
		// If async reads are specified, fire up thread
		#ifdef	_WIN32
		if (hr == S_OK && hFile != INVALID_HANDLE_VALUE && bAsync == true)
			{
			// Create read thread
			CCLOK(ResetEvent(hevStop);)
			CCLTRY(COCREATE(L"Sys.Thread", IID_IThread, &pThrd ));
			CCLTRY(pThrd->threadStart ( this, 5000 ));
			}	// if
		#endif

		}	// else if
	else if (_RCP(Stream))
		{
		adtIUnknown		unkV(v);
		_RELEASE(pStmIo);
		_QISAFE(unkV,IID_IByteStream,&pStmIo);
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

HRESULT File :: tickAbort ( void )
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

	// Signal stop
	#ifdef	_WIN32
	if (hevStop != NULL)
		SetEvent ( hevStop );
	#endif

	return S_OK;
	}	// tickAbort

HRESULT File :: tick ( void )
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
	DWORD		uXfer,uLeft;

	// Debug
//	dbgprintf ( L"File::tick {\n" );

	// Sanity check to avoid fast looping
	if (hr == S_OK && iSzIo == 0)
		{
		dbgprintf ( L"File::tick:WARNING no transfer size specified\r\n" );
		#ifdef 	_WIN32
		Sleep(1000);
		#endif
		}	// if

	// Perform transfer
	uLeft = iSzIo;
	while (hr == S_OK && uLeft > 0)
		{
		// Read from end point. 
		CCLTRY (fileIo ( FALSE, iSzBfr, INFINITE, &uXfer ));

		// Ok if no data was read as long as there is not an error.
		// For example this happens on a serial port with a timeout.
		if (hr == S_OK && uXfer > 0)
			{
			// Write to destination stream
			CCLTRY ( pStmIo->write ( pcBfr, uXfer, NULL ) );

			// Next block
			CCLOK ( uLeft -= uXfer; )

			// Assume a short read means end of transfer
			if (hr == S_OK && uXfer < iSzBfr)
				break;
			}	// if

		}	// while

	// Send out read stream
	if (hr == S_OK)
		_EMT(Read,adtIUnknown(pStmIo));
	else
		_EMT(Error,adtInt(hr));

	// Debug
//	dbgprintf ( L"} File::tick (0x%x)\n", hr );

	return hr;
	}	// tick

HRESULT File :: tickBegin ( void )
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
	return S_OK;
	}	// tickBegin

HRESULT File :: tickEnd ( void )
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
	return S_OK;
	}	// tickEnd
/*
void File :: update ( IDictionary *pDesc )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Update internal state of node to handle new configuration.
	//
	//	PARAMETERS
	//		-	pDesc contains the File information
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;
	adtValue	vL;

	// Previous state
	iSzBfr = 0;
	iPipe  = -1;
	_FREEMEM(pcBfr);

	// Query the pipe information
	CCLTRY ( pDesc->load ( adtString(L"Id"), vL ) );
	CCLOK  ( iPipe = vL; )
	CCLTRY ( pDesc->load ( adtString(L"MaximumPacketSize"), vL ) );
	CCLOK  ( iSzBfr = vL; )

	// Allocate enough space for a full packet
	CCLTRYE ( iSzBfr > 0, E_UNEXPECTED );
	CCLTRYE ( (pcBfr = (U8 *) _ALLOCMEM(iSzBfr)) != NULL, E_OUTOFMEMORY );
	}	// update

*/