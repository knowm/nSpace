////////////////////////////////////////////////////////////////////////
//
//									ENDP.CPP
//
//				Implementation of the USB endpoint node.
//
////////////////////////////////////////////////////////////////////////

#include "usbl_.h"
#include <stdio.h>

// Globals

Endpoint :: Endpoint ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	hIntf		= INVALID_HANDLE_VALUE;
	iPipe		= -1;
	pStmIo	= NULL;
	iSzIo		= 0;
	pcBfrPkt	= NULL;
	hevWr		= NULL;
	hevRd		= NULL;
	hevStop	= NULL;
	hevRst	= NULL;
	bAsync	= false;
	}	// Endpoint

HRESULT Endpoint :: onAttach ( bool bAttach )
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
		if (pnDesc->load ( adtString(L"Id"), vL ) == S_OK)
			iPipe = vL;
		if (pnDesc->load ( adtString(L"AsyncRead"), vL ) == S_OK)
			bAsync = vL;

		// I/O events
		CCLTRYE ( (hevWr = CreateEvent ( NULL, FALSE, FALSE, NULL )) != NULL,
						GetLastError() );
		CCLTRYE ( (hevRd = CreateEvent ( NULL, FALSE, FALSE, NULL )) != NULL,
						GetLastError() );
		CCLTRYE ( (hevRst = CreateEvent ( NULL, FALSE, FALSE, NULL )) != NULL,
						GetLastError() );
		CCLTRYE ( (hevStop = CreateEvent ( NULL, TRUE, FALSE, NULL )) != NULL,
						GetLastError() );
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
		if (hevRst != NULL)
			{
			CloseHandle ( hevRst );
			hevRst = NULL;
			}	// if
		_FREEMEM(pcBfrPkt);
		_RELEASE(pStmIo);
		}	// else

	return hr;
	}	// onAttach

HRESULT Endpoint :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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
		U32	uLeft	= 0;
		U64	uXferd;
		ULONG uXfer;

		// State check
		CCLTRYE ( hIntf != INVALID_HANDLE_VALUE,	ERROR_INVALID_STATE );
		CCLTRYE ( iPipe != -1,							ERROR_INVALID_STATE );
		CCLTRYE ( pStmIo != NULL,						ERROR_INVALID_STATE );

		// Size of transfer, size of 0 means entire stream
		if (hr == S_OK && iSzIo == 0)
			{
			U64	uAv = 0;
			CCLTRY ( pStmIo->available ( &uAv ) );
			CCLOK  ( iSzIo = (U32) uAv; )
			}	// if

		// TODO: Asynchronous I/O
		// Write packet size data from stream
		CCLOK ( uLeft = iSzIo; )
		while (hr == S_OK && uLeft > 0)
			{
			// Amount to write on next transaction
			uXfer = (uLeft < iSzPkt) ? uLeft : iSzPkt;

			// Read from source stream
			CCLTRY ( pStmIo->read ( pcBfrPkt, uXfer, &uXferd ) );

			// Write out endpoint
			CCLTRYE ( WinUsb_WritePipe ( hIntf, iPipe, pcBfrPkt, (U32)uXferd,
													&uXfer, NULL ) == TRUE, GetLastError() );
			// Next block
			CCLOK ( uLeft -= uXfer; )
			}	// while

		// Result
		if (hr != S_OK)
			dbgprintf ( L"Endpoint::Write:hr 0x%x, I/O %d/%d\r\n", hr, uLeft, (U32)iSzIo );
		if (hr == S_OK)
			_EMT(Write,adtIUnknown(pStmIo));
		else
			_EMT(Error,adtInt(hr));
		}	// if

	// Read
	else if (_RCP(Read))
		{
		U32	uLeft	= 0;
		ULONG uXfer;

		// State check
		CCLTRYE ( hIntf != INVALID_HANDLE_VALUE,	ERROR_INVALID_STATE );
		CCLTRYE ( iPipe != -1,							ERROR_INVALID_STATE );
		CCLTRYE ( pStmIo != NULL && iSzIo > 0,		ERROR_INVALID_STATE );

		// If asynchronous reads are enabled, reads happen in thread
		CCLTRYE ( bAsync == false, ERROR_INVALID_STATE );

		// Read packet size data from endpoint
		CCLOK ( uLeft = iSzIo; )
		while (hr == S_OK && uLeft > 0)
			{
			// Amount to read on next transaction
			uXfer = (uLeft < iSzPkt) ? uLeft : iSzPkt;

			// Read from end point
			CCLTRYE ( WinUsb_ReadPipe ( hIntf, iPipe, pcBfrPkt, iSzPkt,
										&uXfer, NULL ) == TRUE, GetLastError() );

			// Write to destination stream
			CCLTRY ( pStmIo->write ( pcBfrPkt, uXfer, NULL ) );

			// Next block
			CCLOK ( uLeft -= uXfer; )

			// A short packet means end of transfer
			if (hr == S_OK && uXfer < iSzPkt)
				break;
			}	// while

		// Result
		if (hr != S_OK)
			dbgprintf ( L"Endpoint::Read:hr 0x%x, I/O %d/%d\r\n", hr, uLeft, (U32)iSzIo );
		if (hr == S_OK)
			_EMT(Read,adtIUnknown(pStmIo));
		else
			_EMT(Error,adtInt(hr));
		}	// if

	// State
	else if (_RCP(Device))
		{
		IDictionary		*pDev = NULL;
		adtIUnknown		unkV(v);
		adtValue			vL;

		// Currently all that is needed is the interface value
		hIntf = INVALID_HANDLE_VALUE;
		CCLTRY ( _QISAFE(unkV,IID_IDictionary,&pDev) );
		CCLTRY ( pDev->load ( adtString(L"Interface"), vL ) );
		CCLOK  ( hIntf = (WINUSB_INTERFACE_HANDLE)(U64)adtLong(vL); )
		}	// else if
	else if (_RCP(Endpoint))
		{
		IDictionary		*pEndp	= NULL;
		adtIUnknown		unkV(v);

		// Shutdown if async thread was started/specified
		if (pThrd != NULL)
			{
			pThrd->threadStop(5000);
			_RELEASE(pThrd);
			}	// if

		// Access endpoint information
		CCLTRY ( _QISAFE(unkV,IID_IDictionary,&pEndp) );
		CCLOK  ( update (pEndp); )

		// If endpoint is valid and asynchronous reads are requested, start thread
		if (hr == S_OK && iPipe != -1)
			{
			// Create read thread
			CCLOK(ResetEvent(hevStop);)
			CCLTRY(COCREATE(L"Sys.Thread", IID_IThread, &pThrd ));
			CCLTRY(pThrd->threadStart ( this, 5000 ));
			}	// if

		// Clean up
		_RELEASE(pEndp);
		}	// else if
	else if (_RCP(Stream))
		{
		adtIUnknown		unkV(v);
		_RELEASE(pStmIo);
		_QISAFE(unkV,IID_IByteStream,&pStmIo);
		}	// else if
	else if (_RCP(Size))
		{
		// New I/O size
		iSzIo = v;

		// Reset state if async running
		SetEvent ( hevRst );
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

HRESULT Endpoint :: tickAbort ( void )
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

HRESULT Endpoint :: tick ( void )
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
	HRESULT		hr			= S_OK;
	HANDLE		hevs[3]	= { hevRd, hevRst, hevStop };
	OVERLAPPED	ov;
	DWORD			dwRet,uXfer,uLeft;

	// Debug
	dbgprintf ( L"Endpoint::tick {\n" );

	// Perform one 'block' of transfers
	uLeft = iSzIo;
	while (hr == S_OK && uLeft > 0)
		{
		// Perform async read
		memset ( &ov, 0, sizeof(ov) );
		ov.hEvent = hevRd;
		CCLTRYE ( WinUsb_ReadPipe (	hIntf, iPipe, pcBfrPkt, iSzPkt,
												NULL, &ov ) == TRUE, GetLastError() );

		// Waiting for data
		if (hr == ERROR_IO_PENDING)
			{
			// Wait for completion or signal to stop
			hr		= S_OK;
			dwRet = WaitForMultipleObjects ( 3, hevs, FALSE, INFINITE );

			// Reset state
			if (dwRet == WAIT_OBJECT_0+1)
				{
				// Reset I/O transfer
				uLeft = iSzIo;
				continue;
				}	// if

			// Stop
			else if (dwRet == WAIT_OBJECT_0+2)
				{
				// Shutting down
				hr = S_FALSE;
				}	// else if

			// USB Read complete
			else if (dwRet != WAIT_OBJECT_0+0)
				{
				hr = S_FALSE;
				}	// else if

			}	// if

		// Amount transfered
		CCLTRYE ( WinUsb_GetOverlappedResult ( hIntf, &ov, &uXfer, TRUE ) == TRUE,
						GetLastError() );

		// Write to destination stream
		CCLTRY ( pStmIo->write ( pcBfrPkt, uXfer, NULL ) );

		// Next block
		CCLOK ( uLeft -= uXfer; )

		// A short packet means end of transfer
		if (hr == S_OK && uXfer < iSzPkt)
			break;
		}	// while

	// Send out read stream
	if (hr == S_OK)
		_EMT(Read,adtIUnknown(pStmIo));
	else
		_EMT(Error,adtInt(hr));

	// Debug
	dbgprintf ( L"} Endpoint::tick (0x%x)\n", hr );

	return hr;
	}	// tick

HRESULT Endpoint :: tickBegin ( void )
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

HRESULT Endpoint :: tickEnd ( void )
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

void Endpoint :: update ( IDictionary *pDesc )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Update internal state of node to handle new configuration.
	//
	//	PARAMETERS
	//		-	pDesc contains the endpoint information
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;
	adtValue	vL;

	// Previous state
	iSzPkt = 0;
	iPipe  = -1;
	_FREEMEM(pcBfrPkt);

	// Query the pipe information
	CCLTRY ( pDesc->load ( adtString(L"Id"), vL ) );
	CCLOK  ( iPipe = vL; )
	CCLTRY ( pDesc->load ( adtString(L"MaximumPacketSize"), vL ) );
	CCLOK  ( iSzPkt = vL; )

	// Allocate enough space for a full packet
	CCLTRYE ( iSzPkt > 0, E_UNEXPECTED );
	CCLTRYE ( (pcBfrPkt = (U8 *) _ALLOCMEM(iSzPkt)) != NULL, E_OUTOFMEMORY );
	}	// update

