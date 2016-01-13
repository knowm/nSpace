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

		// I/O events
		CCLTRYE ( (hevWr = CreateEvent ( NULL, FALSE, FALSE, NULL )) != NULL,
						GetLastError() );
		CCLTRYE ( (hevRd = CreateEvent ( NULL, FALSE, FALSE, NULL )) != NULL,
						GetLastError() );
		}	// if

	// Detach
	else
		{
		// Shutdown
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
		U32	uLeft;
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
		U32	uLeft;
		ULONG uXfer;

		// State check
		CCLTRYE ( hIntf != INVALID_HANDLE_VALUE,	ERROR_INVALID_STATE );
		CCLTRYE ( iPipe != -1,							ERROR_INVALID_STATE );
		CCLTRYE ( pStmIo != NULL && iSzIo > 0,		ERROR_INVALID_STATE );

		// Read packet size data from endpoint
		// TODO: Asynchronous I/O
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

		// Access endpoint information
		CCLTRY ( _QISAFE(unkV,IID_IDictionary,&pEndp) );
		CCLOK  ( update (pEndp); )

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
		iSzIo = v;
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

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

/*
			// Write out endpoint
			if (hr == S_OK)
				{
				OVERLAPPED	ov;

				// Attempt write, ok to fail with overlapped I/O
				memset ( &ov, 0, sizeof(ov) );
				ov.hEvent = hevWr;
				CCLTRYE ( WinUsb_WritePipe ( hIntf, iPipe, pcBfrPkt, iSzPkt,
														NULL, &ov ) == TRUE, GetLastError() );

				// Wait for completion
				if (hr == ERROR_IO_PENDING)
					{
					hr = S_OK;
					CCLTRYE ( (WaitForSingleObject ( hevWr, 5000 ) == WAIT_OBJECT_0),
								ERROR_TIMEOUT );
					}	// if

				// Amount transfered
				CCLTRYE ( WinUsb_GetOverlappedResult ( hIntf, &ov, &uXfer, TRUE ) == TRUE,
								GetLastError() );

				}	// if
*/
