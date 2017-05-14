////////////////////////////////////////////////////////////////////////
//
//									SERIAL.CPP
//
//					Implementation of the serial port setup node
//
////////////////////////////////////////////////////////////////////////

#include "iol_.h"
#include <stdio.h>

Serial :: Serial ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////

	// Defaults
	iBaud			= 9600;
	iBits			= 8;
	strParity	= L"None";
	fStop			= 1.0f;
	pItEv			= NULL;
	pThrd			= NULL;
	bRun			= false;
	#ifdef		_WIN32
	hPort			= INVALID_HANDLE_VALUE;
	hevWait		= NULL;
	#endif
	}	// Serial

void Serial :: destruct ( void )
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
	}	// destruct

HRESULT Serial :: onAttach ( bool bAttach )
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
	HRESULT	hr	= S_OK;
	adtValue vL;

	// Attach
	if (bAttach)
		{
		// Defaults
		if (pnDesc->load ( adtString(L"Baud"), vL ) == S_OK)
			iBaud = vL;
		if (pnDesc->load ( adtString(L"Bits"), vL ) == S_OK)
			iBits = vL;
		if (pnDesc->load ( adtString(L"Parity"), vL ) == S_OK)
			adtValue::toString ( vL, strParity );
		if (pnDesc->load ( adtString(L"StopBits"), vL ) == S_OK)
			fStop = vL;
		if (pnDesc->load ( adtString(L"Monitor"), vL ) == S_OK)
			{
			IContainer		*pCnt	= NULL;
			adtIUnknown		unkV(vL);

			// Monitor list should be a list of strings for each desired event
			CCLTRY(_QISAFE(unkV,IID_IContainer,&pCnt));
			CCLTRY(pCnt->iterate(&pItEv));
			_RELEASE(pCnt);

			// Do not fail attachment due to error
			if (hr != S_OK)
				{
				lprintf ( LOG_ERR, L"Monitor list property invalid, hr 0x%x\r\n", hr );
				hr = S_OK;
				}	// if
			}	// if

		// Event handle for waiting
		CCLTRYE ( (hevWait = CreateEvent ( NULL, TRUE, FALSE, NULL )) != NULL,
						GetLastError() );

		}	// if

	// Detach
	else
		{
		onReceive(prStop,adtInt());
		if (hevWait != NULL)
			{
			CloseHandle ( hevWait );
			hevWait = NULL;
			}	//if
		_RELEASE(pItEv);
		}	// else

	return hr;
	}	// onAttach

HRESULT Serial :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Apply serial settings
	if (_RCP(Fire))
		{
		// Backward compat, set active port if necessary
		if (hPort == INVALID_HANDLE_VALUE)
			onReceive(prPort,v);

		// State check
		CCLTRYE ( hPort != INVALID_HANDLE_VALUE, ERROR_INVALID_STATE );

		// Windows
		#ifdef	_WIN32
		if (hr == S_OK)
			{
			DCB		dcb;

			// Expecting handle to port
			if (hr == S_OK)
				{
				// Current settings
				memset ( &dcb, 0, sizeof(dcb) );
				dcb.DCBlength = sizeof(dcb);
				CCLTRYE ( GetCommState ( hPort, &dcb ) == TRUE, GetLastError() );

				// Adjust according to properties
				if (hr == S_OK)
					{
					dcb.BaudRate	= (iBaud == 110) ? CBR_110 :
											(iBaud == 300) ? CBR_300 :
											(iBaud == 600) ? CBR_600 :
											(iBaud == 1200) ? CBR_1200 :
											(iBaud == 2400) ? CBR_2400 :
											(iBaud == 4800) ? CBR_4800 :
											(iBaud == 9600) ? CBR_9600 :
											(iBaud == 14400) ? CBR_14400 :
											(iBaud == 19200) ? CBR_19200 :
											(iBaud == 38400) ? CBR_38400 :
											(iBaud == 57600) ? CBR_57600 :
											(iBaud == 115200) ? CBR_115200 :
											(iBaud == 128000) ? CBR_128000 :
											(iBaud == 256000) ? CBR_256000 : CBR_9600;
					dcb.ByteSize	=	iBits;
					dcb.StopBits	=	(fStop == 1.5) ? ONE5STOPBITS :
											(fStop == 2.0) ? TWOSTOPBITS : ONESTOPBIT;
					dcb.Parity		=	(!WCASECMP(strParity,L"Even")) ? EVENPARITY :
											(!WCASECMP(strParity,L"Odd")) ? ODDPARITY : NOPARITY;
					}	// if

				// New settings
				CCLTRYE ( SetCommState ( hPort, &dcb ) == TRUE, GetLastError() );

				// Timeouts
				if (hr == S_OK)
					{
					COMMTIMEOUTS	cto;

					// Set communication timeouts TODO: Node properties
					memset ( &cto, 0, sizeof(cto) );
					cto.ReadIntervalTimeout			= 20;
					cto.ReadTotalTimeoutConstant	= 1000;
					cto.WriteTotalTimeoutConstant	= 1000;
					CCLTRYE ( SetCommTimeouts ( hPort, &cto ) == TRUE, GetLastError() );
					}	// if
				}	// if

			}	// if
		#endif

		// Result
		if (hr == S_OK)
			_EMT(Fire,v);
		else
			_EMT(Error,adtInt(hr) );

		}	// if

	// Start monitoring
	else if (_RCP(Start))
		{
		// State check
		CCLTRYE ( pThrd == NULL, ERROR_ALREADY_THREAD );
		CCLTRYE ( pItEv != NULL, ERROR_INVALID_STATE );

		// Start the monitoring thread
		#ifdef	_WIN32
		CCLOK(ResetEvent(hevWait);)
		CCLOK(bRun = true;)
		CCLTRY(COCREATE(L"Sys.Thread", IID_IThread, &pThrd ));
		CCLTRY(pThrd->threadStart ( this, 5000 ));
		#else
		lprintf ( L"Not yet implemented for target OS\r\n" );
		#endif
		}	// else if

	// Stop monitoring
	else if (_RCP(Stop))
		{
		// Worker thread
		if (pThrd != NULL)
			{
			pThrd->threadStop(5000);
			_RELEASE(pThrd);
			}	// if
		}	// else if

	// State
	else if (_RCP(Port))
		{
		IResource	*pRes		= NULL;
		adtValue		vL;
		adtIUnknown	unkV(v);

		// Resource ?
		CCLTRY ( _QISAFE(unkV,IID_IResource,&pRes) );

		// Exract operating system specific Id
		CCLTRY ( pRes->getResId (vL) );

		// Extract port
		#ifdef	_WIN32
		CCLTRYE ( (hPort = (HANDLE)(U64)adtLong(vL)) != INVALID_HANDLE_VALUE,
						E_INVALIDARG );
		#else
		hr = E_NOTIMPL;
		#endif

		// Clean up
		_RELEASE(pRes);
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// onReceive

HRESULT Serial :: tickAbort ( void )
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
	bRun = false;
	#ifdef	_WIN32
	if (hevWait != NULL)
		SetEvent ( hevWait );
	#endif

	return S_OK;
	}	// tickAbort

HRESULT Serial :: tick ( void )
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
	HRESULT		hr		= S_OK;
	#ifdef		_WIN32
	DWORD			dwEv	= 0;
	OVERLAPPED	ov;
	BOOL			bRet;
	DWORD			dwX;

	// Prepare overlapped I/O
	memset ( &ov, 0, sizeof(ov) );
	ov.hEvent	= hevWait;

	// Still running ?
	CCLTRYE ( bRun == true, S_FALSE );

	// Initiate wait
	CCLOK ( bRet = WaitCommEvent ( hPort, &dwEv, &ov ); )

	// Successful but pending
	if (hr == S_OK && !bRet && GetLastError() == ERROR_IO_PENDING)
		{
		// Wait for completion
		CCLTRYE ( WaitForSingleObject ( hevWait, INFINITE ) == WAIT_OBJECT_0,
						GetLastError() );

		// Complete
		CCLTRYE ( GetOverlappedResult ( hPort, &ov, &dwX, TRUE ) == TRUE, GetLastError() );

		// Result
		CCLOK ( bRet = TRUE; )
		}	// if

	// Successful wait
	if (hr == S_OK && bRet)
		{
		// Emit the matching Changes
		if (dwEv & EV_BREAK)
			_EMT(Change,adtString(L"Break"));
		if (dwEv & EV_CTS)
			_EMT(Change,adtString(L"CTS"));
		if (dwEv & EV_DSR)
			_EMT(Change,adtString(L"DSR"));
		if (dwEv & EV_ERR)
			_EMT(Change,adtString(L"Error"));
		if (dwEv & EV_RING)
			_EMT(Change,adtString(L"Ring"));
		if (dwEv & EV_RLSD)
			_EMT(Change,adtString(L"RLSD"));
		if (dwEv & EV_RXCHAR)
			_EMT(Change,adtString(L"RxChar"));
		if (dwEv & EV_RXFLAG)
			_EMT(Change,adtString(L"RxFlag"));
		if (dwEv & EV_TXEMPTY)
			_EMT(Change,adtString(L"TxEmpty"));
		}	// if
	#endif

	return hr;
	}	// tick

HRESULT Serial :: tickBegin ( void )
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
	HRESULT		hr = S_OK;
	adtValue		vL;

	#ifdef	_WIN32
	DWORD			dwEv	= 0;

	// State check
	CCLTRYE ( pItEv != NULL, E_UNEXPECTED );

	// Generate communication mask for events that will trigger wait event
	CCLTRY ( pItEv->begin() );
	while (hr == S_OK && pItEv->read ( vL ) == S_OK)
		{
		adtString strEv(vL);

		// Map string to mask
		if (!WCASECMP(strEv,L"Break"))
			dwEv |=  EV_BREAK;
		if (!WCASECMP(strEv,L"CTS"))
			dwEv |=  EV_CTS;
		if (!WCASECMP(strEv,L"DSR"))
			dwEv |=  EV_DSR;
		if (!WCASECMP(strEv,L"Error"))
			dwEv |=  EV_ERR;
		if (!WCASECMP(strEv,L"Ring"))
			dwEv |=  EV_RING;
		if (!WCASECMP(strEv,L"RLSD"))
			dwEv |=  EV_RLSD;
		if (!WCASECMP(strEv,L"RxChar"))
			dwEv |=  EV_RXCHAR;
		if (!WCASECMP(strEv,L"RxFlag"))
			dwEv |=  EV_RXFLAG;
		if (!WCASECMP(strEv,L"TxEmpty"))
			dwEv |=  EV_TXEMPTY;

		// Next event
		pItEv->next();
		}	// while

	// Any matchin events ?
	CCLTRYE ( dwEv != 0, E_UNEXPECTED );

	// Set mask for waitiable events
	CCLTRYE ( SetCommMask ( hPort, dwEv ) == TRUE, GetLastError() );

	#else
	hr = E_NOTIMPL;
	#endif

	return hr;
	}	// tickBegin

HRESULT Serial :: tickEnd ( void )
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

