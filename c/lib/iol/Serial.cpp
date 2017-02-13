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

	// State check
	if (!bAttach) return S_OK;

	// Defaults
	if (pnDesc->load ( adtString(L"Baud"), vL ) == S_OK)
		iBaud = vL;
	if (pnDesc->load ( adtString(L"Bits"), vL ) == S_OK)
		iBits = vL;
	if (pnDesc->load ( adtString(L"Parity"), vL ) == S_OK)
		adtValue::toString ( vL, strParity );
	if (pnDesc->load ( adtString(L"StopBits"), vL ) == S_OK)
		fStop = vL;

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
		IResource	*pRes = NULL;
		adtIUnknown unkV(v);
		adtValue		vL;

		// Resource ?
		CCLTRY ( _QISAFE(unkV,IID_IResource,&pRes) );

		// Exract operating system specific Id
		CCLTRY ( pRes->getResId (vL) );

		// Windows
		#ifdef	_WIN32
		if (hr == S_OK)
			{
			HANDLE	hPort = INVALID_HANDLE_VALUE;
			DCB		dcb;

			// Expecting handle to port
			CCLTRYE ( (hPort = (HANDLE)(U64)adtLong(vL)) != INVALID_HANDLE_VALUE,
							E_INVALIDARG );
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

		// Clean up
		_RELEASE(pRes);
		}	// if

	// State
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

