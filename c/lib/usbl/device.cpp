////////////////////////////////////////////////////////////////////////
//
//									DEVICE.CPP
//
//				Implementation of the USB device node.
//
////////////////////////////////////////////////////////////////////////

#define	INITGUID
#include "usbl_.h"
#include <stdio.h>

// Globals

Device :: Device ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pRes		= NULL;
	hIntf		= INVALID_HANDLE_VALUE;
	iAltSet	= 0;
	}	// Device

HRESULT Device :: onAttach ( bool bAttach )
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
		if (pnDesc->load ( adtString(L"AlternateSetting"), vL ) == S_OK)
			iAltSet = vL;
		}	// if

	// Detach
	else
		{
		// Shutdown
		if (hIntf != INVALID_HANDLE_VALUE)
			{
			WinUsb_Free ( hIntf );
			hIntf = INVALID_HANDLE_VALUE;
			}	// if
		_RELEASE(pRes);
		}	// else

	return hr;
	}	// onAttach

HRESULT Device :: receive ( IReceptor *pr, const WCHAR *pl, 
										const ADTVALUE &v )
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

	// Open device
	if (_RCP(Open))
		{
		HANDLE						hDev		= INVALID_HANDLE_VALUE;
		IDictionary					*pDev		= NULL;
		adtIUnknown					unkV(v);
		adtValue						vL;
		USB_DEVICE_DESCRIPTOR	udd;
		ULONG							nx;

		// State check
		CCLTRYE ( hIntf == INVALID_HANDLE_VALUE, ERROR_INVALID_STATE );
		CCLTRYE ( pRes != NULL, ERROR_INVALID_STATE );

		// It is assumed that an asynchronous device stream has already 
		// been opened on the device therefore its descriptor is available.
		CCLTRY ( pRes->getResId ( vL ) );

		// Initalize USB on device
		CCLTRYE ( (hDev = (HANDLE)(U64)adtLong(vL)) != INVALID_HANDLE_VALUE, E_UNEXPECTED );
		CCLTRYE ( WinUsb_Initialize ( hDev, &hIntf ) == TRUE, GetLastError() );

		// Device information
		CCLTRYE ( WinUsb_GetDescriptor ( hIntf, USB_DEVICE_DESCRIPTOR_TYPE, 0, 0,
						(PUCHAR) &udd, sizeof(udd), &nx ) == TRUE, GetLastError() );

		// Create dictionary to contain device information
		CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDev ) );
		CCLTRY ( pDev->store ( adtString(L"Interface"), adtLong((U64)hIntf) ) );
		CCLTRY ( pDev->store ( adtString(L"IdVendor"), adtInt(udd.idVendor) ) );
		CCLTRY ( pDev->store ( adtString(L"IdProduct"), adtInt(udd.idProduct) ) );
		CCLTRY ( pDev->store ( adtString(L"SerialNumber"), adtInt(udd.iSerialNumber) ) );
		CCLTRY ( pDev->store ( adtString(L"Manufacturer"), adtInt(udd.iManufacturer) ) );
													
		// Debug
		lprintf ( LOG_INFO, L"Open:pRes %p:hIntf 0x%x:hr 0x%x\r\n",
									pRes, hIntf, hr );

		// Result 
		if (hr == S_OK)
			_EMT(Device,adtIUnknown(pDev));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pDev);
		}	// if

	// Close
	else if (_RCP(Close))
		{
		// Stream object
		_RELEASE(pRes);

		// Interface
		if (hIntf != INVALID_HANDLE_VALUE)
			{
			// Shutting down
			_EMT(Device,adtIUnknown(NULL));

			// Clean up
			WinUsb_Free ( hIntf );
			hIntf = INVALID_HANDLE_VALUE;
			}	// if

		}	// else if

	// Query information about the current open interface
	else if (_RCP(Query))
		{
		IDictionary						*pInfo	= NULL;
		IDictionary						*pEndpts	= NULL;
		USB_INTERFACE_DESCRIPTOR	uid;
		U32								e;
//		U8									b;

		// State check
		CCLTRYE ( hIntf != INVALID_HANDLE_VALUE, ERROR_INVALID_STATE );

		// Create a dictionary to receive the information about the device
		CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pInfo ) );

		// Obtain default interface information
		CCLTRYE ( WinUsb_QueryInterfaceSettings ( hIntf, iAltSet, &uid ) == TRUE, GetLastError() );
		CCLTRY ( pInfo->store ( adtString(L"Number"), adtInt(uid.bInterfaceNumber) ) );
		CCLTRY ( pInfo->store ( adtString(L"Class"), adtInt(uid.bInterfaceClass) ) );
		CCLTRY ( pInfo->store ( adtString(L"SubClass"), adtInt(uid.bInterfaceSubClass) ) );
		CCLTRY ( pInfo->store ( adtString(L"Protocol"), adtInt(uid.bInterfaceProtocol) ) );

		// DEBUG
//		CCLTRYE ( WinUsb_GetCurrentAlternateSetting ( hIntf, &b ) == TRUE, GetLastError() );
//		lprintf ( LOG_INFO, L"Query:hIntf 0x%x:AltSetNow %d:hr 0x%x\r\n", hIntf, b, hr );

		// Retreive the interface

		// Activate specified alternative seting
		CCLTRYE ( WinUsb_SetCurrentAlternateSetting ( hIntf, iAltSet ) == TRUE, GetLastError() );
		if (hr != S_OK)
			lprintf ( LOG_ERR, L"Query:SetAltSetting failed:hIntf 0x%x:hr 0x%x\r\n", hIntf, hr );
//		CCLOK ( WinUsb_SetCurrentAlternateSetting ( hIntf, iAltSet ); )

		// Process end points
		CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pEndpts ) );
		CCLTRY ( pInfo->store ( adtString(L"Endpoints"), adtIUnknown(pEndpts) ) );
		CCLOK ( dbgprintf ( L"Interface endpoints : %d\r\n", uid.bNumEndpoints ); )
		for (e = 0;hr == S_OK && e < uid.bNumEndpoints;++e)
			{
			IDictionary					*pEndpt	= NULL;
			bool							bIn		= false;
			WINUSB_PIPE_INFORMATION	pi;
			adtString					strType;

			// Descriptor for endpoint under index
			CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pEndpt ) );
			CCLTRY ( pEndpts->store ( adtInt(e), adtIUnknown(pEndpt) ) );

			// Pipe information
			CCLTRYE ( WinUsb_QueryPipe ( hIntf, iAltSet, e, &pi ) == TRUE, GetLastError() );

			// Pipe direction
			CCLOK ( bIn = (USB_ENDPOINT_DIRECTION_IN(pi.PipeId) != 0); )

			// Type
			CCLOK ( strType = (pi.PipeType == UsbdPipeTypeControl)		? L"Control" :
									(pi.PipeType == UsbdPipeTypeBulk)			? L"Bulk" :
									(pi.PipeType == UsbdPipeTypeInterrupt)		? L"Interrupt" :
									(pi.PipeType == UsbdPipeTypeIsochronous)	? L"Isochronous" : L"Unknown"; )

			// Fill in dictionary
			CCLTRY ( pEndpt->store ( adtString(L"Id"), adtInt(pi.PipeId) ) );
			CCLTRY ( pEndpt->store ( adtString(L"Type"), strType ) );
			CCLTRY ( pEndpt->store ( adtString(L"Direction"), adtString ( (bIn) ? L"In" : L"Out" ) ) );
			CCLTRY ( pEndpt->store ( adtString(L"MaximumPacketSize"), adtInt(pi.MaximumPacketSize) ) );
			CCLTRY ( pEndpt->store ( adtString(L"Interval"), adtInt(pi.Interval) ) );

			// Debug
			dbgprintf ( L"Pipe %d,%s,%s,%d\r\n", pi.PipeId, (LPCWSTR)strType, (bIn) ? L"Input" : L"Output", pi.MaximumPacketSize );

			// Clean up
			_RELEASE(pEndpt);
			}	// for

		// Result
		if (hr == S_OK)
			_EMT(Query,adtIUnknown(pInfo));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pEndpts);
		_RELEASE(pInfo);
		}	// else if

	// IResource is required to get system Id (Handle, file descriptor, etc) for stream
	else if (_RCP(Stream))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pRes);
		_QISAFE(unkV,IID_IResource,&pRes);
		}	// else if
	else if (_RCP(Setting))
		iAltSet = adtInt(v);
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive
