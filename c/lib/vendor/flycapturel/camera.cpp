////////////////////////////////////////////////////////////////////////
//
//									CAMERA.CPP
//
//		Implementation of the Point Grey camera node
//
////////////////////////////////////////////////////////////////////////

#include "flycapturel_.h"
#include <stdio.h>

// Globals

Camera :: Camera ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	iIdx		= -1;
	bRun		= false;
	pDctImg	= NULL;
	pBits		= NULL;
	}	// Camera

HRESULT Camera :: onAttach ( bool bAttach )
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
		// Create a dictionary and memory block for image data
		CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pDctImg ) );
		CCLTRY ( COCREATE ( L"Io.MemoryBlock", IID_IMemoryMapped, &pBits ) );
		CCLTRY ( pDctImg->store ( adtString(L"Bits"), adtIUnknown(pBits) ) );

		// Event init.
		CCLOK ( evGrab.init(); )
		}	// if

	// Detach
	else
		{
		// Clean up
		_RELEASE(pBits);
		_RELEASE(pDctImg);
		}	// else

	return hr;
	}	// onAttach

void Camera :: onImage ( FlyCapture2::Image *pImg )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when an image is available.
	//
	//	PARAMETERS
	//		-	pImg is the capture image
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;
	VOID		*pvBits	= NULL;
	U32		sz;

	// Size of image data
	sz = pImg->GetRows()*pImg->GetCols()*(pImg->GetBitsPerPixel()/8);

	// Image available
	dbgprintf ( L"Camera::onImage:pImg %p %d X %d:Format 0x%x:Data %p:sz %d (%d)\r\n", 
					pImg, pImg->GetCols(), pImg->GetRows(), pImg->GetPixelFormat(),		
					pImg->GetData(), pImg->GetDataSize(), sz );

	// Ensure correct size, this will be a no-op if already correct size
	CCLTRY ( pBits->setSize ( sz ) );
	CCLTRY ( pBits->lock ( 0, 0, &pvBits, NULL ) );

	// Update bits
	CCLOK ( memcpy ( pvBits, pImg->GetData(), sz ); )

	// Attributes
	CCLTRY ( pDctImg->store ( adtString(L"Width"), adtInt(pImg->GetCols()) ) );	
	CCLTRY ( pDctImg->store ( adtString(L"Height"), adtInt(pImg->GetRows()) ) );	

	// Format TODO: Support other formats as necessary
	if (pImg->GetPixelFormat() == FlyCapture2::PIXEL_FORMAT_MONO8)
		hr = pDctImg->store ( adtString(L"Format"), adtString(L"Grey8") );
	else
		hr = pDctImg->store ( adtString(L"Format"), adtString(L"Unknown") );

	// Clean up
	_UNLOCK(pBits,pvBits);

	// Available
	CCLTRY ( _EMT(Image,adtIUnknown(pDctImg) ) );

	// Image grabbed
	CCLOK ( evGrab.signal(); )
	}	// onImage

void Camera :: onImage ( FlyCapture2::Image *pImg, const void *pv )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Static callback funciton for captured images.
	//
	//	PARAMETERS
	//		-	pImg is the capture image
	//		-	pv is the callback data (ptr to this object)
	//
	////////////////////////////////////////////////////////////////////////

	// Forward to object
	if (pv != NULL)
		((Camera *)pv)->onImage ( pImg );

	}	// onImage

HRESULT Camera :: receive ( IReceptor *pr, const WCHAR *pl, 
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

	// Start capture
	if (_RCP(Start))
		{
		FlyCapture2::BusManager	bus;
		FlyCapture2::PGRGuid		id;

		// State check
		CCLTRYE( bRun == false, ERROR_INVALID_STATE );
		CCLTRYE( iIdx >= 0,		ERROR_INVALID_STATE );

		// Retrieve the Id for the camera
		CCLTRY ( pgrError ( bus.GetCameraFromIndex ( iIdx, &id ) ) );

		// Attempt connection to the camera
		CCLTRY ( pgrError ( cam.Connect ( &id ) ) );
/*
		// BIN testing
		if (hr == S_OK)
			{
			bool								bSupp = false;
			FlyCapture2::Format7Info	info;

			// Test for 8-bit mono, 2x2 binning
			info.mode = FlyCapture2::MODE_1;
			CCLTRY ( pgrError ( cam.GetFormat7Info ( &info, &bSupp ) ) );

			// Supported ?
			if (hr == S_OK && bSupp && (info.pixelFormatBitField & FlyCapture2::PIXEL_FORMAT_MONO8))
				{
				bool											bValid = false;
				FlyCapture2::Format7ImageSettings	set;
				FlyCapture2::Format7PacketInfo		pkt;

				// Desired settings
				memset ( &set, 0, sizeof(set) );
				set.mode				= FlyCapture2::MODE_1;
				set.width			= 1928/2;				// Full res for the test camera
				set.height			= 1448/2;				// Full res for the test camera
				set.pixelFormat	= FlyCapture2::PIXEL_FORMAT_MONO8;
				CCLTRY ( pgrError ( cam.ValidateFormat7Settings ( &set, &bValid, &pkt ) ) );

				// Valid settings ?
				if (hr == S_OK && bValid)
					{
					// Set as new settings
					CCLTRY ( pgrError ( cam.SetFormat7Configuration ( &set, pkt.recommendedBytesPerPacket ) ) );
					}	// if

				}	// if
			}	// if
*/

		// Begin capture with own callback and object
		CCLTRY ( pgrError ( cam.StartCapture ( onImage, this ) ) );

		// Wait until the first image is captured to ensure startup
		CCLOK ( evGrab.reset(); )
		CCLTRYE ( evGrab.wait ( 10000 ) == true, ERROR_TIMEOUT );

		// Result
		dbgprintf ( L"Camera::onStore:Start:hr 0x%x\r\n", hr );
		CCLOK ( bRun = true; )
		if (hr == S_OK)
			_EMT(Start,adtInt(iIdx));
		else
			_EMT(Error,adtInt(hr));
		}	// if
/*
	// Grab a single frame Necessary any more with nSpace ? Just grab one from the 'onImage' storage
	else if (_RCP(Grab))
		{
		adtInt	iTo(v);
		adtValue vL;

		// State check
		CCLTRYE( bRun == true, ERROR_INVALID_STATE );

		// Reset/wait for event
		CCLOK ( evGrab.reset(); )
		CCLTRYE ( evGrab.wait ( (iTo > 0) ? iTo : 1000 ) == true, ERROR_TIMEOUT );

		// Current image is grabbed image
		CCLOK ( _EMT(Grab,adtIUnknown(pDctImg)); )
		}	// else if
*/
	// Stop capture
	else if (_RCP(Stop))
		{
		// State check
		CCLTRYE( bRun == true, ERROR_INVALID_STATE );

		// Shutdown camera acquisition
		CCLOK ( bRun = false; )
		CCLOK ( cam.StopCapture(); )
		CCLOK ( cam.Disconnect(); )
		}	// else if

	// Information about current camera
	else if (_RCP(Info))
		{
		IDictionary					*pInfo	= NULL;
		bool							bConn		= false;
		FlyCapture2::BusManager	bus;
		FlyCapture2::PGRGuid		id;
		FlyCapture2::CameraInfo	info;

		// Valid camera index ?
		CCLTRYE( iIdx >= 0, ERROR_INVALID_STATE );

		// Information dictionary
		CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pInfo ) );

		// Retrieve the Id for the camera
		CCLTRY ( pgrError ( bus.GetCameraFromIndex ( iIdx, &id ) ) );

		// Attempt connection to the camera
		CCLTRY ( pgrError ( cam.Connect ( &id ) ) );
		CCLOK  ( bConn = true; )

		// Retrieve camera information
		CCLTRY ( pgrError ( cam.GetCameraInfo ( &info ) ) );

		// Debug
		if (hr == S_OK)
			{
			dbgprintf ( L"PtGrey::Camera:Camera %d\r\n", iIdx );
			dbgprintf ( L"Serial number : 0x%x\r\n", info.serialNumber );
			dbgprintf ( L"Camera model  : %S\r\n", info.modelName ); 
			dbgprintf ( L"Camera vendor : %S\r\n", info.vendorName );
			dbgprintf ( L"Sensor        : %S\r\n", info.sensorInfo );
			dbgprintf ( L"Resolution    : %S\r\n", info.sensorResolution );
			dbgprintf ( L"Firmware ver  : %S\r\n", info.firmwareVersion );
			dbgprintf ( L"Firmware time : %S\r\n", info.firmwareBuildTime );
			}	// if

		// Disconnect from camera
		if (bConn)
			cam.Disconnect();

		// Camera information dictionary
		if (hr == S_OK)
			{
			// Fill context for camera
			CCLTRY ( pInfo->store ( adtString(L"Index"),				adtInt(iIdx) ) );
			CCLTRY ( pInfo->store ( adtString(L"SerialNumber"),	adtInt(info.serialNumber) ) );
			CCLTRY ( pInfo->store ( adtString(L"Model"),				adtString(info.modelName) ) );
			CCLTRY ( pInfo->store ( adtString(L"Vendor"),			adtString(info.vendorName) ) );
			CCLTRY ( pInfo->store ( adtString(L"Sensor"),			adtString(info.sensorInfo) ) );
			CCLTRY ( pInfo->store ( adtString(L"Resolution"),		adtString(info.sensorResolution) ) );
			CCLTRY ( pInfo->store ( adtString(L"FirmwareRev"),		adtString(info.firmwareVersion) ) );
			CCLTRY ( pInfo->store ( adtString(L"FirmwareDate"),	adtString(info.firmwareBuildTime) ) );
			}	// if

		// Result
		if (hr == S_OK)
			_EMT(Info,adtIUnknown(pInfo));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pInfo);
		}	// if

	// Camera index
	else if (_RCP(Index))
		iIdx = adtInt(v);
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive
