////////////////////////////////////////////////////////////////////////
//
//									UUID.CPP
//
//					Implementation of the UUID node
//
////////////////////////////////////////////////////////////////////////

#include "miscl_.h"
#if		defined(__unix__) || defined(__APPLE__)
#include	<uuid/uuid.h>
#endif

UUIDOp :: UUIDOp ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	}	// UUIDOp

HRESULT UUIDOp :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Fire
	if (_RCP(Fire))
		{
		WCHAR		*psuuid	= NULL;
		GUID		uuid		= GUID_NULL;

		// Handle platform differences...
		#if	(defined(_WIN32) && !defined(UNDER_CE)) || (UNDER_CE >= 400)

		// Standard API
		CCLTRY(CoCreateGuid(&uuid));

		// Work around for Windows CE 3.0 (does not support 'CoCreateGuid')
		#elif defined(UNDER_CE) && (UNDER_CE < 400)
		U8				cID[255];
		DEVICE_ID	*pdid = (DEVICE_ID *) cID;
		DWORD			dwOut;

		// Internal method
		CCLTRYE ( KernelIoControl ( IOCTL_HAL_GET_DEVICEID, 0, 0,
						cID, sizeof(cID), &dwOut ) == TRUE, GetLastError() );

		// So far sizeof GUID == sizeof ID
		if (hr == S_OK && (dwOut-sizeof(DEVICE_ID)) == sizeof(uuid))
			{
			// Copy bytes
			memcpy ( &uuid, &(cID[sizeof(cID)]), sizeof(uuid) );

			// This node is to create a unique ID over time as well so
			// 'randomize' part of the ID using local 'random' number.
			uuid.Data1 = GetTickCount();
			}	// if
		#elif		defined(__unix__) || defined(__APPLE__)
		{
		uuid_t	uuuid;
		char		*up = (char *) &(uuuid);
//		uint32_t	stat;

		// Generate a UUID
		uuid_generate ( uuuid );
//		uuid_create ( &uuuid, &stat );

		// Convert to Win32 UUID (byte order ?)
//		uuid.Data1	= *((U32 *)&(uuuid[0]));
//		uuid.Data2	= *((U16 *)&(uuuid[4]));
//		uuid.Data3	= *((U16 *)&(uuuid[6]));
//		memcpy ( uuid.Data4, &(uuuid[8]), 8 );

		// Convert to Win32 UUID (byte order ?)
		uuid.Data1	= *((U32 *)&(up[0]));
		uuid.Data2	= *((U16 *)&(up[4]));
		uuid.Data3	= *((U16 *)&(up[6]));
		memcpy ( uuid.Data4, &(up[8]), 8 );
		}
		#else
		#error	"Unsupported platform"
		#endif

		// Emit string version of UUID
		CCLTRY(StringFromCLSID(uuid,&psuuid));
		CCLOK (_EMT(Fire,adtString ( psuuid ) );)

		// Clean up
		_FREEMEM(psuuid);
		}	// if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive


