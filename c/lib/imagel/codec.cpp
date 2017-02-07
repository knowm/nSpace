////////////////////////////////////////////////////////////////////////
//
//									Codec.CPP
//
//				Implementation of the image encoding/decoding node.
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"
#include <stdio.h>

// Globals

Codec :: Codec ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pDct		= NULL;
	strType	= L"";
	}	// Codec

HRESULT Codec :: onAttach ( bool bAttach )
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
		if (pnDesc->load ( adtString(L"Type"), vL ) == S_OK)
			adtValue::toString ( vL, strType );

		// Internal objects
		CCLTRY ( jpeg.construct() );
		CCLTRY ( png.construct() );
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pDct);
		}	// else

	return hr;
	}	// onAttach

HRESULT Codec :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Encode
	if (_RCP(Encode))
		{
		// State check
		CCLTRYE ( (pDct != NULL), ERROR_INVALID_STATE );
		CCLTRYE ( strType.length() > 0, ERROR_INVALID_STATE );

		// Encode/compress based on type
		if (hr == S_OK)
			{
			// JPEG library
			if (!WCASECMP(strType,L"JPEG") || !WCASECMP(strType,L"JPG"))
				hr = jpeg.compress(pDct);
			else
				hr = E_NOTIMPL;
			}	// if

		// Result
		if (hr == S_OK)
			_EMT(Encode,adtIUnknown(pDct));
		else
			_EMT(Error,adtInt(hr));
		}	// if

	// Decode
	else if (_RCP(Decode))
		{
		adtValue		vL;
		adtString	strFmt;

		// State check
		CCLTRYE ( (pDct != NULL), ERROR_INVALID_STATE );

		// Extract format of provided image
		CCLTRY ( pDct->load ( adtString(L"Format"), vL ) );
		CCLTRYE( (strFmt = vL).length() > 0, E_UNEXPECTED );

		// Run appropriate decoder
		if (hr == S_OK && (!WCASECMP(strFmt,L"JPEG") || !WCASECMP(strFmt,L"JPG")))
			hr = jpeg.decompress(pDct);
		else if (hr == S_OK && !WCASECMP(strFmt,L"PNG"))
			hr = png.decompress(pDct);
		else
			{
			hr = E_NOTIMPL;
			lprintf ( LOG_WARN, L"Unable to decode format %s\r\n", (LPCWSTR)strFmt );
			}	// else

		// Result
		if (hr == S_OK)
			_EMT(Decode,adtIUnknown(pDct));
		else
			_EMT(Error,adtInt(hr));
		}	// else if

	// State
	else if (_RCP(Image))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pDct);
		_QISAFE(unkV,IID_IDictionary,&pDct);
		}	// else if
	else if (_RCP(Type))
		hr = adtValue::toString ( v, strType );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

