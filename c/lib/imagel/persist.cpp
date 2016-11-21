////////////////////////////////////////////////////////////////////////
//
//									PersistImage.CPP
//
//				Implementation of the image PersistImageence node.
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"
#include <stdio.h>

// Globals

PersistImage :: PersistImage ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pDctImg	= NULL;
	strLoc	= L"";
	}	// PersistImage

HRESULT PersistImage :: onAttach ( bool bAttach )
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
		if (pnDesc->load ( adtString(L"Location"), vL ) == S_OK)
			adtValue::toString ( vL, strLoc );
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pDctImg);
		}	// else

	return hr;
	}	// onAttach

HRESULT PersistImage :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Load
	if (_RCP(Load))
		{
		// State check
		CCLTRYE ( (pDctImg != NULL), ERROR_INVALID_STATE );
		CCLTRYE ( strLoc.length() > 0, ERROR_INVALID_STATE );

		// Necessary ?
//		CCLOK ( strLoc.replace ( '/', '\\' ); )

		// Load image from source
		CCLTRY ( image_load ( strLoc, pDctImg ) );

		// Result
		if (hr == S_OK)
			_EMT(Load,adtIUnknown(pDctImg));
		else
			_EMT(Error,adtInt(hr));
		}	// if

	// Save
	else if (_RCP(Save))
		{
		// State check
		CCLTRYE ( (pDctImg != NULL), ERROR_INVALID_STATE );
		CCLTRYE ( strLoc.length() > 0, ERROR_INVALID_STATE );

		// Save image to destination
		CCLTRY ( image_save ( pDctImg, strLoc ) );

		// Result
		if (hr == S_OK)
			_EMT(Save,adtIUnknown(pDctImg));
		else
			_EMT(Error,adtInt(hr));
		}	// else if

	// State
	else if (_RCP(Image))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pDctImg);
		_QISAFE(unkV,IID_IDictionary,&pDctImg);
		}	// else if
	else if (_RCP(Location))
		hr = adtValue::toString ( v, strLoc );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

