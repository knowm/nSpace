////////////////////////////////////////////////////////////////////////
//
//									FFT.CPP
//
//				Implementation of the image FFT node.
//
////////////////////////////////////////////////////////////////////////

#define	INITGUID
#include "imagel_.h"
#include <stdio.h>

// Globals

FFT :: FFT ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pDctImg	= NULL;
	bZeroDC	= false;
	}	// FFT

HRESULT FFT :: onAttach ( bool bAttach )
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
		if (pnDesc->load ( adtString(L"ZeroDC"), vL ) == S_OK)
			bZeroDC = adtBool(vL);
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pDctImg);
		}	// else

	return hr;
	}	// onAttach

HRESULT FFT :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

	// Execute
	if (_RCP(Fire))
		{
		IDictionary		*pDctUse = NULL;

		// Was an image previously specified ?
		CCLOK ( pDctUse = pDctImg; )
		_ADDREF(pDctUse);
		if (hr == S_OK && pDctUse == NULL)
			{
			adtIUnknown unkV(v);

			// See if an image is provided
			CCLTRY ( _QISAFE(unkV,IID_IDictionary,&pDctUse) );
			}	// if

		// Execute
		CCLTRY ( image_fft ( pDctUse, bZeroDC ) );

		// Result
//		if (hr != S_OK)
//			dbgprintf ( L"FFT::Write:hr 0x%x, I/O %d/%d\r\n", hr, uLeft, (U32)iSzIo );
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pDctUse));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pDctUse);
		}	// if

	// State
	else if (_RCP(Image))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pDctImg);
		_QISAFE(unkV,IID_IDictionary,&pDctImg);
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

