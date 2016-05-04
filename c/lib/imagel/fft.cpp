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
	pImg	= NULL;
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
		_RELEASE(pImg);
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
		IDictionary	*pImgUse = pImg;
		adtValue		vL;

		// Image to use
		if (pImgUse == NULL)
			{
			adtIUnknown unkV(v);
			CCLTRY(_QISAFE(unkV,IID_IDictionary,&pImgUse));
			}	// if
		else
			pImgUse->AddRef();

		//
		// Perform calculation based on matrix type
		//
		/*
		// OpenCl
		if (hr == S_OK && pImgUse->load ( adtString(L"cv::ocl::oclMat"), vL ) == S_OK)
			{
			cv::ocl::oclMat	*pMat = NULL;

			// Extract matrix
			CCLTRYE ( (pMat = (cv::ocl::oclMat *)(U64)adtLong(vL)) != NULL, E_INVALIDARG );

			// Execute
			CCLTRY ( image_fft ( pMat, true, bZeroDC ) );
			}	// if

		// CPU
		else 
			{
			cv::Mat		*pMat		= NULL;

			// CPU matrix required at a minimum
			CCLTRY ( pImgUse->load ( adtString(L"cv::Mat"), vL ) );
			CCLTRYE ( (pMat = (cv::Mat *)(U64)adtLong(vL)) != NULL, E_INVALIDARG );

			// Execute
			CCLTRY ( image_fft ( pMat, true, bZeroDC ) );
			}	// else
		*/

		// Perform FFT
		CCLTRY ( image_fft ( pImgUse, bZeroDC ) );

		// Result
//		if (hr != S_OK)
//			dbgprintf ( L"FFT::Write:hr 0x%x, I/O %d/%d\r\n", hr, uLeft, (U32)iSzIo );
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pImgUse));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pImgUse);
		}	// if

	// State
	else if (_RCP(Image))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pImg);
		_QISAFE(unkV,IID_IDictionary,&pImg);
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

