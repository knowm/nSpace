////////////////////////////////////////////////////////////////////////
//
//									CREATE.CPP
//
//				Implementation of the image creation node.
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"
#include <stdio.h>

// Globals
extern bool	bCuda;									// CUDA enabled
extern bool	bUMat;									// UMat/OpenCL enabled

Create :: Create ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pImg		= NULL;
	strFmt	= L"U8X2";
	iW			= 128;
	iH			= 128;
	bCPU		= false;
	strType	= L"";
	}	// Create

HRESULT Create :: create ( IDictionary *pDct, U32 w, U32 h, U32 f,
									cvMatRef **ppMat, bool bCpu )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Create an appropiate matrix reference object for the image.
	//
	//	PARAMETERS
	//		-	pDct is an optional dictionary to receive the information
	//		-	w,h are the dimensions of the image
	//		-	f is the OpenCV format
	//		-	ppMat is optional and will receive the referenced matrix object.
	//		-	bCpu is true to create a CPU bound cv::Mat regardless of
	//			the current GPU settings.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr		= S_OK;
	cvMatRef	*pMat	= NULL;

	// Ensure GPU initialization has taken place
	CCLOK ( Prepare::gpuInit(); )

	// Create a matrix based on the GPU mode
	CCLTRYE( (pMat = new cvMatRef()) != NULL, E_OUTOFMEMORY );
	if (!bCpu && bUMat)
		{
		CCLTRYE ( (pMat->umat = new cv::UMat ( h, w, f )) != NULL,
						E_OUTOFMEMORY );
		CCLOK ( pMat->umat->setTo ( cv::Scalar(0) ); )
		}	// else if
	#ifdef	WITH_CUDA
	else if (!bCpu && bCuda)
		{
		CCLTRYE ( (pMat->gpumat = new cv::cuda::GpuMat ( h, w, f )) != NULL,
						E_OUTOFMEMORY );
		CCLOK ( pMat->gpumat->setTo ( cv::Scalar(0) ); )
		}	// if
	#endif
	else
		{
		CCLTRYE ( (pMat->mat = new cv::Mat ( h, w, f )) != NULL,
						E_OUTOFMEMORY );
		CCLOK ( pMat->mat->setTo ( cv::Scalar(0) ); )
		}	// else

	// Store image in image dictionary
	if (hr == S_OK && pDct != NULL)
		hr = pDct->store ( adtString(L"cvMatRef"), adtIUnknown(pMat) );

	// Result
	if (hr == S_OK && ppMat != NULL)
		{
		(*ppMat) = pMat;
		_ADDREF((*ppMat));
		}	// if

	// Clean up
	_RELEASE(pMat);
	return hr;
	}	// create

HRESULT Create :: onAttach ( bool bAttach )
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
		if (pnDesc->load ( adtString(L"Format"), vL ) == S_OK)
			hr = adtValue::toString ( vL, strFmt );
		if (pnDesc->load ( adtString(L"Width"), vL ) == S_OK)
			iW = vL;
		if (pnDesc->load ( adtString(L"Height"), vL ) == S_OK)
			iH = vL;
		if (pnDesc->load ( adtString(L"CPU"), vL ) == S_OK)
			bCPU = vL;
		if (pnDesc->load ( adtString(L"Type"), vL ) == S_OK)
			hr = adtValue::toString ( vL, strType );
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pImg);
		}	// else

	return hr;
	}	// onAttach

HRESULT Create :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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
		IDictionary	*pImgUse = NULL;
		cvMatRef		*pMat		= NULL;
		U32			cvFmt;

		// Obtain image refence
		CCLTRY ( Prepare::extract ( pImg, v, &pImgUse, NULL ) );

		// Process
		// A 'type' can be specified if a certain type of matrix is to
		// be created, other a blank image is created.
		if (hr == S_OK)
			{
			// Map requested format into OpenCV format
			if (!WCASECMP(strFmt,L"F32x2"))
				cvFmt = CV_32FC1;
			else if (!WCASECMP(strFmt,L"U16x2"))
				cvFmt = CV_16UC1;
			else if (!WCASECMP(strFmt,L"S16x2"))
				cvFmt = CV_16SC1;
			else if (!WCASECMP(strFmt,L"U8x2"))
				cvFmt = CV_8UC1;
			else if (!WCASECMP(strFmt,L"S8x2"))
				cvFmt = CV_8SC1;
			else
				cvFmt = CV_8UC1;

			// Create a blank matrix based on the GPU mode
			CCLTRY ( create ( pImgUse, iW, iH, cvFmt, &pMat, bCPU ) );

			// Gaussian kernel
			if (!WCASECMP(strType,L"Gaussian"))
				{
				cv::Mat	matK;

				// Create a CPU bound kernel
				CCLOK ( matK = cv::getGaussianKernel(iW,0,cvFmt); )

				// Gaussian kernel gives 1D version, create 2D version
				CCLOK ( cv::mulTransposed ( matK, matK, false ); )

				// Copy to blank image
				if (pMat->isUMat())
					matK.copyTo ( *(pMat->umat) );
				#ifdef	WITH_CUDA
				else if (pMat->isGPU())
					pMat->gpumat->upload ( matK );
				#endif
				else
					matK.copyTo ( *(pMat->mat) );
				}	// if

			}	// if

		// Result
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pImgUse));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pMat);
		_RELEASE(pImgUse);
		}	// else if

	// State
	else if (_RCP(Image))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pImg);
		_QISAFE(unkV,IID_IDictionary,&pImg);
		}	// else if
	else if (_RCP(Format))
		adtValue::toString ( v, strFmt );
	else if (_RCP(Width))
		iW = v;
	else if (_RCP(Height))
		iH = v;
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

