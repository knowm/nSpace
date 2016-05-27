////////////////////////////////////////////////////////////////////////
//
//									PREPARE.CPP
//
//				Implementation of the image preparation node.
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"
#include <stdio.h>

// Globals
static bool	bGPUInit = false;							// GPU detectiion/initialization has occured

Prepare :: Prepare ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pImg		= NULL;
	bCuda		= false;
	bOcl		= false;
	}	// Prepare

HRESULT Prepare :: gpuInit ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Initialize any GPU support on the local machine.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////

	// Already initialized ?
	if (bGPUInit)
		return S_OK;

	#if	CV_MAJOR_VERSION == 3
	// Open CV 3.X has automatic OpenCL support via the new "UMat" object
	if (cv::ocl::haveOpenCL())
		cv::ocl::setUseOpenCL(false);
	#endif

	bGPUInit = true;

	return S_OK;
	}	// gpuInit

HRESULT Prepare :: onAttach ( bool bAttach )
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
//		if (pnDesc->load ( adtString(L"ZeroDC"), vL ) == S_OK)
//			bZeroDC = adtBool(vL);
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pImg);
		}	// else

	return hr;
	}	// onAttach

HRESULT Prepare :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

	// Upload
	if (_RCP(Upload))
		{
		IDictionary	*pImgUse = pImg;
		cvMatRef		*pMat		= NULL;

		// Image to use
		if (pImgUse == NULL)
			{
			adtIUnknown unkV(v);
			CCLTRY(_QISAFE(unkV,IID_IDictionary,&pImgUse));
			}	// if
		else
			pImgUse->AddRef();

		// State check
		CCLTRYE ( pImgUse != NULL, ERROR_INVALID_STATE );

		// Since nothing in the image library can be done without 'uploading' an
		// image first, initialize GPU support here
		if (hr == S_OK && !bGPUInit)
			gpuInit();

		// Wrap image bits in OpenCv matrix
		CCLTRYE( (pMat = new cvMatRef()) != NULL, E_OUTOFMEMORY );
		CCLTRY ( image_to_mat ( pImgUse, &(pMat->mat) ) );

		// Since uploading to a GPU involves a copy, 'own' the pixel
		// data for self in case the download goes back into the same image bits.
		CCLOK ( *(pMat->mat) = pMat->mat->clone(); )

		// Store 'uploaded' image in image dictionary
		CCLTRY ( pImgUse->store (	adtString(L"cvMatRef"), adtLong((U64)pMat) ) );

		// Result
		if (hr == S_OK)
			_EMT(Upload,adtIUnknown(pImgUse));
		else
			{
			lprintf ( LOG_ERR, L"Unable to prepare image %d", hr );
			_EMT(Error,adtInt(hr));
			}	// else

		// Clean up
		_RELEASE(pImgUse);
		}	// if

	// Download
	else if (_RCP(Download))
		{
		IDictionary	*pImgUse = pImg;
		cvMatRef		*pMat		= NULL;
		adtValue		vL;

		// Image to use
		if (pImgUse == NULL)
			{
			adtIUnknown unkV(v);
			CCLTRY(_QISAFE(unkV,IID_IDictionary,&pImgUse));
			}	// if
		else
			pImgUse->AddRef();

		// State check
		CCLTRYE ( pImgUse != NULL, ERROR_INVALID_STATE );

		// Image must be 'uploaded'
		CCLTRY ( pImgUse->load (	adtString(L"cvMatRef"), vL ) );
		CCLTRYE( (pMat = (cvMatRef *)(U64)adtLong(vL)) != NULL,
					ERROR_INVALID_STATE );

		// Store resulting image in dictionary
		CCLTRY ( image_from_mat ( pMat->mat, pImgUse ) );

		// Result
		if (hr == S_OK)
			_EMT(Download,adtIUnknown(pImgUse));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pImgUse);
		}	// else if

	// Release
	else if (_RCP(Release))
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

		// Clean up
		if (pImgUse != NULL)
			pImgUse->remove ( adtString(L"cvMatRef") );

		// Result
		if (hr == S_OK)
			_EMT(Release,adtIUnknown(pImgUse));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pImgUse);
		}	// else if

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


		/*
		// GPU
		if (!bGPUInit)
			{
			cvMatRef matTst ( 1, 1, CV_8UC1 );

			// One time only
			bGPUInit = true;
			bCuda		= false;
			bOcl		= false;

			//
			// For some reason the 'getOpenCLPlatforms', etc
			// crashes with invalid data.  However creating an
			// OpenCL matrix works so just use that as a test if
			// OpenCL and Cuda is enabled.
			//

			// Open CL
			try
				{
				// Attempt to upload matrix
				cv::ocl::oclMat	matUp;
				matUp.upload ( matTst );

				// If here, success
				bOcl = true;
				lprintf ( LOG_INFO, L"OpenCl acceleration enabled\r\n" );
				}	// try
			catch ( cv::Exception & )
				{
				lprintf ( LOG_INFO, L"OpenCl acceleration NOT available\r\n" );
				}	// catch

			// Cuda
			try
				{
				// Attempt to upload matrix
				cv::gpu::GpuMat	matUp;
				matUp.upload ( matTst );

				// If here, success
				bCuda = true;
				lprintf ( LOG_INFO, L"Cuda acceleration enabled\r\n" );
				}	// try
			catch ( cv::Exception & )
				{
				lprintf ( LOG_INFO, L"Cuda acceleration NOT available\r\n" );
				}	// catch

			}	// if

		// DEBUG.  Until a valid test environment is available.
		bOcl = bCuda = false;

		// Wrap image bits in OpenCv matrix
		CCLTRY ( image_to_mat ( pImgUse, &pMat ) );

		// Upload into GPU if enabled
		if (hr == S_OK)
			{
			// Open Cl
			if (hr == S_OK && bOcl)
				{
				cv::ocl::oclMat	*pmatUp = NULL;

				// Create new GPU based matrix
				CCLTRYE ( (pmatUp = new cv::ocl::oclMat()) 
								!= NULL, E_OUTOFMEMORY );

				// Upload image data for future processing
				CCLOK ( pmatUp->upload ( *pMat ); )

				// Using uploaded version
				if (hr == S_OK)
					{
					// Free old one
					delete pMat;
					pMat = NULL;

					// Store new one
					CCLTRY ( pImgUse->store (	adtString(L"cv::ocl::oclMat"), 
														adtLong((U64)pmatUp) ) );
					}	// if
				}	// if

			// CUDA: ToDo

			// CPU
			else
				{
				// Will stay in CPU mode
				CCLTRY ( pImgUse->store (	adtString(L"cvMatRef"), 
													adtLong((U64)pMat) ) );
				}	// else
			}	// if
		*/

		/*
		// OpenCl
		if (hr == S_OK && pImgUse->load ( adtString(L"cv::ocl::oclMat"), vL ) == S_OK)
			{
			cv::ocl::oclMat	*pMat = NULL;
			cvMatRef				mat;

			// Extract matrix
			CCLTRYE ( (pMat = (cv::ocl::oclMat *)(U64)adtLong(vL)) != NULL, E_INVALIDARG );

			// Copy to memory
			CCLOK ( pMat->download ( mat ); )

			// Convert to dictionary
			CCLTRY ( image_from_mat ( &mat, pImgUse ) );

			// Clean up
			if (pMat != NULL)
				delete pMat;
			pImgUse->remove ( adtString(L"cv::ocl::oclMat") );
			}	// if

		// CPU
		else if (hr == S_OK)
			{
			// Valid OpenCv matrix ?
			CCLTRY ( pImgUse->load ( adtString(L"cvMatRef"), vL ) );
			CCLTRYE( (pMat = (cvMatRef *)(U64)adtLong(vL)) != NULL, E_UNEXPECTED );

			// Store resulting image in dictionary
			CCLTRY ( image_from_mat ( pMat, pImgUse ) );

			// Clean up
			if (pMat != NULL)
				delete pMat;
			pImgUse->remove ( adtString(L"cvMatRef") );
			}	// else if
		*/
