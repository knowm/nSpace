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
bool	bCuda				= false;							// CUDA enabled
bool	bUMat				= false;							// UMat/OpenCL enabled

Prepare :: Prepare ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pImg		= NULL;
	}	// Prepare

HRESULT Prepare :: extract (	IDictionary *pDct, const ADTVALUE &vAlt,
										IDictionary **ppDct, cvMatRef **ppMat )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Prepare image dictionary ptr and optionally the embedded
	//			cvMatRef object.
	//
	//	PARAMETERS
	//		-	pDct is the default image dictionary to use
	//		-	vAlt is checked for a dictionary if 'pDct' is NULL.
	//		-	ppDct will receive the image dictionary
	//		-	ppMat (if non-NULL) will receive the embedded cvMatRef object
	//			inside the dictionary
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT hr = S_OK;

	// Image dictionary to use
	if ( (*ppDct = pDct) == NULL )
		{
		adtIUnknown unkV(vAlt);
		CCLTRY(_QISAFE(unkV,IID_IDictionary,ppDct));
		}	// if
	else
		(*ppDct)->AddRef();

	// Caller want matrix object ?
	if (hr == S_OK && ppMat != NULL)
		{
		adtValue vL;

		// Reference counted object
		CCLTRY ( (*ppDct)->load (	adtString(L"cvMatRef"), vL ) );
		CCLTRYE( (*ppMat = (cvMatRef *)(IUnknown *)adtIUnknown(vL)) != NULL,
					ERROR_INVALID_STATE );
		CCLOK  ( (*ppMat)->AddRef(); )
		}	// if

	return hr;
	}	// extract

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
	int	ret;

	// Already initialized ?
	if (bGPUInit)
		return S_OK;

	// Any CUDA-enabled devices ?
	bCuda = false;
	if ((ret = cv::cuda::getCudaEnabledDeviceCount()) > 0)
		{
		// Perform some sort of GPU operation to intiailize Cuda for the
		// first time which can take while (many seconds on a fresh system)
		cv::cuda::GpuMat	gpumat;
		cv::Mat				mat(10,10,CV_8UC1);
		gpumat.upload(mat);

		// Enable Cuda
		lprintf ( LOG_INFO, L"Cuda enabled devices : %d\r\n", ret );
		bCuda = true;

		// For debug to force CPU mode
//		bCuda = false;

		// Do not use OpenCL
		bUMat = false;
		cv::ocl::setUseOpenCL(false);
		}	// if
	else
		{
		lprintf ( LOG_INFO, L"No Cuda enabled devices\r\n" );

		// Open CV 3.X has automatic OpenCL support via the new "UMat" object
		// UMat/OpenCL support currently disabled due to missing UMat function
		// in library.
		bUMat = false;
		if (cv::ocl::haveOpenCL())
			{
			// Do not use OpenCL until there is a chance to test
			lprintf ( LOG_INFO, L"OpenCL enabled device detected\r\n" );
			cv::ocl::setUseOpenCL(false);
//			cv::ocl::setUseOpenCL(true);
//			bUMat = true;
			}	// if

		}	// else

	// GPU initialized
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
		IDictionary	*pImgUse = NULL;
		cvMatRef		*pMat		= NULL;
		cv::Mat		*mat		= NULL;

		// Obtain image refence
		CCLTRY ( extract ( pImg, v, &pImgUse, NULL ) );

		// Since nothing in the image library can be done without 'uploading' an
		// image first, initialize any GPU support here
		if (hr == S_OK && !bGPUInit)
			gpuInit();

		// Wrap image bits in OpenCv matrix
		CCLTRY ( image_to_mat ( pImgUse, &mat ) );

		// Create a blank matrix type
		CCLTRY ( Create::create ( pImgUse, mat->cols, mat->rows, mat->type(), &pMat ) );

		// GPU
		if (hr == S_OK && pMat->isGPU())
			pMat->gpumat->upload ( *mat );

		// OCL
		else if (hr == S_OK && pMat->isUMat())
			mat->copyTo ( *(pMat->umat) );

		// CPU
		else if (hr == S_OK)
			{
			// Since uploading to a GPU involves a copy, 'own' the pixel
			// data for CPU in case the download goes back into the same image bits.
			*(pMat->mat) = mat->clone();
			}	// else

		// Store 'uploaded' image in image dictionary
		CCLTRY ( pImgUse->store (	adtString(L"cvMatRef"), adtIUnknown(pMat) ) );

		// Result
		if (hr == S_OK)
			_EMT(Upload,adtIUnknown(pImgUse));
		else
			{
			lprintf ( LOG_ERR, L"Unable to prepare image %d", hr );
			_EMT(Error,adtInt(hr));
			}	// else

		// Clean up
		if (mat != NULL)
			delete mat;
		_RELEASE(pMat);
		_RELEASE(pImgUse);
		}	// if

	// Download
	else if (_RCP(Download))
		{
		IDictionary	*pImgUse = NULL;
		cvMatRef		*pMat		= NULL;
		cv::Mat		mat;

		// Obtain image refence
		CCLTRY ( extract ( pImg, v, &pImgUse, &pMat ) );

		// Download into local matri if GPU is enabled
		if (pMat->isGPU())
			pMat->gpumat->download(mat);
		else if (pMat->isUMat())
			mat = pMat->umat->getMat(cv::ACCESS_READ);
		else
			mat = *(pMat->mat);

		// Store resulting image in dictionary
		CCLTRY ( image_from_mat ( &mat, pImgUse ) );

		// Ensure any matrix object is removed
		if (pImgUse != NULL)
			pImgUse->remove ( adtString(L"cvMatRef") );

		// Result
		if (hr == S_OK)
			_EMT(Download,adtIUnknown(pImgUse));
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
