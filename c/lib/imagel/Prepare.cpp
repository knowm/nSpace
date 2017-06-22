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
	pImg	= NULL;
	bCPU	= false;
	bRel	= false;
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
	static sysCS	csPrep;

	// Thread safety
	csPrep.enter();

	// Since performance is not an issue when debugging default to 
	// CPU only.  The only time to enable this in debug mode is if this 
	// specifically is being debugged.
	// NOTE: Running from the debugger is very slow to start cuda/open CL.
//	#if 0
//	#ifndef	_DEBUG
	#ifdef	_DEBUG
	bCuda		= false;
	bUMat		= false;
	bGPUInit = true;
	#endif

	// Already initialized ?
	if (!bGPUInit)
		{
		// Any CUDA-enabled devices ?
		bCuda = false;
		#ifdef	HAVE_OPENCV_CUDA
		int	ret;
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
			}	// if
		else
		#endif
			{
			// Debug
			lprintf ( LOG_INFO, L"No Cuda enabled devices\r\n" );
			bUMat = false;

			// Open CV 3.X has automatic OpenCL support via the new "UMat" object
			#ifdef	HAVE_OPENCV_UMAT
			if (cv::ocl::haveOpenCL())
				bUMat = true;

			// Debug information
			cv::ocl::Device	
			dev = cv::ocl::Device::getDefault();
			lprintf ( LOG_DBG, L"Name : %S : %S : %d.%d : %S\r\n", dev.name().c_str(), dev.vendorName().c_str(),
							dev.deviceVersionMajor(), dev.deviceVersionMinor(), dev.name().c_str() );

			// Debug
			lprintf ( LOG_INFO, L"OpenCL %s\r\n", (bUMat) ? L"enabled" : L"disabled" );

			// Perform some sort of GPU operation to intiailize Cuda for the
			// first time which can take while (many seconds on a fresh system)
			cv::ocl::setUseOpenCL(bUMat);
			if (bUMat)
				{
				cv::UMat	umat;
				cv::Mat	mat(10,10,CV_8UC1);
				mat.copyTo ( umat );
				}	// if
			#endif

			}	// else

		// GPU initialized
		bGPUInit = true;
		}	// if

	// Thread safety
	csPrep.leave();

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
		if (pnDesc->load ( adtString(L"CPU"), vL ) == S_OK)
			bCPU = vL;
		if (pnDesc->load ( adtString(L"Release"), vL ) == S_OK)
			bRel = vL;
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pImg);
		}	// else

	return hr;
	}	// onAttach

HRESULT Prepare :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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
		CCLTRY ( Create::create ( pImgUse, mat->cols, mat->rows, mat->type(), &pMat, bCPU ) );

		// CPU
		if (hr == S_OK && pMat->isMat())
			{
			// Since uploading to a GPU involves a copy, 'own' the pixel
			// data for CPU in case the download goes back into the same image bits.
			*(pMat->mat) = mat->clone();

			// DEBUG
//			pImgUse->remove ( adtString(L"Bits") );
			}	// else

		// OCL
		#ifdef	HAVE_OPENCV_UMAT
		else if (hr == S_OK && pMat->isUMat())
			mat->copyTo ( *(pMat->umat) );
		#endif

		// GPU
		#ifdef	HAVE_OPENCV_CUDA
		else if (hr == S_OK && pMat->isGPU())
			pMat->gpumat->upload ( *mat );
		#endif

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

		// Obtain image refence
		CCLTRY ( extract ( pImg, v, &pImgUse, &pMat ) );

		// Copy image from source
		if (hr == S_OK && bRel == false)
			{
			// Store resulting image in dictionary
			hr = image_from_mat ( pMat, pImgUse );
			}	// if

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
