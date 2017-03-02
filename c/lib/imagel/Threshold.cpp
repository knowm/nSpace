////////////////////////////////////////////////////////////////////////
//
//									THRESH.CPP
//
//				Implementation of the image threshold node.
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"
#include <stdio.h>

// Globals

Threshold :: Threshold ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pImg	= NULL;
	vT		= 0;
	strOp	= L"Zero";
	}	// Threshold

HRESULT Threshold :: onAttach ( bool bAttach )
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

		// Defaults (optional)
		if (pnDesc->load ( adtString(L"Op"), vL ) == S_OK)
			adtValue::toString ( vL, strOp );
		pnDesc->load ( adtString(L"Value"), vT );
		pnDesc->load ( adtString(L"Max"), vMax );
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pImg);
		}	// else

	return hr;
	}	// onAttach

HRESULT Threshold :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

		// Obtain image refence
		CCLTRY ( Prepare::extract ( pImg, v, &pImgUse, &pMat ) );


		// Perform operation
//		CCLOK ( image_to_debug ( pMat, L"Threshold 1", L"c:/temp/thresh1.png" ); )
		if (hr == S_OK)
			{
			//
			// UMat
			//
			if (pMat->isUMat())
				{
				if (!WCASECMP(strOp,L"Zero"))
					cv::threshold ( *(pMat->umat), *(pMat->umat), adtDouble(vT), 0, cv::THRESH_TOZERO );
				else if (!WCASECMP(strOp,L"Truncate"))
					cv::threshold ( *(pMat->umat), *(pMat->umat), adtDouble(vT), 0, cv::THRESH_TRUNC );
				else if (!WCASECMP(strOp,L"Binary"))
					cv::threshold ( *(pMat->umat), *(pMat->umat), adtDouble(vT), adtDouble(vMax), cv::THRESH_BINARY );
				else if (!WCASECMP(strOp,L"BinaryInv"))
					cv::threshold ( *(pMat->umat), *(pMat->umat), adtDouble(vT), adtDouble(vMax), cv::THRESH_BINARY_INV );
				}	// else if

			//
			// GPU
			//
			#ifdef	WITH_CUDA
			else if (pMat->isGPU())
				{
				if (!WCASECMP(strOp,L"Zero"))
					cv::cuda::threshold ( *(pMat->gpumat), *(pMat->gpumat), adtDouble(vT), 0, cv::THRESH_TOZERO );
				else if (!WCASECMP(strOp,L"Truncate"))
					cv::cuda::threshold ( *(pMat->gpumat), *(pMat->gpumat), adtDouble(vT), 0, cv::THRESH_TRUNC );
				else if (!WCASECMP(strOp,L"Binary"))
					cv::cuda::threshold ( *(pMat->gpumat), *(pMat->gpumat), adtDouble(vT), adtDouble(vMax), cv::THRESH_BINARY );
				else if (!WCASECMP(strOp,L"BinaryInv"))
					cv::cuda::threshold ( *(pMat->gpumat), *(pMat->gpumat), adtDouble(vT), adtDouble(vMax), cv::THRESH_BINARY_INV );
				}	// if
			#endif

			//
			// Mat
			//
			else
				{
				if (!WCASECMP(strOp,L"Zero"))
					cv::threshold ( *(pMat->mat), *(pMat->mat), adtDouble(vT), 0, cv::THRESH_TOZERO );
				else if (!WCASECMP(strOp,L"Truncate"))
					cv::threshold ( *(pMat->mat), *(pMat->mat), adtDouble(vT), 0, cv::THRESH_TRUNC );
				else if (!WCASECMP(strOp,L"Binary"))
					cv::threshold ( *(pMat->mat), *(pMat->mat), adtDouble(vT), adtDouble(vMax), cv::THRESH_BINARY );
				else if (!WCASECMP(strOp,L"BinaryInv"))
					cv::threshold ( *(pMat->mat), *(pMat->mat), adtDouble(vT), adtDouble(vMax), cv::THRESH_BINARY_INV );
				}	// else

			}	// if

		// Debug
//		CCLOK ( image_to_debug ( pMat, L"Threshold 2", L"c:/temp/thresh2.png" ); )

		// Result
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pImgUse));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pMat);
		_RELEASE(pImgUse);
		}	// if

	// State
	else if (_RCP(Image))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pImg);
		_QISAFE(unkV,IID_IDictionary,&pImg);
		}	// else if
	else if (_RCP(Value))
		adtValue::copy ( v, vT );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

