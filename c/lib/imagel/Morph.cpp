////////////////////////////////////////////////////////////////////////
//
//									MORPH.CPP
//
//				Implementation of the image morphology node.
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"
#include <stdio.h>

// Globals

Morph :: Morph ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pImg		= NULL;
	#ifdef	HAVE_OPENCV_CUDA
	pfOpen	= NULL;
	pfClose	= NULL;
	pfDi		= NULL;
	pfEr		= NULL;
	#endif
//	pKer	= NULL;
	}	// Morph

HRESULT Morph :: onAttach ( bool bAttach )
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
		if (pnDesc->load ( adtString(L"Type"), vL ) == S_OK)
			hr = adtValue::toString ( vL, strType );
		}	// if

	// Detach
	else
		{
		// Clean up
		#ifdef	HAVE_OPENCV_CUDA
		if (pfOpen != NULL)
			{
			delete pfOpen;
			pfOpen = NULL;
			}	// if
		if (pfClose != NULL)
			{
			delete pfClose;
			pfClose = NULL;
			}	// if
		if (pfEr != NULL)
			{
			delete pfEr;
			pfEr = NULL;
			}	// if
		if (pfDi != NULL)
			{
			delete pfDi;
			pfDi = NULL;
			}	// if
		#endif

		// Shutdown
		_RELEASE(pImg);
//		_RELEASE(pKer);
		}	// else

	return hr;
	}	// onAttach

HRESULT Morph :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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
		cv::Mat		matKer;
		int			op;

		// Obtain image refence
		CCLTRY ( Prepare::extract ( pImg, v, &pImgUse, &pMat ) );

		// TODO: Allow kernel image to be specified, for now use default
		CCLOK ( matKer = cv::getStructuringElement ( cv::MORPH_RECT, cv::Size(3,3) ); )

		// Operation
		if (hr == S_OK)
			{
			op =	(!WCASECMP(strType,L"Open")) ?	cv::MORPH_OPEN :
					(!WCASECMP(strType,L"Close")) ?	cv::MORPH_CLOSE :
					(!WCASECMP(strType,L"Erode")) ?	cv::MORPH_ERODE :
					(!WCASECMP(strType,L"Dilate")) ? cv::MORPH_DILATE : -1;
			if (op == -1)
				hr = E_NOTIMPL;
			}	// if

		// Perform operation
		if (hr == S_OK)
			{
			if (pMat->isMat())
				cv::morphologyEx ( *(pMat->mat), *(pMat->mat), op, matKer );
			#ifdef	HAVE_OPENCV_UMAT
			else if (pMat->isUMat())
				cv::morphologyEx ( *(pMat->umat), *(pMat->umat), op, matKer );
			#endif
			#ifdef	HAVE_OPENCV_CUDA
			else if (pMat->isGPU())
				{
				// Need filters ?
				if (pfOpen == NULL)
					{
					// Create filter objects
					CCLTRYE ( (pfOpen = cv::cuda::createMorphologyFilter ( cv::MORPH_OPEN,
									pMat->gpumat->type(), matKer )) != NULL, E_OUTOFMEMORY );
					CCLTRYE ( (pfClose = cv::cuda::createMorphologyFilter ( cv::MORPH_CLOSE,
									pMat->gpumat->type(), matKer )) != NULL, E_OUTOFMEMORY );
					CCLTRYE ( (pfEr = cv::cuda::createMorphologyFilter ( cv::MORPH_ERODE,
									pMat->gpumat->type(), matKer )) != NULL, E_OUTOFMEMORY );
					CCLTRYE ( (pfDi = cv::cuda::createMorphologyFilter ( cv::MORPH_DILATE,
									pMat->gpumat->type(), matKer )) != NULL, E_OUTOFMEMORY );
					}	// if

				// Execute filter
				switch (op)
					{
					case cv::MORPH_OPEN :
						pfOpen->apply ( *(pMat->gpumat), *(pMat->gpumat) );
						break;
					case cv::MORPH_CLOSE :
						pfClose->apply ( *(pMat->gpumat), *(pMat->gpumat) );
						break;
					case cv::MORPH_ERODE :
						pfEr->apply ( *(pMat->gpumat), *(pMat->gpumat) );
						break;
					case cv::MORPH_DILATE :
						pfDi->apply ( *(pMat->gpumat), *(pMat->gpumat) );
						break;
					default :
						hr = E_NOTIMPL;
					}	// switch
				}	// if
			#endif
			}	// if

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
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

