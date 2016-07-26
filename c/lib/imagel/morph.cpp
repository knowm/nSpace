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
	pImg	= NULL;
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
//		if (pnDesc->load ( adtString(L"Size"), vL ) == S_OK)
//			iSz = vL;
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pImg);
//		_RELEASE(pKer);
		}	// else

	return hr;
	}	// onAttach

HRESULT Morph :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

	// Open/close
	if (_RCP(Open) || _RCP(Close))
		{
		IDictionary	*pImgUse = NULL;
		cvMatRef		*pMat		= NULL;
		cv::Mat		matKer;

		// Obtain image refence
		CCLTRY ( Prepare::extract ( pImg, v, &pImgUse, &pMat ) );

		// TODO: Allow kernel image to be specified, for now use default
		CCLOK ( matKer = cv::getStructuringElement ( cv::MORPH_RECT, cv::Size(3,3) ); )

		// Perform operation
		if (hr == S_OK)
			{
			if (pMat->isGPU())
				{
				cv::cuda::Filter *pFilter = NULL;

				// Create filter object
				CCLTRYE ( (pFilter = cv::cuda::createMorphologyFilter (
									(_RCP(Open)) ? cv::MORPH_OPEN : cv::MORPH_CLOSE,
									pMat->gpumat->type(), matKer )) != NULL, E_OUTOFMEMORY );

				// Apply 
				CCLOK ( pFilter->apply ( *(pMat->gpumat), *(pMat->gpumat) ); )

				// Clean up
				if (pFilter != NULL)
					delete pFilter;
				}	// if
			else if (pMat->isUMat())
				cv::morphologyEx ( *(pMat->umat), *(pMat->umat), 
					(_RCP(Open)) ? cv::MORPH_OPEN : cv::MORPH_CLOSE, matKer );
			else
				cv::morphologyEx ( *(pMat->mat), *(pMat->mat), 
					(_RCP(Open)) ? cv::MORPH_OPEN : cv::MORPH_CLOSE, matKer );
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

