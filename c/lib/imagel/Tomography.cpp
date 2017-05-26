////////////////////////////////////////////////////////////////////////
//
//									TOMOGRAPHY.CPP
//
//				Implementation of the image tomography node.
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"
#include <stdio.h>

// Globals

Tomography :: Tomography ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pImg		= NULL;
	pImgX		= NULL;
	pImgY		= NULL;
	pImgPts	= NULL;
	}	// Tomography

HRESULT Tomography :: onAttach ( bool bAttach )
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
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pImg);
		_RELEASE(pImgX);
		_RELEASE(pImgY);
		_RELEASE(pImgPts);
		}	// else

	return hr;
	}	// onAttach

HRESULT Tomography :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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
		IDictionary	*pImgUse		= NULL;
		IDictionary	*pImgUseX	= NULL;
		IDictionary	*pImgUseY	= NULL;
		cvMatRef		*pMat			= NULL;
		cvMatRef		*pMatX		= NULL;
		cvMatRef		*pMatY		= NULL;
		cvMatRef		*pMatPts		= NULL;
		U32			npts			= 0;
		float			fX,fY,fZ,fI;

		// State check
		CCLTRYE ( pImgX != NULL && pImgY != NULL && pImgPts != NULL, ERROR_INVALID_STATE );

		// Obtain image refences
		CCLTRY ( Prepare::extract ( pImg, v, &pImgUse, &pMat ) );
		CCLTRY ( Prepare::extract ( pImgX, adtInt(), &pImgUseX, &pMatX ) );
		CCLTRY ( Prepare::extract ( pImgY, adtInt(), &pImgUseY, &pMatY ) );

		// Lots of inidividual access, so must be local to CPU.
		CCLTRYE ( (pMat->isMat() && pMatX->isMat() && pMatY->isMat()),
						ERROR_INVALID_STATE );

		// Current just works with floating point values.
		CCLTRYE (	(pMat->type()		== CV_32FC1) &&
						(pMatX->type()		== CV_32FC1) &&
						(pMatY->type()		== CV_32FC1),
						ERROR_INVALID_STATE );

		// There needs to be a coordinate for each column in the image
		CCLTRYE (	(pMatX->rows() >= pMat->cols()) &&
						(pMatY->rows() >= pMat->cols()), ERROR_INVALID_STATE );

		//
		// Generate a list of 3D points from the provided images.
		//
		// Assumptions : 
		//	- pImgX,pImgy are single columns values that
		//		contain the X/Y coordinates of each row of the main image.
		// - pImg contains the tomography data with X/Y information along
		//		the columns, and Z information along the rows.
		//	- pImgPts will receive the list of 4 tuples for each point that
		//		contains the X,Y,Z position and the intensity.
		//

		// The number of non-zero pixels will be the number of points
		CCLTRYE ( (npts = cv::countNonZero ( *(pMat->mat) )) > 0, E_UNEXPECTED );

		// Create pre-sized destination, 4 columns (X,Y,Z,I) by number of pts.
		CCLTRYE( (pMatPts = new cvMatRef()) != NULL, E_OUTOFMEMORY );
		CCLTRYE( (pMatPts->mat = new cv::Mat ( npts, 4, CV_32FC1 )) != NULL, E_OUTOFMEMORY );

		// Scan each row of source image
		for (U32 r = 0,idx = 0;hr == S_OK && r < (U32)pMat->rows();++r)
			{
			// Check for non-zero pixels in columns
			for (U32 c = 0;hr == S_OK && c < (U32)pMat->cols();++c)
				{
				// Intensity value
				fI = pMat->mat->at<float>(r,c);
				if (fI <= 0)
					continue;

				// Non-zero pixel.  Use provided X and Y values as X,Y coordinates
				// and the current row as the Z-position
				fX = pMatX->mat->at<float>(c,0);
				fY = pMatY->mat->at<float>(c,0);
				fZ = (float)r;

				// Write values to destination
//				if (idx < 10)
//					lprintf ( LOG_DBG, L"%g %g %g %g\r\n", fX, fY, fZ, fI );
				pMatPts->mat->at<float>(idx,0) = fX;
				pMatPts->mat->at<float>(idx,1) = fY;
				pMatPts->mat->at<float>(idx,2) = fZ;
				pMatPts->mat->at<float>(idx,3) = fI;
				++idx;
				}	// for

			}	// for
	
		// Result
		CCLTRY ( pImgPts->store ( adtString(L"cvMatRef"), adtIUnknown(pMatPts) ) );
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pImgPts));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pMatPts);
		_RELEASE(pMatY);
		_RELEASE(pMatX);
		_RELEASE(pMat);
		_RELEASE(pImgUseY);
		_RELEASE(pImgUseX);
		_RELEASE(pImgUse);
		}	// if

	// State
	else if (_RCP(Image))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pImg);
		_QISAFE(unkV,IID_IDictionary,&pImg);
		}	// else if
	else if (_RCP(ImageX))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pImgX);
		_QISAFE(unkV,IID_IDictionary,&pImgX);
		}	// else if
	else if (_RCP(ImageY))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pImgY);
		_QISAFE(unkV,IID_IDictionary,&pImgY);
		}	// else if
	else if (_RCP(Points))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pImgPts);
		_QISAFE(unkV,IID_IDictionary,&pImgPts);
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

