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
		IDictionary	*pImgUsePts	= NULL;
		cvMatRef		*pMat			= NULL;
		cvMatRef		*pMatX		= NULL;
		cvMatRef		*pMatY		= NULL;
		cvMatRef		*pMatPts		= NULL;

		// Obtain image refences
		CCLTRY ( Prepare::extract ( pImg, v, &pImgUse, &pMat ) );
		CCLTRY ( Prepare::extract ( pImgX, adtInt(), &pImgUseX, &pMatX ) );
		CCLTRY ( Prepare::extract ( pImgY, adtInt(), &pImgUseY, &pMatY ) );
		CCLTRY ( Prepare::extract ( pImgPts, adtInt(), &pImgUsePts, &pMatPts ) );

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

