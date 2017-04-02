////////////////////////////////////////////////////////////////////////
//
//									NORMAL.CPP
//
//				Implementation of the image normalization node.
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"
#include <stdio.h>

// Globals

CascadeClassifier :: CascadeClassifier ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pImg	= NULL;
	pRct	= NULL;
	bLoad	= false;
	}	// CascadeClassifier

HRESULT CascadeClassifier :: onAttach ( bool bAttach )
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

		// Create dictionary to receive location results
		CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, &pRct ) );

		// Defaults (optional)
		if (pnDesc->load ( adtString(L"Location"), vL ) == S_OK)
			adtValue::toString ( vL, strLoc );
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pRct);
		_RELEASE(pImg);
		}	// else

	return hr;
	}	// onAttach

HRESULT CascadeClassifier :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// First/next detected areas
	if (_RCP(First) || _RCP(Next))
		{

		// For first time, perform analysis
		if (_RCP(First))
			{
			IDictionary	*pImgUse = NULL;
			cvMatRef		*pMat		= NULL;

			// State check
			CCLTRYE ( bLoad == true || strLoc.length() > 0, ERROR_INVALID_STATE );

			// Obtain image refence
			CCLTRY ( Prepare::extract ( pImg, v, &pImgUse, &pMat ) );
			if (hr == S_OK)
				{
				// OpenCV uses exceptions
				try
					{
					// Does the classifier need to be loaded ?
					if (!bLoad)
						{
						char *paLoc = NULL;

						// Attempt load
						CCLTRY( strLoc.toAscii(&paLoc) );
						CCLOK	( cc.load ( paLoc ); )

						// Clean up
						_FREEMEM(paLoc);
						}	// if

					// Perform detection
					if (pMat->isMat())
						cc.detectMultiScale ( *(pMat->mat), rcts );
					#ifdef	HAVE_OPENCV_UMAT
					else if (pMat->isUMat())
						cc.detectMultiScale ( *(pMat->umat), rcts );
					#endif
					#ifdef	HAVE_OPENCV_CUDA
					else if (pMat->isGPU())
						{
						}	// else if
					#endif

					// Reset enumeration index
					lprintf ( LOG_INFO, L"Count : %d", rcts.size() );
					idx = 0;
					}	// try
				catch ( cv::Exception &ex )
					{
					lprintf ( LOG_WARN, L"%S\r\n", ex.err.c_str() );
					hr = E_UNEXPECTED;
					}	// catch
				}	// if

			// Clean up
			_RELEASE(pMat);
			_RELEASE(pImgUse);
			}	// if

		// Next available recetangle
		CCLTRYE ( idx < rcts.size(), ERROR_NOT_FOUND );
		if (hr == S_OK)
			{
			// Fill result dictionary
			CCLTRY ( pRct->store ( adtString("X0"), adtInt(rcts[idx].x) ) );
			CCLTRY ( pRct->store ( adtString("Y0"), adtInt(rcts[idx].y) ) );
			CCLTRY ( pRct->store ( adtString("Width"), adtInt(rcts[idx].width) ) );
			CCLTRY ( pRct->store ( adtString("Height"), adtInt(rcts[idx].height) ) );
			CCLTRY ( pRct->store ( adtString("X1"), adtInt(rcts[idx].x+rcts[idx].width) ) );
			CCLTRY ( pRct->store ( adtString("Y1"), adtInt(rcts[idx].y+rcts[idx].height) ) );

			// Next index
			++idx;
			}	// if

		// Result
		if (hr == S_OK)
			_EMT(Next,adtIUnknown(pRct));
		else
			_EMT(End,adtInt(hr));
		}	// if

	// State
	else if (_RCP(Image))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pImg);
		_QISAFE(unkV,IID_IDictionary,&pImg);
		}	// else if
	else if (_RCP(Location))
		hr = adtValue::toString ( v, strLoc );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

