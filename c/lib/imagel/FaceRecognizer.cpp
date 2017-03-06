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

FaceRecognizer :: FaceRecognizer ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pImg		= NULL;
	pImgs		= NULL;
	strType	= L"Fisher";
	}	// FaceRecognizer

HRESULT FaceRecognizer :: onAttach ( bool bAttach )
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
			adtValue::toString ( vL, strType );
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pImg);
		_RELEASE(pImgs);
		}	// else

	return hr;
	}	// onAttach

HRESULT FaceRecognizer :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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


	// Perform prediction
	if (_RCP(Fire))
		{
		IDictionary	*pImgUse = NULL;
		cvMatRef		*pMat		= NULL;
		int			lbl		= 0;
		double		conf		= 0;

		// Trained ?
		CCLTRYE ( recog != NULL, ERROR_INVALID_STATE );

		// Obtain image refence
		CCLTRY ( Prepare::extract ( pImg, v, &pImgUse, &pMat ) );

		// Perform operation
		if (hr == S_OK)
			{
			// OpenCV uses exceptions
			try
				{
				if (pMat->isUMat())
					{
					}	// if
				#ifdef	WITH_CUDA
				else if (pMat->isGPU())
					{
					}	// if
				#endif
				else if (pMat->isMat())
					{
					// Perform prediction
					recog->predict ( *(pMat->mat), lbl, conf );
					lprintf ( LOG_INFO, L"Label %d Distance %g\r\n", lbl, conf );
					}	// if

				}	// try
			catch ( cv::Exception &ex )
				{
				lprintf ( LOG_ERR, L"%S\r\n", ex.err.c_str() );
				hr = E_UNEXPECTED;
				}	// catch

			}	// if

		// Results
		if (hr == S_OK)
			{
			_EMT(Distance,adtDouble(conf));
			_EMT(At,adtInt(lbl));
			_EMT(Fire,adtIUnknown(pImgUse));
			}	// if
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pMat);
		_RELEASE(pImgUse);
		}	// if

	// Train on a set of images
	else if (_RCP(Train))
		{
		IDictionary	*pImgUse = NULL;
		cvMatRef		*pMat		= NULL;
		IIt			*pIt		= NULL;
		adtValue		vL;
		adtIUnknown	unkV;

		// State check
		CCLTRYE ( pImgs != NULL, ERROR_INVALID_STATE );

		// Iterate images
		CCLTRY ( pImgs->iterate(&pIt) );

		// Extract first image to use to determine basic information
		CCLTRY ( pIt->read (vL) );

		// Obtain image refence
		CCLTRY ( Prepare::extract ( NULL, vL, &pImgUse, &pMat ) );
		if (hr == S_OK)
			{
			// OpenCV uses exceptions
			try
				{
				S32	maxW	= 0;
				S32	maxH	= 0;

				// Create specified type of recognizer
				if (!WCASECMP(strType,L"Fisher"))
					recog = cv::face::createFisherFaceRecognizer();
				else if (!WCASECMP(strType,L"Eigen"))
					recog = cv::face::createEigenFaceRecognizer();
				else if (!WCASECMP(strType,L"LBPH"))
					recog = cv::face::createLBPHFaceRecognizer();
				else
					hr = E_NOTIMPL;

				// Determine the largest size image
				CCLTRY(pIt->begin());
				while (hr == S_OK && pIt->read ( vL ) == S_OK)
					{
					IDictionary		*pImgT	= NULL;
					cvMatRef			*pMatT	= NULL;

					// Obtain image refence
					CCLTRY ( Prepare::extract ( NULL, vL, &pImgT, &pMatT ) );

					// Add to list
					if (hr == S_OK)
						{
						if (pMatT->cols() > maxW)
							maxW = pMatT->cols();
						if (pMatT->rows() > maxH)
							maxH = pMatT->rows();
						}	// if

					// Clean up
					_RELEASE(pMatT);
					_RELEASE(pImgT);
					pIt->next();
					}	// while

 				// Train
				if (pMat->isUMat())
					{
					}	// if
				#ifdef	WITH_CUDA
				else if (pMat->isGPU())
					{
					}	// else if
				#endif
				else if (pMat->isMat())
					{
					U32							i	= 1;
					std::vector<cv::Mat>		images;
					std::vector<int>			labels;

					// Create a list of images and labels to feed to the trainer
					CCLTRY(pIt->begin());
					while (hr == S_OK && pIt->read ( vL ) == S_OK)
						{
						IDictionary		*pImgT	= NULL;
						cvMatRef			*pMatT	= NULL;

						// Obtain image refence
						CCLTRY ( Prepare::extract ( NULL, vL, &pImgT, &pMatT ) );

						// Add to list
						if (hr == S_OK && pMatT->isMat())
							{
							// As a service to the graph, ensure images are the same size
							// Copy image into the larger version of itself.
							cv::Mat	matBig(maxH,maxW,pMatT->mat->type());
							pMatT->mat->copyTo ( matBig(cv::Rect(0,0,pMatT->cols(),pMatT->rows())) );
							images.push_back(matBig);
							labels.push_back(i++);
							}	// if

						// Clean up
						_RELEASE(pMatT);
						_RELEASE(pImgT);
						pIt->next();
						}	// while

					// Perform training
					recog->train ( images, labels );
					lprintf ( LOG_INFO, L"Trained : %d images", images.size() );
					}	// else if

				}	// try
			catch ( cv::Exception &ex )
				{
				lprintf ( LOG_ERR, L"%S\r\n", ex.err.c_str() );
				hr = E_UNEXPECTED;
				}	// catch
			}	// if

		// Result
		if (hr == S_OK)
			_EMT(Train,adtIUnknown(pImgs));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pMat);
		_RELEASE(pImgUse);
		_RELEASE(pIt);
		}	// if

	// State
	else if (_RCP(Image))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pImg);
		_QISAFE(unkV,IID_IDictionary,&pImg);
		}	// else if
	else if (_RCP(Images))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pImgs);
		_QISAFE(unkV,IID_IContainer,&pImgs);
		}	// else if
	else if (_RCP(Type))
		hr = adtValue::toString ( v, strType );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

