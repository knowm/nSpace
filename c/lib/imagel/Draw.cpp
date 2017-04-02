////////////////////////////////////////////////////////////////////////
//
//									Draw.CPP
//
//				Implementation of the draw shape node.
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"
#include <stdio.h>

// Globals

Draw :: Draw ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pImg		= NULL;
	fR			= fB = fG = 0.0f;
	fX0		= fY0 = fX1 = fY1 = 0.0f;
	iThick	= 1;
	fRad		= 1.0f;
	fA			= 0.0f;
	fA0		= 0.0f;
	fA1		= 0.0f;
	strShp	= L"Line";
	}	// Draw

HRESULT Draw :: onAttach ( bool bAttach )
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
		if (pnDesc->load ( adtString(L"Shape"), vL ) == S_OK)
			adtValue::toString ( vL, strShp );
		if (pnDesc->load ( adtString(L"Red"), vL ) == S_OK)
			fR = vL;
		if (pnDesc->load ( adtString(L"Green"), vL ) == S_OK)
			fG = vL;
		if (pnDesc->load ( adtString(L"Blue"), vL ) == S_OK)
			fB = vL;
		if (pnDesc->load ( adtString(L"Color"), vL ) == S_OK)
			onReceive ( prColor, vL );
		if (pnDesc->load ( adtString(L"Intensity"), vL ) == S_OK)
			onReceive ( prIntensity, vL );
		if (pnDesc->load ( adtString(L"Thickness"), vL ) == S_OK)
			iThick = vL;
		if (pnDesc->load ( adtString(L"Width"), vL ) == S_OK)
			fW = vL;
		if (pnDesc->load ( adtString(L"Radius"), vL ) == S_OK)
			fRad = vL;
		if (pnDesc->load ( adtString(L"Height"), vL ) == S_OK)
			fH = vL;
		if (pnDesc->load ( adtString(L"Angle"), vL ) == S_OK)
			fA = vL;
		if (pnDesc->load ( adtString(L"Angle0"), vL ) == S_OK)
			fA0 = vL;
		if (pnDesc->load ( adtString(L"Angle1"), vL ) == S_OK)
			fA1 = vL;
		if (pnDesc->load ( adtString(L"X0"), vL ) == S_OK)
			fX0 = vL;
		if (pnDesc->load ( adtString(L"Y0"), vL ) == S_OK)
			fY0 = vL;
		if (pnDesc->load ( adtString(L"X1"), vL ) == S_OK)
			fX1 = vL;
		if (pnDesc->load ( adtString(L"Y1"), vL ) == S_OK)
			fY1 = vL;
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pImg);
		}	// else

	return hr;
	}	// onAttach

HRESULT Draw :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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
		cv::Scalar	clr		= CV_RGB(fR,fG,fB);
		cv::Mat		matNoGpu;

		// Obtain image refence
		CCLTRY ( Prepare::extract ( pImg, v, &pImgUse, &pMat ) );

		// No GPU support for drawing
		// Download the current image into a local matrix for processing
		#ifdef	HAVE_OPENCV_CUDA
		if (hr == S_OK && pMat->isGPU())
			pMat->gpumat->download ( matNoGpu );
		#endif

		// Perform operation
		if (hr == S_OK && !WCASECMP(strShp,L"Line"))
			{
			if (pMat->isGPU())
				cv::line ( matNoGpu, cv::Point((int)fX0,(int)fY0), 
								cv::Point((int)fX1,(int)fY1), clr, iThick );
			#ifdef	HAVE_OPENCV_UMAT
			else if (pMat->isUMat())
				cv::line ( *(pMat->umat), cv::Point((int)fX0,(int)fY0), 
								cv::Point((int)fX1,(int)fY1), clr, iThick );
			#endif
			else
				cv::line ( *(pMat->mat), cv::Point((int)fX0,(int)fY0), 
								cv::Point((int)fX1,(int)fY1), clr, iThick );
			}	// if
		else if (hr == S_OK && !WCASECMP(strShp,L"Ellipse"))
			{
			// Rotated rectangle for which to bound the ellipse
			cv::RotatedRect	rct ( cv::Point2f(fX0,fY0), cv::Size2f(fW,fH), fA );

			// Perform draw
			if (pMat->isGPU())
				cv::ellipse ( matNoGpu, rct, clr, iThick );
			#ifdef	HAVE_OPENCV_UMAT
			else if (pMat->isUMat())
				cv::ellipse ( (*pMat->umat), rct, clr, iThick );
			#endif
			else
				cv::ellipse ( (*pMat->mat), rct, clr, iThick );
			}	// else if
		else if (hr == S_OK && !WCASECMP(strShp,L"Arc"))
			{
			// Perform draw
			if (pMat->isGPU())
				cv::ellipse ( matNoGpu, cv::Point((int)fX0,(int)fY0), 
									cv::Size((int)fX1,(int)fY1),
									fA, fA0, fA1, clr, iThick );
			#ifdef	HAVE_OPENCV_UMAT
			else if (pMat->isUMat())
				cv::ellipse ( (*pMat->umat), cv::Point((int)fX0,(int)fY0), 
									cv::Size((int)fX1,(int)fY1),
									fA, fA0, fA1, clr, iThick );
			#endif
			else
				cv::ellipse ( (*pMat->mat), cv::Point((int)fX0,(int)fY0), 
									cv::Size((int)fX1,(int)fY1),
									fA, fA0, fA1, clr, iThick );
			}	// else if
		else if (hr == S_OK && !WCASECMP(strShp,L"Circle"))
			{
			if (pMat->isGPU())
				cv::circle ( matNoGpu, cv::Point((int)(float)fX0,(int)(float)fY0), 
									(int)(float)fRad, clr, iThick );
			#ifdef	HAVE_OPENCV_UMAT
			else if (pMat->isUMat())
				cv::circle ( *(pMat->umat), cv::Point((int)(float)fX0,(int)(float)fY0), 
									(int)(float)fRad, clr, iThick );
			#endif
			else
				cv::circle ( *(pMat->mat), cv::Point((int)(float)fX0,(int)(float)fY0), 
									(int)(float)fRad, clr, iThick );
			}	// else if
		else if (hr == S_OK && !WCASECMP(strShp,L"Rectangle"))
			{
			if (pMat->isGPU())
				cv::rectangle ( matNoGpu, cv::Point((int)fX0,(int)fY0), 
									cv::Point((int)fX1,(int)fY1), clr, iThick );
			#ifdef	HAVE_OPENCV_UMAT
			else if (pMat->isUMat())
				cv::rectangle ( *(pMat->umat), cv::Point((int)fX0,(int)fY0), 
									cv::Point((int)fX1,(int)fY1), clr, iThick );
			#endif
			else
				cv::rectangle ( *(pMat->mat), cv::Point((int)fX0,(int)fY0), 
									cv::Point((int)fX1,(int)fY1), clr, iThick );
			}	// else if

		// Upload changes back to GPU
		#ifdef	HAVE_OPENCV_CUDA
		if (hr == S_OK && pMat->isGPU())
			pMat->gpumat->upload ( matNoGpu );
		#endif

		// Result
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pImgUse));
		else
			_EMT(Error,adtInt(hr));

		// Debug
		if (hr != S_OK)
			lprintf ( LOG_DBG, L"draw '%s' failed : 0x%x\r\n", (LPCWSTR)strShp, hr );

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
	else if (_RCP(Color))
		{
		adtInt	iClr(v);

		// Alternative way to specify a color
		fB = (float)((iClr >> 0) & 0xff);
		fG = (float)((iClr >> 8) & 0xff);
		fR = (float)((iClr >> 16) & 0xff);
		}	// if
	else if (_RCP(Intensity))
		{
		adtFloat	fClr(v);

		// Alternative way to specify a color
		fB = fClr;
		fG = fClr;
		fR = fClr;
		}	// if
	else if (_RCP(X0))
		fX0 = v;
	else if (_RCP(X1))
		fX1 = v;
	else if (_RCP(Y0))
		fY0 = v;
	else if (_RCP(Y1))
		fY1 = v;
	else if (_RCP(Angle))
		fA = v;
	else if (_RCP(Angle0))
		fA0 = v;
	else if (_RCP(Angle1))
		fA1 = v;
	else if (_RCP(Height))
		fH = v;
	else if (_RCP(Width))
		fW = v;
	else if (_RCP(Thick))
		iThick = v;
	else if (_RCP(Radius))
		fRad = v;
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

