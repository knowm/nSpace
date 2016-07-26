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
	fAngle	= 0.0f;
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
			{
			adtInt	iClr(vL);

			// Alternative way to specify a color
			fB = (float)((iClr >> 0) & 0xff);
			fG = (float)((iClr >> 8) & 0xff);
			fR = (float)((iClr >> 16) & 0xff);
			}	// if
		if (pnDesc->load ( adtString(L"Thickness"), vL ) == S_OK)
			iThick = vL;
		if (pnDesc->load ( adtString(L"Width"), vL ) == S_OK)
			fW = vL;
		if (pnDesc->load ( adtString(L"Radius"), vL ) == S_OK)
			fRad = vL;
		if (pnDesc->load ( adtString(L"Height"), vL ) == S_OK)
			fH = vL;
		if (pnDesc->load ( adtString(L"Angle"), vL ) == S_OK)
			fAngle = vL;
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pImg);
		}	// else

	return hr;
	}	// onAttach

HRESULT Draw :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

		// Obtain image refence
		CCLTRY ( Prepare::extract ( pImg, v, &pImgUse, &pMat ) );

		// Perform operation
		if (hr == S_OK && !WCASECMP(strShp,L"Line"))
			{
			if (pMat->isGPU())
				hr = E_NOTIMPL;
			else if (pMat->isUMat())
				cv::line ( *(pMat->umat), cv::Point((int)fX0,(int)fY0), 
								cv::Point((int)fX1,(int)fY1), clr, iThick );
			else
				cv::line ( *(pMat->mat), cv::Point((int)fX0,(int)fY0), 
								cv::Point((int)fX1,(int)fY1), clr, iThick );
			}	// if
		else if (hr == S_OK && !WCASECMP(strShp,L"Ellipse"))
			{
			// Rotated rectangle for which to bound the ellipse
			cv::RotatedRect	rct ( cv::Point2f(fX0,fY0), cv::Size2f(fW,fH), fAngle );

			// Perform draw
			if (pMat->isGPU())
				hr = E_NOTIMPL;
			else if (pMat->isUMat())
				cv::ellipse ( (*pMat->umat), rct, clr, iThick );
			else
				cv::ellipse ( (*pMat->mat), rct, clr, iThick );
			}	// else if
		else if (hr == S_OK && !WCASECMP(strShp,L"Circle"))
			{
			if (pMat->isGPU())
				hr = E_NOTIMPL;
			else if (pMat->isUMat())
				cv::circle ( *(pMat->umat), cv::Point((int)(float)fX0,(int)(float)fY0), 
									(int)(float)fRad, clr, iThick );
			else
				cv::circle ( *(pMat->mat), cv::Point((int)(float)fX0,(int)(float)fY0), 
									(int)(float)fRad, clr, iThick );
			}	// else if
		else if (hr == S_OK && !WCASECMP(strShp,L"Rectangle"))
			{
			if (pMat->isGPU())
				hr = E_NOTIMPL;
			else if (pMat->isUMat())
				cv::rectangle ( *(pMat->umat), cv::Point((int)fX0,(int)fY0), 
									cv::Point((int)fX1,(int)fY1), clr, iThick );
			else
				cv::rectangle ( *(pMat->mat), cv::Point((int)fX0,(int)fY0), 
									cv::Point((int)fX1,(int)fY1), clr, iThick );
			}	// else if

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
	else if (_RCP(X0))
		fX0 = v;
	else if (_RCP(X1))
		fX1 = v;
	else if (_RCP(Y0))
		fY0 = v;
	else if (_RCP(Y1))
		fY1 = v;
	else if (_RCP(Angle))
		fAngle = v;
	else if (_RCP(Height))
		fH = v;
	else if (_RCP(Width))
		fW = v;
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

