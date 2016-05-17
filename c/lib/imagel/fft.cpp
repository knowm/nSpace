////////////////////////////////////////////////////////////////////////
//
//									FFT.CPP
//
//				Implementation of the image FFT node.
//
////////////////////////////////////////////////////////////////////////

#define	INITGUID
#include "imagel_.h"
#include <stdio.h>

// Globals

FFT :: FFT ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pImg		= NULL;
	bZeroDC	= false;
	pWnd		= NULL;
	}	// FFT

HRESULT FFT :: onAttach ( bool bAttach )
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
		if (pnDesc->load ( adtString(L"ZeroDC"), vL ) == S_OK)
			bZeroDC = adtBool(vL);
		if (pnDesc->load ( adtString(L"Window"), vL ) == S_OK)
			adtValue::toString ( vL, strWnd );
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pImg);
		if (pWnd != NULL)
			{
			delete pWnd;
			pWnd = NULL;
			}	// if
		}	// else

	return hr;
	}	// onAttach

HRESULT FFT :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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
		IDictionary	*pImgUse = pImg;
		cv::Mat		*pMat		= NULL;
		adtValue		vL;

		// Image to use
		if (pImgUse == NULL)
			{
			adtIUnknown unkV(v);
			CCLTRY(_QISAFE(unkV,IID_IDictionary,&pImgUse));
			}	// if
		else
			pImgUse->AddRef();

		// Image must be 'uploaded'
		CCLTRY ( pImgUse->load (	adtString(L"cv::Mat"), vL ) );
		CCLTRYE( (pMat = (cv::Mat *)(U64)adtLong(vL)) != NULL,
					ERROR_INVALID_STATE );

		//
		// Windowing.  Only generate a single row window function as
		// things change.  If the window function or number of columns
		// change it is time to re-create the window
		//

		// Thread safey
		csSync.enter();

		// Verify size
		if (	hr == S_OK		&& 
				(pWnd != NULL	&& (pWnd->cols != pMat->cols || pWnd->rows != pMat->rows)) ||
				(pWnd == NULL	&& strWnd.length() > 0 && WCASECMP(strWnd,L"None")) )
			{
			// Previous window
			if (pWnd != NULL)
				{
				delete pWnd;
				pWnd = NULL;
				}	// if

			// Create new window.  Assuming it is faster to just do per element
			// multiplication rather than row by row.
			CCLTRYE ( (pWnd = new cv::Mat(pMat->rows,pMat->cols,CV_32FC1)) != NULL, E_OUTOFMEMORY );

			// Generate function
			if (!WCASECMP(strWnd,L"Blackman-Nutall"))
				{
				// Coefficients for window
				float a0 = 0.3635819f;
				float a1 = 0.4891775f;
				float a2 = 0.1365995f;
				float a3 = 0.0106441f;
				float ev = 0.0f;

				// Compute window (slow)
				for (int r = 0;r < pMat->rows;++r)
					for (int c = 0;c < pMat->cols;++c)
						{
						// Evaulate
						ev =	(float) (a0 -
											a1 * cos ( (2*CV_PI*c)/(pMat->cols-1) ) +
											a2 * cos ( (4*CV_PI*c)/(pMat->cols-1) ) -
											a3 * cos ( (6*CV_PI*c)/(pMat->cols-1) ) );

						// Set
						pWnd->at<float>(cv::Point(c,r)) = ev;

						// Debug
//						pWnd->at<float>(cv::Point(c,r)) = 0;
						}	// for

				}	// if

			// Unknown function
			else
				{
				lprintf ( LOG_ERR, L"Unimplemented window function : %s", (LPCWSTR) strWnd );
				hr = E_NOTIMPL;
				}	// else

			}	// if

		// Compute FFT/Magnitude
		CCLTRY ( image_fft ( pMat, pWnd, true, bZeroDC ) );

		// Thread safey
		csSync.leave();

		// Result
//		if (hr != S_OK)
//			dbgprintf ( L"FFT::Write:hr 0x%x, I/O %d/%d\r\n", hr, uLeft, (U32)iSzIo );
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pImgUse));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pImgUse);
		}	// if

	// State
	else if (_RCP(Image))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pImg);
		_QISAFE(unkV,IID_IDictionary,&pImg);
		}	// else if
	else if (_RCP(Window))
		{
		// New window name
		adtValue::toString ( v, strWnd );

		// Assume a new window is needed
		dbgprintf ( L"New window : %s\r\n", (LPCWSTR)strWnd );
		if (pWnd != NULL)
			{
			// Thread safey
			csSync.enter();

			// Clean up
			delete pWnd;
			pWnd = NULL;

			// Thread safey
			csSync.leave();
			}	// if
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive



		//
		// Perform calculation based on matrix type
		//
		/*
		// OpenCl
		if (hr == S_OK && pImgUse->load ( adtString(L"cv::ocl::oclMat"), vL ) == S_OK)
			{
			cv::ocl::oclMat	*pMat = NULL;

			// Extract matrix
			CCLTRYE ( (pMat = (cv::ocl::oclMat *)(U64)adtLong(vL)) != NULL, E_INVALIDARG );

			// Execute
			CCLTRY ( image_fft ( pMat, true, bZeroDC ) );
			}	// if

		// CPU
		else 
			{
			cv::Mat		*pMat		= NULL;

			// CPU matrix required at a minimum
			CCLTRY ( pImgUse->load ( adtString(L"cv::Mat"), vL ) );
			CCLTRYE ( (pMat = (cv::Mat *)(U64)adtLong(vL)) != NULL, E_INVALIDARG );

			// Execute
			CCLTRY ( image_fft ( pMat, true, bZeroDC ) );
			}	// else
		*/

/*
		// TODO: TEMPORARY.  This thresholding will be moved into a node
		// in dB
		CCLOK ( threshold ( matMag, matMag, 50, 0, THRESH_TRUNC ); )
		CCLOK ( threshold ( matMag, matMag, 15, 0, THRESH_TOZERO ); )
*/