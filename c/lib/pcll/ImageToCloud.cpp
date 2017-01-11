////////////////////////////////////////////////////////////////////////
//
//									IMAGETOCLOUD.CPP
//
//			Implementation of the image to point cloud conversion node.
//
////////////////////////////////////////////////////////////////////////

#include "pcll_.h"
#include <stdio.h>

// Globals

ImageToCloud :: ImageToCloud ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pDct	= NULL;
	pItX	= NULL;
	pItY	= NULL;
	pItZ	= NULL;
	}	// ImageToCloud

HRESULT ImageToCloud :: onAttach ( bool bAttach )
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

		// How to assign each axis
		if (pnDesc->load ( adtString(L"X"), vL ) == S_OK)
			strX = vL;
		if (pnDesc->load ( adtString(L"Y"), vL ) == S_OK)
			strY = vL;
		if (pnDesc->load ( adtString(L"Z"), vL ) == S_OK)
			strZ = vL;

		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pItX);
		_RELEASE(pItY);
		_RELEASE(pItZ);
		_RELEASE(pDct);
		}	// else

	return hr;
	}	// onAttach

HRESULT ImageToCloud :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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
		// State check
		CCLTRYE ( pDct != NULL, ERROR_INVALID_STATE );

		// Access image information

		// Assign each axis for non-zero points
//		for (size_t i = 0;i < cloud.size();++i)
//			{
//			}	// for

		}	// else if

	// State
	else if (_RCP(Dictionary))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pDct);
		_QISAFE(unkV,IID_IDictionary,&pDct);
		}	// else if
	else if (_RCP(Xaxis) || _RCP(Yaxis) || _RCP(Zaxis))
		{
		IContainer	*pCnt = NULL;
		IIt			*pIt	= NULL;
		adtIUnknown	unkV(v);

		// Obtain iterator for axis
		CCLTRY(_QISAFE(unkV,IID_IContainer,&pCnt));
		CCLTRY(pCnt->iterate(&pIt));

		// Assign
		_ADDREF(pIt);										// For assignment
		if (_RCP(Xaxis))
			{
			_RELEASE(pItX);
			pItX = pIt;
			}	// if
		else if (_RCP(Yaxis))
			{
			_RELEASE(pItY);
			pItY = pIt;
			}	// if
		else 
			{
			_RELEASE(pItZ);
			pItZ = pIt;
			}	// if

		// Clean up
		_RELEASE(pIt);
		_RELEASE(pCnt);
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

