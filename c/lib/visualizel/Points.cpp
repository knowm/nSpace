////////////////////////////////////////////////////////////////////////
//
//									POINTS.CPP
//
//				Implementation of the points creation node.
//
////////////////////////////////////////////////////////////////////////

#define	INITGUID
#include "visualizel_.h"
#include <stdio.h>

// Globals

Points :: Points ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pDct	= NULL;
	pImg	= NULL;
	}	// Points

HRESULT Points :: onAttach ( bool bAttach )
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
		_RELEASE(pDct);
		}	// else

	return hr;
	}	// onAttach

HRESULT Points :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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
		visObjRef	*pRef		= NULL;
		IIt			*pItX		= NULL;
		IIt			*pItY		= NULL;
		IIt			*pItZ		= NULL;
		ImageDct		dctImg;

		//
		// NOTE: Current assumes points are contained in the image
		// with each row being a point and the colums can be X,Y,Z,I
		//

		// State check
		CCLTRYE	( pDct != NULL && pImg != NULL, ERROR_INVALID_STATE );

		// Extract information about image
		CCLTRY ( dctImg.lock ( pImg ) );

		// Current requires non-zero number of floating points
		CCLTRYE ( dctImg.iFmt == IMGFMT_F32X2, ERROR_INVALID_STATE );
		CCLTRYE ( dctImg.iH > 0, ERROR_INVALID_STATE );

		// Create point collection
		CCLTRYE	( (pRef = new visObjRef()) != NULL, E_OUTOFMEMORY );
		CCLTRYE	( (pRef->pts = vtkSmartPointer<vtkPoints>::New())
						!= NULL, E_OUTOFMEMORY );
		CCLOK		( pRef->pts->SetDataTypeToFloat(); )

		// Total number of points will be the number of rows in the image
		CCLOK		( pRef->pts->SetNumberOfPoints(dctImg.iH); )
//		CCLOK		( pRef->pts->SetNumberOfPoints(10000); )

		// Add points to cloud from memory
		if (hr == S_OK)
			{
			float			*pfBits	= (float *)(dctImg.pvBits);
			adtDouble	dX,dY,dZ;

			// All rows
			for (U32 idx = 0;hr == S_OK && idx < (U32)dctImg.iH;++idx)//&& idx < 10000;++idx)
				{
				// Assign coordinates
				if (dctImg.iW > 0)	dX = pfBits[0];
				if (dctImg.iW > 1)	dY = pfBits[1];
				if (dctImg.iW > 2)	dZ = pfBits[2];

				// Add point to collection
//				lprintf ( LOG_DBG, L"%g %g %g\r\n", (double)dX, (double)dY, (double)dZ );
				pRef->pts->SetPoint(idx,dX,dY,dZ);

				// Next row
				pfBits += dctImg.iW;
				}	// for

			}	// if

		// DEBUG
/*
if (hr == S_OK)
{
pRef->pts->SetNumberOfPoints(100);
for (U32 i = 0;i < 100;++i)
	pRef->pts->SetPoint(i,i,i,i);
//pRef->pts->InsertNextPoint(0,0,0);
//pRef->pts->InsertNextPoint(1,0,0);
//pRef->pts->InsertNextPoint(0,1,0);
//pRef->pts->InsertNextPoint(0,0,1);
}
*/
		// Result
		CCLTRY(pDct->store ( adtString(L"visObjRef"), adtIUnknown(pRef)));
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pDct));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pRef);
		}	// else if

	// State
	else if (_RCP(Dictionary))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pDct);
		_QISAFE(unkV,IID_IDictionary,&pDct);
		}	// else if
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

