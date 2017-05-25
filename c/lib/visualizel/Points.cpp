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
	pCntX	= NULL;
	pCntY	= NULL;
	pCntZ	= NULL;
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
		adtValue		vL;

		// How to assign each axis
		if (pnDesc->load ( adtString(L"Xaxis"), vL ) == S_OK)
			onReceive(prXaxis,vL);
		if (pnDesc->load ( adtString(L"Yaxis"), vL ) == S_OK)
			onReceive(prYaxis,vL);
		if (pnDesc->load ( adtString(L"Zaxis"), vL ) == S_OK)
			onReceive(prZaxis,vL);
		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pCntX);
		_RELEASE(pCntY);
		_RELEASE(pCntZ);
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
		U32			npts		= 0;
		U32			nptsx		= 0;

		// State check
		CCLTRYE	( pDct != NULL, ERROR_INVALID_STATE );
		CCLTRYE	( pCntX != NULL && pCntY != NULL && pCntZ != NULL, ERROR_INVALID_STATE );

		// Create point collection
		CCLTRYE	( (pRef = new visObjRef()) != NULL, E_OUTOFMEMORY );
		CCLTRYE	( (pRef->pts = vtkSmartPointer<vtkPoints>::New())
						!= NULL, E_OUTOFMEMORY );
		CCLOK		( pRef->pts->SetDataTypeToFloat(); )

		// Total number of pts will be minimum of all three lists
		CCLTRY ( pCntX->size ( &npts ) );
		if (hr == S_OK && pCntY->size ( &nptsx ) == S_OK && nptsx < npts)
			npts = nptsx;
		if (hr == S_OK && pCntZ->size ( &nptsx ) == S_OK && nptsx < npts)
			npts = nptsx;
		CCLTRYE	( npts > 0, E_INVALIDARG );
		CCLOK		( pRef->pts->SetNumberOfPoints(npts); )

		// Add points to cloud from lists
		if (hr == S_OK)
			{
			U32			idx = 0;
			adtValue		vX,vY,vZ;
			adtDouble	dX,dY,dZ;

			// Add point from each list until a list runs dry
			CCLTRY ( pCntX->iterate ( &pItX ) );
			CCLTRY ( pCntY->iterate ( &pItY ) );
			CCLTRY ( pCntZ->iterate ( &pItZ ) );
			while (	hr == S_OK && 
						pItX->read(vX) == S_OK &&
						pItY->read(vY) == S_OK &&
						pItZ->read(vZ) == S_OK )
				{
				// Add point to collection
				pRef->pts->SetPoint(idx++,(dX=vX),(dY=vY),(dZ=vZ));

				// Next tuple
				pItX->next();
				pItY->next();
				pItZ->next();
				}	// while

			// Clean up
			_RELEASE(pItZ);
			_RELEASE(pItY);
			_RELEASE(pItX);
			}	// if

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
	else if (_RCP(Xaxis) || _RCP(Yaxis) || _RCP(Zaxis))
		{
		adtIUnknown	unkV(v);

		// Obtain iterator for axis
		if (_RCP(Xaxis))
			{
			_RELEASE(pCntX);
			CCLTRY(_QISAFE(unkV,IID_IContainer,&pCntX));
			}	// if
		else if (_RCP(Yaxis))
			{
			_RELEASE(pCntY);
			CCLTRY(_QISAFE(unkV,IID_IContainer,&pCntY));
			}	// if
		else if (_RCP(Zaxis))
			{
			_RELEASE(pCntZ);
			CCLTRY(_QISAFE(unkV,IID_IContainer,&pCntZ));
			}	// if

		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

