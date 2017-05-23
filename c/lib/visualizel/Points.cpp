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
	pItX	= NULL;
	pItY	= NULL;
	pItZ	= NULL;
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
		_RELEASE(pItX);
		_RELEASE(pItY);
		_RELEASE(pItZ);
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

		// State check
		CCLTRYE	( pDct != NULL, ERROR_INVALID_STATE );
		CCLTRYE	( pItX != NULL && pItY != NULL && pItZ != NULL, ERROR_INVALID_STATE );

		// Create point collection
		CCLTRYE ( (pRef = new visObjRef()) != NULL, E_OUTOFMEMORY );
		CCLTRYE	( (pRef->pts = vtkSmartPointer<vtkPoints>::New())
						!= NULL, E_OUTOFMEMORY );

		// Add points to cloud from lists
		if (hr == S_OK)
			{
			adtValue		vX,vY,vZ;
			adtDouble	dX,dY,dZ;

			// Add point from each list until a list runs dry
			CCLTRY ( pItX->begin() );
			CCLTRY ( pItY->begin() );
			CCLTRY ( pItZ->begin() );
			while (	hr == S_OK && 
						pItX->read(vX) == S_OK &&
						pItY->read(vY) == S_OK &&
						pItZ->read(vZ) == S_OK )
				{
				// Add point to collection
				pRef->pts->InsertNextPoint((dX=vX),(dY=vY),(dZ=vZ));

				// Next tuple
				pItX->next();
				pItY->next();
				pItZ->next();
				}	// while

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

