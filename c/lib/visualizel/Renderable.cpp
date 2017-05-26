////////////////////////////////////////////////////////////////////////
//
//									Renderable.CPP
//
//				Implementation of the renderable item node.
//
////////////////////////////////////////////////////////////////////////

#define	INITGUID
#include "visualizel_.h"
#include <stdio.h>

// Globals

Renderable :: Renderable ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pDct	= NULL;
	pItm	= NULL;
	}	// Renderable

HRESULT Renderable :: onAttach ( bool bAttach )
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
		_RELEASE(pItm);
		_RELEASE(pDct);
		}	// else

	return hr;
	}	// onAttach

HRESULT Renderable :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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
		visObjRef	*pRefItm = NULL;
		visObjRef	*pRefAct = NULL;
		adtValue		vL;

		// State check
		CCLTRYE ( pDct != NULL && pItm != NULL, ERROR_INVALID_STATE );

		// Access item
		CCLTRY (pItm->load(adtString(L"visObjRef"),vL));
		CCLTRYE( (pRefItm = (visObjRef *)(IUnknown *)(adtIUnknown(vL)))
						!= NULL, ERROR_INVALID_STATE );
		CCLOK  ( pRefItm->AddRef(); )

		// Create object for renderable
		CCLTRYE	( (pRefAct = new visObjRef()) != NULL, E_OUTOFMEMORY );

		//
		// Create renderable (actor) object based on type
		//

		// Points
		if (hr == S_OK && pRefItm->pts != NULL)
			{
			// Create a mapper for the points

			// Object for points
			vtkSmartPointer<vtkPolyData> ptsPoly =
				vtkSmartPointer<vtkPolyData>::New();
			ptsPoly->SetPoints ( pRefItm->pts );

			// Vertices
			vtkSmartPointer<vtkVertexGlyphFilter> vf =
				vtkSmartPointer<vtkVertexGlyphFilter>::New();
			vf->SetInputData(ptsPoly);
			vf->Update();

			// Object for vertices
			vtkSmartPointer<vtkPolyData> vrtsPoly =
				vtkSmartPointer<vtkPolyData>::New();
			vrtsPoly->ShallowCopy ( vf->GetOutput() );

			// Mapper for primitives
			vtkSmartPointer<vtkPolyDataMapper> mapper =
				vtkSmartPointer<vtkPolyDataMapper>::New();
			mapper->SetInputData ( vrtsPoly );

			// Renderable actor
			pRefAct->actor = vtkSmartPointer<vtkActor>::New();
			pRefAct->actor->SetMapper(mapper);
			}	// if

		// Result
		CCLTRY(pDct->store ( adtString(L"visObjRef"), adtIUnknown(pRefAct)));
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pDct));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pRefAct);
		_RELEASE(pRefItm);
		}	// else if

	// State
	else if (_RCP(Dictionary))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pDct);
		_QISAFE(unkV,IID_IDictionary,&pDct);
		}	// else if
	else if (_RCP(Item))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pItm);
		_QISAFE(unkV,IID_IDictionary,&pItm);
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

