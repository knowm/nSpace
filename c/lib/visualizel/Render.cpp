////////////////////////////////////////////////////////////////////////
//
//									RENDER.CPP
//
//			Implementation of the visualization render node
//
////////////////////////////////////////////////////////////////////////

#include "visualizel_.h"
#include <stdio.h>
#include	<vtkAutoInit.h>

// NOTE NOTE NOTE.  VTK seems to rely a lot on static
// intialization.  This seems to easily crash the system since doing
// things from a COM dll.  Currently only building/linking with 
// static version of libraries
VTK_MODULE_INIT(vtkRenderingOpenGL2);

// Globals
static bool bFirst = true;

Render :: Render ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pDct	= NULL;
	pImg	= NULL;
	pBits	= NULL;
	}	// Render

HRESULT Render :: onAttach ( bool bAttach )
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
//		adtValue		vL;

		// Defaults
//		if (pnDesc->load ( adtString(L"Xaxis"), vL ) == S_OK)
//			strX = vL;

		// Create output image dictionary
		CCLTRY ( COCREATE(L"Adt.Dictionary",IID_IDictionary,&pImg) );
		CCLTRY ( COCREATE(L"Io.MemoryBlock",IID_IMemoryMapped,&pBits) );
		CCLTRY ( pImg->store(adtString(L"Bits"),adtIUnknown(pBits)) );

		// Does this belong here ?  For now nodes just support off-screen
		// rendering to an image for later usage.  Perform one-time
		// initialization to setup offscreen rendering.
		if (bFirst)
			{
			vtkSmartPointer<vtkGraphicsFactory>	gfct;
//			vtkSmartPointer<vtkImagingFactory>	ifct;

			// Options for graphics factory
			CCLTRYE ( (gfct = vtkSmartPointer<vtkGraphicsFactory>::New())
							!= NULL, E_OUTOFMEMORY );
			CCLOK ( gfct->SetOffScreenOnlyMode(1); )
			CCLOK ( gfct->SetUseMesaClasses(1); )

			// Options for imaging factory
//			CCLTRYE((ifct = vtkSmartPointer<vtkImagingFactory>::New())
//				!= NULL, E_OUTOFMEMORY);
//			CCLOK(ifct->SetUseMesaClasses(1);)

			// Done
			bFirst = false;
			}	// if

		}	// if

	// Detach
	else
		{
		// Shutdown
		_RELEASE(pDct);
		_RELEASE(pBits);
		_RELEASE(pImg);

		// NOTE NOTE NOTE.  VTK seems to rely a lot on static
		// intialization.  This seems to easily crash the system since doing
		// things from a COM dll.  Calling this directly seems to clear it up (?)
//		vtkObjectFactory::UnRegisterAllFactories();
		}	// else

	return hr;
	}	// onAttach

HRESULT Render :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Render
	if (_RCP(Fire))
		{
		visObjRef	*pRef = NULL;
		adtValue		vL;

		// State check
		CCLTRYE ( pDct != NULL, ERROR_INVALID_STATE );

		// Access reference object
		CCLTRY (pDct->load(adtString(L"visObjRef"),vL));
		CCLTRYE( (pRef = (visObjRef *)(IUnknown *)(adtIUnknown(vL)))
						!= NULL, ERROR_INVALID_STATE );
		CCLOK  ( pRef->AddRef(); )

		// Render and grab snapshot of image
		if (hr == S_OK)
			{
			// Execute render
			pRef->renderw->Render();

			// Update the image
			pRef->wif->Update();
			}	// if

		//
		// Update the bits of image
		//
		if (hr == S_OK)
			{
			vtkImageData	*pout		= NULL;
			VOID				*pvBits	= NULL;
			int				dims[3];
			S32				sz;

			// Retrieve the image data object and its dimensions
			CCLTRYE ( (pout = pRef->wif->GetOutput()) != NULL, E_UNEXPECTED );
			CCLOK ( pout->GetDimensions(dims); )

			// Ensure target has enough room and copy bits, assuming format used below.
			CCLTRYE( (sz = dims[0]*dims[1]*(3*sizeof(U8))) > 0, E_UNEXPECTED );
			CCLTRY ( pBits->setSize ( sz ) );
			CCLTRY ( pBits->lock ( 0, 0, &pvBits, NULL ) );
			CCLOK  ( memcpy ( pvBits, pout->GetScalarPointer(), sz ); )

			// Update dimensions (do this once on creation ?)
			CCLTRY ( pImg->store ( adtString(L"Width"), adtInt(dims[0]) ) );
			CCLTRY ( pImg->store ( adtString(L"Height"), adtInt(dims[1]) ) );

			// Clean up
			_UNLOCK(pBits,pvBits);
			}	// if

		// Result
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pImg));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pRef);
		}	// else if

	// Open a new render object
	else if (_RCP(Open))
		{
		visObjRef	*pRef = NULL;
		adtValue		vL;

		// State check
		CCLTRYE ( pDct != NULL, ERROR_INVALID_STATE );
		CCLTRYE ( pDct->load(adtString(L"visObjRef"),vL) != S_OK,
						ERROR_INVALID_STATE );

		// Create new object to hold references
		CCLTRYE ( (pRef = new visObjRef()) != NULL, E_OUTOFMEMORY );

		// Create the renderer and off-screen render window objects
		CCLTRYE	( (pRef->renderer = vtkSmartPointer<vtkRenderer>::New())
						!= NULL, E_OUTOFMEMORY );
		CCLTRYE	( (pRef->renderw = vtkSmartPointer<vtkRenderWindow>::New())
						!= NULL, E_OUTOFMEMORY );
		CCLOK		( pRef->renderw->SetOffScreenRendering(1); )
		CCLOK		( pRef->renderw->AddRenderer(pRef->renderer); )

		// Debug
		CCLOK ( pRef->renderer->SetBackground(0,0,0); )

		// Add the window to image filter that will be used to take a snapshot
		// of the rendered image pixel data
		CCLTRYE	( (pRef->wif = vtkSmartPointer<vtkWindowToImageFilter>::New()) 
						!= NULL, E_OUTOFMEMORY );
		CCLOK		( pRef->wif->SetInput(pRef->renderw); )
//		CCLOK		( pRef->wif->SetInputBufferTypeToRGBA(); )
		CCLOK		( pRef->wif->SetInputBufferTypeToRGB(); )
		CCLOK		( pRef->wif->ReadFrontBufferOff(); )
		CCLTRY	( pImg->store ( adtString(L"Format"),adtString(L"R8G8B8") ) );

		//
		// DEBUG
		//
		if (hr == S_OK)
			{
			// Create a random point cloud
			vtkSmartPointer<vtkPointSource>	pts = 
				vtkSmartPointer<vtkPointSource>::New();
			pts->SetCenter(0,0,0);
			pts->SetNumberOfPoints(500);
			pts->SetRadius(5.0);
			pts->Update();

			// Mapper and actor
			vtkSmartPointer<vtkPolyDataMapper> mapper =
				vtkSmartPointer<vtkPolyDataMapper>::New();
			mapper->SetInputConnection(pts->GetOutputPort());

			vtkSmartPointer<vtkActor> actor =
				vtkSmartPointer<vtkActor>::New();
			actor->SetMapper(mapper);

			pRef->renderer->AddActor(actor);

			}	// if

		// Result
		CCLTRY	( pDct->store ( adtString(L"visObjRef"), adtIUnknown(pRef) ) );
		if (hr == S_OK)
			_EMT(Open,adtIUnknown(pDct));
		else
			_EMT(Error,adtInt(hr));
		}	// else if

	// Close down render object
	else if (_RCP(Close))
		{
		// Remove reference object
		if (pDct != NULL)
			pDct->remove ( adtString(L"visObjRef") );
		}	// else if

	// State
	else if (_RCP(Dictionary))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pDct);
		_QISAFE(unkV,IID_IDictionary,&pDct);
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

