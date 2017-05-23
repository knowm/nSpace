////////////////////////////////////////////////////////////////////////
//
//										VISUALIZEL_.H
//
//		Implementation include file for visualization library
//
////////////////////////////////////////////////////////////////////////

#ifndef	VISUALIZEL__H
#define	VISUALIZEL__H

// Includes
#include	"visualizel.h"

//
// Currently using the Visualization Toolkit (VTK)
//
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkGraphicsFactory.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPointSource.h>
#include <vtkPolyDataMapper.h>

///////////
// Objects
///////////

//
// Class - visObjRef.  Object reference container.
//

class visObjRef :
	public CCLObject										// Base class
	{
	public :
	visObjRef ( void ) { AddRef(); }					// Constructor

	// Run-time data
	vtkSmartPointer<vtkRenderer>		renderer;	// Renderer object
	vtkSmartPointer<vtkRenderWindow>	renderw;		// Rendere window object
	vtkSmartPointer<vtkWindowToImageFilter>
												wif;			// Window to image filter
	vtkSmartPointer<vtkPoints>			pts;			// Points collection

	// CCL
	CCL_OBJECT_BEGIN_INT(visObjRef)
	CCL_OBJECT_END()
	};

/////////
// Nodes
/////////

//
// Class - Points.  Point cloud generation.
//

class Points :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Points ( void );										// Constructor

	// Run-time data
	IDictionary	*pDct;									// Target dictionary
	IIt			*pItX,*pItY,*pItZ;					// Pre-assigned coordinates
	
	// CCL
	CCL_OBJECT_BEGIN(Points)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_RCP(Dictionary)
	DECLARE_CON(Fire)
	DECLARE_EMT(Error)
	DECLARE_RCP(Xaxis)
	DECLARE_RCP(Yaxis)
	DECLARE_RCP(Zaxis)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Dictionary)
		DEFINE_CON(Fire)
		DEFINE_EMT(Error)
		DEFINE_RCP(Xaxis)
		DEFINE_RCP(Yaxis)
		DEFINE_RCP(Zaxis)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Internal utilities
	float getAxisPt ( U8, U32, U32, IIt * );

	};

//
// Class - Render.  Rendering node.
//

class Render :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Render ( void );										// Constructor

	// Run-time data
	IDictionary		*pDct;								// Context
	IDictionary		*pImg;								// Rendered image
	IMemoryMapped	*pBits;								// Image bits

	// CCL
	CCL_OBJECT_BEGIN(Render)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_RCP(Close)
	DECLARE_RCP(Dictionary)
	DECLARE_CON(Fire)
	DECLARE_EMT(Error)
	DECLARE_CON(Open)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Close)
		DEFINE_RCP(Dictionary)
		DEFINE_CON(Fire)
		DEFINE_EMT(Error)
		DEFINE_CON(Open)
	END_BEHAVIOUR_NOTIFY()
	};

#endif
