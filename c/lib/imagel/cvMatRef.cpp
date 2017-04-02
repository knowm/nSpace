////////////////////////////////////////////////////////////////////////
//
//									CVMATREF.CPP
//
//				Implementation of the OpenCV matrix container
//
////////////////////////////////////////////////////////////////////////

#include "imagel_.h"

cvMatRef :: cvMatRef ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	mat		= NULL;
	#ifdef	HAVE_OPENCV_CUDA
	gpumat	= NULL;
	#endif
	#ifdef	HAVE_OPENCV_UMAT
	umat		= NULL;
	#endif

	// So creators do not have to do the initial AddRef
	AddRef();
	}	// cvMatRef

cvMatRef :: ~cvMatRef ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Destructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	if (mat != NULL)
		{
		delete mat;
		mat = NULL;
		}	 // if
	#ifdef	HAVE_OPENCV_CUDA
	if (gpumat != NULL)
		{
		delete gpumat;
		gpumat = NULL;
		}	 // if
	#endif
	#ifdef	HAVE_OPENCV_UMAT
	if (umat != NULL)
		{
		delete umat;
		umat = NULL;
		}	 // if
	#endif
	}	// ~cvMatRef

