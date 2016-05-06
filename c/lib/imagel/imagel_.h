////////////////////////////////////////////////////////////////////////
//
//										IMAGEL_.H
//
//				Implementation include file for image processing library
//
////////////////////////////////////////////////////////////////////////

#ifndef	IMAGEL__H
#define	IMAGEL__H

// Includes
#include "imagel.h"
#include "../../lib/nspcl/nspcl.h"

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/ocl/ocl.hpp>

// Operations
#define	MATHOP_NOP		-1

// Arithmetic
#define	MATHOP_ADD		1
#define	MATHOP_SUB		2
#define	MATHOP_MUL		3
#define	MATHOP_DIV		4

///////////
// Objects
///////////


/////////
// Nodes
/////////

//
// Class - Binary.  Perform a binary operation involving images.
//

class Binary :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Binary ( void );										// Constructor

	// Run-time data
	adtValue		vL,vR;									// Parameters
	int			iOp;										// Math operation

	// CCL
	CCL_OBJECT_BEGIN(Binary)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Fire)
	DECLARE_RCP(Left)
	DECLARE_RCP(Right)
	DECLARE_EMT(Error)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
		DEFINE_RCP(Left)
		DEFINE_RCP(Right)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - Convert.  Convert between formats.
//

class Convert :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Convert ( void );										// Constructor

	// Run-time data
	IDictionary	*pImg;									// Source image
	adtString	strTo;									// Convert 'to' format

	// CCL
	CCL_OBJECT_BEGIN(Convert)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Fire)
	DECLARE_RCP(Image)
	DECLARE_EMT(Error)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
		DEFINE_RCP(Image)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - FFT.  FFT processing node.
//

class FFT :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	FFT ( void );											// Constructor

	// Run-time data
	IDictionary	*pImg;									// Image dictionary
	adtString	strWnd;									// Window function
	adtBool		bZeroDC;									// Zero DC component
	cv::Mat		*pWnd;									// Window function

	// CCL
	CCL_OBJECT_BEGIN(FFT)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_EMT(Error)
	DECLARE_CON(Fire)
	DECLARE_RCP(Image)
	DECLARE_RCP(Window)
	BEGIN_BEHAVIOUR()
		DEFINE_EMT(Error)
		DEFINE_CON(Fire)
		DEFINE_RCP(Image)
		DEFINE_RCP(Window)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - Normalize.  Image normalization node.
//

class Normalize :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Normalize ( void );									// Constructor

	// Run-time data
	IDictionary	*pImg;									// Image dictionary
	adtValue		vFrom,vTo;								// Normalization range

	// CCL
	CCL_OBJECT_BEGIN(Normalize)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_EMT(Error)
	DECLARE_CON(Fire)
	DECLARE_RCP(Image)
	BEGIN_BEHAVIOUR()
		DEFINE_EMT(Error)
		DEFINE_CON(Fire)
		DEFINE_RCP(Image)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - Persist.  Image persistence node.
//

class PersistImage :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	PersistImage ( void );								// Constructor

	// Run-time data
	IDictionary	*pDctImg;								// Image dictionary
	adtString	strLoc;									// Persistence location

	// CCL
	CCL_OBJECT_BEGIN(PersistImage)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Load)
	DECLARE_CON(Save)
	DECLARE_RCP(Image)
	DECLARE_RCP(Location)
	DECLARE_EMT(Error)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Load)
		DEFINE_CON(Save)
		DEFINE_RCP(Image)
		DEFINE_RCP(Location)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - Prepare.  Image preparation for processing.
//

class Prepare :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Prepare ( void );										// Constructor

	// Run-time data
	IDictionary	*pImg;									// Image dictionary
	bool			bCuda,bOcl;								// Cuda/OpenCl enabled

	// CCL
	CCL_OBJECT_BEGIN(Prepare)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_RCP(Image)
	DECLARE_CON(Download)
	DECLARE_CON(Upload)
	DECLARE_EMT(Error)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Image)
		DEFINE_CON(Download)
		DEFINE_CON(Upload)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - Threshold.  Thresholding node.
//

class Threshold :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Threshold ( void );									// Constructor

	// Run-time data
	IDictionary	*pImg;									// Image dictionary
	adtValue		vT;										// Treshold value
	adtString	strOp;									// Operation

	// CCL
	CCL_OBJECT_BEGIN(Threshold)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_EMT(Error)
	DECLARE_CON(Fire)
	DECLARE_RCP(Image)
	DECLARE_RCP(Value)
	BEGIN_BEHAVIOUR()
		DEFINE_EMT(Error)
		DEFINE_CON(Fire)
		DEFINE_RCP(Image)
		DEFINE_RCP(Value)
	END_BEHAVIOUR_NOTIFY()
	};


// Prototypes
HRESULT image_fft			( cv::Mat *, cv::Mat *, bool = false, bool = false );
//HRESULT image_fft			( cv::ocl::oclMat *, bool = false, bool = false );
HRESULT image_from_mat	( cv::Mat *, IDictionary * );
HRESULT image_load		( const WCHAR *, IDictionary * );
HRESULT image_save		( IDictionary *, const WCHAR * );
HRESULT image_to_mat		( IDictionary *, cv::Mat ** );

#endif
