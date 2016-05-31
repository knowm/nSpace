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
//#include <opencv2/gpu/gpu.hpp>
#if		CV_MAJOR_VERSION == 3
#include <opencv2/core/ocl.hpp>
#endif

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

//
// Class - cvMatRef.  Object to cache reference counted OpenCV matrices.
//

class cvMatRef :
	public CCLObject										// Base class
	{
	public :
	cvMatRef ( void );									// Constructor

	// Run-time data
	#if	CV_MAJOR_VERSION == 3
	cv::UMat		*mat;
	#else
	cv::Mat		*mat;										// Matrix - CPU
	#endif

	// CCL
	CCL_OBJECT_BEGIN_INT(cvMatRef)
	CCL_OBJECT_END()
	virtual void		destruct	( void );			// Destruct object

	};

/////////
// Nodes
/////////

//
// Class - At.  Node to address individual pixels.
//

class At :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	At ( void );											// Constructor

	// Run-time data
	IDictionary	*pImg;									// Image dictionary
	adtInt		iX,iY;									// Positions
	adtValue		vAt;										// Value

	// CCL
	CCL_OBJECT_BEGIN(At)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_EMT(Error)
	DECLARE_RCP(Image)
	DECLARE_CON(Load)
	DECLARE_RCP(X)
	DECLARE_CON(Store)
	DECLARE_RCP(Y)
	DECLARE_RCP(Value)
	BEGIN_BEHAVIOUR()
		DEFINE_EMT(Error)
		DEFINE_RCP(Image)
		DEFINE_CON(Load)
		DEFINE_RCP(X)
		DEFINE_CON(Store)
		DEFINE_RCP(Y)
		DEFINE_RCP(Value)
	END_BEHAVIOUR_NOTIFY()
	};

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
	adtValue		vRes;										// Result
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
// Class - Create.  Create a new image.
//

class Create :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Create ( void );										// Constructor

	// Run-time data
	IDictionary	*pImg;									// Image dictionary
	adtString	strFmt;									// Format
	adtInt		iW,iH;									// Size

	// CCL
	CCL_OBJECT_BEGIN(Create)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_CON(Fire)
	DECLARE_RCP(Format)
	DECLARE_RCP(Height)
	DECLARE_RCP(Image)
	DECLARE_EMT(Error)
	DECLARE_RCP(Width)
	BEGIN_BEHAVIOUR()
		DEFINE_CON(Fire)
		DEFINE_RCP(Format)
		DEFINE_RCP(Height)
		DEFINE_RCP(Image)
		DEFINE_EMT(Error)
		DEFINE_RCP(Width)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - Flip.  Image flip node.
//

class Flip :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Flip ( void );											// Constructor

	// Run-time data
	IDictionary	*pImg;									// Image dictionary
	adtBool		bHorz,bVert;							// Flip options

	// CCL
	CCL_OBJECT_BEGIN(Flip)
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
	cvMatRef		*pWnd;									// Window function
	sysCS			csSync;									// Thread safety

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
// Class - Line.  Line drawing node.
//

class Line :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Line ( void );											// Constructor

	// Run-time data
	IDictionary	*pImg;									// Image dictionary
	adtInt		iR,iB,iG;								// Color
	adtInt		iX0,iX1,iY0,iY1;						// Endpoints
	adtInt		iThick;									// Thickness

	// CCL
	CCL_OBJECT_BEGIN(Line)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_EMT(Error)
	DECLARE_CON(Fire)
	DECLARE_RCP(Image)
	DECLARE_RCP(X0)
	DECLARE_RCP(Y0)
	DECLARE_RCP(X1)
	DECLARE_RCP(Y1)
	BEGIN_BEHAVIOUR()
		DEFINE_EMT(Error)
		DEFINE_CON(Fire)
		DEFINE_RCP(Image)
		DEFINE_RCP(X0)
		DEFINE_RCP(Y0)
		DEFINE_RCP(X1)
		DEFINE_RCP(Y1)
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
	DECLARE_CON(Release)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Image)
		DEFINE_CON(Download)
		DEFINE_CON(Upload)
		DEFINE_EMT(Error)
		DEFINE_CON(Release)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Internal utilities
	HRESULT gpuInit ( void );
	};

//
// Class - Roi.  Region of interest node.
//

class Roi :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Roi ( void );											// Constructor

	// Run-time data
	IDictionary	*pSrc,*pDst;							// Image dictionaries
	adtInt		iL,iT,iR,iB;							// ROI
	adtBool		bCopy;									// Copy ROI into own image

	// CCL
	CCL_OBJECT_BEGIN(Roi)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_RCP(Bottom)
	DECLARE_RCP(Destination)
	DECLARE_EMT(Error)
	DECLARE_CON(Fire)
	DECLARE_RCP(Left)
	DECLARE_RCP(Right)
	DECLARE_RCP(Source)
	DECLARE_RCP(Top)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Bottom)
		DEFINE_RCP(Destination)
		DEFINE_EMT(Error)
		DEFINE_CON(Fire)
		DEFINE_RCP(Left)
		DEFINE_RCP(Right)
		DEFINE_RCP(Source)
		DEFINE_RCP(Top)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - Stats.  Compute statistics about an image.
//

class Stats :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Stats ( void );										// Constructor

	// Run-time data
	IDictionary	*pImg;									// Image dictionary

	// CCL
	CCL_OBJECT_BEGIN(Stats)
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
//HRESULT image_fft			( cv::ocl::oclMat *, bool = false, bool = false );
HRESULT image_load		( const WCHAR *, IDictionary * );
HRESULT image_save		( IDictionary *, const WCHAR * );
#if		CV_MAJOR_VERSION == 3
HRESULT image_fft			( cv::UMat *, cv::UMat *, bool = false, bool = false );
HRESULT image_from_mat	( cv::UMat *, IDictionary * );
HRESULT image_to_mat		( IDictionary *, cv::UMat ** );
#else
HRESULT image_fft			( cv::Mat *, cv::Mat *, bool = false, bool = false );
HRESULT image_from_mat	( cv::Mat *, IDictionary * );
HRESULT image_to_mat		( IDictionary *, cv::Mat ** );
#endif

// From 'mathl'
HRESULT mathOp			( const WCHAR *, int * );

#endif
