////////////////////////////////////////////////////////////////////////
//
//										PCLL_.H
//
//	Implementation include file for Point Cloud Library (PCL) node library
//
////////////////////////////////////////////////////////////////////////

#ifndef	PCLL__H
#define	PCLL__H

// Includes
#include "pcll.h"
#include "../../lib/nspcl/nspcl.h"

// Point Cloud Library - Needs SDK_PCL defined
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

///////////
// Objects
///////////
/*
//
// Class - cvMatRef.  Object to cache reference counted OpenCV matrices.
//

class cvMatRef :
	public CCLObject										// Base class
	{
	public :
	cvMatRef ( void );									// Constructor
	virtual ~cvMatRef ( void );						// Destructor

	// Utilities
	bool isGPU	( void ) { return (gpumat != NULL); }
	bool isUMat ( void ) { return (umat != NULL); }
	S32	rows ( void )
			{ return (mat != NULL) ? mat->rows :
						(umat != NULL) ? umat->rows :
						(gpumat != NULL) ? gpumat->rows : 0; }
	S32	cols ( void )
			{ return (mat != NULL) ? mat->cols :
						(umat != NULL) ? umat->cols :
						(gpumat != NULL) ? gpumat->cols : 0; }
	S32	channels ( void )
			{ return (mat != NULL) ? mat->channels() :
						(umat != NULL) ? umat->channels() :
						(gpumat != NULL) ? gpumat->channels() : 0; }

	// Run-time data
	cv::Mat				*mat;								// CPU matrix
	cv::cuda::GpuMat	*gpumat;							// GPU matrix
	cv::UMat				*umat;							// Universal/OpenCL matrix

	// CCL
	CCL_OBJECT_BEGIN_INT(cvMatRef)
	CCL_OBJECT_END()
	};
*/

/////////
// Nodes
/////////

//
// Class - Create.  Create new objects.
//

class Create :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Create ( void );										// Constructor

	// Run-time data
	IDictionary	*pDct;									// Target dictionary
	adtInt		iW,iH,iSz;								// Size

	// CCL
	CCL_OBJECT_BEGIN(Create)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_RCP(Dictionary)
	DECLARE_CON(Fire)
	DECLARE_RCP(Height)
	DECLARE_EMT(Error)
	DECLARE_RCP(Size)
	DECLARE_RCP(Width)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Dictionary)
		DEFINE_CON(Fire)
		DEFINE_RCP(Height)
		DEFINE_EMT(Error)
		DEFINE_RCP(Size)
		DEFINE_RCP(Width)
	END_BEHAVIOUR_NOTIFY()
	};

//
// Class - ImageToCloud.  Convert image data to point cloud using
//		supported rules.
//

class ImageToCloud :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	ImageToCloud ( void );								// Constructor

	// Run-time data
	IDictionary	*pDct;									// Image dictionary
	adtString	strX,strY,strZ;						// Axis specifications
	IIt			*pItX,*pItY,*pItZ;					// Pre-assigned coordinates
	
	// CCL
	CCL_OBJECT_BEGIN(ImageToCloud)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_RCP(Dictionary)
	DECLARE_CON(Fire)
	DECLARE_RCP(Image)
	DECLARE_EMT(Error)
	DECLARE_RCP(Xaxis)
	DECLARE_RCP(Yaxis)
	DECLARE_RCP(Zaxis)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Dictionary)
		DEFINE_CON(Fire)
		DEFINE_RCP(Image)
		DEFINE_EMT(Error)
		DEFINE_RCP(Xaxis)
		DEFINE_RCP(Yaxis)
		DEFINE_RCP(Zaxis)
	END_BEHAVIOUR_NOTIFY()
	};

#endif
