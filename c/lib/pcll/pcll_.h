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
#define	NOMINMAX											// Disable unused legacy macros
#include	"../../lib/imagel/imagel.h"

// Point Cloud Library - Needs SDK_PCL defined
#ifdef	_WIN32
#pragma	warning(disable:4996)							// _CRT_SECURE_NO_WARNINGS for PCL/Flann
#endif
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>

///////////
// Objects
///////////

//
// Class - pclObjRef.  Object reference container.
//

class pclObjRef :
	public CCLObject										// Base class
	{
	public :
	pclObjRef ( void ) { AddRef(); }					// Constructor

	// Utilities
//	bool isCloud	( void ) { return (cloud != NULL); }

	// Run-time data
	pcl::PointCloud<pcl::PointXYZ>	cloud;

	// CCL
	CCL_OBJECT_BEGIN_INT(pclObjRef)
	CCL_OBJECT_END()
	};

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
	IDictionary	*pDct;									// Target dictionary
	IDictionary	*pImg;									// Image object
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

	private :

	// Internal utilities
	float getAxisPt ( U8, U32, U32, IIt * );

	};

//
// Class - Normal.  Compute/estimate normals for a cloud.
//

class Normal :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Normal ( void );										// Constructor

	// Run-time data
	IDictionary	*pDct;									// Target dictionary
	
	// CCL
	CCL_OBJECT_BEGIN(Normal)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_RCP(Dictionary)
	DECLARE_CON(Fire)
	DECLARE_EMT(Error)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(Dictionary)
		DEFINE_CON(Fire)
		DEFINE_EMT(Error)
	END_BEHAVIOUR_NOTIFY()
	};

#endif
