////////////////////////////////////////////////////////////////////////
//
//										FLYCAPTUREL_.H
//
//				Implementation include file for USB library
//
////////////////////////////////////////////////////////////////////////

#ifndef	FLYCAPTUREL__H
#define	FLYCAPTUREL__H

// Includes
#include "flycapturel.h"
#include "../../../lib/nspcl/nspcl.h"

// 3rd party includes
#include "\Program Files (x86)\Point Grey Research\FlyCapture2\include\FlyCapture2.h"

///////////
// Objects
///////////


/////////
// Nodes
/////////

//
// Class - Camera.  Point grey / FlyCapture SDK single camera implementation.
//

class Camera :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Camera ( void );										// Constructor

	// Run-time data
	IDictionary		*pDctImg;							// Image dictionary
	IMemoryMapped	*pBits;								// Memory for bits
	U32				iIdx;									// Camera index
	FlyCapture2::Camera
						cam;									// Internal camera object
	bool				bRun;									// Capture running
	sysEvent			evGrab;								// Grab event

	// CCL
	CCL_OBJECT_BEGIN(Camera)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_EMT(Error)
//	DECLARE_CON(Grab)
	DECLARE_EMT(Image)
	DECLARE_RCP(Index)
	DECLARE_CON(Info)
	DECLARE_CON(Start)
	DECLARE_RCP(Stop)
	BEGIN_BEHAVIOUR()
		DEFINE_EMT(Error)
//		DEFINE_CON(Grab)
		DEFINE_EMT(Image)
		DEFINE_RCP(Index)
		DEFINE_CON(Info)
		DEFINE_CON(Start)
		DEFINE_RCP(Stop)
	END_BEHAVIOUR_NOTIFY()

	private :

	// Internal utilities
				void onImage ( FlyCapture2::Image * );
	static	void onImage ( FlyCapture2::Image *, const void * );
	};

//
// Class - Enum.  Point grey enumeration implementation.
//

class Enum :
	public CCLObject,										// Base class
	public IBehaviour										// Interface
	{
	public :
	Enum ( void );											// Constructor
	virtual ~Enum ( void ) {}							// Destructor

	// Run-time data
	U32			iIdx;										// Enumeration index

	// CCL
	CCL_OBJECT_BEGIN(Enum)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_RCP(First)
	DECLARE_CON(Next)
	DECLARE_EMT(End)
	BEGIN_BEHAVIOUR()
		DEFINE_RCP(First)
		DEFINE_CON(Next)
		DEFINE_EMT(End)
	END_BEHAVIOUR_NOTIFY()

	private :
	};

// Utilities
HRESULT pgrError ( FlyCapture2::Error );

#endif
