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

// External API
//#include <winusb.h>

///////////
// Objects
///////////


/////////
// Nodes
/////////

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
	IDictionary	*pDctImg;								// Image dictionary
	adtBool		bZeroDC;									// Zero DC component

	// CCL
	CCL_OBJECT_BEGIN(FFT)
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

// Prototypes
HRESULT image_fft		( IDictionary *, bool );
HRESULT image_save	( IDictionary *, const WCHAR * );

#endif
