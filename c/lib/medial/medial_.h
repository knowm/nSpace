////////////////////////////////////////////////////////////////////////
//
//										MEDIAL_.H
//
//				Implementation include file for the media library
//
////////////////////////////////////////////////////////////////////////

#ifndef	MEDIAL__H
#define	MEDIAL__H

// Includes
#include "medial.h"

// SAPI
#include	<sapi.h>

///////////
// Objects
///////////

/////////
// Nodes
/////////

//
// Class - Speak.  Client node for generating plot Speaks.
//								(Currently uses Speak).
//

class Speak :
	public CCLObject,										// Base class
	public Behaviour										// Interface
	{
	public :
	Speak ( void );										// Constructor

	// Run-time data
	adtValue		vSpk;										// Speak value
	ISpVoice		*pVoice;									// SAPI voice

	// CCL
	CCL_OBJECT_BEGIN(Speak)
		CCL_INTF(IBehaviour)
	CCL_OBJECT_END()

	// Connections
	DECLARE_EMT(Error)
	DECLARE_CON(Fire)
	DECLARE_RCP(Value)
	BEGIN_BEHAVIOUR()
		DEFINE_EMT(Error)
		DEFINE_CON(Fire)
		DEFINE_RCP(Value)
	END_BEHAVIOUR_NOTIFY()
	};

#endif
