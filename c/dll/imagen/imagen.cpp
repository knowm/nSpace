////////////////////////////////////////////////////////////////////////
//
//									IMAGE.CPP
//
//				Main file for the image node library
//
////////////////////////////////////////////////////////////////////////

#define	CCL_OBJ_PREFIX		L"nSpace"
#define	CCL_OBJ_MODULE		L"Image"

// Library implementations
#include "../../lib/imagel/imagel_.h"

// Objects in this module
CCL_OBJLIST_BEGIN()
	// Objects

	// Nodes
	CCL_OBJLIST_ENTRY	(At)
	CCL_OBJLIST_ENTRY	(Binary)
	CCL_OBJLIST_ENTRY	(CascadeClassifier)
	CCL_OBJLIST_ENTRY	(Codec)
	CCL_OBJLIST_ENTRY	(Contours)
	CCL_OBJLIST_ENTRY	(Convert)
	CCL_OBJLIST_ENTRY	(Create)
	CCL_OBJLIST_ENTRY	(Distance)
	CCL_OBJLIST_ENTRY	(Draw)
	CCL_OBJLIST_ENTRY	(FaceRecognizer)
	CCL_OBJLIST_ENTRY	(Features)
	CCL_OBJLIST_ENTRY	(Flip)
	CCL_OBJLIST_ENTRY	(FFT)
	CCL_OBJLIST_ENTRY	(Gradient)
	CCL_OBJLIST_ENTRY	(Match)
	CCL_OBJLIST_ENTRY	(Morph)
	CCL_OBJLIST_ENTRY	(Normalize)
	CCL_OBJLIST_ENTRY	(PersistImage)
	CCL_OBJLIST_ENTRY	(Prepare)
	CCL_OBJLIST_ENTRY	(Resize)
	CCL_OBJLIST_ENTRY	(Roi)
	CCL_OBJLIST_ENTRY	(Smooth)
	CCL_OBJLIST_ENTRY	(Stats)
	CCL_OBJLIST_ENTRY	(Threshold)
	CCL_OBJLIST_ENTRY	(VideoCapture)
	CCL_OBJLIST_ENTRY	(VideoWriter)

CCL_OBJLIST_END()

