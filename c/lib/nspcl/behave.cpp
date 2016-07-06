////////////////////////////////////////////////////////////////////////
//
//									BEHAVE.CPP
//
//					Implementation of the behaviour wrapper class
//
////////////////////////////////////////////////////////////////////////

#include "nspcl_.h"

Behaviour :: Behaviour ( IBehaviour *_pBehave )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pBehave = _pBehave;
	_ADDREF(pBehave);

	// Auto add-ref self
	AddRef();
	}	// Behaviour

HRESULT Behaviour :: attach ( IDictionary *pnLoc, bool bAttach )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	FROM	IBehaviour
	//
	//	PURPOSE
	//		-	Called when the behaviour is attached to a location.
	//
	//	PARAMETERS
	//		-	pnLoc is the location
	//		-	bAttach is true to attach, false to detach
	//
	//	RETURN Behaviour
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT hr = S_OK;

	// Thread safety
	csRx.enter();

	// Contained object
	hr = (pBehave != NULL) ? pBehave->attach ( pnLoc, bAttach ) : E_UNEXPECTED;

	// Thread safety
	csRx.leave();

	return hr;
	}	// attach

void Behaviour :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		CCLObject
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed
	//
	////////////////////////////////////////////////////////////////////////
	_RELEASE(pBehave);
	}	// destruct

HRESULT Behaviour :: receive ( IReceptor *pr, const WCHAR *pl, 
											const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	FROM	IReceptor
	//
	//	PURPOSE
	//		-	A location has received a Behaviour on the specified receptor.
	//
	//	PARAMETERS
	//		-	pr is the receptor
	//		-	pl is the location
	//		-	v is the Behaviour
	//
	//	RETURN Behaviour
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT hr = S_OK;

	// Thread safety
	csRx.enter();

	// Contained object
	hr = (pBehave != NULL) ? pBehave->receive ( pr, pl, v ) : E_UNEXPECTED;

	// Thread safety
	csRx.leave();

	return hr;
	}	// receive

