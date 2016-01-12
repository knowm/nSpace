////////////////////////////////////////////////////////////////
//
//									RECEPX.CPP
//
//			Implementation of the external receptor helper object
//
////////////////////////////////////////////////////////////////

#include "nshl_.h"

ReceptorX :: ReceptorX ( NamespaceX *_pSpc, const WCHAR *_pRoot, 
								ILocation *_pLoc )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	//	PARAMETERS
	//		-	_pSpc is the parent object
	//		-	_pRoot is the root of the connected location
	//		-	_pLoc is the target location
	//
	////////////////////////////////////////////////////////////////////////
	pSpc		= _pSpc;
	strRoot	= _pRoot;
	pLoc		= _pLoc;
	AddRef();

	// 'Own' the root to ensure own copy of path.
	strRoot.at();
	}	// ReceptorX

ReceptorX :: ~ReceptorX ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Destructor for the node
	//
	////////////////////////////////////////////////////////////////////////
//	dbgprintf ( L"ReceptorX::~ReceptorX\r\n" );
	}	// ReceptorX

HRESULT ReceptorX :: receive ( IReceptor *pRcp, const WCHAR *pl, 
											const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IReceptor
	//
	//	PURPOSE
	//		-	Called when a location receives a value.
	//
	//	PARAMETERS
	//		-	pRcp is the receptor
	//		-	pl is the location
	//		-	v is the emitted value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
//	dbgprintf ( L"ReceptorX::receive:Path %s:%s {\r\n", (LPCWSTR)strRoot, pl );

	// Thread proection
	csRx.enter();

	// Notify parent
	if (pSpc != NULL)
//		pSpc->received ( strRoot, pl, v );
		pSpc->received ( strRoot, pl, &(varX = v) );

	// Thread proection
	csRx.leave();

//	dbgprintf ( L"} ReceptorX::receive:Path %s:%s\r\n", (LPCWSTR)strRoot, pl );
	return (pSpc != NULL) ? S_OK : ERROR_INVALID_STATE;
	}	// receive

