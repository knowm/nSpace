////////////////////////////////////////////////////////////////////////
//
//									GLBNSPC.CPP
//
//						Implementation of the global nSpace object.
//
////////////////////////////////////////////////////////////////////////

#include "nspcl_.h"
#include <stdio.h>

// For direct creation of ADT objects to avoid DLL unloading race conditions
#include "../adtl/adtl_.h"

// Globals
GlobalNspc	nspcglb;

GlobalNspc :: GlobalNspc ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	refcnt		= 0;
	pStkAbs		= NULL;
	pItAbs		= NULL;

	// AddRef self
	AddRef();
	}	// GlobalNspc

GlobalNspc :: ~GlobalNspc ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Destructor for the object.
	//
	////////////////////////////////////////////////////////////////////////

	// In case it did not get called
	refcnt = 1;
	Release();
	}	// GlobalNspc

ULONG GlobalNspc :: AddRef ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called to increase reference on the object.
	//
	//	RETURN VALUE
	//		Reference count
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;
	Stack		*pStk		= NULL;

	// Reference count
	if (++refcnt > 1)
		return refcnt;

	// Class factories
//	CCLTRY ( cclGetFactory ( L"nSpace.Adt.Dictionary", IID_IClassFactory, 
//										  (void **) &pcfDct ) );
	
	// Create stack and iterator for strings
	CCLTRYE ( (pStk = new Stack()) != NULL, E_OUTOFMEMORY );
	CCLOK   ( pStk->AddRef(); )
	CCLTRY  ( pStk->construct() );
	CCLTRY  ( pStk->iterate ( &pItAbs ) );

	// Assign to internal variable
	if (hr == S_OK)
		{
		pStkAbs = pStk;
		_ADDREF(pStkAbs);
		}	// if

	// Clean up
	_RELEASE(pStk);

	// Debug
	if (hr != S_OK)
		lprintf ( LOG_ERR, L"Unable to initialize global objects, 0x%x" );

	return refcnt;
	}	// AddRef

ULONG GlobalNspc :: Release ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Decrement reference count on object.
	//
	////////////////////////////////////////////////////////////////////////

	// Reference count
	if (refcnt == 0 || --refcnt > 0)
		return refcnt;

	// Clean up
	_RELEASE(pStkAbs);
	_RELEASE(pItAbs);

	return 0;
	}	// destruct

