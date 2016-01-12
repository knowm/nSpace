////////////////////////////////////////////////////////////////////////
//
//									GLBNSPC.CPP
//
//						Implementation of the global nSpace object.
//
////////////////////////////////////////////////////////////////////////

#include "nspcl_.h"
#include <stdio.h>

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
	pcfDct		= NULL;
	pLstStVal	= NULL;
	pLstLd		= NULL;
	refcnt		= 0;
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
	HRESULT	hr		= S_OK;

	// Reference count
	if (++refcnt > 1)
		return refcnt;

	// Class factories
	CCLTRY ( cclGetFactory ( L"nSpace.Adt.Dictionary", IID_IClassFactory, 
										  (void **) &pcfDct ) );
	
	// Re-usable list for generating token list for 'nspcStoreValue'
	CCLTRY ( COCREATE ( L"Adt.List", IID_IList, &pLstStVal ) );
	CCLTRY ( COCREATE ( L"Adt.List", IID_IList, &pLstLd ) );

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
	_RELEASE(pLstStVal);
	_RELEASE(pLstLd);
	_RELEASE(pcfDct);

	return 0;
	}	// destruct

