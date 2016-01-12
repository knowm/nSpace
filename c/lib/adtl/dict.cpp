////////////////////////////////////////////////////////////////////////
//
//									DICT.CPP
//
//			Implementation of an AVL binary search tree (dictionary)
//
////////////////////////////////////////////////////////////////////////

#include "adtl_.h"
#include <stdio.h>

Dictionary :: Dictionary ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IContainer
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	pCnt	= NULL;
	}	// Dictionary

HRESULT Dictionary :: construct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		CCLObject
	//
	//	PURPOSE
	//		-	Called when the object is being created.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;

	// Create default implementation of a dictionary
//	CCLTRYE ( (pCnt = new KeyValArray()) != NULL, E_OUTOFMEMORY );
	CCLTRYE ( (pCnt = new AVL()) != NULL, E_OUTOFMEMORY );
	CCLOK	  ( pCnt->AddRef(); )
	CCLTRY  ( pCnt->construct() );

	return hr;
	}	// construct

void Dictionary :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed.
	//
	////////////////////////////////////////////////////////////////////////
	_RELEASE(pCnt);
	}	// destruct

//
// Contained functions
//

HRESULT Dictionary :: clear ( void )
	{
	return pCnt->clear();
	}	// clear

HRESULT Dictionary :: clone ( IUnknown **ppUnk )
	{
	return pCnt->clone ( ppUnk );
	}	// clone

HRESULT Dictionary :: copyTo ( IContainer *pCont )
	{
	return pCnt->copyTo ( pCont );
	}	// copyTo

HRESULT Dictionary :: isEmpty ( void )
	{
	return pCnt->isEmpty();
	}	// isEmpty

HRESULT Dictionary :: iterate ( IIt **ppIt )
	{
	return pCnt->iterate ( ppIt );
	}	// iterate

HRESULT Dictionary :: keys ( IIt **ppKeys )
	{
	return pCnt->keys ( ppKeys );
	}	// keys

HRESULT Dictionary :: load ( const ADTVALUE &vKey, ADTVALUE &vValue )
	{
	return pCnt->load ( vKey, vValue );
	}	// load

HRESULT Dictionary :: remove ( const ADTVALUE &v )
	{
	return pCnt->remove ( v );
	}	// remove

HRESULT Dictionary :: size ( U32 *s )
	{
	return pCnt->size ( s );
	}	// size

HRESULT Dictionary :: store ( const ADTVALUE &vKey, const ADTVALUE &vValue )
	{
	return pCnt->store ( vKey, vValue );
	}	// store

