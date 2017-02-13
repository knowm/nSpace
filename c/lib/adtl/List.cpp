////////////////////////////////////////////////////////////////////////
//
//										LIST.CPP
//
//					Implementation of the list container class
//
////////////////////////////////////////////////////////////////////////

#include "adtl_.h"
#include <stdio.h>

/////////////
// 'List'
/////////////

List :: List ( void )
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
	pVals		= NULL;
	iCnt		= 0;
	iAlloc	= 0;
	}	// List

HRESULT List :: addObject ( const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Adds the specified object to the end of list.
	//
	//	PARAMETERS
	//		-	v is the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;

	// Thread safe
	if (hr == S_OK && cs.enter())
		{
		// Need more space ?
		if (hr == S_OK && (iCnt+1) >= iAlloc)
			{
			// Re-size the value array
			CCLTRYE ( (pVals = (ADTVALUE *) _REALLOCMEM(pVals,
						(iAlloc+10)*sizeof(ADTVALUE))) != NULL, E_OUTOFMEMORY );
			CCLOK   ( memset ( &(pVals[iAlloc]), 0, 10*sizeof(ADTVALUE) ); )
			CCLOK   ( iAlloc += 10; )
			}	// if

		// Place value at next entry
		CCLTRY ( adtValue::copy ( v, pVals[iCnt] ) );
		CCLOK  ( iCnt++; )

		cs.leave();
		}	// if

	return hr;
	}	// addObject

HRESULT List :: clear ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IContainer
	//
	//	PURPOSE
	//		-	Resets the container.
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Thread safe
	if (hr == S_OK && cs.enter())
		{
		// Might as well keep the array allocated so don't free memory here,
		// just release the values.

		// Clear list
		for (U32 i = 0;i < iCnt;++i)
			adtValue::clear(pVals[i]);
		iCnt		= 0;

		cs.leave();
		}	// if

	return hr;
	}	// clear

HRESULT List :: clone ( IUnknown **ppUnk )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ICloneable
	//
	//	PURPOSE
	//		-	Clones the object.
	//
	//	PARAMETERS
	//		-	ppUnk will receive the clone
	//
	//	RETURN
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	IList			*pLst	= NULL;
	IIt			*pIt	= NULL;
	adtValue		vV;

	// Create target object
	CCLTRY ( COCREATE ( L"Adt.List", IID_IList, &pLst ) );

	// Iterate and store values into target
	CCLTRY ( iterate ( &pIt ) );
	while (hr == S_OK && pIt->read ( vV ) == S_OK)
		{
		adtValue	vVc;

		// Clone value
		CCLTRY ( adtValue::clone ( vV, vVc ) );

		// Store in target
		CCLTRY ( pLst->write ( vVc ) );

		// Next key
		pIt->next();
		}	// while

	// Result
	(*ppUnk) = pLst;
	_ADDREF(*ppUnk);

	// Clean up
	_RELEASE(pIt);
	_RELEASE(pLst);
		
	return hr;
	}	// clone

HRESULT List :: copyTo ( IContainer *pCont )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ICopyTo
	//
	//	PURPOSE
	//		-	Copies values from this container to another container.
	//
	//	PARAMETERS
	//		-	pCont will receive the values
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	return E_NOTIMPL;
	}	// copyTo

void List :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed.
	//
	////////////////////////////////////////////////////////////////////////

	// Clear and free memory
	clear();
	_FREEMEM(pVals);
	iAlloc = 0;
	}	// destruct

HRESULT List :: isEmpty ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IContainer
	//
	//	PURPOSE
	//		-	Returns the empty state of the container
	//
	//	RETURN VALUE
	//		S_OK if container is empty
	//
	////////////////////////////////////////////////////////////////////////
	return (iCnt <= 0) ? S_OK : S_FALSE;
	}	// isEmpty

HRESULT List :: iterate ( IIt **ppIt )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IContainer
	//
	//	PURPOSE
	//		-	Returns an iterator for the container.
	//
	//	PARAMETERS
	//		-	ppIt will receive a ptr. to the iterator
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Create iterator
	CCLTRYE ( ((*ppIt) = new ListIt ( this, false )) != NULL, E_OUTOFMEMORY );
	_ADDREF((*ppIt));

	return hr;
	}	// iterator

HRESULT List :: keys ( IIt **ppIt )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IDictionary
	//
	//	PURPOSE
	//		-	Returns an object to iterate through the keys in the tree.
	//
	//	PARAMETERS
	//		-	ppIt will receive the iterator
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Create iterator
	CCLTRYE ( ((*ppIt) = new ListIt ( this, true )) != NULL, E_OUTOFMEMORY );
	_ADDREF((*ppIt));

	return hr;
	}	// keys

HRESULT List :: load ( const ADTVALUE &vKey, ADTVALUE &vValue )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IDictionary
	//
	//	PURPOSE
	//		-	Loads a value from the dictionary with the given key.
	//
	//	PARAMETERS
	//		-	vKey is the key
	//		-	vValue will receive the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr		= S_OK;

	// Thread safety
	if (cs.enter())
		{
		// Do not check the key type in order to allow for 'path' access to values.
		adtLong	lIdx(vKey);

		// Check range of index
		CCLTRYE ( lIdx >= 1 && lIdx <= iCnt, ERROR_NOT_FOUND );

		// Return vlaue
		CCLOK ( adtValue::copy ( pVals[(int)lIdx-1], vValue ); )

		// Thread safe
		cs.leave();
		}	// if

	return hr;
	}	// load

HRESULT List :: remove ( const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IContainer
	//
	//	PURPOSE
	//		-	Removes an item from the container identified by the specified
	//			value.
	//
	//	PARAMETERS
	//		-	v identifies the value to remove
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr		= S_OK;

	// TODO: Separate remove by Key and remove by vals ?

	// Thread safe
	if (hr == S_OK && cs.enter())
		{
		int	iIdx	= -1;

		// Find item
		for (U32 i = 0;i < iCnt && iIdx == -1;++i)
			if (adtValue::compare ( pVals[i], v ) == 0)
				iIdx = i;

		// Found ?
		CCLTRYE ( iIdx != -1, ERROR_NOT_FOUND );

		// Debug
		#ifdef	_DEBUG
//		WCHAR dbgbufr[MAX_PATH];
//		swprintf ( dbgbufr, L"List::remove:pF %p, nentries %d\n", pF, nentries );
//		OutputDebugString ( dbgbufr );
		#endif

		// Remove from list
		if (hr == S_OK)
			{
			// Clear existing entry
			adtValue::clear ( pVals[iIdx] );

			// Move remaining entries down by one
			if (iCnt > 1)
				memmove ( &(pVals[iIdx]), &(pVals[iIdx+1]), (iCnt-1)*sizeof(ADTVALUE) );

			// One less value
			--iCnt;
			}	// if

		cs.leave();
		}	// if

	return hr;
	}	// remove

HRESULT List :: size ( U32 *s )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IContainer
	//
	//	PURPOSE
	//		-	Returns the # of items in the container.
	//
	//	PARAMETERS
	//		-	s will return the size of the container
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	(*s) = iCnt;
	return S_OK;
	}	// size

HRESULT List :: store ( const ADTVALUE &vKey, const ADTVALUE &vValue )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IDictionary
	//
	//	PURPOSE
	//		-	Stores a value in the dictionary with the given key.
	//
	//	PARAMETERS
	//		-	vKey is the key
	//		-	vValue is the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr		= S_OK;

	// Thread safe
	if (cs.enter())
		{
		// Do not check the key type in order to allow for 'path' access to values.
		adtLong	lIdx(vKey);

		// Check range of index
		CCLTRYE ( lIdx >= 1 && lIdx <= iCnt, E_INVALIDARG );

		// Replace value
		CCLTRY ( adtValue::copy ( vValue, pVals[(int)lIdx-1] ) );

		cs.leave();
		}	// if

	return hr;
	}	// store

HRESULT List :: write ( const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IList
	//
	//	PURPOSE
	//		-	Writes the specified object to the end of the list and
	//			increments the iterator
	//
	//	PARAMETERS
	//		-	v is the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT hr			= S_OK;

	// Thread safe
	if (hr == S_OK && cs.enter())
		{
		hr = addObject ( v );
		cs.leave();
		}	// if

	return hr;
	}	// write

////////////////////
// 'ListIt'
////////////////////

ListIt :: ListIt ( List *_cont, bool _bKeys )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IIt
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	//	PARAMETERS
	//		-	_cont is a ptr. to the container for this iterator.
	//		-	_bKeys is true to iterate keys, false for values
	//
	////////////////////////////////////////////////////////////////////////
	cont		= _cont; cont->AddRef();				// Keep container alive
	bKeys		= _bKeys;
	iIdx		= 0;
	}	// ListIt

HRESULT ListIt :: begin ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IIt
	//
	//	PURPOSE
	//		-	Resets the iterator position within the container.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Iteration index
	iIdx = 0;

	return hr;
	}	// begin

void ListIt :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed.
	//
	////////////////////////////////////////////////////////////////////////
	cont->Release();
	}	// destruct

HRESULT ListIt :: end ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IIt
	//
	//	PURPOSE
	//		-	Resets the iterator position within the container.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Iteration index
	iIdx = cont->iCnt;

	return hr;
	}	// end

HRESULT ListIt :: next ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IIt
	//
	//	PURPOSE
	//		-	Moves to the next position within the container.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Thread safe
	if (hr == S_OK && cont->cs.enter())
		{
		// Valid position ?
		CCLTRYE ( iIdx < (int)cont->iCnt,	E_UNEXPECTED );

		// Next postion
		CCLOK ( ++iIdx; )

		cont->cs.leave();
		}	// if

	return hr;
	}	// next

HRESULT ListIt :: prev ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IIt
	//
	//	PURPOSE
	//		-	Moves to the previous position within the container.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr = S_OK;

	// Thread safe
	if (hr == S_OK && cont->cs.enter())
		{
		// Valid position ?
		CCLTRYE ( iIdx > 0,	E_UNEXPECTED );

		// Previous postion
		CCLOK ( --iIdx; )

		cont->cs.leave();
		}	// if

	return hr;
	}	// prev

HRESULT ListIt :: read ( ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IIt
	//
	//	PURPOSE
	//		-	Reads the next item from the stream and moves to the next one.
	//
	//	PARAMETERS
	//		-	v will receive the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr = S_OK;

	// Thread safe
	if (hr == S_OK && cont->cs.enter())
		{
		// Valid position ?
		CCLTRYE ( iIdx >= 0 && iIdx < (int)cont->iCnt,	E_UNEXPECTED );

		// Return key/value
		CCLTRY ( (bKeys) ?	adtValue::copy ( adtInt(iIdx+1), v ) :
									adtValue::copy ( cont->pVals[iIdx], v ) );

		cont->cs.leave();
		}	// if

 	return hr;
	}	// read

