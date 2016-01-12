////////////////////////////////////////////////////////////////////////
//
//								CONNLST.CPP
//
//				Implementation of a ConnList list.
//
////////////////////////////////////////////////////////////////////////

#include "nspcl_.h"

/*
// Macros for connector entries
#define	_EMTRCPINIT(a)			(a)->next = NULL; (a)->pRecep = NULL; (a)->refcnt = 1;
#define	_EMTRCPFREE(a)			_FREEMEM((a));
#define	_EMTRCPADDREF(a)		if ((a) != NULL) InterlockedIncrement ( &((a)->refcnt) );
#define	_EMTRCPRELEASE(a)		if ((a) != NULL) { if (!InterlockedDecrement ( &((a)->refcnt) ))	\
											{ _EMTRCPFREE((a)); } (a) = NULL; }
*/

ConnList :: ConnList ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	h	= NULL;
	t	= NULL;
	}	// ConnList

HRESULT ConnList :: add ( IUnknown *punkC, bool bRx )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Add a connector to the list
	//
	//	PARAMETERS
	//		-	punkC is the connector
	//		-	bRx is true if reception should receive values
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;
	CNNE		*e	= NULL;

	// Thread safety
	cs.enter();

	// Allocate new entry
	CCLTRYE ( (e = (CNNE *)_ALLOCMEM(sizeof(CNNE))) != NULL, E_OUTOFMEMORY );
	
	// Add to list
	if (hr == S_OK)
		{
		// Fill structure
		e->next		= NULL;
		e->pConn		= punkC;
		e->bRx		= bRx;

		// Add to list
		if (h == NULL)	h			= e;						// New head
		if (t != NULL) t->next	= e;						// Old tail
		t = e;													// New tail
		}	// if

	// Thread safety
	cs.leave();

	return hr;
	}	// add

HRESULT ConnList :: remove ( IUnknown *punkC )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Remove a connector from the list
	//
	//	PARAMETERS
	//		-	punkC is the connector
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;
	CNNE		*e	= NULL;
	CNNE		*p	= NULL;

	// Thread safety
	cs.enter();

	// Find specified connector
	for (e = h,p = NULL;e != NULL;e = e->next)
		{
		if (e->pConn == punkC)	break;
		else							p = e;
		}	// for

	// Debug
//	if (e == NULL)
//		dbgprintf (L"ConnList::remove:Connector not found:%p\r\n", punkC );

	// Delete entry
	if (e != NULL)
		{
		// Remove from list
		if (p == NULL)		h			= e->next;	// New head
		else					p->next	= e->next;
		if (t == e)			t			= p;			// New tail
		e->next							= NULL;		// No longer in list
		_FREEMEM(e);
		}	// if

	// Thread safety
	cs.leave();

	return hr;
	}	// remove

