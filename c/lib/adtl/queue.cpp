////////////////////////////////////////////////////////////////////////
//
//									QUEUE.CPP
//
//					Implementation of a queue container class
//
////////////////////////////////////////////////////////////////////////

#include "adtl_.h"

//////////////
// 'Queue'
//////////////

Queue :: Queue ( void )
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

	// Setup
	pQ			= NULL;
	fidx		= 0;
	bidx		= 0;
	qsize		= 0;
	}	// Queue

HRESULT Queue :: clear ( void )
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
//	dbgprintf ( L"Queue::clear:%d,%d\r\n", fidx, bidx );
	if (hr == S_OK && cs.enter())
		{
		// Clear out values
		for (U32 i = 0;i < qsize;++i)
			if (pQ[i] != NULL)
				delete pQ[i];

		// Done
		_FREEMEM(pQ);
		qsize	= 0;
		fidx	= bidx = 0;
		cs.leave();
		}	// if

	return S_OK;
	}	// clear

HRESULT Queue :: copyTo ( IContainer *pCont )
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

void Queue :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed.
	//
	////////////////////////////////////////////////////////////////////////
//	dbgprintf ( L"Queue::destruct:%d,%d\r\n", fidx, bidx );

	// Default
	CCLObject::destruct();

	// Do not call clear in the destructor, mutex issues
	for (U32 i = 0;i < qsize;++i)
		if (pQ[i] != NULL)
			delete pQ[i];
	_FREEMEM(pQ);
	}	// destruct

HRESULT Queue :: isEmpty ( void )
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
	return (fidx == bidx) ? S_OK : S_FALSE;
	}	// isEmpty

HRESULT Queue :: iterate ( IIt **ppIt )
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
	(*ppIt) = new QueueIt ( this );
	(*ppIt)->AddRef();
	return S_OK;
	}	// iterator

HRESULT Queue :: remove ( const ADTVALUE &v )
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
	return E_NOTIMPL;;
	}	// remove

HRESULT Queue :: resize ( U32 newsize )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Makes room in the queue for the specified # of items.
	//
	//	PARAMETERS
	//		-	newsize specifies the new size of the array
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;
	adtValue	**pNewQ	= NULL;
	U32		src,dst;

	// Already room ?
	if (!newsize || newsize <= qsize) return S_OK;
//	dbgprintf ( L"Queue::resize:%d -> %d\r\n", qsize, newsize );

	// Allocate new array
	CCLTRYE ( (pNewQ = (adtValue **) _ALLOCMEM(newsize*sizeof(adtValue *))) != NULL,
					E_OUTOFMEMORY );

	// Copy current entries (assumes new queue is bigger)
	if (hr == S_OK)
		{
		for (src = bidx,dst = 0;src != fidx;++dst,src=(src+1)%qsize)
			pNewQ[dst] = pQ[src];
		bidx	= 0;
		fidx	= dst;
		for (;dst < newsize;++dst)
			pNewQ[dst] = NULL;
		}	// if

	// Done
	if (hr == S_OK)
		{
		_FREEMEM(pQ);
		pQ		= pNewQ;
		qsize	= newsize;
		}	// if

	return hr;
	}	// resize

HRESULT Queue :: size ( U32 *s )
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
	(*s) = (qsize) ? ((fidx+qsize-bidx)%qsize) : 0;
	return S_OK;
	}	// size

HRESULT Queue :: write ( const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IOutIt
	//
	//	PURPOSE
	//		-	Writes the specified object to the end of the list and
	//			increments the iterator
	//
	//	PARAMETERS
	//		-	v is the value to write
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;

	// Thread safe
	if (hr == S_OK && cs.enter())
		{
		// Need more room ?
		if (	!qsize ||
				((fidx+1)%qsize == bidx) )
			hr = resize(qsize+100);

		// Store in front of queue
		CCLTRYE	( (pQ != NULL), E_UNEXPECTED );
		CCLTRYE	( (pQ[fidx] = new adtValue()) != NULL, E_OUTOFMEMORY );
		CCLOK		( adtValue::copy ( v, *(pQ[fidx]) ); )
		CCLOK		( fidx = (fidx+1)%qsize; )
		cs.leave();
		}	// if

	return hr;
	}	// write

///////////////////
// 'QueueIt'
///////////////////

QueueIt :: QueueIt ( Queue *_queue )
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
	//		-	_queue is a ptr. to the container for this iterator.
	//
	////////////////////////////////////////////////////////////////////////
	queue	= _queue; queue->AddRef();					// Keep container alive
	}	// QueueIt

QueueIt :: ~QueueIt ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Destructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	queue->Release();
	}	// QueueIt

HRESULT QueueIt :: begin ( void )
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
	// Reads always from beginning, writes always at end
	return S_OK;
	}	// begin

HRESULT QueueIt :: end ( void )
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
	// Reads always from beginning, writes always at end
	return S_OK;
	}	// end

HRESULT QueueIt :: next ( void )
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
	if (hr == S_OK && queue->cs.enter())
		{
		// Move to next item in queue
		CCLTRYE	( (queue->bidx != queue->fidx) , ERROR_NOT_FOUND );
		if (hr == S_OK && queue->pQ[queue->bidx] != NULL)
			{
			delete queue->pQ[queue->bidx];
			queue->pQ[queue->bidx] = NULL;
			}	// if
		CCLOK		( queue->bidx = (queue->bidx+1)%queue->qsize; )
		CCLTRYE	( (queue->bidx != queue->fidx) , ERROR_NOT_FOUND );
		queue->cs.leave();
		}	// if

	return hr;
	}	// next

HRESULT QueueIt :: prev ( void )
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
	// Previous item gone...
	return ERROR_NOT_FOUND;
	}	// prev

HRESULT QueueIt :: read ( ADTVALUE &v )
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
	//		-	v will receive a ptr to the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;

	// Thread safe
	if (hr == S_OK && queue->cs.enter())
		{
		// Available ?
		CCLTRYE( (queue->bidx != queue->fidx) , ERROR_NOT_FOUND );
		CCLTRYE( (queue->pQ != NULL), E_UNEXPECTED );
		CCLOK ( adtValue::copy ( *(queue->pQ[queue->bidx]), v ); )
		queue->cs.leave();
		}	// if

	return hr;
	}	// read
