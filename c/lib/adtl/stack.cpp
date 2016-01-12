////////////////////////////////////////////////////////////////////////
//
//									STACK.CPP
//
//					Implementation of a stack container class
//
////////////////////////////////////////////////////////////////////////

#include "adtl_.h"

//////////////
// 'Stack'
//////////////

Stack :: Stack ( void )
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
	pS			= NULL;
	ssize		= 0;
	salloc	= 0;
	}	// Stack

HRESULT Stack :: clear ( void )
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

	// Reset (might as well keep array allocated)
	cs.enter();
	for (U32 s = 0;s < ssize;++s)
		adtValue::clear ( pS[s] );
	ssize = 0;
	cs.leave();

	return hr;
	}	// clear

HRESULT Stack :: copyTo ( IContainer *pCont )
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

void Stack :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed.
	//
	////////////////////////////////////////////////////////////////////////

	// Default
	CCLObject::destruct();

	// Empty container
	for (U32 s = 0;s < ssize;++s)
		adtValue::clear(pS[s]);
	_FREEMEM(pS);

	}	// destruct

HRESULT Stack :: isEmpty ( void )
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
	return (ssize == 0) ? S_OK : S_FALSE;
	}	// isEmpty

HRESULT Stack :: iterate ( IIt **ppIt )
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
	(*ppIt) = new StackIt ( this );
	(*ppIt)->AddRef();
	return S_OK;
	}	// iterator

HRESULT Stack :: remove ( const ADTVALUE &v )
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

HRESULT Stack :: resize ( U32 newsize )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Makes room in the stack for the specified # of items.
	//
	//	PARAMETERS
	//		-	newsize specifies the new size of the array
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;
	ADTVALUE	*pNewS	= NULL;
	U32		i;

	// Already room ?
	if (!newsize || newsize <= salloc) return S_OK;

	// Allocate new array
	CCLTRYE ( (pNewS = (ADTVALUE *) _ALLOCMEM(newsize*sizeof(adtValue))) != NULL,
					E_OUTOFMEMORY );

	// Copy current entries (assumes new queue is bigger)
	if (hr == S_OK)
		{
		for (i = 0;i < ssize;++i)
			pNewS[i] =  pS[i];
		for (;i < newsize;++i)
			_ADTVINIT(pNewS[i]);
		}	// if

	// Done
	if (hr == S_OK)
		{
		_FREEMEM(pS);
		pS			= pNewS;
		salloc	= newsize;
		}	// if

	return S_OK;
	}	// resize

HRESULT Stack :: size ( U32 *s )
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
	(*s) = (ssize);
	return S_OK;
	}	// size

HRESULT Stack :: write ( const ADTVALUE &v )
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
	//		-	v is the value to write
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;

	// 'Push' item onto stack
	cs.enter();
	CCLTRY(	resize(ssize+10) );
	CCLOK (	adtValue::copy ( v, pS[ssize++] ); )
	cs.leave();

	return hr;
	}	// write

///////////////////
// 'StackIt'
///////////////////

StackIt :: StackIt ( Stack *_stack )
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
	//		-	_stack is a ptr. to the container for this iterator.
	//
	////////////////////////////////////////////////////////////////////////
	stack	= _stack; stack->AddRef();					// Keep container alive
	}	// StackIt

StackIt :: ~StackIt ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Destructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	stack->Release();
	}	// StackIt

HRESULT StackIt :: begin ( void )
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
	// Stack always at 'beginning'
	return S_OK;
	}	// begin

HRESULT StackIt :: end ( void )
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
	return S_OK;
	}	// end

HRESULT StackIt :: next ( void )
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

	stack->cs.enter();
	CCLTRYE	( (stack->ssize > 0) , ERROR_NOT_FOUND );
	CCLOK		( adtValue::clear ( stack->pS[stack->ssize-1] ); )
	CCLOK		( --(stack->ssize); )
	stack->cs.leave();

	return hr;
	}	// next

HRESULT StackIt :: prev ( void )
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

HRESULT StackIt :: read ( ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		IIt
	//
	//	PURPOSE
	//		-	Reads the next item from the container and moves to the next one.
	//
	//	PARAMETERS
	//		-	v will receive the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;

	// 'Peek' at item on stack
	stack->cs.enter();
	CCLTRYE( (stack->ssize > 0) , ERROR_NOT_FOUND );
	CCLOK ( adtValue::copy ( stack->pS[stack->ssize-1], v ); )
	stack->cs.leave();

	return hr;
	}	// read
