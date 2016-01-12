////////////////////////////////////////////////////////////////////////
//
//									MEMORY.CPP
//
//						Memory management implementation
//
////////////////////////////////////////////////////////////////////////

#include "sysl.h"
#include <stdio.h>

// Globals
static sysALLOC	*pAlloc	= NULL;

void *sysMemAlloc ( U32 sz )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Allocates a block of memory of the specified size from the
	//			custom or standard heap.
	//
	//	PARAMETERS
	//		-	sz is the amount of memory to allocate
	//
	//	RETURN VALUE
	//		Ptr. to memory
	//
	////////////////////////////////////////////////////////////////////////
	#ifdef	_WIN32
	return (pAlloc != NULL) ? pAlloc->alloc ( sz ) : CoTaskMemAlloc ( sz );
	#else
	return (pAlloc != NULL) ? pAlloc->alloc ( sz ) : malloc ( sz );
	#endif
	}	// sysMemAlloc

void sysMemFree ( void *ptr )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Frees a block of memory.
	//
	//	PARAMETERS
	//		-	ptr is the memory
	//
	////////////////////////////////////////////////////////////////////////
	if (ptr != NULL)
		{
		if (pAlloc != NULL)
			pAlloc->free ( ptr );
		else
			{
			#ifdef	_WIN32
			CoTaskMemFree ( ptr );
			#else
			free ( ptr );
			#endif
			}	// else
		}	// if
	}	// sysMemFree

void *sysMemRealloc ( void *ptr, U32 sz )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Resizes a block of allocated memory.
	//
	//	PARAMETERS
	//		-	ptr is the current block of memory
	//		-	sz is the new size
	//
	//	RETURN VALUE
	//		Ptr. to memory
	//
	////////////////////////////////////////////////////////////////////////
	#ifdef	_WIN32
	return (pAlloc != NULL) ? pAlloc->realloc ( ptr, sz ) : 
				CoTaskMemRealloc ( ptr, sz );
	#else
	return (pAlloc != NULL) ? pAlloc->realloc ( ptr, sz ) : realloc ( ptr, sz );
	#endif
	}	// sysMemAlloc

void sysMemUse ( sysALLOC *_pAlloc )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called to use an alternative set of functions for memory
	//			management.
	//
	//	PARAMETERS
	//		-	_pAlloc specifies the new functions.
	//
	////////////////////////////////////////////////////////////////////////
	pAlloc	= _pAlloc;
	}	// sysMemAlloc
