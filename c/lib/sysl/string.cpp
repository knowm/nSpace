////////////////////////////////////////////////////////////////////////
//
//									STRING.CPP
//
//						String management implementation
//
////////////////////////////////////////////////////////////////////////

#include "sysl.h"
#include <stdio.h>
#ifdef	__APPLE__
#import <libkern/OSAtomic.h>
#endif

// Invalid reference count for debugging
#define	REFCNTINV	0xffeeddcc

S32 sysStringAddRef ( sysSTRING *pstr )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Add a reference to an existing string.
	//
	//	PARAMETERS
	//		-	pstr is the string
	//
	//	RETURN VALUE
	//		Ptr. to memory
	//
	////////////////////////////////////////////////////////////////////////

	// Debug
	#ifdef	_DEBUG
	if (pstr == NULL || pstr->refcnt == 0 || (U32)pstr->refcnt == REFCNTINV)
		{
		dbgprintf ( L"sysStringAddRef::AddRef string with invalid reference count (%d)\r\n",
						(pstr != NULL) ? pstr->nalloc : -1 );
		#ifdef	_WIN32
		DebugBreak();
		#endif
		}	// if
	#endif

	// State check
	if (pstr == NULL) return 0;

	// Increment
	return InterlockedIncrement ( &(pstr->refcnt) );
	}	// sysStringAddRef

S32 sysStringRelease ( sysSTRING *pstr )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Remove a reference to an existing string.
	//
	//	PARAMETERS
	//		-	pstr is the string
	//
	//	RETURN VALUE
	//		Ptr. to memory
	//
	////////////////////////////////////////////////////////////////////////

	// Debug
	#ifdef	_DEBUG
	if (pstr != NULL && (pstr->refcnt == 0 || (U32)pstr->refcnt == REFCNTINV))
		{
		dbgprintf ( L"sysStringRelease::Releasing string with no reference count (%d)\r\n",
						pstr->nalloc );
		#ifdef	_WIN32
		DebugBreak();
		#endif
		}	// if
	#endif

	// State check
	if (pstr == NULL || pstr->refcnt == 0)
		return 0;

	// Decrement
	if (!InterlockedDecrement ( &(pstr->refcnt) ))
		{
		// String is now invalid, release structure
		pstr->refcnt = REFCNTINV;
		_FREEMEM(pstr);
		return 0;
		}	// if

	return pstr->refcnt;
	}	// sysStringRelease

sysSTRING *sysStringAlloc ( const wchar_t *pStr )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Allocate and initialize a string to contain the specified
	//			characters.
	//
	//	PARAMETERS
	//		-	pstr is the string
	//
	//	RETURN VALUE
	//		Ptr. to memory
	//
	////////////////////////////////////////////////////////////////////////
	return sysStringAllocLen ( pStr, (pStr) ? (U32)wcslen(pStr) : 0 );
	}	// sysStringAlloc

sysSTRING *sysStringAllocLen ( const wchar_t *pStr, U32 cch )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Allocate a string of the specified length and optionally
	//			initialize it with an initial set of characters
	//
	//	PARAMETERS
	//		-	pstr is the string
	//		-	cch is the number of character to allocate for the string
	//
	//	RETURN VALUE
	//		Ptr. to memory
	//
	////////////////////////////////////////////////////////////////////////
	sysSTRING	*ret;

	// Allocate memory for the structure and the string (plus null termination)
	if ( (ret = (sysSTRING *) _ALLOCMEM (	sizeof(sysSTRING) +
														((cch+1)*sizeof(wchar_t)) ) ) != NULL )
		{
		wchar_t	*cp,*str;

		// Initialize structure
		ret->refcnt	= 1;
		ret->nalloc = cch;
		str			= (wchar_t *) &(ret[1]);
		str[0]		= wchar_t('\0');

		// Copy requested length from source string if specified
		if (pStr != NULL)
			{
			// Copy
			for (cp = str;cch > 0 && *pStr != 0;++cp,++pStr,--cch)
				*cp = *pStr;
			*cp = '\0';
			}	// if

		}	// if

	return ret;
	}	// sysStringAllocLen

sysSTRING *sysStringReallocLen ( sysSTRING *str, U32 cch )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Reallocate space for the provided string to the specified 
	//			number of characters.
	//
	//	PARAMETERS
	//		-	str is the current string
	//		-	cch is the new number of character to reallocate for the string
	//
	//	RETURN VALUE
	//		Ptr. to memory
	//
	////////////////////////////////////////////////////////////////////////
	sysSTRING	*ret	= NULL;

	// For a new string use 'alloc' to avoid duplicating initialization code
	if (str == NULL)
		ret = sysStringAllocLen ( NULL, cch );

	// Allocate memory for the structure and the string (plus null termination)
	else if ( (ret = (sysSTRING *) sysMemRealloc ( str, sizeof(sysSTRING) +
												((cch+1)*sizeof(wchar_t)) ) ) != NULL )
		ret->nalloc = cch;

	return ret;
	}	// sysStringReallocLen
