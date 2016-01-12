////////////////////////////////////////////////////////////////////////
//
//									DL.CPP
//
//							Dynamic library object
//
////////////////////////////////////////////////////////////////////////

#include "sysl.h"
#if      __unix__ || __APPLE__
#include <dlfcn.h>
#include	<sys/time.h>
#endif

sysDl :: sysDl ( const wchar_t *_pwLib )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	//	PARAMETERS
	//		-	_pwLib is the path to the library
	//
	////////////////////////////////////////////////////////////////////////

	// Setup
	pwLib		= _pwLib;
	lRefCnt	= 0;

	#ifdef	_WIN32
	hLib	= NULL;
	#elif		__unix__ || __APPLE__
	pvLib	= NULL;
	#endif
	}	// sysDl

sysDl :: ~sysDl ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Destructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	#ifdef	_WIN32
	if (hLib != NULL) 
		FreeLibrary ( hLib );
	#elif		__unix__ || __APPLE__
	if (pvLib != NULL)
		dlclose ( pvLib );
	#endif
	}	// ~sysDl

LONG sysDl :: AddRef ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Adds a reference count to the library.  Attempts load
	//			on first count.
	//
	//	RETURN VALUE
	//		Current reference count
	//
	////////////////////////////////////////////////////////////////////////
	// Protect
	if (!cs.enter())
		return -1;

	// Next count
	++lRefCnt;

	// First one ?
	if (lRefCnt == 1)
		{
		#ifdef	_WIN32
		hLib = LoadLibrary ( pwLib );
		if (hLib == NULL)
			{
			dbgprintf ( L"sysDl::AddRef:Unable to load library:%d:%s\r\n",
								GetLastError(), pwLib );
			lRefCnt = 0;
			}	// if
		#elif		__unix__ || __APPLE__
		char	cLib[1024];
		wcstombs ( cLib, pwLib, sizeof(cLib) );
		pvLib = dlopen ( cLib, RTLD_NOW|RTLD_LOCAL );
		if (pvLib == NULL)
			lRefCnt = 0;
		#endif
		}	// if

	// Clean up
	cs.leave();

	return lRefCnt;
	}	// AddRef

LONG sysDl :: Release ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Adds a reference count to the library.  Attempts load
	//			on first count.
	//
	//	RETURN VALUE
	//		Current reference count
	//
	////////////////////////////////////////////////////////////////////////

	// Protect
	if (!cs.enter())
		return -1;

	// Next count
	--lRefCnt;

	// First one ?
	if (lRefCnt == 0)
		{
		#ifdef	_WIN32
		if (hLib != NULL) 
			{
			FreeLibrary ( hLib );
			hLib = NULL;
			}	// if
		#elif		__unix__ || __APPLE__
		if (pvLib != NULL)
			{
			dlclose ( pvLib );
			pvLib = NULL;
			}	// if
		#endif

		}	// if

	// Clean up
	cs.leave();

	return lRefCnt;
	}	// Release
