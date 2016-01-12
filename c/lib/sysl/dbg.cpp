////////////////////////////////////////////////////////////////////////
//
//									DBG.CPP
//
//								Debug routines
//
////////////////////////////////////////////////////////////////////////

#include "sysl.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

// Globals
static wchar_t wDbgBfr[32768];
static wchar_t wFmt[32768];
static sysCS	csDbgprintf;

extern "C"
int dbgprintf ( const wchar_t *format, ... )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Debug string output.
	//
	//	PARAMETERS
	//		-	format is the list of values to format
	//
	//	RETURN VALUE
	//		-	# of characters printed
	//
	////////////////////////////////////////////////////////////////////////
	int		len	= 0;
	va_list	args;

	// Thread protection
	if (!csDbgprintf.enter())
		return 0;
      
	// Format string
	va_start ( args, format );
	#ifdef	_WIN32
	len = _vsnwprintf_s ( wDbgBfr, sizeof(wDbgBfr)/sizeof(wchar_t), _TRUNCATE, format, args );
	#elif		__unix__ || __APPLE__
	// Under WIN32 the '%s' modifier defaults to wide string for wide functions.
	// Under GNUC the '%s' modifier remains as ASCII.  The equivalent modifier
	// is '%ls' or '%S'.  Since Win32 was first, adjust format string for GNUC.
	wcscpy ( wFmt, format );
	for (len = 0;wFmt[len] != wchar_t('\0');++len)
		{
		if 		(wFmt[len] == wchar_t('\%') && wFmt[len+1] == wchar_t('s'))
			wFmt[len+1] = wchar_t('S');
		else if 	(wFmt[len] == wchar_t('\%') && wFmt[len+1] == wchar_t('S'))
			wFmt[len+1] = wchar_t('s');
		}	// for
	len = vswprintf ( wDbgBfr, sizeof(wDbgBfr)/sizeof(wchar_t), wFmt, args );
	#endif
	va_end ( args );

	// Convert to log entry
//	lprintf ( LOG_DBG, wDbgBfr );

	// Output
	#ifdef	_WIN32
	OutputDebugString ( wDbgBfr );
	#elif		__unix__ || __APPLE__
	printf ( "%S", wDbgBfr );
	#endif


	// DEBUG
//	{
//	FILE			*f		= NULL;

	// Access log
//	if ( (f = fopen ( "debug.txt", "a" )) == NULL )
//		f = fopen ( "debug.txt", "w" );

	// Write
//	if (f != NULL) fprintf ( f, "%S", wDbgBfr );

//	if (f != NULL) fclose ( f );
//	}


	// Clean up
	csDbgprintf.leave();

	return len;
	}	// dbgprintf
