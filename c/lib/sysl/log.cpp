////////////////////////////////////////////////////////////////////////
//
//									LOG.CPP
//
//								Logging routines
//
////////////////////////////////////////////////////////////////////////

#include "sysl.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

// Globals
static cLogInfo	*pLogInfo	= NULL;
static WCHAR		wDbgBfr[32768];
static WCHAR		wFmt[32768];

void logOpen ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Access the process global logging information.
	//
	////////////////////////////////////////////////////////////////////////
	#ifdef	_WIN32
	BOOL		bOk		= TRUE;
	BOOL		bNew		= FALSE;
	HANDLE	hMap		= NULL;
	cLogInfo	**ppInfo	= NULL;
	WCHAR		wName[81];

	// Valid ?
	if (pLogInfo != NULL)
		return;

	// Name for shared memory location for this process
	swprintf_s ( wName, L"nSpaceLog_%d", GetCurrentProcessId() );

	// Access pre-defined file mapping
	if (bOk && (hMap = OpenFileMapping ( FILE_MAP_ALL_ACCESS, FALSE, 
			wName )) == NULL)
		{
		// Create a memory mapped file location to hold the info object
		//	so that anyone in the process (other DLLs, etc) can access the
		// functionality.
		if (bOk) bOk = ( (hMap = CreateFileMapping ( INVALID_HANDLE_VALUE, NULL,
						PAGE_READWRITE, 0, sizeof(cLogInfo *), wName )) 
						!= NULL );

		// New location
		bNew = bOk;
		}	// if

	// Get a pointer to the location
	if (bOk) bOk = ((ppInfo = (cLogInfo **) MapViewOfFile ( hMap, 
						FILE_MAP_ALL_ACCESS, 0, 0, 0 )) != NULL);

	// Create new object
	if (bOk && bNew)
		(*ppInfo) = new cLogInfo();

	// Access object
	if (bOk)
		pLogInfo = (*ppInfo);

	// Clean up
	if (ppInfo != NULL)
		UnmapViewOfFile ( ppInfo );
	if (!bNew && hMap != NULL)
		CloseHandle ( hMap );
	#endif
	}	// logOpen

void logSink ( logCallback pCB, void *pvCB )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Set a callback for log entries.
	//	
	//	PARAMETERS
	//		-	pCB is the new callback
	//		-	pvCB is the callback parameter
	//
	////////////////////////////////////////////////////////////////////////

	// Logging
	logOpen();
	if (!pLogInfo)
		return;

	// Assign
	pLogInfo->pCB	= pCB;
	pLogInfo->pvCB	= pvCB;

	// Initial flush
	pLogInfo->csLog.enter();
	pLogInfo->bBusy	= true;
	pLogInfo->flush ( &(pLogInfo->pHead) );
	pLogInfo->bBusy	= false;
	pLogInfo->csLog.leave();
	}	// logSink

extern "C"
int logPrintf ( const WCHAR *file, int line, const WCHAR *func, 
						int level , const WCHAR *fmt, ... )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Log a variable argument string.
	//
	//	PARAMETERS
	//		-	file is the source filename
	//		-	line is the line number
	//		-	func is the function name
	//		-	level is the logging level (LOG_XXX)
	//		-	fmt is the list of values to format
	//
	//	RETURN VALUE
	//		-	# of characters printed
	//
	////////////////////////////////////////////////////////////////////////
	int			len		= 0;
	cLogEntry	*entry	= NULL;
	va_list		args;

	// Logging
	logOpen();

	// Format string to buffer
	va_start ( args, fmt );
	#ifdef	_WIN32
	len = _vsnwprintf_s ( wDbgBfr, sizeof(wDbgBfr)/sizeof(WCHAR), _TRUNCATE, fmt, args );
	#elif		__unix__ || __APPLE__
	// Under WIN32 the '%s' modifier defaults to wide string for wide functions.
	// Under GNUC the '%s' modifier remains as ASCII.  The equivalent modifier
	// is '%ls' or '%S'.  Since Win32 was first, adjust format string for GNUC.
	wcscpy ( wFmt, fmt );
	for (len = 0;wFmt[len] != WCHAR('\0');++len)
		{
		if 		(wFmt[len] == WCHAR('\%') && wFmt[len+1] == WCHAR('s'))
			wFmt[len+1] = WCHAR('S');
		else if 	(wFmt[len] == WCHAR('\%') && wFmt[len+1] == WCHAR('S'))
			wFmt[len+1] = WCHAR('s');
		}	// for
	len = vswprintf ( wDbgBfr, sizeof(wDbgBfr)/sizeof(WCHAR), wFmt, args );
	#endif
	va_end ( args );

	// Logging
	if (pLogInfo != NULL)
		WCSCPY ( pLogInfo->wLogBfr, sizeof(pLogInfo->wLogBfr)/sizeof(pLogInfo->wLogBfr[0]),
					wDbgBfr );

	// Output to debug
	swprintf ( SWPF(wFmt,sizeof(wFmt)/sizeof(WCHAR)),
					#ifdef	_WIN32
					L"<%s:%s> %s",
					#elif		__unix__ || __APPLE__
					L"<%S:%S> %S",
					#endif
					(level == LOG_DBG)  ? L"DBG" :
					(level == LOG_INFO) ? L"INF" :
					(level == LOG_WARN) ? L"WRN" :
					(level == LOG_ERR)  ? L"ERR" : L"FTL", func, wDbgBfr );

	#ifdef	_WIN32
	OutputDebugString ( wFmt );
	if (wFmt[wcslen(wFmt)-1] != '\n')
		OutputDebugString ( L"\r\n" );
	#else
	printf ( "%S", wFmt );
	if (wDbgBfr[wcslen(wFmt)-1] != '\n')
		printf ( "\r\n" );
	#endif

	// Avoid recursive log entries
	if (pLogInfo != NULL && !pLogInfo->bBusy)
		{
		// List protection
		pLogInfo->csLog.enter();

		// Logging system busy
		pLogInfo->bBusy	= true;

		// Create a new log entry and append to end of list
		if ((entry = new cLogEntry ( file, line, func, level, pLogInfo->wLogBfr )) != NULL)
			{
			// Add to list
			entry->next			= pLogInfo->pHead;
			pLogInfo->pHead	= entry;

			// Flush output
			pLogInfo->flush(&(pLogInfo->pHead));
			}	// if

		// Clean up
		pLogInfo->bBusy	= false;
		pLogInfo->csLog.leave();
		}	// if

	return len;
	}	// lprintf

/////////////
// cLogEntry
/////////////

cLogEntry :: cLogEntry ( const WCHAR *_file, int _line, const WCHAR *_func, 
									int _level , const WCHAR *_str )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	//	PARAMETERS
	//		-	file is the source filename
	//		-	line is the line number
	//		-	func is the function name
	//		-	level is the logging level (LOG_XXX)
	//		-	str is the string to log
	//
	////////////////////////////////////////////////////////////////////////
	file	= sysStringAlloc ( _file );
	line	= _line;
	func	= sysStringAlloc ( _func );
	level	= _level;
	str	= sysStringAlloc ( _str );
	next	= NULL;
	}	// cLogEntry

cLogEntry :: ~cLogEntry ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Destructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	if (file != NULL)
		sysStringRelease(file);
	if (func != NULL)
		sysStringRelease(func);
	if (str != NULL)
		sysStringRelease(str);
	}	// cLogEntry

////////////
// cLogInfo
////////////

cLogInfo :: cLogInfo ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	pHead	= NULL;
	pCB	= NULL;
	bBusy	= false;
	}	// cLogInfo

cLogInfo :: ~cLogInfo ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Destructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	cLogEntry	*n,*c;

	// Clear existing log
	for (c = pHead;c != NULL;c = n)
		{
		n = c->next;
		delete c;
		}	// for

	}	// ~cLogInfo

void cLogInfo :: flush ( cLogEntry **pEntry )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Send log entry to callback.
	//
	//	PARAMETERS
	//		-	pEntry is the current entry to send
	//
	////////////////////////////////////////////////////////////////////////

	// Valid callback ?
	if (pLogInfo->pCB == NULL)
		return;

	// Entries are stored newest first so send next entry first
	if ((*pEntry)->next != NULL)
		flush ( &((*pEntry)->next) );

	// Send this log entry
	pLogInfo->pCB ( *pEntry, pLogInfo->pvCB );

	// Done
	delete (*pEntry);
	*pEntry	= NULL;
	}	// flush
