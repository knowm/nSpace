////////////////////////////////////////////////////////////////////////
//
//								MAIN.CPP
//
//					Entry point into application
//
////////////////////////////////////////////////////////////////////////

#define	INITGUID
#include "nsh.h"
#include "../../lib/nshl/nshl_.h"
#include <stdio.h>
#include <stdlib.h>

#ifdef	_WIN32
#include <commctrl.h>

// Memory leak detection
#ifdef	_DEBUG
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#endif
#endif

// Objects in this module (for CCL linkage, no objects created in app)
CCL_OBJLIST_BEGIN()
CCL_OBJLIST_END()

// Prototypes

// For 'modern'/XP look at feel
#if defined(_WIN32) && !defined(_WIN64)
//#pragma comment(linker,"\"/manifestdependency:type='win32' name='Microsoft.Windows.Common-Controls'" \
//					"version='6.0.0.0' processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")
#endif

#ifdef	_WIN32
int WINAPI wWinMain(HINSTANCE hInst, HINSTANCE hPrevInst,
								LPWSTR lpCmdLine, int nCmdShow )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Entry point into program.
	//
	//	PARAMETERS
	//		-	hInst is a handle to the instance of the program
	//		-	hPrevInst is handle to the previous instance of the program
	//		-	lpCmdLine specifies the command line
	//		-	nCmdShow specifies initial show parameters
	//
	//	RETURN VALUE	
	//		0 on failure
	//
	////////////////////////////////////////////////////////////////////////
#else
int main ( int argc, char *argv[] )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Entry point into program.
	//
	//	PARAMETERS
	//		-	argc,argv are the command line parameters
	//
	//	RETURN VALUE	
	//		Exit code
	//
	////////////////////////////////////////////////////////////////////////
	WCHAR	lpCmdLine[1024];
   
	// Normalize command line processing
	WCSCPY ( lpCmdLine, sizeof(lpCmdLine)/sizeof(lpCmdLine[0]), L"" );
	for (int i = 1;i < argc;++i)
		{
		adtString	strArg(argv[i]);

		// Append next argument
		WCSCAT ( lpCmdLine, sizeof(lpCmdLine)/sizeof(lpCmdLine[0]), strArg );
		if (i+1 < argc)
			WCSCAT ( lpCmdLine, sizeof(lpCmdLine)/sizeof(lpCmdLine[0]), L" " );
		}	// for
	dbgprintf ( L"Command line : %s\r\n", lpCmdLine );
 	
#endif
	HRESULT		hr				= S_OK;
	bool			bCoInitd		= false;
	bool			bEmbed		= false;
	bool			bReg			= false;
	bool			bUnreg		= false;
	IDictionary	*pDctCmd		= NULL;
	int			iCmdLine		= 0;

	// Unix
	#if      __unix__ || __APPLE__
/*
	char		cwd[1024];

	// Currently implementation assumes the object database file is
	// located in the current working directory.
	CCLTRYE	( getcwd ( cwd, sizeof(cwd)-13 ) >= 0, E_UNEXPECTED );
	CCLOK		( strcat ( cwd, "/objects.txt" ); )

	// Initialize COM database
	CCLTRY	( CoSetDatabase ( cwd ) );
*/
	#endif

	// Initialize COM
	CCLTRY ( CoInitializeEx ( NULL, COINIT_MULTITHREADED ) );
	CCLOK  ( bCoInitd = true; )

	// Windows/COM has some predefined command line parameters that are
	// passed regardless of design of application.  Look for those
	// first before parsing the command line.
	CCLOK ( bEmbed = (!WCASECMP ( lpCmdLine, L"-Embedding" ) || !WCASECMP ( lpCmdLine, L"/Embedding" )); )
	CCLOK ( bReg	= (!WCASECMP ( lpCmdLine, L"-Register" )	|| !WCASECMP ( lpCmdLine, L"/Register" )); )
	CCLOK ( bUnreg	= (!WCASECMP ( lpCmdLine, L"-Unregister" )|| !WCASECMP ( lpCmdLine, L"/Unregister" )); )

	// Based on type of execute, compute start of "own" command line
	CCLOK ( iCmdLine =	(bEmbed || bReg || bUnreg) ? (int)wcslen(lpCmdLine) : 0; )

	// Command line options
	if (hr == S_OK)
		{
		adtString	strType;
		adtValue		vL;

		// Default(empty) command line
		CCLTRY ( COCREATE(L"Adt.Dictionary",IID_IDictionary,&pDctCmd) );

		// Additional command line ?
		if (iCmdLine < (int)wcslen(lpCmdLine))
			{
			IDictionary	*pDct	= NULL;
			adtIUnknown	unkV;

			// Command line dictionary
			CCLTRY ( strToVal ( adtString(&(lpCmdLine[iCmdLine])), vL ) );
			CCLTRY ( _QISAFE((unkV=vL),IID_IDictionary,&pDct) );
			if (hr == S_OK)
				{
				_RELEASE(pDctCmd);
				pDctCmd = pDct;
				_ADDREF(pDctCmd);
				}	// if

			// Clean up
			_RELEASE(pDct);
			}	// if

		// Command line options provided by Windows
		if (hr == S_OK && (bEmbed || bReg || bUnreg))
			{
			// Embedded type
			CCLTRY ( pDctCmd->store ( adtString(L"Type"), adtString(L"Embedding") ) );
			if (hr == S_OK && bReg)
				hr = pDctCmd->store ( adtString(L"Execute"), adtString(L"Register") );
			else if (hr == S_OK && bUnreg)
				hr = pDctCmd->store ( adtString(L"Execute"), adtString(L"Unregister") );
			}	// if

		// Local instance
		#ifdef	_WIN32
		CCLTRY ( pDctCmd->store ( adtString(L"Instance"), adtLong((U64)hInst) ) );
		#endif

		// Executable type
		if (hr == S_OK && pDctCmd->load ( adtString(L"Type"), vL ) == S_OK)
			strType = vL;

		// Run appropriate main
		if (hr == S_OK)
			{
			#ifdef	_WIN32
			if (!WCASECMP(strType,L"Embedding"))
				hr = xMain ( pDctCmd );
			else if (!WCASECMP(strType,L"Service"))
				hr = svcMain ( pDctCmd );
			else
			#endif
				hr = defMain ( pDctCmd );
			}	// if
		}	// if

	// Clean up
	dbgprintf ( L"nsh::WinMain:hr 0x%x\r\n", hr );
	_RELEASE(pDctCmd);
	if (bCoInitd)	CoUninitialize();
	#ifdef	_DEBUG
	//_CrtDumpMemoryLeaks();
	#endif

	return (hr == S_OK) ? 0 : hr;
	}	// WinMain/main


