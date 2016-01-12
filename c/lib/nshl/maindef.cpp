////////////////////////////////////////////////////////////////////////
//
//									MAINDEF.CPP
//
//						Default main for nSpace.
//
////////////////////////////////////////////////////////////////////////

#include "nshl_.h"
#ifdef	_WIN32
#include <commctrl.h>
#endif

// Globals

HRESULT WINAPI defMain ( IDictionary *pDctCmd )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Entry point into program.
	//
	//	PARAMETERS
	//		-	pDctCmd contains command line information
	//
	//	RETURN VALUE	
	//		0 on failure
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;
	Shell		*pShell	= NULL;

	// Initialize common controls for Win32
	#if	!defined(__NOGDI__) || (UNDER_CE >= 400)
	INITCOMMONCONTROLSEX	icce;
	memset ( &icce, 0, sizeof(icce) );
	icce.dwSize = sizeof(icce);
	icce.dwICC	= ICC_TREEVIEW_CLASSES;
	InitCommonControlsEx ( &icce );
	#endif

	// Shell object
	CCLTRYE	( (pShell = new Shell(pDctCmd)) != NULL, E_OUTOFMEMORY );
	CCLOK		( pShell->AddRef(); )
	CCLTRY	( pShell->construct() );

	// Run shell
	if (hr == S_OK && pShell->tickBegin() == S_OK)
		{
		// Continue until shell exits
		while (pShell->tick() == S_OK)
			{
			}	// while

		}	// if

	// Clean up
	if (pShell != NULL)
		{
		pShell->tickEnd();
		_RELEASE(pShell);
		}	// if

	return hr;
	}	// defMain
