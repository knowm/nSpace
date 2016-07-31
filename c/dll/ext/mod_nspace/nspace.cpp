////////////////////////////////////////////////////////////////////////
//
//									NSPACE.CPP
//
//				nSpace side of Apache interface module
//
////////////////////////////////////////////////////////////////////////

#include "nspace.h"

// Globals
static nSpaceLink	*pLink = NULL;

nSpaceLink :: nSpaceLink ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pThrd = NULL;
	pTick	= NULL;
	AddRef();
	}	// nSpaceLink

HRESULT nSpaceLink :: construct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called to construct the object.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr	= S_OK;

	// Worker object
	CCLTRYE ( (pTick = new nSpaceLink_t()) != NULL, E_OUTOFMEMORY );
	CCLOK   ( pTick->pThis = this; )

	// Start worker thread
	CCLTRY(COCREATE(L"Sys.Thread", IID_IThread, &pThrd));
	CCLTRY(pThrd->threadStart(pTick, 10000));

	return hr;
	}	// construct

void nSpaceLink :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		CCLObject
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed
	//
	////////////////////////////////////////////////////////////////////////
	if (pThrd != NULL)
		{
		pThrd->threadStop(5000);
		_RELEASE(pThrd);
		}	// if
	_RELEASE(pTick);
	}	// destruct

HRESULT nSpaceLink :: onReceive (	const WCHAR *pwRoot, 
												const WCHAR *pwLoc,
												const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		nSpaceClientCB
	//
	//	PURPOSE
	//		-	Called when a listened location receives a value.
	//
	//	PARAMETERS
	//		-	pwRoot is the path to the listened location
	//		-	pwLoc is the location relative to the root for the value
	//		-	v is the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	lprintf ( LOG_INFO, L"%s:%s", pwRoot, pwLoc );
	return S_OK;
	}	// onReceive

//
// nSpaceLink_t
//

nSpaceLink_t :: nSpaceLink_t ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pnCli		= NULL;
	pThis		= NULL;
	bWork		= true;

	// Addref self
	AddRef();
	}	// nSpaceLink_t

HRESULT nSpaceLink_t :: tickAbort ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITickable
	//
	//	PURPOSE
	//		-	Notifies the object that 'ticking' should abort.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////

	// Shtudown
	bWork = false;

	return S_OK;
	}	// tickAbort

HRESULT nSpaceLink_t :: tick ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITickable
	//
	//	PURPOSE
	//		-	Perform one 'tick's worth of work.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr	= S_OK;

	// Clean up
	Sleep(1000);

	// Continue ?
	CCLTRYE ( bWork == true, S_FALSE );

	return hr;
	}	// tick

HRESULT nSpaceLink_t:: tickBegin ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITickable
	//
	//	PURPOSE
	//		-	Notifies the object that it should get ready to 'tick'.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Initialize COM
	CCLTRY ( CoInitializeEx ( NULL, COINIT_MULTITHREADED ) );

	// Create nSpace connection
	CCLTRYE ( (pnCli = new nSpaceClient()) != NULL, E_OUTOFMEMORY );
	CCLTRY ( pnCli->open ( L"{ Namespace Apache }", TRUE ) );

	return hr;
	}	// tickBegin

HRESULT nSpaceLink_t :: tickEnd ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		ITickable
	//
	//	PURPOSE
	//		-	Notifies the object that 'ticking' is to stop.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	lprintf ( LOG_INFO, L"End" );

	// Clean up
	if (pnCli != NULL)
		{
		pnCli->close();
		delete pnCli;
		pnCli = NULL;
		}	// if

	// Uninitialize COM
	CoUninitialize();

	return S_OK;
	}	// tickEnd

//
// nSpaceLink
//

bool nspace_init ( bool bInit )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Initialize/uninitialize nSpace link.
	//
	//	PARAMETERS
	//		-	bInit is true to initialize, false to uninitialize.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Debug
	lprintf(LOG_INFO, L"nspace_init %d\r\n", bInit);

	// Initialize
	if (bInit)
		{
		// Initialize COM if it is not already initialized
		CCLTRY ( CoInitializeEx ( NULL, COINIT_MULTITHREADED ) );

		// State check
		CCLTRYE ( (pLink == NULL), ERROR_INVALID_STATE );

		// Create the link object
		CCLTRYE ( (pLink = new nSpaceLink()) != NULL, E_OUTOFMEMORY );
		CCLTRY  ( pLink->construct() );

		// Clean up
		if (hr != S_OK)
			{
			_RELEASE(pLink);
			}	// if
		}	// if

	// Uninitialize
	else
		{
		// Shutdown
		_RELEASE(pLink);

		// Clean up
		CoUninitialize();
		}	// else

	return (hr == S_OK);
	}	// nspace_init

