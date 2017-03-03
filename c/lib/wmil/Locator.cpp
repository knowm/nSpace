////////////////////////////////////////////////////////////////////////
//
//									LOCATOR.CPP
//
//				Implementation of the WBEM locator node
//
////////////////////////////////////////////////////////////////////////

#include "wmil_.h"
#include <stdio.h>

// Globals

Locator :: Locator ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	}	// Locator

HRESULT Locator :: onAttach ( bool bAttach )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Called when this behaviour is assigned to a node
	//
	//	PARAMETERS
	//		-	bAttach is true for attachment, false for detachment.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Attach
	if (bAttach)
		{
		}	// if

	// Detach
	else
		{
		}	// else

	return hr;
	}	// onAttach

HRESULT Locator :: onReceive ( IReceptor *pr, const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	The node has received a value on the specified receptor.
	//
	//	PARAMETERS
	//		-	pr is the receptor
	//		-	v is the value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Connection
	if (_RCP(Connect))
		{
		IWbemLocator	*pLoc		= NULL;
		IWbemServices	*pSvc		= NULL;
		BSTR				bstrRsrc	= NULL;
		adtString		strRsrc ( L"root\\cimv2" );

		// Create object
		CCLTRY ( CoCreateInstance ( CLSID_WbemLocator, NULL,
												CLSCTX_ALL, IID_IWbemLocator, (void **) &pLoc ) );

		// Connector
		CCLTRY ( strRsrc.toBSTR ( &bstrRsrc ) );
		CCLTRY ( pLoc->ConnectServer ( bstrRsrc, NULL, NULL, NULL, 0,
													NULL, NULL, &pSvc ) );


		// Result
		if (hr == S_OK)
			_EMT(Connect,adtIUnknown(pSvc) );
		else
			_EMT(Error,adtInt(hr) );

		// Clean up
		_RELEASE(pLoc);
		_FREEBSTR(bstrRsrc);
		}	// if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive
