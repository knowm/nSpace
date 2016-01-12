////////////////////////////////////////////////////////////////////////
//
//									SERVICE.CPP
//
//				Implementation of the WBEM services node
//
////////////////////////////////////////////////////////////////////////

#include "wmil_.h"
#include <stdio.h>

// Globals

Service :: Service ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pSvc	= NULL;
	}	// Service

HRESULT Service :: onAttach ( bool bAttach )
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
		adtValue	vL;

		// Default query string
		if (pnDesc->load ( adtString(L"Text"), vL ) == S_OK)
			strTxt = vL;		
		}	// if

	// Detach
	else
		{
		// Clean up
		_RELEASE(pSvc);
		}	// else

	return hr;
	}	// onAttach

HRESULT Service :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

	// Query
	if (_RCP(Query))
		{
		IEnumWbemClassObject	*pEnum	= NULL;
		BSTR						bstrLang	= NULL;
		BSTR						bstrTxt	= NULL;
		adtString				strLang(L"WQL");

		// State check
		CCLTRYE ( pSvc != NULL, ERROR_INVALID_STATE );
		CCLTRYE ( strTxt.length() > 0, ERROR_INVALID_STATE );

		// Needed strings
		CCLTRY ( strLang.toBSTR ( &bstrLang ) );
		CCLTRY ( strTxt.toBSTR ( &bstrTxt ) );

		// Execute query
		CCLTRY ( pSvc->ExecQuery ( bstrLang, bstrTxt, WBEM_FLAG_FORWARD_ONLY,
											NULL, &pEnum ) );

		// Result
		if (hr == S_OK)
			_EMT(Query,adtIUnknown(pEnum) );
		else
			_EMT(Error,adtInt(hr) );

		// Clean up
		_RELEASE(pEnum);
		_FREEBSTR(bstrTxt);
		_FREEBSTR(bstrLang);
		}	// if

	// State
	else if (_RCP(Service))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pSvc);
		_QISAFE(unkV,IID_IWbemServices,&pSvc);
		}	// else if
	else if (_RCP(Text))
		strTxt = v;
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive
