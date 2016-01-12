////////////////////////////////////////////////////////////////////////
//
//									ENUM.CPP
//
//				Implementation of the WBEM enumertor node
//
////////////////////////////////////////////////////////////////////////

#include "wmil_.h"
#include <stdio.h>

// Globals

Enumerator :: Enumerator ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	pEnum		= NULL;
	pObj		= NULL;
	pNames	= NULL;
	lEnumIdx	= 0;
	}	// Enumerator

HRESULT Enumerator :: onAttach ( bool bAttach )
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
		// Clean up
		if (pNames != NULL)
			{
			SafeArrayDestroy ( pNames );
			pNames	= NULL;
			}	// if
		_RELEASE(pEnum);
		_RELEASE(pObj);
		}	// else

	return hr;
	}	// onAttach

HRESULT Enumerator :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

	// First
	if (_RCP(First))
		{
		// Previous state
		if (pNames != NULL)
			{
			SafeArrayDestroy ( pNames );
			pNames = NULL;
			}	// if

		// Class object enumerator
		// This may fail but let enumerator continue to keep model of first/next
		if (hr == S_OK && pEnum != NULL)
			pEnum->Reset();

		// Class object
		else if (hr == S_OK && pObj != NULL)
			{
			// Obtain list of names ('BeginEnumeration' does not seem to work ?)
			CCLTRY ( pObj->GetNames ( NULL, 0, NULL, &pNames ) );
			CCLOK  ( lEnumIdx = 0; )
			}	// else if

		// Invalid
		else
			hr = ERROR_INVALID_STATE;

		// Result
		if (hr == S_OK)
			hr = receive ( prNext, pl, v );
		else
			_EMT(Last,adtInt(hr));
		}	// if

	// Next item
	else if (_RCP(Next))
		{
		IWbemClassObject	*pEnumObj	= NULL;
		BSTR					bstrName		= NULL;
		ULONG					lRet;
		adtValue				vL;

		// Next object
		if (hr == S_OK && pEnum != NULL)
			{
			CCLTRY ( pEnum->Next ( 60000, 1, &pEnumObj, &lRet )) ;
			CCLTRYE( lRet == 1, E_UNEXPECTED );
			CCLTRY ( adtValue::copy ( adtIUnknown(pEnumObj), vL ) );
			}	// if
		else if (hr == S_OK && pNames != NULL)
			{
			BSTR			bstr	= NULL;

			// Access property string
			CCLTRY ( SafeArrayGetElement ( pNames, &lEnumIdx, &bstr ) );
			CCLOK  ( ++lEnumIdx; )

			// Result
			CCLOK  ( adtValue::copy ( adtString(bstr), vL ); )
			}	// else if
		else
			hr = ERROR_INVALID_STATE;

		// Result
		if (hr == S_OK)
			_EMT(Next,vL );
		else
			_EMT(Last,adtInt(hr) );

		// Clean up
		_FREEBSTR(bstrName);
		_RELEASE(pEnumObj);
		}	// else if

	// State
	else if (_RCP(Enumerator))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pEnum);
		_QISAFE(unkV,IID_IEnumWbemClassObject,&pEnum);
		if (pEnum == NULL)
			_QISAFE(unkV,IID_IWbemClassObject,&pObj);
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive
