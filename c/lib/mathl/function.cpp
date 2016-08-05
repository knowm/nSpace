////////////////////////////////////////////////////////////////////////
//
//									FUNCTION.CPP
//
//					Implementation of the function node
//
////////////////////////////////////////////////////////////////////////

#include "mathl_.h"
#include <stdio.h>
#include <math.h>
/*
Function :: Function ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	strName	= L"";
	pLstP		= NULL;
	}	// Function

HRESULT Function :: onAttach ( bool bAttach )
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
	adtValue	vL;

	// Attach
	if (bAttach)
		{
		adtIUnknown	unkV;

		// Defaults
		if (pnDesc->load ( adtString(L"Function"), vL ) == S_OK)
			strName = vL;

		// Parameter list
		if (pnDesc->load ( adtString(L"Param"), vL ) == S_OK)
			{
			hr = _QISAFE((unkV = vL),IID_IList,&pLstP);
			}	// if
		else
			hr = COCREATE(L"Adt.List",IID_IList,&pLstP);
		}	// if

	// Detach
	else
		{
		// Clean up
		_RELEASE(pLstP);
		}	// else

	return hr;
	}	// onAttach

HRESULT Function :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Evaluate
	if (_RCP(Fire))
		{
		adtValue	vRes;
		IIt		*pIt	= NULL;
		U32		pcnt;

		// State check
		CCLTRYE ( strName.length() > 0, ERROR_INVALID_STATE );

		// Parameter list 
		CCLTRY ( pLstP->size ( &pcnt ) );
		CCLTRY ( pLstP->iterate ( &pIt ) );

		//
		// Evaluate
		//

		// Binary operations
		if (	!WCASECMP(strName,L"+") ||
				!WCASECMP(strName,L"-") ||
				!WCASECMP(strName,L"*") ||
				!WCASECMP(strName,L"/") ||
				!WCASECMP(strName,L"dot") )
			{
			adtValue	vL,vR;

			// State check
			CCLTRYE ( pcnt == 2, E_UNEXPECTED );

			// Parameters
			CCLTRY ( pIt->read ( vL ) );
			CCLOK  ( pIt->next(); )
			CCLTRY ( pIt->read ( vR ) );
			CCLOK  ( pIt->next(); )

			// Perform operation
			CCLTRY ( mathBinary (	!WCASECMP(strName,L"-")		?	MATHOP_SUB :
											!WCASECMP(strName,L"*")		?	MATHOP_MUL :
											!WCASECMP(strName,L"/")		?	MATHOP_DIV : 
											!WCASECMP(strName,L"dot")	?	MATHOP_DOT : 
																					MATHOP_ADD,
											vL, vR, vRes ) );
			}	// if

		// Single param
		else if (	!WCASECMP(strName,L"cos") ||
						!WCASECMP(strName,L"abs") )
			{
			adtValue	vP;

			// State check
			CCLTRYE ( pcnt == 1, E_UNEXPECTED );

			// Parameters
			CCLTRY ( pIt->read ( vP ) );
			CCLOK  ( pIt->next(); )

			// Perform
			if (hr == S_OK)
				{
				if (!WCASECMP(strName,L"cos"))
					adtValue::copy ( adtDouble(cos(adtDouble(vP))), vRes );
				else if (!WCASECMP(strName,L"abs"))
					adtValue::copy ( adtDouble((float)abs(adtDouble(vP))), vRes );
				}	// if
			}	// else if

		else
			{
			dbgprintf ( L"Function::receive:Not implemented:%s\r\n", (LPCWSTR)strName );
			hr = E_NOTIMPL;
			}	// else

		// Clean up
		_RELEASE(pIt);

		// Result
		if (hr == S_OK)
			_EMT(Fire,vRes );
		else
			_EMT(Error,adtInt(hr) );
		}	// if

	// Clear parameters
	else if (_RCP(Clear))
		pLstP->clear();

	// Add parameter to list
	else if (_RCP(Param))
		pLstP->write ( v );

	// State
	else if (_RCP(Function))
		strName = v;
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

*/