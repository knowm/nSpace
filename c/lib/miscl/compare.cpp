////////////////////////////////////////////////////////////////////////
//
//									COMPARE.CPP
//
//					Implementation of the comparison node
//
////////////////////////////////////////////////////////////////////////

#include "miscl_.h"
#include <stdio.h>

Compare :: Compare ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	}	// Compare

HRESULT Compare :: onAttach ( bool bAttach )
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

	// State check
	if (!bAttach) return S_OK;

	// Default states
	pnDesc->load ( adtString(L"Left"),	vLeft );
	pnDesc->load ( adtString(L"Right"), vRight );

	return S_OK;
	}	// onAttach

HRESULT Compare :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

	// Fire
	if (_RCP(Fire))
		{
		int				res		= 0;
		const ADTVALUE	*pvRight	= NULL;

		// Use incoming value if right side was not previously specified
		pvRight = (adtValue::empty ( vRight )) ? &v : &vRight;

//		if (	hr == S_OK && vLeft.vtype == VTYPE_STR && vLeft.pstr != NULL &&
//			!WCASECMP(vLeft.pstr, L"State/Graph/Instance" ))
//		dbgprintf ( L"Hi\r\n" );
//		if (!WCASECMP(strnName,L"IsPort"))
//			dbgprintf ( L"Hi\r\n" );

		// Compare values
		CCLOK ( res = adtValue::compare ( vLeft, *pvRight ); )

		// Valid result ?
		CCLTRYE ( (res >= -1 && res <= +1), ERROR_INVALID_OPERATION );

		// Debug
		#ifdef	_DEBUG
		if (hr != S_OK)
			dbgprintf ( L"Compare::receiveFire:%s:Error! %d (0x%x)\n", (LPCWSTR)strnName, res, hr );
		#endif

		// Results
		if (hr == S_OK)
			{
			// Equal
			if (res == 0)
				_EMT(Equal,*pvRight);

			// Greater than
			else if (res == 1)
				{
				_EMT(NotEqual,*pvRight);
				_EMT(Greater,*pvRight);
				}	// else if

			// Less than
			else if (res == -1)
				{
				_EMT(NotEqual,*pvRight);
				_EMT(Less,*pvRight);
				}	// else if

			}	// if
		else
			_EMT(Error,adtInt(res));
		}	// if

	// Context
	else if (_RCP(Left))
		hr = adtValue::copy ( v, vLeft );
	else if (_RCP(Right))
		hr = adtValue::copy ( v, vRight );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive


