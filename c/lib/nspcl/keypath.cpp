////////////////////////////////////////////////////////////////////////
//
//									KEYPATH.CPP
//
//				Implementation of the key path management node
//
////////////////////////////////////////////////////////////////////////

#include "nspcl_.h"
#include <stdio.h>

KeyPath :: KeyPath ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pDct		= NULL;
	}	// KeyPath

void KeyPath :: destruct ( void )
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
	_RELEASE(pDct);
	}	// destruct

HRESULT KeyPath :: onAttach ( bool bAttach )
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
	HRESULT		hr		= S_OK;
	adtIUnknown	unkV;
	adtString	strEmit;
	adtValue		v;

	// Attach
	if (bAttach)
		{
		// Attributes
		if (pnDesc->load ( adtString(L"Key"), v ) == S_OK)
			strKey = v;
		pnDesc->load ( adtString(L"Value"), vValue );
		}	// if

	// Detach
	else
		{
		_RELEASE(pDct);
		}	// else

	return hr;
	}	// onAttach

HRESULT KeyPath :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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
	HRESULT		hr		= S_OK;

	// Load value at path
	if (_RCP(Load))
		{
		// State check
		CCLTRYE ( pDct != NULL, ERROR_INVALID_STATE );

		// Attempt access
		CCLTRY ( nspcLoadPath ( pDct, strKey, vL ) );

		// Result
		if (hr == S_OK)
			_EMT(Load,vL);
		else
			_EMT(NotFound,strKey);
		}	// if

	// Store value at path
	else if (_RCP(Store))
		{
		const ADTVALUE	*puseV	= (!adtValue::empty(vValue)) ? &vValue : &v;

		// State check
		CCLTRYE ( pDct != NULL, ERROR_INVALID_STATE );

		// Store value
		CCLTRY ( nspcStoreValue ( pDct, strKey, *puseV ) );

		// Result
		CCLOK ( _EMT(Store,(unkV=pDct) ); )
		}	// else if

	// Visit
	else if (_RCP(Visit))
		{
		// State check
		CCLTRYE ( pDct != NULL, ERROR_INVALID_STATE );

		// Perform visit
		CCLOK ( visit ( pDct, pl, L"/" ); )

		// Done
		CCLOK ( _EMT(VisitEnd,v); )
		}	// else if

	// State
	else if (_RCP(Dictionary))
		{
		_RELEASE(pDct);
		CCLTRY(_QISAFE((unkV=v),IID_IDictionary,&pDct) );
		}	// else if
	else if (_RCP(Key))
		strKey = v;
	else if (_RCP(Value))
		hr = adtValue::copy ( v, vValue );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

HRESULT KeyPath :: visit ( IDictionary *pDct, const WCHAR *pl, const WCHAR *wPath )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Perform a visitation of all paths in the dictionary.
	//
	//	PARAMETERS
	//		-	pDct is the dictionary to visit
	//		-	pl is the location path
	//		-	wPath is the current path of the provided dictionary
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	IIt			*pIt		= NULL;
	adtValue		vK;

	// State check
	CCLTRYE ( pDct != NULL, ERROR_INVALID_STATE );

	// Visit values in graph
	CCLTRY ( pDct->keys ( &pIt ) );
	while (hr == S_OK && pIt->read ( vK ) == S_OK)
		{
		IDictionary	*pDctV	= NULL;
		adtString	strKey,strPath(wPath);
		adtValue		vV;
		adtIUnknown	unkV;

		// Announce current path
		CCLTRY ( adtValue::toString ( vK, strKey ) );
		CCLTRY ( strPath.append ( strKey ) );
		CCLTRY ( strPath.append ( L"/" ) );
		CCLOK  ( _EMT(Visit,strPath); )

		// If value at current key is a dictionary, time to visit that dictionary
		if (hr == S_OK								&&
				pDct->load ( vK, vV ) == S_OK &&
				vV.vtype == VTYPE_UNK			&&
				_QI((unkV=vV),IID_IDictionary,&pDctV) == S_OK)
			visit ( pDctV, pl, strPath );

		// Clean up
		_RELEASE(pDctV);
		pIt->next();
		}	// while

	// Clean up
	_RELEASE(pIt);

	return hr;
	}	// visit
