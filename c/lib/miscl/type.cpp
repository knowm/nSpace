////////////////////////////////////////////////////////////////////////
//
//									TYPE.CPP
//
//					Implementation of the type node.
//
////////////////////////////////////////////////////////////////////////

#include "miscl_.h"
#include <stdio.h>

// Round to integers (as opposed to truncating which the default during type cast)
#define	RNDINT(a)	((S32)( (a) + ( ((a) > 0.0) ? 0.5 : -0.5 )))

// String references

Type :: Type ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	}	// Type

HRESULT Type :: getType ( const ADTVALUE &v, IDictionary **ppDct )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Returns a string descriptor of the value type.
	//
	//	PARAMETERS
	//		-	v is the value
	//		-	ppDct will receive the dictionary with the types
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr	= S_OK;

	// Type dictionary
	CCLTRY ( COCREATE ( L"Adt.Dictionary", IID_IDictionary, ppDct ) );

	// Object ?
	if (hr == S_OK && (v.vtype & VTYPE_TYPEMASK) == VTYPE_UNK)
		{
		IDictionary	*pDict	= NULL;
		ILocation	*pLoc		= NULL;
		IList			*pList	= NULL;
		IContainer	*pCont	= NULL;
		IReceptor	*pRecep	= NULL;
		adtIUnknown	unkV(v);

		// Start at the most specific interface and move down

		// Always an object
		(*ppDct)->store ( adtString(L"Object"), adtInt(0) );

		// Location ?
		if (	unkV.punk != NULL &&
				_QI(unkV.punk,IID_ILocation,&pLoc) == S_OK)
			(*ppDct)->store ( adtString(L"Location"), adtInt(0) );

		// Receptor ?
		if (	unkV.punk != NULL &&
				_QI(unkV.punk,IID_IReceptor,&pRecep) == S_OK)
			(*ppDct)->store ( adtString(L"Receptor"), adtInt(0) );

		// Dictionary ?
		if (	unkV.punk != NULL &&
				_QI(unkV.punk,IID_IDictionary,&pDict) == S_OK )
			{
			// Dictionary
			(*ppDct)->store ( adtString(L"Dictionary"), adtInt(0)	);

			// Graph ?  A graph is a special case dictionary
			adtValue	vL;
			if (	pDict->load ( adtString(L"_Type"), vL ) == S_OK &&
					adtValue::type(vL) == VTYPE_STR						&&
					vL.pstr != NULL											&&
					!WCASECMP(vL.pstr,L"Graph") )
				(*ppDct)->store ( adtString(L"Graph"), adtInt(0) );

			}	// if

		// List ?
		if (	unkV.punk != NULL &&
				_QI(unkV.punk,IID_IList,&pList) == S_OK)
			(*ppDct)->store ( adtString(L"List"), adtInt(0) );

		// Container ?
		if (	unkV.punk != NULL &&
				_QI(unkV.punk,IID_IContainer,&pCont) == S_OK)
			(*ppDct)->store ( adtString(L"Container"), adtInt(0) );

		// Clean up
		_RELEASE(pRecep);
		_RELEASE(pCont);
		_RELEASE(pList);
		_RELEASE(pDict);
		_RELEASE(pLoc);
		}	// if

	// Empty
	else if (hr == S_OK && v.vtype == VTYPE_EMPTY)
		(*ppDct)->store ( adtString(L"Empty"), adtInt(0) );
		
	// Non-object
	else if (hr == S_OK)
		{
		// Normal types
		(*ppDct)->store ( adtString(L"Value"), adtInt(0) );
		if (v.vtype == VTYPE_I4)
			(*ppDct)->store ( adtString(L"Integer"), adtInt(0) );
		else if (v.vtype == VTYPE_I8)
			(*ppDct)->store ( adtString(L"Long"), adtInt(0) );
		else if (v.vtype == VTYPE_R4)
			(*ppDct)->store ( adtString(L"Float"), adtInt(0) );
		else if (v.vtype == VTYPE_R8)
			(*ppDct)->store ( adtString(L"Double"), adtInt(0) );
		else if (v.vtype == VTYPE_DATE)
			(*ppDct)->store ( adtString(L"Date"), adtInt(0) );
		else if (v.vtype == VTYPE_BOOL)
			(*ppDct)->store ( adtString(L"Boolean"), adtInt(0) );
		else if (adtValue::type(v) == VTYPE_STR)
			(*ppDct)->store ( adtString(L"String"), adtInt(0) );
		}	// else

	return hr;
	}	// getType

HRESULT Type :: onAttach ( bool bAttach )
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
	pnDesc->load ( adtString(L"Type"), sType );

	return S_OK;
	}	// onAttach

HRESULT Type :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Determine if value is of the current type
	if (_RCP(Fire))
		{
		bool	bMatch	= false;

		// Value to use
		const ADTVALUE	*pUseV	= (!adtValue::empty(vVal)) ? &vVal : &v;

		// Type
		if (hr == S_OK)
			{
			ILocation	*pLoc		= NULL;
			IDictionary	*pDct		= NULL;
			IContainer	*pCnt		= NULL;
			IList			*pLst		= NULL;
			IReceptor	*pRcp		= NULL;
			adtValue		vL;

			// Check for types

			// Objects
			if (!WCASECMP(L"Object",sType))
				bMatch = (	adtValue::type(*pUseV) == VTYPE_UNK && pUseV->punk != NULL );
			else if (!WCASECMP(L"Location",sType))
				bMatch = (	pUseV->vtype == VTYPE_UNK && pUseV->punk != NULL &&
								_QI(pUseV->punk,IID_ILocation,&pLoc) == S_OK );
			else if (!WCASECMP(L"Dictionary",sType))
				bMatch = (	pUseV->vtype == VTYPE_UNK && pUseV->punk != NULL &&
								_QI(pUseV->punk,IID_IDictionary,&pDct) == S_OK);
			else if (!WCASECMP(L"Container",sType))
				bMatch = (	pUseV->vtype == VTYPE_UNK && pUseV->punk != NULL &&
								_QI(pUseV->punk,IID_IContainer,&pCnt) == S_OK);
			else if (!WCASECMP(L"List",sType))
				bMatch = (	pUseV->vtype == VTYPE_UNK && pUseV->punk != NULL &&
								_QI(pUseV->punk,IID_IList,&pLst) == S_OK);
			else if (!WCASECMP(L"Receptor",sType))
				bMatch = (	pUseV->vtype == VTYPE_UNK && pUseV->punk != NULL &&
								_QI(pUseV->punk,IID_IReceptor,&pRcp) == S_OK);

			// Values
			else if (!WCASECMP(L"String",sType))
				bMatch = (adtValue::type(*pUseV) == VTYPE_STR);
			else if (!WCASECMP(L"Integer",sType))
				bMatch = (pUseV->vtype == VTYPE_I4);
			else if (!WCASECMP(L"Long",sType))
				bMatch = (pUseV->vtype == VTYPE_I8);
			else if (!WCASECMP(L"Float",sType))
				bMatch = (pUseV->vtype == VTYPE_R4);
			else if (!WCASECMP(L"Double",sType))
				bMatch = (pUseV->vtype == VTYPE_R8);
			else if (!WCASECMP(L"Date",sType))
				bMatch = (pUseV->vtype == VTYPE_DATE);
			else if (!WCASECMP(L"Boolean",sType))
				bMatch = (pUseV->vtype == VTYPE_BOOL);

			// Clean up
			_RELEASE(pLoc);
			_RELEASE(pDct);
			_RELEASE(pCnt);
			_RELEASE(pLst);
			_RELEASE(pRcp);
			}	// if

		// Result
		if (bMatch)	_EMT(Equal,*pUseV);
		else			_EMT(NotEqual,*pUseV);
		}	// if

	// Convert from one type to another
	else if (_RCP(Convert))
		{
		// Value to use
		const ADTVALUE	*pUseV	= (!adtValue::empty(vVal)) ? &vVal : &v;

		// State check
		CCLTRYE	(sType.length() > 0,E_UNEXPECTED);
		CCLTRYE	(pUseV->vtype != VTYPE_EMPTY,E_UNEXPECTED);

		// Attempt conversion
		if (hr == S_OK)
			{
			// Anything to a string
			if (!WCASECMP ( sType, L"String" ))
				{
				adtString	vStr;
				adtValue::toString ( *pUseV, vStr );
				adtValue::copy ( vStr, vConv );
				}	// if
			else
				{
				// More can be added over time...
				switch (adtValue::type(*pUseV))
					{
					case VTYPE_I4 :
						if (!WCASECMP ( sType, L"Integer" ))
							adtValue::copy ( *pUseV, vConv );
						else if (!WCASECMP ( sType, L"Float" ))
							adtValue::copy ( adtFloat((float)pUseV->vint), vConv );
						else if (!WCASECMP ( sType, L"Double" ))
							adtValue::copy ( adtDouble((double)pUseV->vint), vConv );
						else
							hr = E_NOTIMPL;
						break;

					case VTYPE_I8 :
						if (!WCASECMP ( sType, L"Double" ))
							adtValue::copy ( adtDouble((double)pUseV->vlong), vConv );
						else if (!WCASECMP ( sType, L"Object" ))
							adtValue::copy ( adtIUnknown((IUnknown *)pUseV->vlong), vConv );
						else
							hr = E_NOTIMPL;
						break;

					case VTYPE_R4 :
						if (!WCASECMP ( sType, L"Float" ))
							adtValue::copy ( *pUseV, vConv );
						else if (!WCASECMP ( sType, L"Double" ))
							adtValue::copy ( adtDouble((double)pUseV->vflt), vConv );
						else if (!WCASECMP ( sType, L"Integer" ))
							adtValue::copy ( adtInt(RNDINT(pUseV->vflt)), vConv );
						else
							hr = E_NOTIMPL;
						break;

					case VTYPE_R8 :

						// Double to date, direct conversion
						if (!WCASECMP ( sType, L"Date" ))
							{
							adtValue::copy ( adtDate(pUseV->vdate), vConv );
							}	// if
						else if (!WCASECMP ( sType, L"Float" ))
							adtValue::copy ( adtFloat((float)pUseV->vdbl), vConv );
						else if (!WCASECMP ( sType, L"Integer" ))
							adtValue::copy ( adtInt(RNDINT(pUseV->vdbl)), vConv );
						else if (!WCASECMP ( sType, L"Double" ))
							adtValue::copy ( *pUseV, vConv );
						else
							hr = E_NOTIMPL;
						break;

					case VTYPE_DATE :
						if (!WCASECMP ( sType, L"Double" ))
							adtValue::copy ( adtDouble(pUseV->vdate), vConv );
						else if (!WCASECMP ( sType, L"Date" ))
							adtValue::copy ( *pUseV, vConv );
						else if (!WCASECMP ( sType, L"Integer" ))
							adtValue::copy ( adtInt(RNDINT(pUseV->vdate)), vConv );
						else
							hr = E_NOTIMPL;
						break;

					case VTYPE_STR :
						{
						adtString	s(*pUseV);
						VALUETYPE	type;
						type =	(!WCASECMP ( sType, L"Double"))		? VTYPE_R8 :
									(!WCASECMP ( sType, L"Float"))		? VTYPE_R4 :
									(!WCASECMP ( sType, L"Integer" ))	? VTYPE_I4 :
									(!WCASECMP ( sType, L"Long" ))		? VTYPE_I8 :
									(!WCASECMP ( sType, L"Date" ))		? VTYPE_DATE :
									(!WCASECMP ( sType, L"Boolean" ))	? VTYPE_BOOL : VTYPE_STR;
						CCLTRY ( adtValue::fromString ( s, type, vConv ) );
						}	// VTYPE_STR
						break;

					case VTYPE_BOOL :

						// Boolean to string...
						if (!WCASECMP ( sType, L"Boolean" ))
							{
							CCLTRY ( adtValue::copy ( *pUseV, vConv ) );
							}	// if
						else
							hr = E_NOTIMPL;
						break;

					case VTYPE_UNK :
						{
						IUnknown	*pUnk	= NULL;

						// To ensure conversion is always the same, query for the base IUnknown
						// interface before conversion.

						// Base IUnknown
						CCLTRY ( _QISAFE(pUseV->punk,IID_IUnknown,&pUnk) );

						// Convert to supported types
						if (!WCASECMP ( sType, L"Long" ))
							adtValue::copy ( adtLong((U64)pUnk), vConv );
						else
							hr = E_NOTIMPL;

						// Clean up
						_RELEASE(pUnk);
						}	// VTYPE_UNK
						break;

					default :
						hr = E_NOTIMPL;
					}	// switch
				}	// else

			}	// if

		// Result
		if (hr == S_OK)
			_EMT(Fire,vConv);
		else
			{
			// Debug
			dbgprintf ( L"Type::receiveFire:Unable to convert type from 0x%x to %s\n",
							pUseV->vtype, (LPCWSTR)sType );
			_EMT(Error,adtInt(hr));
			}	// if

		}	// else if

	// Query the current type of value
	else if (_RCP(Query))
		{
		IDictionary	*pDct	= NULL;

		// Value to use
		const ADTVALUE *pUseV	= (!adtValue::empty(vVal)) ? &vVal : &v;

		// Obtain the type
		CCLTRY ( getType ( *pUseV, &pDct ) );

		// Result
		CCLOK ( _EMT(Type,adtIUnknown(pDct)); )

		// Clean up
		_RELEASE(pDct);
		}	// else if

	// Set the type
	else if (_RCP(Type))
		hr = adtValue::copy ( adtString(v), sType );

	// Set the value
	else if (_RCP(Value))
		hr = adtValue::copy ( v, vVal );
	else
		hr = ERROR_NO_MATCH;
		
	return hr;
	}	// receive
