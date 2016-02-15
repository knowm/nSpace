////////////////////////////////////////////////////////////////////////
//
//									TOKENIT.CPP
//
//					Implementation of the token iterator.
//
////////////////////////////////////////////////////////////////////////

#include "miscl_.h"
#include <stdio.h>
#if		defined(__unix__) || defined(__APPLEE__)
#include <wctype.h>
#endif

TokenIt :: TokenIt ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object
	//
	////////////////////////////////////////////////////////////////////////
	ptokLast = NULL;
	}	// TokenIt

WCHAR *TokenIt :: iwcstok (	WCHAR *strToken,
										const WCHAR *strDelimit )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Internal implementation of 'wcstok' to avoid C-library
	//			static memory issues.
	//
	//	PARAMETERS
	//		-	strToken is the string containing tokens.
	//		-	strDelimit is the set of delimiter characters.
	//
	//	RETURN VALUE
	//		Ptr. to next token
	//
	////////////////////////////////////////////////////////////////////////
	WCHAR			*token;
	const WCHAR	*ctl;

	// If no strToken, continue with previous one
	if (strToken == NULL)
		strToken = ptokLast;
	if (strToken == NULL)
		return NULL;

	// Find beginning of token (skip over leading dilimiters)
	while (*strToken)
		{
		for (ctl=strDelimit;*ctl && *ctl != *strToken;ctl++)
			;
		if (!*ctl) break;
		strToken++;
		}	// while

	// Next strToken starts here
	token = strToken;

	// Find the end of the token
	for ( ; *strToken ; strToken++ )
		{
		for (ctl=strDelimit;*ctl && *ctl != *strToken;ctl++)
			;
		if (*ctl)
			{
			*strToken++ = WCHAR('\0');
			break;
			}	// if
		}	// for

	// Update next token
	ptokLast = strToken;

	// Found ?
	if (token == strToken)
		return NULL;
	else
		return token;
	}	// iwcstok

HRESULT TokenIt :: onAttach ( bool bAttach )
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
	pnDesc->load ( adtString(L"Delimiter"), strDelim );

	return S_OK;
	}	// onAttach

HRESULT TokenIt :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

	// Next token
	if (_RCP(Next))
		{
		bool		bFirst = false;
		WCHAR		*pwToken;

		// State check
		CCLTRYE ( strDelim.length() > 0, ERROR_INVALID_STATE );

		// First 'next' ?
		if (hr == S_OK && strCpy[0] == '\0')
			bFirst = true;

		// Make copy of original string for own use
		if (hr == S_OK && bFirst)
			hr = adtValue::copy ( strOrg, strCpy );

		// First/next token
		if (hr == S_OK && (pwToken = iwcstok ( (bFirst) ? &strCpy.at() : NULL, strDelim )) != NULL)
			_EMT(Next,adtString(pwToken) );
		else
			_EMT(End,strOrg );
		}	// else if

	// Reset
	else if (_RCP(Reset))
		{
		// Reset state by clearing string copy
		strCpy = L"";
		}	// if

	// Count
	else if (_RCP(Count))
		{
		U32		cnt	= 0;
		WCHAR		*pwToken;

		// State check
		CCLTRYE ( strDelim.length() > 0, ERROR_INVALID_STATE );
		CCLTRYE ( strOrg.length() > 0, ERROR_INVALID_STATE );

		// Count strings
		CCLTRY ( adtValue::copy ( strOrg, strCpy ) );
		if (hr == S_OK)
			{
			for (	pwToken = iwcstok ( &strCpy.at(), strDelim );
					pwToken != NULL;
					pwToken = iwcstok ( NULL, strDelim ) )
				++cnt;
			}	// if

		// Result
		CCLOK ( _EMT(Count,adtInt(cnt) ); )
		}	// else if

	// Generate entire list of tokens
	else if (_RCP(List))
		{
		IList	*pLst	= NULL;
		WCHAR	*pwToken;

		// State check
		CCLTRYE ( strDelim.length() > 0, ERROR_INVALID_STATE );
		CCLTRYE ( strOrg.length() > 0, ERROR_INVALID_STATE );

		// Create list to receive the tokens
		CCLTRY ( COCREATE ( L"Adt.List", IID_IList, &pLst ) );

		// Add tokens to list
		CCLTRY ( adtValue::copy ( strOrg, strCpy ) );
		if (hr == S_OK)
			for (pwToken = iwcstok ( &strCpy.at(), strDelim );
					pwToken != NULL;
					pwToken = iwcstok ( NULL, strDelim ))
				pLst->write ( adtString(pwToken) );

		// Result
		CCLOK ( _EMT(List,adtIUnknown(pLst)); )

		// Clean up
		_RELEASE(pLst);
		}	// else if

	// State
	else if (_RCP(String))
		{
		strOrg = v;
		strCpy = L"";
		}	// else if
	else if (_RCP(Delimiter))
		strDelim = v;
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

