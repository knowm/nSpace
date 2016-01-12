////////////////////////////////////////////////////////////////////////
//
//									StringOp.CPP
//
//				Implementation of the generic StringOp node
//
////////////////////////////////////////////////////////////////////////

#include "miscl_.h"
#include <stdio.h>

StringOp :: StringOp ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	iFrom	= 0;
	iTo	= 0;
	bTo	= false;
	pMap	= NULL;
	}	// StringOp

void StringOp :: destruct ( void )
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
	_RELEASE(pMap);
	}	// destruct

HRESULT StringOp :: onAttach ( bool bAttach )
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
	adtValue		v;
	adtIUnknown	unkV;

	// Attach ?
	if (!bAttach) return S_OK;

	// Attributes
	pnDesc->load ( adtString ( L"From" ),			iFrom );
	bTo = (pnDesc->load ( adtString ( L"To" ), iTo ) == S_OK);
	pnDesc->load ( adtString ( L"Source" ),		strSrc );
	pnDesc->load ( adtString ( L"Destination" ),	strDst );
	pnDesc->load ( adtString ( L"Type" ),			strType );

	// A 'map' context can be provided for things like string replacement.
	if (pnDesc->load ( adtString(L"Map"), v ) == S_OK)
		{
		_QISAFE((unkV=v),IID_IDictionary,&pMap);
		}	// if

	return S_OK;
	}	// onAttach

HRESULT StringOp :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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

	// Generate a subStringOp
	if (_RCP(Substring))
		{
		S32			l,b,e;

//		if (!WCASECMP(strnName,L"LastName"))
//			dbgprintf ( L"Hi\r\n" );

		// State check
		CCLTRYE ( (l = strSrc.length()) > 0, ERROR_INVALID_STATE );
		if (hr == S_OK)
			{
			// Start index, positive means from start, negative means from end.
			if (((S32)iFrom) >= 0)
				b = (((S32)iFrom) < l) ? (S32)iFrom : l;
			else if (((S32)(iFrom)) < 0)
				{
				b = -((S32)iFrom);
				if (b < l) 	b = (l - b);
				else			b = 0;
				}	// else if
			
			// Stop index, positive means from start, negative means from end.
			if (!bTo)
				e = l;
			else if (((S32)iTo) >= 0)
				{
				e = iTo;
				if (e > l) e = l;
				}	// if
			else if (((S32)iTo) < 0)
				{
				e = l + iTo;								// iTo is negative
				if (e < 0) e = 0;
				}	// else if
			}	// if

		// Generate subStringOp
		CCLTRY ( strSrc.substring ( b, e, strRes ) );

		// Result
		if (hr != S_OK)
			strRes = L"";
		_EMT(Fire,strRes);
		}	// if

	// Index of/Last index of
	else if (_RCP(IndexOf) || _RCP(LastIndexOf))
		{
		S32			iOf	= 0;
		WCHAR			*pw	= NULL;
		adtString	strSrci,strDsti;

		// Returns the first/last occurence of 'Source' in 'Destination' from 'Start'.
		// State check
		CCLTRYE ( iFrom <= strDst.length(), ERROR_INVALID_STATE );

		// By default StringOp comparisons are case insensitive
		CCLOK  ( strSrci = strSrc; )
		CCLOK  ( strSrci.toUpper(); )
		CCLOK  ( strDsti = strDst; )
		CCLOK  ( strDsti.toUpper(); )

//if (!WCASECMP(strSrci,L"://"))
//	dbgprintf ( L"Hi\r\n" );
		// Find StringOp
		if (hr == S_OK && _RCP(IndexOf))
			{
			CCLTRYE ( (iOf = strDsti.indexOf ( strSrci, iFrom )) != -1, ERROR_NOT_FOUND );
			}	// if
		else if (hr == S_OK && _RCP(LastIndexOf))
			{
			WCHAR	*pp;
			for (	pp = wcsstr ( &(strDsti.at(iFrom)), strSrci ),pw = NULL;
					pp != NULL;
					pp = wcsstr ( pp+1, strSrci ))
				pw = pp;

			// Result
			CCLTRYE ( pw != NULL, ERROR_NOT_FOUND );
			CCLOK   ( iOf = (U32)(pw-(&(strDsti.at(0)))); )
			}	// else if
//		dbgprintf ( L"StringOp::receive:IndexOf:%d:%s:%s\r\n", iOf, (LPCWSTR) strSrci, (LPCWSTR) strDsti );

		// Result
		if (hr == S_OK)
			_EMT(Fire,adtInt(iOf) );
		else
			_EMT(NotFound,strSrc );
		}	// if

	// Is type
	else if (_RCP(IsType))
		{
		bool	bRes = false;

		// Add new types as necessary
		if (((S32)iFrom) >= 0 && iFrom < strSrc.length())
			{
//			bRes = (!WCASECMP(strType,L"DIGIT") ? (iswdigit ( strSrc[iStart] ) != 0) : false);
			bRes = (!WCASECMP(strType,L"DIGIT") ? (strSrc[iFrom] >= '0' && strSrc[iFrom] <= '9') : false);
			}	// if

		// Result
		if (bRes)
			_EMT(True,strType);
		else
			_EMT(False,strType);
		}	// else if

	// Replace 'source' with 'destination'
	else if (_RCP(Replace))
		{
		adtString	str(v);
		int			len;
		WCHAR			*pwRes,*pwStr,*pwAt,*pwStr0;
		WCHAR			*pwRes0 = NULL;

		// State check
		CCLTRYE ( (len = str.length()) > 0, ERROR_INVALID_STATE );

		// Map/whole string replacement specified ?
		if (hr == S_OK && pMap != NULL)
			{
			IIt			*pIt	= NULL;
			adtString	sFrom,sTo,sSrc,sDst;
			U32			rsz,dstidx,tlen,flen;
			WCHAR			*pwsrc;
			adtValue		vL;
			U32			i,j;

			// The keys of the dictionary contain the 'from' strings
			CCLTRY(pMap->keys ( &pIt ) );

			// Perform replacement one string at a time.  A little slower
			// but makes logic easier to follow.
			CCLTRY ( adtValue::copy ( str, sSrc ) );
			while (hr == S_OK && pIt->read ( vL ) == S_OK)
				{
				// From string
				CCLTRYE ( (sFrom = vL).length() > 0, E_UNEXPECTED );

				// Load the 'to' string
				CCLTRY ( pMap->load ( sFrom, sTo ) );
				CCLOK ( flen = sFrom.length(); )
				CCLOK ( tlen = sTo.length(); )

				// Two passes, one to calculate how much space will be needed, the
				// next to perform substitution.
				for (i = 0,rsz = 0,dstidx = 0,pwsrc = &sSrc.at();hr == S_OK && i < 2;++i)
					{
					WCHAR	*pwf;

					// Find occurences of string
					for (	pwf = wcsstr ( &sSrc.at(), sFrom );
							pwf != NULL && hr == S_OK;
							pwf = wcsstr ( pwf, sFrom ) )
						{
						// First pass, count replacement size
						if (i == 0)	rsz += sTo.length();

						// Second pass, replace string
						else
							{
							// Copy character up to found string
							while (pwsrc != pwf && *pwsrc != WCHAR('\0'))
								sDst.at(dstidx++) = *pwsrc++;

							// Now replace with to string
							for (j = 0;j < tlen;++j)
								sDst.at(dstidx++) = sTo[j];

							// Skip over replaced string
							pwsrc += flen;
							}	// else

						// Jump over from string
						pwf += flen;
						}	// for

					// Allocate memory for result string if there are any replacements
					if (hr == S_OK && i == 0)
						{
						if (rsz == 0)
							break;
						else
							hr = sDst.allocate ( sSrc.length() + rsz );
						}	// if
					}	// for

				// Copy remaining unmatched text
				if (hr == S_OK && rsz)
					{
					while (hr == S_OK && *pwsrc != WCHAR('\0'))
						sDst.at(dstidx++) = *pwsrc++;
					CCLOK ( sDst.at(dstidx) = WCHAR('\0'); )

					// New source string is destination
					CCLTRY ( adtValue::copy ( sDst, sSrc ) );
					}	// if

				// Next from string
				pIt->next();
				}	// while

			// Clean up
			_RELEASE(pIt);

			// Result
			CCLOK ( _EMT(Fire,sSrc ); )
			}	// if

		// Character replacement
		else if (hr == S_OK)
			{
			// State check
			CCLTRYE ( strDst.length() > 0, ERROR_INVALID_STATE );
			CCLTRYE ( strSrc.length() > 0, ERROR_INVALID_STATE );

			// Worst case is that every character in StringOp is replaced by destination StringOp
			CCLTRY  ( strRes.allocate ( str.length()*strDst.length() ) );

			// Replace subStringOps
			if (hr == S_OK)
				{
				// Starting point
				pwStr0	= pwStr	= &str.at();
				pwRes0	= pwRes	= &strRes.at();
				while ((pwStr-pwStr0) < len)
					{
					// Next match
					pwAt = wcsstr ( pwStr, strSrc );
					if (pwAt == NULL)
						break;

					// Copy StringOp unmodified up to matching position
					while (pwStr != pwAt)
						*pwRes++ = *pwStr++;

					// Replace found StringOp
					for (U32 i = 0;i < strDst.length();++i)
						*pwRes++ = strDst.at(i);

					// Skip source match
					pwStr += strSrc.length();
					}	// while

				// Copy remaining characters umodified
				while ((pwStr-pwStr0) < len)
					*pwRes++ = *pwStr++;

				// Done
				*pwRes = WCHAR('\0');
				}	// if

			// Result
			CCLOK ( _EMT(Fire,adtString(pwRes0)); )		
			}	// else if

		}	// else if

	// Remove trailing characters
	else if (_RCP(Trailing))
		{
		int			i,j,tlen,len = 0;
		adtString	strRes;

		// State check
		CCLTRYE ( strDst.length() > 0, ERROR_INVALID_STATE );
		CCLTRYE ( strSrc.length() > 0, ERROR_INVALID_STATE );
		CCLTRYE ( (len = strDst.length()) > 0, ERROR_INVALID_STATE );

		// Strip trailing characters
		for (i = len;hr == S_OK && i > 0;--i)
			{
			// Current character match ?
			CCLOK ( tlen = strSrc.length(); )
			for (j = 0;j < tlen;++j)
				if (strDst.at(i-1) == strSrc.at(j))
					break;

			// End of trailing StringOp ?
			if (j >= tlen)
				break;
			}	// for

		// Result
		CCLTRY( adtValue::copy ( strDst, strRes ) );
		CCLOK ( strRes.at(i) = '\0'; )
		CCLOK	( _EMT(Fire,adtString(strRes)); )
		}	// else if
 
	// Length of source StringOp
	else if (_RCP(Length))
		_EMT(Fire,adtInt ( strSrc.length() ) );

	// State
	else if (_RCP(Source))
		hr = adtValue::copy ( adtString(v), strSrc );
	else if (_RCP(Destination))
		hr = adtValue::copy ( adtString(v), strDst );
	else if (_RCP(From))
		iFrom	= v;
	else if (_RCP(To))
		{
		iTo	= v;
		bTo	= true;
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

