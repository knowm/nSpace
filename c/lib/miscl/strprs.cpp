////////////////////////////////////////////////////////////////////////
//
//									STRPRS.CPP
//
//				Implementation of the string parse node
//
////////////////////////////////////////////////////////////////////////

#include "miscl_.h"
#include <stdio.h>
#if	__unix__ || __APPLE__ 
#include	<wctype.h>
#endif

#define	WCHAR2HEX(a)																	\
	((a) >= WCHAR('0') && (a) <= WCHAR('9')) ? (a)-'0' 				:		\
	((a) >= WCHAR('a') && (a) <= WCHAR('f')) ? (a)-WCHAR('a')+10 	:		\
	((a) >= WCHAR('A') && (a) <= WCHAR('F')) ? (a)-WCHAR('A')+10	: 0

StringParse :: StringParse ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pFmt		= NULL;
	pDctRs	= NULL;
	}	// StringParse

HRESULT StringParse :: onAttach ( bool bAttach )
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

	// Attach
	if (bAttach)
		{
		adtIUnknown	unkV;

		// Allow format to specified as a node property
		if (pnDesc->load ( adtString(L"Format"),	unkV ) == S_OK)
			_QISAFE(unkV,IID_IContainer,&pFmt);
		}	// if

	// Detach
	else
		{
		_RELEASE(pDctRs);
		_RELEASE(pFmt);
		}	// if
 
	return S_OK;
	}	// onAttach

HRESULT StringParse :: parseString ( IUnknown *pSpecs, 
													WCHAR *pwStr, U32 *stridx )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Parse the current string with the list of format specifiers.
	//
	//	PARAMETERS
	//		-	pSpecs is the format list
	//		-	pwStr is the current string
	//		-	stridx is the current position of the string index
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr				= S_OK;
	IContainer	*pCont		= NULL;
	IDictionary	*pDict		= NULL;
	IDictionary	*pDictFmt	= NULL;
	IDictionary	*pMap			= NULL;
	IIt			*pIt			= NULL;
	WCHAR			wTmp			= 0;
	U32			dpos			= 0;
	U32			dlen,idx;
	adtIUnknown	unkV;
	adtString	strName,strRes,strDelim,strType;
	adtInt		iSz;
	adtBool		bNum;

	// Iterator
	CCLTRY(_QISAFE(pSpecs,IID_IContainer,&pCont));
	CCLTRY(pCont->iterate ( &pIt ));

	// Continue parsing string until end of tokens or a 'substring' is
	// specified
	CCLTRY(pIt->begin());
	while (	hr == S_OK &&
				pwStr[*stridx] != WCHAR('\0') && 
				pIt->read ( unkV ) == S_OK )
		{
		// Active dictionary spec.
		CCLTRY(_QISAFE(unkV,IID_IDictionary,&pDict));

		// Property ?
		if (hr == S_OK && pDict->load ( adtString(L"Name"), strName ) == S_OK)
			{
			adtValue	ValueUn,Value;

			// Does the format specify a size ?  If so we read exactly the number
			// of characters specified.  If not we read until the next delimiter.
			// Read exact # of chars.
			if (pDict->load ( adtString(L"Size"), iSz ) == S_OK && iSz > (U32)0)
				{
				// Remember and NULL last character
				wTmp	= pwStr[(*stridx)+iSz];
				pwStr[(*stridx)+iSz]	= WCHAR('\0');

				// Store
				CCLOK ( strRes = &(pwStr[(*stridx)]); )

				// Restore character and move to next block
				pwStr[(*stridx)+iSz]	= wTmp;
				(*stridx) += iSz;
				}	// if
			// Read until delimiter
			else
				{
				// Find end of string
				if (hr == S_OK)
					{
					// If end is not specified, assume '\0'
					if (pDict->load ( adtString(L"End"), strDelim ) != S_OK)
						strDelim = L"";

					// If 'Number' is true then scan until an invalid number character is reached
					if (pDict->load ( adtString(L"IsNumber"), bNum ) != S_OK)
						bNum = false;

					// Find end of string
					CCLOK ( dpos = (*stridx); )
					CCLOK ( dlen = strDelim.length(); )
					while (hr == S_OK && pwStr[dpos] != WCHAR('\0'))
						{
						// Search possible delimiters
						for (idx = 0;idx < dlen;++idx)
							if (pwStr[dpos] == strDelim[idx])
								break;

						// Number ? See if character is a valid 'number character'
						if (bNum == true)
							{
							// Check character
							if (	pwStr[dpos] != WCHAR('.') &&
									pwStr[dpos] != WCHAR('+') &&
									pwStr[dpos] != WCHAR('-') &&
									(pwStr[dpos] < WCHAR('0') || pwStr[dpos] > WCHAR('9')) )
								break;
							}	// if

						// If end of string not found move to next char, otherwise break out of loop
						if (!dlen || (idx == dlen))	++dpos;
						else									break;
						}	// while

					}	// if

				// Remember previous char and NULL
				CCLOK ( wTmp = pwStr[dpos]; )
				CCLOK ( pwStr[dpos]	= WCHAR('\0'); )

				// Delimiter or end of string found.
				CCLOK ( strRes = &(pwStr[(*stridx)]); )
				CCLOK ( strRes.at(); )

				// Restore char.
				CCLOK ( pwStr[dpos]	= wTmp; )

				// One past end of string
				CCLOK ( (*stridx) = dpos; )
				}	// else

			// Store result value based on type (default string)
			if (hr == S_OK)
				{
				// Load type
				if ( pDict->load ( adtString(L"Type"), strType ) == S_OK )
					{
					if			(!WCASECMP ( strType, L"Integer" ))	Value.vtype	= VTYPE_I4;
					else if	(!WCASECMP ( strType, L"Long" ))		Value.vtype	= VTYPE_I8;
					else if	(!WCASECMP ( strType, L"Float" ))		Value.vtype	= VTYPE_R4;
					else if	(!WCASECMP ( strType, L"Double" ))		Value.vtype	= VTYPE_R8;
					else if	(!WCASECMP ( strType, L"String" ))		Value.vtype = VTYPE_STR;
					else if	(!WCASECMP ( strType, L"Binary" ))		Value.vtype = VTYPE_UNK;
					else														hr				= E_NOTIMPL;
					}	// if
				else
					Value.vtype = VTYPE_STR;

				// Format value from string.  Cannot use 'adtValue::fromString' because
				// finer control over formatting is allowed..
				if (hr == S_OK)
					{
					switch (Value.vtype)
						{
						// Integer
						case VTYPE_I4 :
							{
							adtInt	oInt;
							adtBool	bHex(false);
							adtBool	bChr(false);

							// Hex ?
							CCLOK ( pDict->load ( adtString(L"Hex"), bHex ); )

							// CHR ?
							CCLOK ( pDict->load ( adtString(L"Chr"), bChr ); )

							// Convert based on hex or decimal...
							if ( bHex == true )
								oInt = (U32)(wcstoul ( strRes, NULL, 16 ));
							// Direct character/ASCII
							else if (bChr == true)
								oInt = (U32) ((U8)(strRes[0] & 0xff));
							else
								oInt = (U32)(wcstoul ( strRes, NULL, 10 ));
							adtValue::copy ( oInt, Value );
							}	// VTYPE_I4
							break;
						// Long
						case VTYPE_I8 :
							{
							adtInt	oInt;
							adtBool	bHex(false);
							adtBool	bChr(false);

							// Hex ?
							CCLOK ( pDict->load ( adtString(L"Hex"), bHex ); )

							// CHR ?
							CCLOK ( pDict->load ( adtString(L"Chr"), bChr ); )

							// Convert based on hex or decimal...
							if ( bHex == true )
								oInt = (U32)(wcstoul ( strRes, NULL, 16 ));
							// Direct character/ASCII
							else if (bChr == true)
								oInt = (U32) ((U8)(strRes[0] & 0xff));
							else
								oInt = (U32)(wcstoul ( strRes, NULL, 10 ));
							adtValue::copy ( oInt, Value );
							}	// VTYPE_I8
						// Float
						case VTYPE_R4 :
							{
							adtFloat	oFlt;
							oFlt = (float)wcstod ( strRes, NULL );
							adtValue::copy ( oFlt, Value );
							}	// VTYPE_R4
							break;
						// Double
						case VTYPE_R8 :
							{
							adtDouble	oDbl;
							oDbl = wcstod ( strRes, NULL );
							adtValue::copy ( oDbl, Value );
							}	// VTYPE_R8
							break;
						// String
						case VTYPE_STR :
							// Allow for leading and trailing characters to be removed.
							if (hr == S_OK && pDict->load ( adtString(L"Trail"), Value ) == S_OK)
								{
								adtString	strTrail(Value);
								U32			tlen = strTrail.length();
								U32			slen = strRes.length();
								idx	= 0;
								while (idx < tlen)
									{
									for (idx = 0;idx < tlen;++idx)
										if (strRes[slen-1] == strTrail[idx])
											{
											// Remove character and update length
											strRes.at(slen-1) = '\0';
											slen = strRes.length();
											break;
											}	// if
									}	// while

								}	// if

							CCLOK  ( adtValue::clear ( Value ); )
							CCLTRY ( adtValue::copy ( strRes, Value ) );
							break;

						// Object/Binary stream
						case VTYPE_UNK :
							{
							// ASCII hex encoded binary stream
							IByteStream	*pStm	= NULL;
							U32			i,len	= strRes.length();
							U8				byte;

							// Create memory based stream to receive data
							CCLTRY ( COCREATE ( L"Io.StmMemory", IID_IByteStream, &pStm ) );
							CCLOK  ( Value.punk = pStm; )
							_ADDREF(Value.punk);

							// Parse string
							if (hr == S_OK && iSz > (U32)0)
								{
								// Pre-size to known length (will be half of ASCII version)
								CCLTRY ( pStm->setSize ( iSz/(U32)2 ) );
								for (i = 0;hr == S_OK && i < len;)
									{
									// Convert next char. to byte
									byte =	(WCHAR2HEX(strRes[i])) << 4; ++i;
									byte |=	(WCHAR2HEX(strRes[i])) << 0; ++i;

									// To stream
									CCLTRY ( pStm->write ( &byte, 1, NULL ) );

									// Done ?
									if (i == 0) break;
									}	// for

								// Always reset position of stream so its ready to use
								CCLTRY ( pStm->seek ( 0, STREAM_SEEK_SET, NULL ) );
								}	// if

							// Clean up
							_RELEASE(pStm);
							}	// VTYPE_UNKNOWN
							break;

						default :
							hr = E_NOTIMPL;
						}	// switch
					}	// if

				// Mapping specified ?  Allow format to specify a dictionary that will 'map'
				// incoming 'from' values 'to' output dictionary values.
				if (hr == S_OK && pDict->load ( adtString(L"Map"), unkV ) == S_OK)
					{
					CCLTRY(_QISAFE(unkV,IID_IDictionary,&pMap));
					CCLTRY(adtValue::copy ( Value, ValueUn ));
					CCLTRY(pMap->load ( ValueUn, Value ));
					_RELEASE(pMap);
					}	// if
				else if (hr == S_OK)
					hr = adtValue::copy ( Value, ValueUn );

				// Store
				CCLTRY ( pDctRs->store ( strName, Value ) );
				}	// if

			// Format specify a 'subformat' ?  If so we look for the specified key.
			// If substring is specified but no key is found to match the result we
			// stop the parsing there.  Use unmapped value as key.
			if (hr == S_OK && pDict->load ( adtString(L"Subformat"), unkV ) == S_OK)
				{
				// Substring list
				CCLTRY(_QISAFE(unkV,IID_IDictionary,&pDictFmt));

				// Check for unmapped value, if not present check for 'default' case
				if (hr == S_OK && pDictFmt->load ( ValueUn, unkV ) != S_OK)
					hr = pDictFmt->load ( adtString(L""), unkV );

				// Continue parsing in sub string
				CCLTRY(parseString ( unkV, pwStr, stridx ));

				// Clean up
				_RELEASE(pDictFmt);
				}	// if

			}	// if

		// Constant ?
		else if (hr == S_OK && pDict->load ( adtString(L"Constant"), strName ) == S_OK)
			{
			// If current character matches constant, proceed forward, otherwise error
			for (U32 i = 0;hr == S_OK && strName[i] != WCHAR('\0');++i)
				{
				if (	towupper(pwStr[(*stridx)]) == 
						towupper(strName[i]))				++(*stridx);
				else												hr = E_UNEXPECTED;
				}	// for
			}	// if

		// Skip ?
		else if (hr == S_OK && pDict->load ( adtString(L"Skip"), strName ) == S_OK)
			{
			// Skip over any matching characters
			while (hr == S_OK && pwStr[(*stridx)] != WCHAR('\0'))
				{
				U32	i		= 0;
				U32	len;

				// Current character match any skip characters ?
				for (i = 0,len = strName.length();i < len;++i)
					if (towupper(pwStr[(*stridx)]) == towupper(strName[i]))
						break;

				// Skip ?
				if (i < len)	++(*stridx);
				else				break;
				}	// while

			}	// else if

		// Clean up
		_RELEASE(pDict);
		pIt->next();
		}	// while

	// Clean up
	_RELEASE(pIt);
	_RELEASE(pCont);

	return hr;
	}	// parseString

HRESULT StringParse :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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
		U32			idx		= 0;
		adtString	strParseMe;

		// State check
		CCLTRYE ( (pDctRs != NULL),		ERROR_INVALID_STATE );
		CCLTRYE ( (pFmt != NULL),			ERROR_INVALID_STATE );
		CCLTRYE ( strParse.length() > 0,	ERROR_INVALID_STATE );

		// Make a copy of the string for our use
		CCLTRY	( adtValue::copy ( strParse, strParseMe ) );

		// DEBUG
//		if (hr == S_OK && !wcsnicmp(strParse,L"BEGIN",5))
//			{
//			dbgprintf ( L"String : %s\r\n", (LPCWSTR)strParse );
//			DebugBreak();
//			}	// if

		// Begin parsing at the 'top' level list
		CCLTRY	( parseString ( pFmt, &strParseMe.at(), &idx ) );

		// Result.  Since parsing may abnormally abort still emit what was generated.
		_EMT(Position,adtInt(idx) );
		_EMT(Fire,adtIUnknown(pDctRs) );

		// DEBUG
//		if (hr != S_OK)
//			{
//			dbgprintf ( L"StringParse::receiveFire:Parsed failed:" );
//			dbgprintf ( (pDictRes == NULL) ? L"No result dictionary" : L"." );
//			dbgprintf ( (pFmt == NULL) ? L"No format dictionary" : L"." );
//			dbgprintf ( (strParse.length() == 0) ? L"No string to parse" : L"." );
//			dbgprintf ( L"\r\n" );
//			}	// if

		}	// if

	// State
	else if (_RCP(Dictionary))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pDctRs);
		hr = _QISAFE(unkV,IID_IDictionary,&pDctRs);
		}	// else if
	else if (_RCP(Format))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pFmt);
		hr = _QISAFE(unkV,IID_IContainer,&pFmt);
		}	// else if
	else if (_RCP(String))
		hr = adtValue::copy ( adtString(v), strParse );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

