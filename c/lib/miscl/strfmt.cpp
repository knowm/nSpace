////////////////////////////////////////////////////////////////////////
//
//									STRFMT.CPP
//
//				Implementation of the string format node
//
////////////////////////////////////////////////////////////////////////

#include "miscl_.h"
#include <stdio.h>

#define	HEX2WCHAR(a)													\
	(((a) < 10) ? ((a) + WCHAR('0')) : ((a) - 10 + WCHAR('A')))
#define	SIZE_BFR		4096
#define	COUNT_BFR	(SIZE_BFR/sizeof(WCHAR))

StringFormat :: StringFormat ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pFmt		= NULL;
	pDctSrc	= NULL;
	pBfr		= NULL;
	pwBfr		= NULL;
	}	// StringFormat

HRESULT StringFormat :: formatString ( IUnknown *pSpecs, 
													WCHAR *pwStr, U32 *stridx )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Format the current dictionary with the list of format
	//			specifiers.
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
	IDictionary	*pDictSub	= NULL;
	IDictionary	*pMap			= NULL;
	IIt			*pIt			= NULL;
	adtIUnknown	unkV;
	adtString	strName,strValue;
	adtInt		iSz;
	adtValue		vValue,vValueUn,vKey;

	// Iterator specifications
	CCLTRY(_QISAFE(pSpecs,IID_IContainer,&pCont));
	CCLTRY(pCont->iterate ( &pIt ));

	// Continue formatting string until end of tokens or a 'substring' is specified
	CCLTRY(pIt->begin());
	while (	hr == S_OK && pIt->read ( unkV ) == S_OK )
		{
		// Active dictionary spec.
		CCLTRY(_QISAFE(unkV,IID_IDictionary,&pDict));

		// Property ?
		if (hr == S_OK && pDict->load ( adtString(L"Name"), strName ) == S_OK)
			{
			// Optional 'size' parameter
			CCLOK	( iSz = (U32)-1; )
			CCLOK ( pDict->load ( adtString(L"Size"), iSz ); )

			// Attempt to load the value for the specified name from the source dictionary
			if (hr == S_OK)
				{
				hr = pDctSrc->load ( strName, vValue );
				if (hr != S_OK)
					{
					U32	sz;
					pDict->size ( &sz );
					dbgprintf ( L"StringFormat::formatString:Name not found %s (Dictionary size %d)\r\n",
										(LPCWSTR) strName, sz );
					}	// if
				}	// if
			CCLOK  ( adtValue::copy ( vValue, vValueUn ); )

			// Mapping specified ?  Allow format to specify a dictionary that will 'map'
			// incoming 'from' dictionary values to 'to' output values to use during format.
			if (hr == S_OK && pDict->load ( adtString(L"Map"), unkV ) == S_OK)
				{
				CCLTRY(_QISAFE(unkV,IID_IDictionary,&pMap));
				CCLTRY(adtValue::copy ( vValue, vKey ));
				CCLTRY(pMap->load ( vKey, vValue ));
				_RELEASE(pMap);
				}	// if

			// Format value to string.  Cannot use 'adtValueImpl::toString' because
			// finer control over formatting is allowed..
			// Ok to specify a size of 0, just means no information gets transferred
			// to string in current specifier.
			if (hr == S_OK && iSz != 0)
				{
				switch ((int)adtValue::type(vValue))
					{
					// String
					case VTYPE_STR :
						{
						adtString	sValue(vValue);

						// If size not specified, use whole string
						U32 srcidx = 0;
						while (	hr == S_OK &&
									sValue[srcidx] != WCHAR('\0') &&
									(iSz == (U32)-1 || srcidx < iSz) )
							pwStr[(*stridx)++] = sValue[srcidx++];
						pwStr[(*stridx)] = WCHAR('\0');
						}	// VTYPE_BSTR	
						break;
					// Integer
					case VTYPE_I4 :
						{
						WCHAR		wNumBfr[21];
						adtBool	bHex(false);
						adtBool	bChr(false);

						// Hex ?
						CCLOK ( pDict->load ( adtString(L"Hex"), bHex ); )

						// CHR ?
						CCLOK ( pDict->load ( adtString(L"Chr"), bChr ); )

						// Format number
						if (bHex == true)
							{
							if (iSz == (U32)-1)
								swprintf ( SWPF(wNumBfr,20), L"%0X", vValue.vint );

							// Comparison to specific sizes is done to handle negative numbers
							else if (iSz == 2)
								swprintf ( SWPF(wNumBfr,20), L"%02X", (U8)(vValue.vint) );
							else if (iSz == 4)
								swprintf ( SWPF(wNumBfr,20), L"%04X", (U16)(vValue.vint) );
							else
								swprintf ( SWPF(wNumBfr,20), L"%0*X", (U32)(iSz), vValue.vint );
							}	// if
						// Direct character/ASCII
						else if (bChr == true)
							{
							wNumBfr[0] = (WCHAR)((U32)vValue.vint);
							wNumBfr[1] = WCHAR('\0');
							}	// else if
						else
							{
							if (iSz == (U32)-1)
								swprintf ( SWPF(wNumBfr,20), L"%0d", vValue.vint );
							else
								swprintf ( SWPF(wNumBfr,20), L"%0*d", (U32)(iSz), vValue.vint );
							}	// if

						// Append
						WCSCAT ( pwStr, 100, wNumBfr );
						(*stridx) += (U32)wcslen(wNumBfr);
						}	// VTYPE_I4
						break;
					// Long integer
					case VTYPE_I8 :
						{
						WCHAR		wNumBfr[21];
						adtBool	bHex(false);

						// Hex ?
						CCLOK ( pDict->load ( adtString(L"Hex"), bHex ); )

						// Format number
						if (bHex == true)
							{
							if (iSz == (U32)-1)
								swprintf ( SWPF(wNumBfr,20), L"%0llX", vValue.vlong );

							// Comparison to specific sizes is done to handle negative numbers
							else if (iSz == 2)
								swprintf ( SWPF(wNumBfr,20), L"%02X", (U16)(vValue.vlong) );
							else if (iSz == 4)
								swprintf ( SWPF(wNumBfr,20), L"%04lX", (U32)(vValue.vlong) );
							else
								swprintf ( SWPF(wNumBfr,20), L"%0*llX", (U32)(iSz), vValue.vlong );
							}	// if
						else
							{
							if (iSz == (U32)-1)
								swprintf ( SWPF(wNumBfr,20), L"%0lld", vValue.vlong );
							else
								swprintf ( SWPF(wNumBfr,20), L"%0*lld", (U32)(iSz), vValue.vlong );
							}	// if

						// Append
						WCSCAT ( pwStr, COUNT_BFR, wNumBfr );
						(*stridx) += (U32)wcslen(wNumBfr);
						}	// VTYPE_I4
						break;
					case VTYPE_R4 :
						{
						WCHAR		wNumBfr[41];
						adtInt	iPrec;

						// Format number
						if (iSz == (U32)-1)
							swprintf ( SWPF(wNumBfr,40), L"%g", vValue.vflt );
						else if (pDict->load ( adtString(L"Precision"), iPrec ) == S_OK)
							swprintf ( SWPF(wNumBfr,40), L"%0*.*f", (U32)(iSz), iPrec.vint, vValue.vflt );
						else
							swprintf ( SWPF(wNumBfr,40), L"%0*f", (U32)(iSz), vValue.vflt );

						// Append
						WCSCAT ( pwStr, COUNT_BFR, wNumBfr );
						(*stridx) += (U32)wcslen(wNumBfr);
						}	// VTYPE_R4
						break;
					case VTYPE_R8 :
						{
						WCHAR		wNumBfr[41];
						adtInt	iPrec;

						// Format number
						if (iSz == (U32)-1)
							swprintf ( SWPF(wNumBfr,40), L"%g", vValue.vdbl );
						else if (pDict->load ( adtString(L"Precision"), iPrec ) == S_OK)
							swprintf ( SWPF(wNumBfr,40), L"%0*.*f", (U32)(iSz), iPrec.vint, vValue.vdbl );
						else
							swprintf ( SWPF(wNumBfr,40), L"%0*f", (U32)(iSz), vValue.vdbl );

						// Append
						WCSCAT ( pwStr, COUNT_BFR, wNumBfr );
						(*stridx) += (U32)wcslen(wNumBfr);
						}	// VTYPE_R4
						break;

					// Object
					case VTYPE_UNK :
						{
						IByteStream	*pStm	= NULL;
						adtIUnknown	unkV(vValue);

						// Byte stream ?
						if (	unkV != (IUnknown *)(NULL) &&
								_QI(unkV,IID_IByteStream,&pStm) == S_OK)
							{
							adtString	sEncode;

							// Encoding type (required ?, default ?)
							CCLTRY ( pDict->load ( adtString(L"Encode"), sEncode ); )

							// Base64ish ?
							if (hr == S_OK && !wcsncmp ( sEncode, L"Base64", 6 ))
								{
								char	*res     = NULL;
								U64	sz			= 0;
								int	i;//,len;
								char	*d			= NULL;
								BYTE	b[3];

								// Support an altered Base64 encoding that is 'web friendly'.
								const char *map	= "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
								const char spc		= '=';
								const char *mapw	= "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789._";
								const char spcw	= '-';

								// Figure out which mapping to use
								const char	*mapu	= (!WCASECMP(sEncode,L"Base64Web")) ? mapw : map;
								const char	spcu	= (!WCASECMP(sEncode,L"Base64Web")) ? spcw : spc;

								// Stream length calculation 
								CCLTRY ( pStm->available ( &sz ) );

								// Encode
								// Modelled from 'http://bugs.musicbrainz.org/browser/libdiscid/trunk/src/base64.c'
								CCLOK ( i = (int)((sz + 2) / 3) * 4; )
//								CCLOK ( len = i += 2 * ((i / 60) + 1); )
								CCLTRYE ( (d = res = (char *) _ALLOCMEM ( ++i ) ) != NULL, E_OUTOFMEMORY );
								for (i = 0;sz;)
									{
									// Next 3 bytes
									if ( (hr = pStm->read ( b, (sz < sizeof(b)) ? sz : sizeof(b), NULL ) ) != S_OK )
										break;

									// Generate string
									*d++ = mapu[b[0] >> 2];
									*d++ = mapu[((b[0] << 4) + (--sz ? (b[1] >> 4) : 0)) & 0x3f];
									*d++ = sz ? mapu[((b[1] << 2) + (--sz ? (b[2] >> 6) : 0)) & 0x3f] : spcu;
									*d++ = sz ? mapu[b[2] & 0x3f] : spcu;
									if (sz) sz--;
									if ((++i) == 15)
										{
										i = 0;
										*d++ = '\015'; *d++ = '\012';
										}	// if
									}	// for
								CCLOK ( *d = '\0'; )

								// Result
								if (hr == S_OK)
									{
									adtString	sRes;

									// Append
									sRes = res;
									WCSCAT ( pwStr, COUNT_BFR, sRes );
									(*stridx) += (U32)sRes.length();
									}	// if
								}	// if

							// Hex ?
							else if (hr == S_OK && !wcsncmp ( sEncode, L"HEX", 3 ))
								{
								adtString	sRes;
								U64			sz;
								U32			i;
								BYTE			b;

								// Convert bytes to ASCII hex

								// Size to convert
								if (iSz != (U32)-1)
									sz = iSz;
								else
									{
									// Stream length calculation
									CCLTRY ( pStm->available ( &sz ) );
									}	// else

								// Allocate memory for string (2*size)
								CCLTRY ( sRes.allocate ( (U32)(2*sz) ) );

								// Convert stream
								if (hr == S_OK)
									{
									for (i = 0;i < 2*sz;)
										{
										// Next byte
										if ( pStm->read ( &b, 1, NULL ) != S_OK )
											break;

										// Store
										sRes.at(i++) = HEX2WCHAR( ((b>>4) & 0xf) );
										sRes.at(i++) = HEX2WCHAR( ((b>>0) & 0xf) );
										}	// for

									// Ensure termination in case stream runs out
									sRes.at(i) = WCHAR('\0');
									}	// if

								// Append to string
								if (hr == S_OK)
									{
									WCSCAT ( pwStr, COUNT_BFR, sRes );
									(*stridx) += (U32)sRes.length();
									}	// if
								}	// else if

							// Clean up
							_RELEASE(pStm);
							}	// if

						}	// VTYPE_UNKNOWN
						break;
					}	// switch
				}	// if

			// Format specify a 'subformat' ?  If so, look for the specified key.
			// If substring is specified but no key is found to match the result we
			// stop the formatting here.
			if (hr == S_OK && pDict->load ( adtString(L"Subformat"), unkV ) == S_OK)
				{
				// Substring list (used unmapped value for lookup) (ok if not found, no addtional param)
				CCLTRY(_QISAFE(unkV,IID_IDictionary,&pDictSub));
				if (hr == S_OK && pDictSub->load ( vValueUn, unkV ) == S_OK)
					{
					// Continue formatting in sub string
					CCLTRY(formatString ( unkV, pwStr, stridx ));
					}	// if

				// Clean up
				_RELEASE(pDictSub);
				}	// if

			}	// if

		// Constant ?
		else if (hr == S_OK && pDict->load ( adtString(L"Constant"), strName ) == S_OK)
			{
			// Place constant in string
			for (U32 i = 0;hr == S_OK && strName[i] != WCHAR('\0');++i)
				pwStr[(*stridx)++] = strName[i];
			pwStr[(*stridx)] = WCHAR('\0');
			}	// if

		// Clean up
		_RELEASE(pDict);
		pIt->next();
		}	// while

	// Clean up
	_RELEASE(pIt);
	_RELEASE(pIt);
	_RELEASE(pCont);

	return hr;
	}	// formatString

HRESULT StringFormat :: onAttach ( bool bAttach )
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
		adtIUnknown	unkV;

		// String buffer
		CCLTRY(COCREATE(L"Io.MemoryBlock",IID_IMemoryMapped,&pBfr));
		CCLTRY(pBfr->setSize ( SIZE_BFR ));
		CCLTRY(pBfr->lock ( 0, 0, (void **) &pwBfr, NULL ));

		// Allow format to specified as a node property
		if (pnDesc->load ( adtString(L"Format"),	unkV ) == S_OK)
			_QISAFE(unkV,IID_IContainer,&pFmt);
		}	// if

	// Detach
	else
		{
		_UNLOCK(pBfr,pwBfr);
		_RELEASE(pBfr);
		_RELEASE(pDctSrc);
		_RELEASE(pFmt);
		}	// else
 
	return S_OK;
	}	// onAttach

HRESULT StringFormat :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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
		U32		idx	= 0;

		// State check
		CCLTRYE ( (pDctSrc != NULL),	ERROR_INVALID_STATE );
		CCLTRYE ( (pFmt != NULL),		ERROR_INVALID_STATE );

		// Initialize initial string
		CCLOK ( pwBfr[0] = WCHAR('\0'); )

		// Format string
		CCLTRY	( formatString ( pFmt, pwBfr, &idx ) );
		pwBfr[idx] = WCHAR('\0');

		// Result.
		_EMT(Fire,adtString(pwBfr) );
		}	// if

	// Context
	else if (_RCP(Dictionary))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pDctSrc);
		hr = _QISAFE(unkV,IID_IDictionary,&pDctSrc);
		}	// else if
	else if (_RCP(Format))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pFmt);
		hr = _QISAFE(unkV,IID_IContainer,&pFmt);
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

