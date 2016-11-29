////////////////////////////////////////////////////////////////////////
//
//									DCTPRS.CPP
//
//		Implementation of the stream or string to dictionary parse node
//
////////////////////////////////////////////////////////////////////////

#include "miscl_.h"
#include <stdio.h>

DictParse :: DictParse ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pFmt			= NULL;
	pStm			= NULL;
	pDict			= NULL;
//	strParse		= "";
	}	// DictParse

HRESULT DictParse :: onAttach ( bool bAttach )
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
	HRESULT			hr		= S_OK;

	// Attach
	if (bAttach)
		{
		adtValue		v;
		adtIUnknown	unkV;

		// Allow format to specified as a node property
		if (pnDesc->load ( adtString(L"Format"),	v ) == S_OK)
			_QISAFE((unkV=v),IID_IContainer,&pFmt);
		}	// if

	// Detach
	else
		{
		_RELEASE(pFmt);
		_RELEASE(pDict);
		_RELEASE(pStm);
		}	// else

	return hr;
	}	// onAttach

HRESULT DictParse :: parse ( IContainer *pFmt )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Parses a binary stream or string into a dictionary using 
	//			the specified format.
	//
	//	PARAMETERS
	//		-	pFmt is the format specification
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr				= S_OK;
	IDictionary	*pDictSpec	= NULL;
	IDictionary	*pDictSub	= NULL;
	IContainer	*pFmtSub		= NULL;
	IIt			*pFmtIn		= NULL;
	adtIUnknown	unkV;
	adtValue		vSz;
	adtString	sName,sType;
	adtValue		vVal,vValUn,vValChk,vL;
	adtInt		uSz;

	// Iterator
	CCLTRY ( pFmt->iterate ( &pFmtIn ) );

	// Convert each field from stream according to specification
	CCLTRY ( pFmtIn->begin() );
	while (hr == S_OK && pFmtIn->read ( unkV ) == S_OK)
		{
		// Property set
		CCLTRY	( _QISAFE ( unkV, IID_IDictionary, &pDictSpec ) );

		// Fields (required)
		CCLTRY	( pDictSpec->load ( adtString(L"Type"), sType ) );

		// Size field required for certain type
		if (pDictSpec->load ( adtString(L"Size"), vSz ) == S_OK)
			{
			// Size, if a string is specified for the 'Size' field then it refers
			// to a field in the dictionary.
			if (hr == S_OK && adtValue::type(vSz) == VTYPE_STR)
				hr = pDict->load ( vSz, uSz );
			else if (hr == S_OK && vSz.vtype == VTYPE_I4)
				uSz = vSz.vint;
			else
				hr = E_UNEXPECTED;
			}	// if

		// Convert field based on type.  Little endian default.
		if (hr == S_OK && !WCASENCMP ( L"Int", sType, 3 ))
			{
			adtInt	uInt;
			adtBool	bSigned(false);

			// State check
			CCLTRYE(uSz <= 4,E_INVALIDARG);

			// Signed option
			if (hr == S_OK && pDictSpec->load ( adtString(L"Signed"), vL ) == S_OK)
				bSigned = vL;

			// Read value
			switch ((U32)uSz)
				{
				// Byte
				case 1 :
					{
					if (bSigned)
						{
						S8		b;
						CCLTRY ( pStm->read ( &b, 1, NULL ) );
						CCLOK  ( uInt.vint = (S32)b; )
						}	// if
					else
						{
						CCLTRY ( pStm->read ( &(uInt.vint), uSz, NULL ) );
						}	// else
					}	// case 1
					break;

				// Short
				case 2 :
					{
					if (bSigned)
						{
						S16	i;
						CCLTRY ( pStm->read ( &i, 2, NULL ) );
						CCLOK  ( uInt.vint = (S32)i; )
						}	// if
					else
						{
						CCLTRY ( pStm->read ( &(uInt.vint), 2, NULL ) );
						}	// else
//					if (bLittleE == false)	uInt.vint = (U16) SWAPS(uInt.vint);
					}	// case 1
					break;

				// Long
				case 4 :
					if (hr == S_OK)
						{
						CCLTRY ( pStm->read ( &(uInt.vint), uSz, NULL ) );
//						if (bLittleE == false)	uInt.vint = (U32) SWAPL(uInt.vint);
						}	// if
					break;

				default :
					dbgprintf ( L"DictParse::receiveFire:Invalid integer size\n" );
					hr = E_UNEXPECTED;
				}	// switch

			// Optional bit masking available
			if (pDictSpec->load ( adtString(L"Bits"), vL ) == S_OK)
				{
				IDictionary	*pBits	= NULL;
				IIt			*pItK		= NULL;

				// Iterate the specified key names
				CCLTRY(_QISAFE((unkV=vL),IID_IDictionary,&pBits));
				CCLTRY(pBits->keys ( &pItK ) );
				while (hr == S_OK && pItK->read ( vL ) == S_OK)
					{
					IDictionary		*pBit		= NULL;
					IDictionary		*pBitMap = NULL;
					adtString		strKey(vL);
					adtValue			vRes;
					U32				iMask;

					// Bit mask descriptor
					CCLTRY(pBits->load ( vL, vL ));
					CCLTRY(_QISAFE((unkV=vL),IID_IDictionary,&pBit));
					
					// Bit mask
					CCLTRY(pBit->load ( adtString(L"Mask"), vL ));

					// Apply mask
					CCLTRY ( adtValue::copy ( adtInt ( ((iMask = adtInt(vL)) & uInt) ), vRes ) );

					// Map result if available
					if (	hr == S_OK													&& 
							pBit->load ( adtString(L"Map"), vL ) == S_OK		&&
							(IUnknown *)(NULL) != (unkV=vL)						&&
							_QI((unkV=vL),IID_IDictionary,&pBitMap) == S_OK &&
							pBitMap->load ( vRes, vL ) == S_OK )
						hr = adtValue::copy ( vL, vRes );

					// Store result under key
					CCLTRY ( pDict->store ( strKey, vRes ) );

					// Clean up
					_RELEASE(pBitMap);
					_RELEASE(pBit);
					pItK->next();
					}	// while

				// Clean up
				_RELEASE(pItK);
				_RELEASE(pBits);
				}	// if

			// Store
			CCLOK  ( adtValue::copy ( uInt, vVal ); )
			}	// if

		else if (hr == S_OK && !WCASECMP ( L"Float", sType ))
			{
			adtFloat	fFlt;

			// Read
			CCLTRY ( pStm->read ( &(fFlt.vflt), sizeof(fFlt.vflt), NULL ) );
			CCLOK  ( adtValue::copy ( fFlt, vVal ); )
			}	// if

		else if (hr == S_OK && !WCASECMP ( L"Double", sType ))
			{
			adtDouble	fDbl;

			// Read
			CCLTRY ( pStm->read ( &(fDbl.vdbl), sizeof(fDbl.vdbl), NULL ) );
			CCLOK  ( adtValue::copy ( fDbl, vVal ); )
			}	// if

		else if (hr == S_OK && !WCASENCMP ( L"Bool", sType, 4 ))
			{
			adtBool bV;

			// Read
			CCLTRY ( pStm->read ( &(bV.vbool), sizeof(bV.vbool), NULL ) );
			CCLOK  ( adtValue::copy ( bV, vVal ); )
			}	// if

		else if (hr == S_OK && !WCASECMP ( L"String", sType ))
			{
			// Converting to internal string from assumed ASCII.
			WCHAR			*pwStr	= NULL;
			U8				c;
			adtValue		vL;
			adtString	sEnd;
			U32			i,j,uAllocd,elen;

			// If the size is set to zero, an ending sequence of
			// characters can be specified.
			uAllocd = 0;
			if (	hr == S_OK && 
					uSz == 0  &&
					pDictSpec->load ( adtString(L"End"), vL ) == S_OK )
				hr = adtValue::toString ( vL, sEnd );
			elen = sEnd.length();

			// Pre-allocate space for string
			i = 0;
			while (hr == S_OK && pStm->read ( &c, 1, NULL ) == S_OK)
				{
				// Ensure enough space in buffer
				if (hr == S_OK && i >= uAllocd)
					{
					// Re-allocate buffer
					uAllocd	+= 128;
					pwStr		= (WCHAR *) _REALLOCMEM ( pwStr, (uAllocd+1)*sizeof(WCHAR) );
					}	// if

				// Add character
				CCLOK ( pwStr[i++] = WCHAR(c); )

				// End of string ?
				if (hr == S_OK && uSz > 0 && i >= uSz)
					break;

				// Check for ending sequence ?
				if (hr == S_OK && uSz == 0)
					{
					// Special case is the end string is a 0 length string
					// which means end at null termination
					if (elen == 0 && c == '\0')
						break;
					else if (elen > 0 && i >= elen)
						{
						bool	bMatch = true;
						for (j = 0;j < sEnd.length() && bMatch;++j)
							if (pwStr[i-sEnd.length()+j] != sEnd.at(j))
								bMatch = false;

						// Match ?
						if (bMatch)
							break;
						}	// else if
					}	// if

				}	// for

			// Terminate string
			CCLOK ( pwStr[i] = WCHAR('\0'); )
			CCLOK ( adtValue::copy ( adtString(pwStr), vVal ); )

			// Clean up
			_FREEMEM(pwStr);
			}	// if

		else if (hr == S_OK && !WCASECMP ( L"Binary", sType ))
			{
			IByteStream	*pStmDst = NULL;
			adtIUnknown	unkV;
			
			// Size must be specified
			CCLTRYE ( uSz > 0, E_INVALIDARG );

			// Unformatted data, store as stream
			CCLTRY ( COCREATE ( L"Io.StmMemory", IID_IByteStream, &pStmDst ) );

			// Copy bytes
			CCLTRY( pStm->copyTo ( pStmDst, uSz, NULL ); )
			CCLTRY( pStmDst->seek ( STREAM_SEEK_SET, 0, NULL ) );
			CCLOK ( adtValue::copy ( adtIUnknown(pStmDst), vVal ); )

			// Clean up
			_RELEASE(pStmDst);
			}	// if

		// Mapping ?  Format specification can contain a mapping of from/to values
		if (hr == S_OK && pDictSpec->load ( adtString(L"Map"), unkV ) == S_OK)
			{
			IDictionary	*pMap	= NULL;

			// Load mapping
			CCLTRY(_QISAFE(unkV,IID_IDictionary,&pMap));
			CCLTRY(adtValue::copy ( vVal, vValUn ));
			// No need to error out on non-map ?
			CCLOK(pMap->load ( vValUn, vVal );)
			_RELEASE(pMap);
			}	// if
		else if (hr == S_OK)
			hr = adtValue::copy ( vVal, vValUn );

		// Process
		if (hr == S_OK)
			{
			// If name is specified, store under specified key
			if (pDictSpec->load ( adtString(L"Name"), sName ) == S_OK)
				hr = pDict->store ( sName, vVal );

			// If a value is specified, make sure it matches
			else if (pDictSpec->load ( adtString(L"Value"), vValChk ) == S_OK)
				hr = (adtValue::compare ( vValChk, vVal ) == 0) ? S_OK : S_FALSE;
			}	// if

		// 'Sub'format specified ?
		if (hr == S_OK && pDictSpec->load ( adtString(L"Subformat"), unkV ) == S_OK)
			{
			// Load the subformat for the current value.  Ok if missing, just means
			// there are no additional parameters for the current value
			CCLTRY ( _QISAFE(unkV,IID_IDictionary,&pDictSub) );
			if (hr == S_OK && pDictSub->load ( vValUn, unkV ) == S_OK)
				{
				// Parse new specification
				CCLTRY ( _QISAFE(unkV,IID_IContainer,&pFmtSub) );
				CCLTRY ( parse ( pFmtSub ) );

				// Clean up
				_RELEASE(pFmtSub);
				}	// if

			// Clean up
			_RELEASE(pDictSub);
			}	// if

		// Clean up
		_RELEASE(pDictSpec);
		pFmtIn->next();
		}	// while

	// Clean up
	_RELEASE(pFmtIn);

	return hr;
	}	// parse

HRESULT DictParse :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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
		adtIUnknown	unkV;
		adtString	sName,sType;
		adtValue		vVal;

		// State check
		CCLTRYE ( (pFmt != NULL),		ERROR_INVALID_STATE );
		CCLTRYE ( (pStm != NULL),		ERROR_INVALID_STATE );
		CCLTRYE ( (pDict != NULL),		ERROR_INVALID_STATE );

		// Parse as much as possible
		CCLTRY ( parse ( pFmt ); )

		// Result
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pDict) );
		else
			_EMT(Error,adtIUnknown(pDict) );
		}	// if

	// State
	else if (_RCP(Dictionary))
		{
		adtIUnknown	vUnk(v);

		// New value
		_RELEASE(pDict);
		CCLTRYE	( (IUnknown *)(NULL) != vUnk, E_INVALIDARG );
		CCLTRY	( _QI(vUnk,IID_IDictionary,&pDict) );
		}	// else if
	else if (_RCP(Format))
		{
		adtIUnknown		vUnk(v);

		// Clean up
		_RELEASE(pFmt);

		// A format specification is a list of property sets containing information
		// about each field
		CCLTRYE	( (IUnknown *)(NULL) != vUnk, E_INVALIDARG );
		CCLTRY	( _QI(vUnk,IID_IContainer,&pFmt) );
		}	// else if
	else if (_RCP(Stream))
		{
		adtIUnknown	vUnk(v);

		// New stream
		_RELEASE(pStm);
		CCLTRYE	( (IUnknown *)(NULL) != vUnk, E_INVALIDARG );
		CCLTRY	( _QI(vUnk,IID_IByteStream,&pStm) );
		}	// else if
//	else if (prStr == pR)
//		hr = adtValue::copy ( adtString(v), strParse );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive
