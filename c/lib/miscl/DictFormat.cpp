////////////////////////////////////////////////////////////////////////
//
//									DICTFMT.CPP
//
//					Implementation of the dictionary format node
//
////////////////////////////////////////////////////////////////////////

#include "miscl_.h"
#include <stdio.h>

DictFormat :: DictFormat ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pStm			= NULL;
	pDct			= NULL;
	pFmt			= NULL;
	bEndianBig	= false;

	// Frequently used keys
	strkSize	= L"Size";
	strkName	= L"Name";
	strkVal	= L"Value";
	strkMap	= L"Map";
	strkType	= L"Type";
	strkSub	= L"Subformat";
	}	// DictFormat

HRESULT DictFormat :: onAttach ( bool bAttach )
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
		if (pnDesc->load ( adtString(L"Endian"),	v ) == S_OK)
			onReceive ( prEndian, v );
		}	// if

	// Detach
	else
		{
		_RELEASE(pFmt);
		_RELEASE(pDct);
		_RELEASE(pStm);
		}	// else

	return hr;
	}	// onAttach

HRESULT DictFormat :: format ( IContainer *pFmt )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Perform formatting for the current list of fields.
	//
	//	PARAMETERS
	//		-	pFmt is the current format list
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr				= S_OK;
	IDictionary		*pDictSpec	= NULL;
	IDictionary		*pDictSub	= NULL;
	IDictionary		*pMap			= NULL;
	IContainer		*pFmtSub		= NULL;
	IIt				*pFmtIt		= NULL;
	adtIUnknown		unkV;
	adtInt			oSz;
	adtString		oName,strType;
	adtValue			vSz,oKey,oVal,oValueUn,v;

	// Iterate format specifications
	CCLTRY ( pFmt->iterate ( &pFmtIt ) );

	// Format fields from specification
	CCLTRY ( pFmtIt->begin() );
	while (hr == S_OK && pFmtIt->read ( v ) == S_OK)
		{
		// Dictionary for field
		CCLTRYE	( (IUnknown *)(NULL) != (unkV = v), E_INVALIDARG );
		CCLTRY	( _QI(unkV,IID_IDictionary,&pDictSpec) );

		// Size of output field
		CCLOK		( oSz = 0; )
		if (hr == S_OK && pDictSpec->load ( strkSize, vSz ) == S_OK)
			{
			// Size, if a string is specified for the 'Size' field then it refers
			// to a field in the dictionary.
			if (adtValue::type(vSz) == VTYPE_STR)
				hr = pDct->load ( vSz, vSz );
			if (vSz.vtype == VTYPE_I4)
				oSz = vSz.vint;
			else if (vSz.vtype == VTYPE_I8)
				oSz = (S32)(vSz.vlong);
			else
				hr = E_UNEXPECTED;
			}	// if

		// If no key name is specified, a predefined 'value' must be specified
		if (hr == S_OK)
			{
			if (pDictSpec->load ( strkName, oName ) == S_OK)
				hr = pDct->load ( oName, oVal );
			else
				hr = pDictSpec->load ( strkVal, oVal );
			}	// if

		// Keep a copy of the 'unmapped' value
		CCLOK  ( adtValue::copy ( oVal, oValueUn ); )

		// Mapping specified ?  Allow format to specify a dictionary that will 'map'
		// incoming 'from' dictionary values to 'to' output values to use during format.
		if (pDictSpec->load ( strkMap, v ) == S_OK)
			{
			CCLTRY(_QISAFE((unkV=v),IID_IDictionary,&pMap));
			CCLTRY(adtValue::copy ( oVal, oKey ));
			CCLTRY(pMap->load ( oKey, oVal ));
			_RELEASE(pMap);
			}	// if

		// If a type if specified for the value, allow conversion to the destination type
		if (hr == S_OK && pDictSpec->load ( strkType, strType ) == S_OK)
			{
			// Destination type
			VALUETYPE
			type =	(!WCASECMP ( strType, L"Double"))	? VTYPE_R8 :
						(!WCASECMP ( strType, L"Float"))		? VTYPE_R4 :
						(!WCASECMP ( strType, L"Integer" ))	? VTYPE_I4 :
						(!WCASECMP ( strType, L"Long" ))		? VTYPE_I8 :
						(!WCASECMP ( strType, L"Boolean" ))	? VTYPE_BOOL : 
						(!WCASECMP ( strType, L"Date" ))		? VTYPE_DATE : VTYPE_STR;

			// Convert
			hr = adtValue::toType ( oValueUn, type, oVal );
			}	// if

		// Write value to output based on type.  Little endian default.
		// Integer
		if (hr == S_OK && (oVal.vtype == VTYPE_I4 || oVal.vtype == VTYPE_I8))
			{
			// Write value
			switch ((U32)oSz)
				{
				// 'No byte' is ok, sort of like a 'no-op'
				case 0 :
					break;

				// Byte
				case 1 :
					{
					U8	b = (oVal.vtype == VTYPE_I8) ? (U8)(oVal.vlong & 0xff) : (U8)(oVal.vint & 0xff);
					CCLTRY ( pStm->write ( &b, 1, NULL ) );
					}	// case 1
					break;

				// Short
				case 2 :
					{
					U16	s = (oVal.vtype == VTYPE_I8) ? (U16)(oVal.vlong & 0xffff) : (U16)(oVal.vint & 0xffff);
					if (bEndianBig)	s = SWAPS(s);
					CCLTRY ( pStm->write ( &s, 2, NULL ) );
					}	// case 2
					break;

				// Long
				case 4 :
					{
					U32	i = (oVal.vtype == VTYPE_I8) ? (U32)(oVal.vlong & 0xffffffffff) : (U32)(oVal.vint);
					if (bEndianBig)	i = SWAPI(i);
					CCLTRY ( pStm->write ( &i, 4, NULL ) );
					}	// case 4
					break;

				// 64-bits
				case 8 :
					{
					U64	i = (oVal.vtype == VTYPE_I8) ? (U64)(oVal.vlong) : (U64)(oVal.vint);
					if (bEndianBig)	i = SWAPL(i);
					CCLTRY ( pStm->write ( &i, 8, NULL ) );
					}	// case 4
					break;

				default :
					dbgprintf ( L"ConvDictToBin::receiveFire:Invalid integer size\n" );
					hr = E_UNEXPECTED;
				}	// switch
			}	// if

		// Long
		else if (hr == S_OK && oVal.vtype == VTYPE_I8)
			{
			U64 v = oVal.vlong;
			CCLTRY ( pStm->write ( &v, sizeof(v), NULL ) );
			}	// if

		// Float
		else if (hr == S_OK && oVal.vtype == VTYPE_R4)
			{
			float	vflt = oVal.vflt;
			CCLTRY ( pStm->write ( &vflt, sizeof(vflt), NULL ) );
			}	// if

		// Double
		else if (hr == S_OK && oVal.vtype == VTYPE_R8)
			{
			double vdbl = oVal.vdbl;
			CCLTRY ( pStm->write ( &vdbl, sizeof(vdbl), NULL ) );
			}	// if

		// String
		else if (hr == S_OK && adtValue::type(oVal) == VTYPE_STR)
			{
			adtString	sVal(oVal);

			// Write each ASCIIified character to the stream
			U32	i,len = sVal.length();
			U8		c,z	= 0;

			// String, allow for variable length string by setting size to zero
			for (i = 0;i < len && (oSz == (U32)0 || i < oSz) && hr == S_OK;++i)
				{
				// Convert
				CCLOK ( c = (U8) (sVal[i] & 0xff); )

				// Write
				CCLTRY ( pStm->write ( &c, 1, NULL ) );
				}	// for

			// Remaining space
			for (;i < oSz && hr == S_OK;++i)
				hr = pStm->write ( &z, 1, NULL );
			}	// else if

		// Boolean
		else if (hr == S_OK && oVal.vtype == VTYPE_BOOL)
			{
			U16 vb = oVal.vbool;
			CCLTRY ( pStm->write ( &vb, sizeof(vb), NULL ) );
			}	// if

		// Object
		else if (hr == S_OK && oVal.vtype == VTYPE_UNK)
			{
			IByteStream	*pStmSrc	= NULL;

			// Byte stream ?
			if (oVal.punk != NULL && _QI(oVal.punk,IID_IByteStream,&pStmSrc) == S_OK)
				{
				U64	pos = 0;

				// Remember original position of stream
				CCLTRY ( pStmSrc->seek ( 0, STREAM_SEEK_CUR, &pos ) );

				// Copy to destination
				CCLTRY ( pStmSrc->copyTo ( pStm, oSz, NULL ) );

				// Restore position
				CCLTRY ( pStmSrc->seek ( pos, STREAM_SEEK_SET, NULL ) );

				// Clean up
				_RELEASE(pStmSrc);
				}	// if

			}	// else if

		// 'Sub'format specified ?
		if (hr == S_OK && pDictSpec->load ( strkSub, v ) == S_OK)
			{
			// Load the subformat for the current value.  Ok if missing, just means
			// there are no additional parameters for the current value
			CCLTRY ( _QISAFE((unkV=v),IID_IDictionary,&pDictSub) );
			if (hr == S_OK && pDictSub->load ( oValueUn, v ) == S_OK)
				{
				// Format new specification
				CCLTRY ( _QISAFE((unkV=v),IID_IContainer,&pFmtSub) );
				CCLTRY ( format ( pFmtSub ) );

				// Clean up
				_RELEASE(pFmtSub);
				}	// if

			// Clean up
			_RELEASE(pDictSub);
			}	// if

		// Clean up
		_RELEASE(pDictSpec);
		pFmtIt->next();
		}	// while

	// Clean up
	_RELEASE(pFmtIt);

	return hr;
	}	// format

HRESULT DictFormat :: onReceive ( IReceptor *pr, const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	The node has received a value on the specified receptor.
	//
	//	PARAMETERS
	//		-	pR is the receptor
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
		// State check
		CCLTRYE ( pFmt != NULL, ERROR_INVALID_STATE );
		CCLTRYE ( pDct != NULL, ERROR_INVALID_STATE );

		// TEMP until string support added
		CCLTRYE ( pStm != NULL, ERROR_INVALID_STATE );

		// Perform format
		CCLTRY ( format ( pFmt ) );

		// Result
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pStm));
		else
			_EMT(Error,adtInt(hr));
		}	// if

	// Format
	else if (_RCP(Format))
		{
		// A format specification is a list of dictionaries describing each field in the output.
		adtIUnknown	unkV(v);
		_RELEASE(pFmt);
		hr = _QISAFE(unkV,IID_IContainer,&pFmt);
		}	// else if

	// Dictionary
	else if (_RCP(Dictionary))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pDct);
		hr = _QISAFE(unkV,IID_IDictionary,&pDct);
		}	// else if

	// Stream
	else if (_RCP(Stream))
		{
		adtIUnknown	unkV(v);
		_RELEASE(pStm);
		hr = _QISAFE(unkV,IID_IByteStream,&pStm);
		}	// else if

	// State
	else if (_RCP(Endian))
		{
		adtString	strEnd(v);
		if (!WCASECMP(strEnd,L"Big"))
			bEndianBig = true;
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive
