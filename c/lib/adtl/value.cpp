////////////////////////////////////////////////////////////////////////
//
//										VALUE.CPP
//
//					Implementation of the 'value' class
//
////////////////////////////////////////////////////////////////////////

#define	INITGUID
#include "adtl_.h"
#include "../ccl/ccl.h"
#include <stdio.h>

// Definitions
#define	SECINDAY		(60.0*60.0*24.0)				// # of seconds in a day
#define	RNDINT(a)	((S32)( (a) + ( ((a) > 0.0) ? 0.5 : -0.5 )))

adtValue :: adtValue ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	_ADTVINIT(*this);
	}	// adtValue

adtValue :: adtValue ( ADTVALUE *pv )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	//	PARAMETERS
	//		-	Ptr. to value reference.
	//
	////////////////////////////////////////////////////////////////////////
	_ADTVINIT(*this);
	pval	= pv;
	vtype	= (VTYPE_VALUE|VTYPE_BYREF);
	}	// adtValue

adtValue :: ~adtValue ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Destructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	clear(*this);
	}	// ~adtValue

void adtValue :: clear ( ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Frees any resources associated with value and sets it back
	//			to 'empty'.
	//
	//	PARAMETERS
	//		-	v is the value to clear
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////

	// Release resources based on type
	if (v.vtype == VTYPE_STR && v.pstr != NULL)
		{
		sysStringRelease ( ((sysSTRING *)v.pstr) - 1 );
		}	// if
	else if (v.vtype == VTYPE_UNK && v.punk != NULL)
		v.punk->Release();

	// Initialize value
	v.vtype	= VTYPE_EMPTY;
	v.vdbl	= 0.0;
	}	// clear

HRESULT adtValue :: clone ( const ADTVALUE &src, ADTVALUE &dst )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Clones a value.
	//
	//	PARAMETERS
	//		-	src is the value to clone
	//		-	dst will receive the cloned value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	ICloneable	*pClone	= NULL;

	// Default to copying the value
	CCLTRY ( adtValue::copy ( src, dst ) );

	// Object - Allow clone of object if it is supported
	if (	src.vtype == VTYPE_UNK	&& 
			src.punk != NULL			&&
			_QI(src.punk,IID_ICloneable,&pClone) == S_OK)
		{
		IUnknown		*pUnk		= NULL;

		// Create clone object
		CCLTRY ( pClone->clone ( &pUnk ) );

		// Place in destination
		_RELEASE(dst.punk);
		dst.punk = pUnk;
		_ADDREF(dst.punk);

		// Clean up
		_RELEASE(pUnk);
		_RELEASE(pClone);
		}	// if

	return hr;
	}	// clone

int adtValue :: compare ( const ADTVALUE &v1, const ADTVALUE &v2 )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Performs a compare of the given two values.  Allows mixing of
	//			types where appropriate.
	//
	//	PARAMETERS
	//		-	v1,v2 are the values to compare
	//
	//	RETURN VALUE
	//		-1 if v1 < v2, 0 if v1 = v2, 1 if v1 > v2,
	//		E_INVALIDARG if values cannot be compared.
	//
	////////////////////////////////////////////////////////////////////////
	int		ret = E_INVALIDARG;

	// 
	// Try and support every combination that makes 'sense'
	//
	switch ( v1.vtype )
		{
		// Integer
		case VTYPE_I4 :
			switch ( v2.vtype )
				{
				case VTYPE_I4 :
					ret = (v1.vint < v2.vint) ? -1 :
							(v1.vint > v2.vint) ? 1 : 0;
					break;
				case VTYPE_I8 :
					ret = (v1.vint < v2.vlong) ? -1 :
							(v1.vint > v2.vlong) ? 1 : 0;
					break;
				case VTYPE_R4 :
					ret = (v1.vint < v2.vflt) ? -1 :
							(v1.vint > v2.vflt) ? 1 : 0;
					break;
				case VTYPE_R8 :
				case VTYPE_DATE :
					ret = (v1.vint < v2.vdbl) ? -1 :
							(v1.vint > v2.vdbl) ? 1 : 0;
					break;
				case VTYPE_VALUE|VTYPE_BYREF :
					ret = (v2.pval != NULL) ? adtValue::compare ( v1, (*v2.pval) ) : 0;
					break;
				}	// switch
			break;

		// Long
		case VTYPE_I8 :
			switch ( v2.vtype )
				{
				case VTYPE_I4 :
					ret = (v1.vlong < v2.vint) ? -1 :
							(v1.vlong > v2.vint) ? 1 : 0;
					break;
				case VTYPE_I8 :
					ret = (v1.vlong < v2.vlong) ? -1 :
							(v1.vlong > v2.vlong) ? 1 : 0;
					break;
				case VTYPE_R4 :
					ret = (v1.vlong < v2.vflt) ? -1 :
							(v1.vlong > v2.vflt) ? 1 : 0;
					break;
				case VTYPE_R8 :
				case VTYPE_DATE :
					ret = (v1.vlong < v2.vdbl) ? -1 :
							(v1.vlong > v2.vdbl) ? 1 : 0;
					break;
				case VTYPE_VALUE|VTYPE_BYREF :
					ret = (v2.pval != NULL) ? adtValue::compare ( v1, (*v2.pval) ) : 0;
					break;
				}	// switch
			break;

		// Float
		case VTYPE_R4 :
			switch ( v2.vtype )
				{
				case VTYPE_I4 :
					ret = (v1.vflt < v2.vint) ? -1 :
							(v1.vflt > v2.vint) ? 1 : 0;
					break;
				case VTYPE_I8 :
					ret = (v1.vflt < v2.vlong) ? -1 :
							(v1.vflt > v2.vlong) ? 1 : 0;
					break;
				case VTYPE_R4 :
					ret = (v1.vflt < v2.vflt) ? -1 :
							(v1.vflt > v2.vflt) ? 1 : 0;
					break;
				case VTYPE_R8 :
				case VTYPE_DATE :
					ret = (v1.vflt < v2.vdbl) ? -1 :
							(v1.vflt > v2.vdbl) ? 1 : 0;
					break;
				case VTYPE_VALUE|VTYPE_BYREF :
					ret = (v2.pval != NULL) ? adtValue::compare ( v1, (*v2.pval) ) : 0;
					break;
				}	// switch
			break;

		// Double/Date (currently uses that fact that a union is used and 'date' is a double).
		case VTYPE_R8 :
		case VTYPE_DATE :
			switch ( v2.vtype )
				{
				case VTYPE_I4 :
					ret = (v1.vdbl < v2.vint) ? -1 :
							(v1.vdbl > v2.vint) ? 1 : 0;
					break;
				case VTYPE_I8 :
					ret = (v1.vdbl < v2.vlong) ? -1 :
							(v1.vdbl > v2.vlong) ? 1 : 0;
					break;
				case VTYPE_R4 :
					ret = (v1.vdbl < v2.vflt) ? -1 :
							(v1.vdbl > v2.vflt) ? 1 : 0;
					break;
				case VTYPE_R8 :
				case VTYPE_DATE :
					ret = (v1.vdbl < v2.vdbl) ? -1 :
							(v1.vdbl > v2.vdbl) ? 1 : 0;
					break;
				case VTYPE_VALUE|VTYPE_BYREF :
					ret = (v2.pval != NULL) ? adtValue::compare ( v1, (*v2.pval) ) : 0;
					break;
				}	// switch
			break;

		// Boolean
		case VTYPE_BOOL :
			switch ( v2.vtype )
				{
				case VTYPE_BOOL :
					ret = (v1.vbool == v2.vbool) ? 0 :
							(!v1.vbool && v2.vbool) ? -1 : 1;
					break;
				case VTYPE_VALUE|VTYPE_BYREF :
					ret = (v2.pval != NULL) ? adtValue::compare ( v1, (*v2.pval) ) : 0;
					break;
				}	// switch
			break;

		// String
		case VTYPE_STR :
		case VTYPE_STR|VTYPE_CONST :
		case VTYPE_STR|VTYPE_BYREF :
			switch ( v2.vtype )
				{
				case VTYPE_STR :
				case VTYPE_STR|VTYPE_CONST :
				case VTYPE_STR|VTYPE_BYREF :
					// Special case.  Two reference strings ptrs can be compared to see if it
					// is literally the same string
					if (v1.pstr == v2.pstr)
						ret = 0;								// Equal
					else
						{
						// NOTE: Accessing string directly to avoid having
						// to construct a string object for every compare.
						ret = WCASECMP ( (v1.pstr != NULL) ? v1.pstr : L"", (v2.pstr != NULL) ? v2.pstr : L"" );
						ret = (ret > 0) ? 1 :
								(ret < 0) ? -1 : 0;
						}	// else 
					break;
				case VTYPE_VALUE|VTYPE_BYREF :
					ret = (v2.pval != NULL) ? adtValue::compare ( v1, (*v2.pval) ) : 0;
					break;
				}	// switch
			break;

		// Value reference
		case VTYPE_VALUE|VTYPE_BYREF :
			ret = (v1.pval != NULL) ? adtValue::compare ( *(v1.pval), v2 ) : 0;
			break;

		// Object
		case VTYPE_UNK :
		case VTYPE_UNK|VTYPE_BYREF :
			switch ( v2.vtype )
				{
				case VTYPE_UNK :
				case VTYPE_UNK|VTYPE_BYREF :
					{
					// Object ptrs.
					IUnknown	*punk1	=	NULL;
					IUnknown	*punk2	=	NULL;

					// To ensure proper object comparison, always compare the same IUnknown interface.
					_QISAFE(v1.punk,IID_IUnknown,&punk1);
					_QISAFE(v2.punk,IID_IUnknown,&punk2);

					// Compare
					ret = (punk1 < punk2) ? -1 :
							(punk1 > punk2) ? 1 : 0;

					// Clean up
					_RELEASE(punk1);
					_RELEASE(punk2);
					}	// VT_UNK
					break;
				case VTYPE_VALUE|VTYPE_BYREF :
					ret = (v2.pval != NULL) ? adtValue::compare ( v1, (*v2.pval) ) : 0;
					break;
				}	// switch
			break;
		}	// switch

	return ret;
	}	// compare

HRESULT adtValue :: copy ( const ADTVALUE &src, ADTVALUE &dst, bool bDeref )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Copies on value to another after clear the target value first.
	//
	//	PARAMETERS
	//		-	src is the value to copy
	//		-	dst will receive the copied value
	//		-	bDeref is true to dereference any referred values into local copies.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr	= S_OK;

	// Sanity check.  Do not proceed if the values are the same location
	// in memory to avoid clearing the value.  By definition, a copy of a
	// value at same memory location will be a 'no-op'.
	if (&src == &dst)
		return S_OK;

	// Clear destination
	adtValue::clear(dst);

	// Copy value
	dst.vtype = src.vtype;

	// Object
	if (src.vtype == VTYPE_UNK)
		{
		if ( (dst.punk = src.punk) )
			dst.punk->AddRef();
		}	// if

	// Value reference
	else if (src.vtype == (VTYPE_VALUE|VTYPE_BYREF))
		{
		// Can just copy the ptr.
		dst.pval = src.pval;

		// De-reference ?
		if (bDeref)
			hr = adtValue::copy ( *(src.pval), dst );
		}	// else if

	// String
	else if (src.vtype == VTYPE_STR)
		{
		// A reference to the string can just be incremented.
		dst.pstr = src.pstr;
		if (dst.pstr != NULL)
			_ADDREFSTR ( ((sysSTRING *)dst.pstr)-1 );
		}	// else if

	// By reference string
	else if (src.vtype == (VTYPE_STR|VTYPE_BYREF))
		{
		// Must be allocated/copied since unknown if const ptr 
		// will remain valid beyond object lifetime.
		adtString	strC = src.pstr;

		// This will force ownership
		strC.at();

		// Copy the now dynamic string
		adtValue::copy ( strC, dst );
		}	// else if

	// Const/static string
	else if (src.vtype == (VTYPE_STR|VTYPE_CONST))
		{
		// Can just copy the ptr.
		dst.pstr = src.pstr;
		}	// else if

	// Default (copy biggest member in union will copy any other value)
	else
		dst.vdbl = src.vdbl;

	return hr;
	}	// copy

HRESULT adtValue :: fromString ( const WCHAR *s, VALUETYPE type, 
											ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Converts the string to its value.
	//
	//	PARAMETERS
	//		-	s is the string
	//		-	type is the type to which to convert.
	//		-  v will receive the value 
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;

	// Convert
	switch ((int)type)
		{
		case VTYPE_STR :
		case VTYPE_STR|VTYPE_BYREF :
		case VTYPE_STR|VTYPE_CONST :
			adtValue::copy ( adtString(s), v );
			break;
		case VTYPE_I4 :
			{
			adtInt	oInt;
			// Convert based on hex or decimal...
			if (s != NULL)
				{
				if (	s[0] == WCHAR('0') &&
						(	s[1] == WCHAR('x') ||
							s[1] == WCHAR('X') ) )
					oInt = (U32)(wcstoul ( s+2, NULL, 16 ));
				else
					oInt = (U32)(wcstoul ( s, NULL, 10 ));
				}	// if
			adtValue::copy ( oInt, v );
			}	// case VTYPE_I4
			break;
		case VTYPE_I8 :
			{
			adtLong	oLong;
			// Convert based on hex or decimal...
			if (s != NULL)
				{
				if (	s[0] == WCHAR('0') &&
						(	s[1] == WCHAR('x') ||
							s[1] == WCHAR('X') ) )
					oLong = (U32)(wcstoul ( s+2, NULL, 16 ));
				else
					oLong = (U32)(wcstoul ( s, NULL, 10 ));
				}	// if
			adtValue::copy ( oLong, v );
			}	// case VTYPE_I8
			break;
		case VTYPE_BOOL :
			{
			adtBool	oBool;
			// TRUE/FALSE
			oBool = (s != NULL && (!WCASECMP ( s, L"true" ) || 
						!WCASECMP ( s, L"1" ))) ? true : false;
			adtValue::copy ( oBool, v );
			}	// case VTYPE_BOOL
			break;
		case VTYPE_R4 :
			{
			adtFloat	oFlt;
			// Convert
//			if (s != NULL) oFlt.vflt = (float)_wtof(s);
			if (s != NULL) SWSCANF ( s, L"%f", &(oFlt.vflt) );
			adtValue::copy ( oFlt, v );
			}	// case VTYPE_R4
			break;
		case VTYPE_R8 :
			{
			adtDouble	oDbl;
			// Convert
//			if (s != NULL) oDbl.vdbl = _wtof(s);
			if (s != NULL) SWSCANF ( s, L"%lf", &(oDbl.vdbl) );
			adtValue::copy ( oDbl, v );
			}	// case VTYPE_R8
			break;
		case VTYPE_DATE :
			CCLTRY ( adtDate::fromString ( s, v ) );
			break;
		default :
			dbgprintf ( L"adtValue::fromString:Unimplemented value type\r\n" );
			hr = E_NOTIMPL;
		}	// switch

	return hr;
	}	// fromString

HRESULT adtValue :: toString ( const ADTVALUE &v, adtString &s )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Converts the value to its string representation.
	//
	//	PARAMETERS
	//		-	v is the value to convert
	//		-	s will receive the representation
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;
	WCHAR		wNumStr[21];

	// Convert
	switch (v.vtype)
		{
		case VTYPE_I4 :
			{
			swprintf ( SWPF(wNumStr,20), L"%d", v.vint );
			s = wNumStr;
			s.at();
			}	// VTYPE_I4
			break;
		case VTYPE_I8 :
			{
			WCHAR	wNumStr[21];
			swprintf ( SWPF(wNumStr,20), L"%lld", v.vlong );
			s = wNumStr;
			s.at();
			}	// VTYPE_I8
			break;
		case VTYPE_R4 :
			{
			WCHAR	wNumStr[21];
			swprintf ( SWPF(wNumStr,20), L"%.6g", v.vflt );
			s = wNumStr;
			s.at();
			}	// VTYPE_R4
			break;
		case VTYPE_R8 :
			{
			WCHAR	wNumStr[31];
			swprintf ( SWPF(wNumStr,20), L"%.15g", v.vdbl );
			s = wNumStr;
			s.at();
			}	// VTYPE_R8
			break;
		case VTYPE_DATE :
			CCLTRY ( adtDate::toString(v,s) );
			s.at();
			break;
		case VTYPE_BOOL :
			{
			s = (v.vbool == TRUE) ? L"true" : L"false";
			}	// VTYPE_BOOL
			break;
		case VTYPE_STR :
		case VTYPE_STR|VTYPE_CONST :
		case VTYPE_STR|VTYPE_BYREF :
			{
			// Use copy logic
			CCLTRY ( adtValue::copy ( v, s ) );
			}	// VTYPE_STR
			break;
		case VTYPE_UNK :
		case VTYPE_UNK|VTYPE_BYREF :
			{
			IUnknown	*punk	= NULL;

			// Always use the same IUnknown so the same object converts to
			// the same string.
			if (v.punk != NULL)
				_QI(v.punk,IID_IUnknown,&punk);
			swprintf ( SWPF(wNumStr,20), L"%llX", (U64)(punk) );
			s = wNumStr;
			s.at();
			_RELEASE(punk);
			}	// VTYPE_UNKNOWN
			break;
		case VTYPE_VALUE|VTYPE_BYREF :
			CCLTRYE( (v.pval != NULL), E_INVALIDARG );
			CCLTRY ( adtValue::toString ( *(v.pval), s ) );
			break;
		default :
			hr = E_INVALIDARG;
		}	// switch

	return hr;
	}	// toString

HRESULT adtValue :: toType ( const ADTVALUE &vF, VALUETYPE type, 
										ADTVALUE &vT )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Converts the value to the specified type.
	//
	//	PARAMETERS
	//		-	vF is the value to convert
	//		-	type is the desired type.
	//		-	vT will receive the converted value
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Anything to a string
	if (type == VTYPE_STR)
		{
		adtString	vStr;
		CCLTRY ( adtValue::toString ( vF, vStr ) );
		CCLTRY ( adtValue::copy ( vStr, vT ) );
		}	// if

	// Non-string
	else
		{
		switch (vF.vtype)
			{
			case VTYPE_STR :
				CCLTRY ( adtValue::fromString ( vF.pstr, type, vT ) );
				break;
			case VTYPE_I4	:
			case VTYPE_I8	:
			case VTYPE_R4	:
			case VTYPE_R8	:
			case VTYPE_DATE:
			case VTYPE_BOOL:
				switch(type)
					{
					case VTYPE_I4 :
						CCLTRY ( adtValue::copy ( adtInt(vF), vT ) );
						break;
					case VTYPE_I8 :
						CCLTRY ( adtValue::copy ( adtLong(vF), vT ) );
						break;
					case VTYPE_R4 :
						CCLTRY ( adtValue::copy ( adtFloat(vF), vT ) );
						break;
					case VTYPE_R8 :
						CCLTRY ( adtValue::copy ( adtDouble(vF), vT ) );
						break;
					case VTYPE_DATE :
						CCLTRY ( adtValue::copy ( adtDate(vF), vT ) );
						break;
					case VTYPE_BOOL :
						CCLTRY ( adtValue::copy ( adtBool(vF), vT ) );
						break;
					default :
						hr = E_NOTIMPL;
					}	// switch	
				break;
			case VTYPE_UNK :
				{
				IUnknown	*pUnk	= NULL;

				// To ensure conversion is always the same, query for the base IUnknown
				// interface before conversion.

				// Base IUnknown
				CCLTRY ( _QISAFE(vF.punk,IID_IUnknown,&pUnk) );

				// Convert to supported types
				if (type == VTYPE_I8)
					adtValue::copy ( adtLong((U64)vF.punk), vT );
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

	return hr;
	}	// toType

//
// Operators
//

adtValue& adtValue::operator= ( const adtValue &v )
	{
	copy ( v, *this );
	return *this;
	}	// operat

#ifdef	_WIN32

adtValue& adtValue::operator= ( const VARIANT &var )
	{
	// Reset state
	adtValue::clear(*this);

	// Map internal types to variant types
	switch (var.vt)
		{
		case VT_EMPTY :
		case VT_NULL :
			break;
		case VT_BOOL :
			vtype		= VTYPE_BOOL;
			vbool		= (var.boolVal == VARIANT_TRUE) ? TRUE : FALSE;
			break;
		case VT_DATE :
			vtype		= VTYPE_DATE;
			vdate		= var.date;
			break;
		case VT_R4 :
			vtype		= VTYPE_R4;
			vflt		= var.fltVal;
			break;
		case VT_R8 :
			vtype		= VTYPE_R8;
			vdbl		= var.dblVal;
			break;
		case VT_I4 :
			vtype		= VTYPE_I4;
			vint		= var.lVal;
			break;
		case VT_I8 :
			vtype		= VTYPE_I8;
			vdbl		= var.dblVal;
			break;
		case VT_BSTR :
			copy ( adtString(var.bstrVal), *this );
			break;
		case VT_UNKNOWN :
			vtype		= VTYPE_UNK;
			punk		= var.punkVal;
			if (punk != NULL)
				punk->AddRef();
			break;
		case VT_DISPATCH :
			vtype		= VTYPE_UNK;
			punk		= var.pdispVal;
			if (punk != NULL)
				punk->AddRef();
			break;
		default :
			dbgprintf ( L"adtValue::adtValue:Unable to handle variant type 0x%x\r\n", var.vt );
		}	// switch

	return *this;
	}	// adtValue

#endif

