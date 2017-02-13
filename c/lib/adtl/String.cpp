////////////////////////////////////////////////////////////////////////
//
//										STRING.CPP
//
//					Implementation of the 'string' class
//
////////////////////////////////////////////////////////////////////////

#include "adtl_.h"
#include <ctype.h>
#include <wctype.h>

adtString :: adtString ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////

	// Initialize interval value
	_ADTVINIT(*this);

	// Null str
	*this = L"";
	}	// adtString

adtString :: adtString ( const adtString &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	//	PARAMETERS
	//		-	v is the string with which to initialize the value
	//
	////////////////////////////////////////////////////////////////////////

	// Initialize interval value
	_ADTVINIT(*this);

	// New string object
	*this = (LPCWSTR)v;
	}	// adtString

adtString :: adtString ( const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	//	PARAMETERS
	//		-	v is the string with which to initialize the value
	//
	////////////////////////////////////////////////////////////////////////

	// Initialize interval value
	_ADTVINIT(*this);

	// New object
	*this = v;
	}	// adtString

adtString :: adtString ( const char *v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	//	PARAMETERS
	//		-	v is the string with which to initialize the value
	//
	////////////////////////////////////////////////////////////////////////

	// Initialize interval value
	_ADTVINIT(*this);

	// Assignment
	*this = v;
	}	// adtString

adtString :: adtString ( const WCHAR *v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	//	PARAMETERS
	//		-	v is the string with which to initialize the value
	//
	////////////////////////////////////////////////////////////////////////

	// Initialize interval value
	_ADTVINIT(*this);

	// Assignment
	*this = v;
	}	// adtString

adtString :: ~adtString ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Destructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
//	dbgprintf ( L"adtString::~adtString:pstr (%p,%p) %s\r\n",
//						((sysSTRING *)pstr)-1, ((sysSTRING *)pstr)-0,
//						(pstr != NULL) ? pstr : L"null" );
	adtValue::clear(*this);
	}	// ~adtString

HRESULT adtString :: allocate ( U32 len )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Allocate a string value that can hold the specified amount
	//			of characters (not including the NULL)
	//
	//	PARAMETERS
	//		-	len is the number of characters the string should be able 
	//			to hold.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	sysSTRING	*sys	= NULL;

	// Must own string 
	own(len);

	// This string
	if (pstr != NULL)
		sys = ((sysSTRING *)pstr)-1;

	// Only re-alloc if string space is insufficient
	if (sys == NULL || sys->nalloc < (LONG)len)
		{
		// Optimize by simply re-allocing any existing buffer
		CCLTRYE ( (sys = sysStringReallocLen ( sys, len )) != NULL,
						E_OUTOFMEMORY );
		}	// if

	// New string ptr.
	CCLOK ( pstr = (WCHAR *)(sys+1); )

	return hr;
	}	// allocate

HRESULT adtString :: append ( const WCHAR *strA )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Append a string to 'this' string.
	//
	//	PARAMETERS
	//		-	strA is the string to append to this string
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr		= S_OK;
	U32		len;

	// Total length needed
	len = length()+(U32)wcslen(strA);

	// Reallocate memory for total string
	CCLTRY ( allocate ( len ) );

	// Append string
	CCLOK ( WCSCAT ( &at(), len+1, strA ); )

	return hr;
	}	// append

HRESULT adtString :: append ( const WCHAR *strA, adtString &strD )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Generate a string comprised of this string plus 'strA'.
	//
	//	PARAMETERS
	//		-	strA is the string to append to this string
	//		-	strD will receive the appended string
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	const WCHAR	*str	= NULL;
	U32			len;

	// State check
	str =	*this;
	CCLTRYE ( str != NULL, E_UNEXPECTED );

	// Total length of both strings
	len = length()+(U32)wcslen(strA);

	// Allocate memory for final string	
	CCLTRY ( strD.allocate ( len ) );

	// Generate full string
	CCLOK ( WCSCPY ( &strD.at(), len+1, str ); )
	CCLOK ( WCSCAT ( &strD.at(), len+1, strA ); )

	return hr;
	}	// append

WCHAR &adtString :: at ( size_t index )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Return a ptr. to the string at the specified index.
	//
	//	PARAMETERS
	//		-	index is the index of the character to return
	//
	//	RETURN VALUE
	//		Ptr to string
	//
	////////////////////////////////////////////////////////////////////////

	// Must own string 
	own();

	// This string information
	sysSTRING	*
	pstrThis	= ((sysSTRING *)pstr)-1;

	// Rail index
	if (index > (size_t)pstrThis->nalloc)
		index = pstrThis->nalloc;

	return pstr[index];
	}	// at

S32 adtString :: indexOf ( WCHAR ch, S32 from )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Searches string forwards from the starting position and
	//			returns the first occurrence of the specified character.
	//
	//	PARAMETERS
	//		-	ch	is the character 
	//		-	from is the starting search position 
	//
	//	RETURN VALUE
	//		Index of character (-1 means not found)
	//
	////////////////////////////////////////////////////////////////////////
	const WCHAR *p;

	// Valid ?
	if (pstr == NULL)
		return -1;

	// Start position
	if (from >= (S32)wcslen(pstr))
		from = (S32)wcslen(pstr);

	// Search for character
	p = &pstr[from];
	while (*p != '\0')
		{
		if (*p == ch)
			break;
		else
			++p;
		}	// while

	return (*p != '\0') ? (S32)(p-pstr) : -1;
	}	// indexOf

S32 adtString :: indexOf ( const WCHAR *psub, S32 from )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Searches string forwards from the starting position and
	//			returns the first occurrence of the specified string.
	//			Default is case insensitive.
	//
	//	PARAMETERS
	//		-	psub is the substring to search for
	//		-	from is the starting search position 
	//
	//	RETURN VALUE
	//		Index of string (-1 means not found)
	//
	////////////////////////////////////////////////////////////////////////
	const WCHAR *pThis = *this;
	int			i,len,lens;

	// Valid ?
	if (pThis == NULL)
		return -1;

	// Start position
	len = (int)wcslen(pThis);
	if (from >= (S32)len)
		from = len;

	// Search internal string for substring
	lens = (int)wcslen(psub);
	while (from < len)
		{
		// Search from current character
		for (i = 0;i < lens && i < (len-from);++i)
			if (towlower(psub[i]) != towlower(pThis[i+from]))
				break;

		// If search made it to the end of the loop then a match was found
		if (i == lens)
			return from;

		// Move search position forward
		from += (i+1);
		}	// while

	// Not found
	return -1;
	}	// indexOf

S32 adtString :: lastIndexOf ( WCHAR ch, S32 from )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Searches string backwards from the starting position and
	//			returns the last occurrence of the specified character.
	//
	//	PARAMETERS
	//		-	ch	is the character 
	//		-	from is the starting search position (-1 means from end of string)
	//
	//	RETURN VALUE
	//		Index of last character (-1 means not found)
	//
	////////////////////////////////////////////////////////////////////////
	const WCHAR *p;
	S32			idx = -1;
	S32			len;

	// Sanity check
	if (pstr == NULL)
		return -1;

	// Length of entire string
	len = (S32)wcslen( pstr );

	// Starting search position
	if (from == -1 || from > len)
		from = len;

	// Search for character
	p = &pstr[from];
	while (p != pstr)
		if (*(--p) == ch)
			break;

	// Position
	idx = (S32)(p-pstr);

	return (*p) == ch ? idx : -1;
	}	// lastIndexOf

adtString &adtString :: operator= ( const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Assigns a value to this object.
	//
	//	PARAMETERS
	//		-	v is the source value
	//
	//	RETURN VALUE
	//		Reference to this object
	//
	////////////////////////////////////////////////////////////////////////
	adtValue::toString ( v, *this );
	return *this;
	}	// operator=

adtString &adtString :: operator= ( const adtString &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Assigns a string to this object.
	//
	//	PARAMETERS
	//		-	v is the source string
	//
	//	RETURN VALUE
	//		Reference to this object
	//
	////////////////////////////////////////////////////////////////////////
	*this	= (const WCHAR *)v;
	return *this;
	}	// operator=

adtString &adtString :: operator= ( const WCHAR *str )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Assigns a string to this object.
	//
	//	PARAMETERS
	//		-	str is the string
	//
	//	RETURN VALUE
	//		Reference to this object
	//
	////////////////////////////////////////////////////////////////////////

	// To avoid multiple possible clearing and reallocating with the same
	// string buffer, just use existing string if possible.
	if (vtype == VTYPE_STR)
		{
		int len = (int)((str != NULL) ? wcslen(str) : 0);

		// Reallocate and copy
		if (allocate ( len ) == S_OK && str != NULL)
			WCSCPY ( pstr, len+1, str );
		}	// if

	// String by reference
	// No need to assume a const WCHAR string will need its own buffer.
	// Wait until ownership is required before allocating a copy.
	// This handles 'local' use of a string object using just a ptr to
	// an existing string.
	else
		{
		// Clear existing value
		adtValue::clear(*this);

		// Reference string
		vtype = VTYPE_STR|VTYPE_BYREF; 
		pstr	= (WCHAR *) str;
		}	// else

	return *this;
	}	// operator=

adtString &adtString :: operator= ( const char *pc )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Assigns a string to this object.
	//			Straight ASCII to Wide conversion (no mbstowcs,etc)
	//
	//	PARAMETERS
	//		-	pc is the source string
	//
	//	RETURN VALUE
	//		Reference to this object
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr	= S_OK;
	U32		len;

	// Previous string
	adtValue::clear ( *this );
	vtype	= VTYPE_STR;

	#ifdef	_WIN32
	// Compute length of string required for conversion
	CCLTRYE ( (len = MultiByteToWideChar ( CP_ACP, 0, pc, -1, NULL, 0 )) != 0,
						GetLastError() );

	// Create a new string so it can be referenced
	CCLTRYE ( (pstr	= (WCHAR *)(sysStringAllocLen(NULL,len)+1)) != NULL,
					E_OUTOFMEMORY );

	// Peform conversion
	CCLTRYE ( MultiByteToWideChar ( CP_ACP, 0, pc, -1, pstr, len+1 ) != 0,
						GetLastError() );
	#else

	// Allocate room for string
	len	= (pc != NULL) ? (U32)strlen(pc) : 0;
	if ( (pstr = (WCHAR *)(sysStringAllocLen(NULL,len)+1)) != NULL )
		{
		// Copy new string
		U32	i;
		for (i = 0;i < len;++i)
			pstr[i] = pc[i];					// Little endian
		pstr[i] = WCHAR('\0');
		}	// if
	#endif

	// Result
	if (hr != S_OK)
		{
		// Contents invalid
		adtValue::clear(*this);
		vtype	= VTYPE_STR;

		// Debug
		lprintf ( LOG_ERR, L"const char *:Failed 0x%x", hr );
		}	// if

	return *this;
	}	// operator=

void adtString :: own ( U32 min )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Ensure this object 'owns' the internal string so 
	//			modifications can be made.
	//
	//	PARAMETERS
	//		-	min is the minimum length required of the string if
	//			reallocation is necessary.
	//
	////////////////////////////////////////////////////////////////////////

	// This string 'owns' its own string if the reference count is one
	if (vtype == VTYPE_STR && (((sysSTRING *)pstr)-1)->refcnt == 1)
		return;

	// Length of current string
	U32
	len = (U32)((pstr != NULL) ? wcslen(pstr) : 0);

	// Allocate memory for own string
	sysSTRING *
	pstrCpy	= sysStringAllocLen ( NULL, (len < min) ? min : len );

	// Copy contents
	if (pstrCpy != NULL && len > 0)
		WCSCPY ( (WCHAR *)(pstrCpy+1), len+1, pstr );

	// Release previous string
	adtValue::clear(*this);

	// New string
	vtype	= VTYPE_STR;
	pstr	= (WCHAR *)(pstrCpy+1);
	}	// own

HRESULT adtString :: prepend ( const WCHAR *strP )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Prepend a string to 'this' string.
	//
	//	PARAMETERS
	//		-	strP is the string to prepend to this string
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr		= S_OK;
	U32		lent,lenp,len;
	int		i;

	// Total length needed
	lenp	= (U32)wcslen(strP);
	lent	= length();
	len	= lenp+lent;

	// Reallocate memory for total string
	CCLTRY ( allocate ( len ) );

	// Move down characters and prepend provided ones
	pstr[len] = '\0';
	for (i = len-1;i >= 0;--i)
		pstr[i] = (i >= (int)lenp) ? pstr[i-lenp] : strP[i];
		
	return hr;
	}	// prepend

void adtString :: replace ( WCHAR from, WCHAR to )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Replaces all characters of one value to another
	//
	//	PARAMETERS
	//		-	from,to are the characters
	//
	////////////////////////////////////////////////////////////////////////
	int	i,l = length();

	// Modifying string
	own();

	// Replace
	for (i = 0;i < l;++i)
		if (pstr[i] == from)
			pstr[i] = to;
	}	// replace

HRESULT adtString :: substring ( S32 begin, S32 end, adtString &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Generates a substring.
	//
	//	PARAMETERS
	//		-	begin is the beginning index of the substring
	//		-	end is the ending index of the substring exclusive.
	//		-	v will receive the substring
	//
	//	RETURN VALUE
	//		true if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr		= S_OK;
	const WCHAR *str	= NULL;
	S32			len,idx;

	// State check
	str =	(adtValue::type(*this) == VTYPE_STR) ? pstr :  NULL;
	CCLTRYE ( str != NULL, E_UNEXPECTED );

	// Process string
	if (hr == S_OK)
		{
		// Length of entire string
		len = (S32)wcslen ( str );

		// Beginning index
		if (begin > len)
			begin = len;

		// Ending index
		if (end == -1 || end > len)
			end = len;

		// Allocate memory for a new string
		CCLTRY ( v.allocate ( end-begin ) );
		}	// if

	// Copy string
	if (hr == S_OK)
		{
		for (idx = begin;idx < end;++idx)
			v.at(idx-begin) = str[idx];
		v.at(idx-begin) = '\0';
		}	// if

	return hr;
	}	// substring

HRESULT adtString :: toAscii ( char **ppcStr ) const
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Allocates/creates an ASCII string version of the object's
	//			unicode string.
	//
	//	PARAMETERS
	//		-	ppcStr will receive the string (CoTaskMemFree to free).
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	const WCHAR *pThis	= *this;
	int			len;
	#ifdef		_WIN32

	// Setup
	*ppcStr = NULL;

	// Valid ?
	if (pThis == NULL || (len = length()) == 0)
		return E_INVALIDARG;

	// Compute length of string required for conversion
	CCLTRYE ( (len = WideCharToMultiByte ( CP_ACP, 0, pThis, -1, NULL, 0, 
					NULL, NULL )) != 0, GetLastError() );

	// Allocate memory for ASCII string
	CCLTRYE ( ((*ppcStr = (char *) _ALLOCMEM(len+1)) != NULL), E_OUTOFMEMORY );

	// Perform conversion
	CCLTRYE ( WideCharToMultiByte ( CP_ACP, 0, pThis, -1, *ppcStr, len+1,
												NULL, NULL ) > 0, GetLastError() );
	#elif	__unix__ || __APPLE__

	// Setup
	(*ppcStr) = NULL;

	// Allocate
	if (hr == S_OK)
		{
		len = length();
		hr = (*ppcStr = (char *) _ALLOCMEM ( len+1 )) != NULL ?
				S_OK : E_OUTOFMEMORY;
		}	// if
	if (hr == S_OK) (*ppcStr)[0] = '\0';

	// Initialize
	if (hr == S_OK && pstr != NULL)
		{
		int i;
		for (i = 0;i < len;++i)
			(*ppcStr)[i] = (char) (pThis[i] & 0xff);
		(*ppcStr)[i] = '\0';
		}	// if

	#endif

	return hr;
	}	// toAscii

#ifdef	_WIN32
HRESULT adtString :: toBSTR ( BSTR *ppbStr ) const
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Allocates/creates an ASCII string version of the object's
	//			unicode string.
	//
	//	PARAMETERS
	//		-	ppcStr will receive the string (CoTaskMemFree to free).
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	const WCHAR *pThis	= *this;
	U32			len;

	// Setup
	*ppbStr = NULL;
	
	// Allocate BSTR
	CCLOK		(  len = (U32)((pThis != NULL) ? wcslen(pThis) : 0); )
	CCLTRYE	( (*ppbStr = SysAllocStringLen ( NULL, len )) != NULL, E_OUTOFMEMORY );

	// Copy string
	if (hr == S_OK && len > 0)
		WCSCPY (	(*ppbStr), len+1, pThis );

	return hr;
	}	// toBSTR
#endif

void adtString :: toLower ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Convert the string to a lower case version.
	//
	////////////////////////////////////////////////////////////////////////

	// Modifying string
	own();

	// Convert
	for (WCHAR *pc = pstr;pc != NULL && *pc != '\0';++pc)
		*pc = towlower ( (*pc) );

	}	// toLower

HRESULT adtString :: toUnquoted ( IList *pLst )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Break the current string into a list of unquoted strings
	//			seperated by whitespace.
	//
	//	PARAMETERS
	//		-	pLst will receive the list of strings
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	const WCHAR *pThis	= *this;
	const WCHAR	*f,*b;
	WCHAR			ec;
	int			len;

	// Valid ?
	if (pThis == NULL)
		return S_OK;

	// Process entire string
	f = pThis;
	while (hr == S_OK && *f != '\0')
		{
		// Skip whitespace
		while (*f != '\0' && iswspace(*f))
			++f;

		// If first character is a quote then assume the entire string is quoted,
		// otherwise the first space encountered will terminate the string.
		if (*f == '\"')
			{
			// Skip quote
			++f;

			// End character
			ec = '\"';
			}	// if
		else
			ec = ' ';

		// Scan until end character found
		b = f;
		while (*b != '\0' && *b != ec)
			++b;

		// Length of new string
		if ( (len = (S32)(b-f)) > 0)
			{
			adtString	str;

			// Create substring, do not include end quote
			CCLTRY ( substring ( 
							(S32)(f-pThis),
							(S32)(b-pThis),
							str ) );

			// Add to list
			CCLTRY ( pLst->write ( str ) );
			}	// if

		// Continue
		f = ((*b) == '\"') ? (b+1) : b;
		}	// while

	return hr;
	}	// toUnquoted

void adtString :: toUpper ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Convert the string to a upper case version.
	//
	////////////////////////////////////////////////////////////////////////

	// Modifying string
	own();

	// Convert
	for (WCHAR *pc = pstr;pc != NULL && *pc != '\0';++pc)
		*pc = towupper ( (*pc) );

	}	// toUpper

//
// Debug
//

bool adtwcasecmp ( const WCHAR *pwF, const WCHAR *pwSrc, const WCHAR *pwDst )
	{
//	if (WCASECMP(pwSrc,L"_Descriptor") && WCASECMP(pwDst,L"_Descriptor"))
//		dbgprintf ( L"adtwcasecmp:%s:%s:%s\r\n", pwF, pwSrc, pwDst );
	return !WCASECMP(pwSrc,pwDst);
	}	// rcp

//
// adtStringRef
//
/*
adtStringRef :: adtStringRef ( const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	//	PARAMETERS
	//		-	v is the value with which to initialize.
	//
	////////////////////////////////////////////////////////////////////////

	// If value is already a string set up as a reference.
	if (v.vtype == VTYPE_STR && v.pstr != NULL)
		{
		pstr	= v.pstr;
		vtype	= (VTYPE_STR|VTYPE_BYREF);
		}	// if

	// otherwise convert to a string as normal.
	else
		adtValue::toString ( v, *this );

	}	// adtStringRef
*/