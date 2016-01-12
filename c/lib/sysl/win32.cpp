////////////////////////////////////////////////////////////////////////
//
//									WIN32.CPP
//
//								Win32 routines
//
////////////////////////////////////////////////////////////////////////

#include "sysl.h"
#include <errno.h>

#define	SECINDAY		(60.0*60.0*24.0)

extern "C"
S32 GetLastError()
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Returns the last reported error.
	//
	//	RETURN VALUE
	//		-	Error value
	//
	////////////////////////////////////////////////////////////////////////
	return errno;
	}	// GetLastError

extern "C"
BOOL IsEqualGUID ( const GUID &id1, const GUID &id2 )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Checks to GUIDs for equality
	//
	//	PARAMETERS
	//		-	id1,id2 are the IDs
	//
	//	RETURN VALUE
	//		-	TRUE if equal
	//
	////////////////////////////////////////////////////////////////////////
	return	(	((U32 *) &id1)[0] == ((U32 *)&id2)[0] &&
					((U32 *) &id1)[1] == ((U32 *)&id2)[1] &&
					((U32 *) &id1)[2] == ((U32 *)&id2)[2] &&
					((U32 *) &id1)[3] == ((U32 *)&id2)[3] );
	}	// IsEqualGUID

S32 MultiByteToWideChar	( U32 cp, S32 sFlags, const char *pStr, S32 iLen,
									WCHAR *pwStr, S32 iLenW )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Maps a character string to a wide string.
	//
	//	PARAMETERS
	//		-	cp is the code page to use
	//		-	sFlags are translation flags
	//		-	pStr is the string to be converted
	//		-	iLen is the length of the string
	//		-	pwStr will receive the string
	//		-	iLenW is the size in wide characters of the buffer
	//
	//	RETURN VALUE
	//		Number of characters written/size needed
	//
	////////////////////////////////////////////////////////////////////////
	S32	lenReq = 0;
	
	// Calculate space required for new string
	lenReq = ((S32)strlen(pStr)+1);

	// Buffer provided ?
	if (iLenW > 0 && lenReq > iLenW)
		lenReq = 0;
		
	// Copy string
	if (lenReq > 0 && iLenW > 0)
		{
		// Perform conversion
		mbstowcs ( pwStr, pStr, lenReq );
		}
	
	return lenReq;
	}	// MultiByteToWideChar

HRESULT	VarDateFromStr ( WCHAR *pwStr, LCID lcid, U32 dwF, DATE *pd )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Extracts a DATE from a string.
	//
	//	PARAMETERS
	//		-	pwStr is the string to convert
	//		-	lcid is not used
	//		-	dwF are flags
	//		-	pd will receive the date
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr = S_OK;
	#ifdef		__APPLE__
	struct tm	tm = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	time_t		t	= 0;
	int			i;
	WCHAR			*c;
	
	// Setup.  If nothing specified make sure it comes out as Jan 1, 1970
	(*pd) 		= 0.0;
	tm.tm_mday	= 1;										// Jan 1
	tm.tm_year	= 70;										// 1970
	
	// Parse optional date
	for (i = 0;hr == S_OK && pwStr[i] != WCHAR('\0');++i)
		if (pwStr[i] == WCHAR('/')) break;
	
	// If a 'slash' was found then assume date has been specified
	if (hr == S_OK && pwStr[i] != WCHAR('\0'))
		{
		// Month
		c = &(pwStr[i]);
		if ((c-2) >= pwStr)
			tm.tm_mon = (S32)wcstoul ( c-2, NULL, 10 );
		else if ((c-1) >= pwStr)
			tm.tm_mon = (S32)wcstoul ( c-1, NULL, 10 );
		else
			hr = E_INVALIDARG;
		if (hr == S_OK) tm.tm_mon--;					// 0-11
		
		// Day
		if (hr == S_OK) tm.tm_mday = (S32)wcstoul ( ++c, NULL, 10 );
		
		// Next '/' (required)
		for (;hr == S_OK && (*c) != WCHAR('\0');++c)
			if ((*c) == WCHAR('/')) break;
		if (hr == S_OK && (*c) == WCHAR('\0')) 	hr = E_INVALIDARG;
		if (hr == S_OK && *(c+1) == WCHAR('\0'))	hr = E_INVALIDARG;
		
		// Year
		if (hr == S_OK)
			{
			tm.tm_year = (S32)wcstoul ( c+1, NULL, 10 );
			if (tm.tm_year >= 1900) tm.tm_year -= 1900;
			}	// if
		}	// if
	
	// Parse optional time
	for (i = 0;hr == S_OK && pwStr[i] != WCHAR('\0');++i)
		if (pwStr[i] == WCHAR(':')) break;
	
	// If a 'colon' was found then assume time has been specified
	if (hr == S_OK && pwStr[i] != WCHAR('\0'))
		{
		// Hours
		c = &(pwStr[i]);
		if ((c-2) >= pwStr)
			tm.tm_hour = (S32)wcstoul ( c-2, NULL, 10 );
		else if ((c-1) >= pwStr)
			tm.tm_hour = (S32)wcstoul ( c-1, NULL, 10 );
		else
			hr = E_INVALIDARG;
		
		// Minutes
		if (hr == S_OK) tm.tm_min = (S32)wcstoul ( ++c, NULL, 10 );
		
		// Next ':' (required)
		for (;hr == S_OK && (*c) != WCHAR('\0');++c)
			if ((*c) == WCHAR(':')) break;
		if (hr == S_OK && (*c) == WCHAR('\0')) 	hr = E_INVALIDARG;
		if (hr == S_OK && *(c+1) == WCHAR('\0'))	hr = E_INVALIDARG;
		
		// Seconds
		if (hr == S_OK)	tm.tm_sec = (S32)wcstoul ( c+1, NULL, 10 );
		
		// Fractions of a second (after decimal point) (optional)
		for (;hr == S_OK && (*c) != WCHAR('\0');++c)
			if ((*c) == WCHAR('.')) break;
		if (hr == S_OK && (*c) == WCHAR('.'))
			{
			// TODO
			dbgprintf ( L"VarDateFromStr::Fractional seconds not implemented\n" );
			hr = E_NOTIMPL;
			}	// if
		
		}	// if
	
	// Convert # of seconds in Jan. 1, 1970
	if (hr == S_OK)
		hr = ((t = mktime ( &tm )) != -1) ? S_OK : E_INVALIDARG;
	
	// Convert to days.  Since 'time_t' is based in 1970 and the 'DATE' data type
	// is based at 1900, remove the 70 year offset (plus 1 day for the 'zero' century)
	//	if (hr == S_OK)	(*pd) = (t/SECINDAY) - ((70.0*365.25) + 1.0);
	if (hr == S_OK)	(*pd) = (t/SECINDAY);
	
	// Debug
	//	printf ( "VarDateFromStr:" );
	//	OutputDebugString (pwStr);
	//	printf ( ":0x%x : %d/%d/%d %d:%02d:%02d : %ld seconds(%lf)\n",
	//				hr, tm.tm_mon, tm.tm_mday, tm.tm_year,
	//				tm.tm_hour, tm.tm_min, tm.tm_sec, t, (*pd) );
	#else
	hr = E_NOTIMPL;
	#endif
	
	return hr;
	}	// VarDateFromStr

S32 WideCharToMultiByte	( U32 cp, S32 sFlags, const WCHAR *pwStr, S32 iLenW,
									char *pStr, S32 iLen, const char *pDef, BOOL *bDefUsed )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Maps a wide string to a character string
	//
	//	PARAMETERS
	//		-	cp is the code page to use
	//		-	sFlags are translation flags
	//		-	pwStr is the string to be converted
	//		-	iLenW is the length of the string
	//		-	pStr will receive the string
	//		-	iLen is the size in wide characters of the buffer
	//		-	pDef,bDefUsed are not used
	//
	//	RETURN VALUE
	//		Number of characters written/size needed
	//
	////////////////////////////////////////////////////////////////////////
	S32	lenReq = 0;
	
	// Calculate space required for new string
	lenReq = ((S32)wcslen(pwStr)+1);
	
	// Buffer provided ?
	if (iLen > 0 && lenReq > iLen)
		lenReq = 0;
	
	// Copy string
	if (lenReq > 0 && iLen > 0)
		{
		// Perform conversion
		wcstombs ( pStr, pwStr, lenReq );
		}
	
	return lenReq;
	}	// WideCharToMultiByte

