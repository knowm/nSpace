////////////////////////////////////////////////////////////////////////
//
//										DATE.CPP
//
//					Implementation of the 'date' class
//
////////////////////////////////////////////////////////////////////////

#include "adtl.h"
#if	__APPLE__ || __unix__
#include <sys/time.h>
#endif
#include <math.h>
#include <stdio.h>

#define	SECINDAY		(60.0*60.0*24.0)				// # of seconds in a day
#define	MSECINDAY	(1000.0*SECINDAY)				// # of ms in a day
#define	SEC20011970	978307200.0						// Seconds since Jan 1, 1970 to Jan 1, 2001

// This is a bit ugly.  Apparently ever since the first version of Excel the VARIANT
// date has considered 1900 a leap year (incorrectly).  The offset from
//	'time_t' to VARIANT is 70 years.  So add 70 years with no leap years then add the
// leap 'days' (plus one day for Dec 31, 1899)
#define	SEC19701899		((70.0*365.0*86400)+((18.0+1.0)*86400.0))

adtDate :: adtDate ( DATE _date )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Default constructor for the object.
	//
	////////////////////////////////////////////////////////////////////////
	vtype		= VTYPE_DATE;
	vdate		= _date;
	}	// adtDate

adtDate :: adtDate ( const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the object.
	//
	//	PARAMETERS
	//		-	v is the value to initialize with
	//
	////////////////////////////////////////////////////////////////////////
	vtype	= VTYPE_DATE;
	vdate	= 0;
	*this = v;
	}	// adtDate

HRESULT adtDate :: fromEpochSeconds ( S32 s, S32 us, DATE *d )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Converts the specified # of seconds since Jan 1, 1970 to the
	//			date format.
	//
	//	PARAMETERS
	//		-	s is the number of seconds since Jan 1, 1970
	//		-	us is the number of microseconds
	//		-	d will receive the date
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////

	// Convert to variant date
	*d	=	(((double)(s))/SECINDAY) +
			(((double)(us)/1000000.0)/SECINDAY)+
			(SEC19701899/SECINDAY);

	// Convert to internal reference
	*d = varToRef(*d);
	
	return S_OK;
	}	// fromSystemTime

#ifdef	_WIN32
HRESULT adtDate :: fromSystemTime ( SYSTEMTIME *st, DATE *date )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Converts the SYSTEM time structure to a variant date.	We cannot use
	//			Variant functions only because they ignore milliseconds.
	//
	//	PARAMETERS
	//		-	st is the system time
	//		-	date will receive the date
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;

	// Convert main fields
	if (hr == S_OK)
		hr = (SystemTimeToVariantTime ( st, date ) != 0) ? S_OK : E_UNEXPECTED;

	// Add in milliseconds
	if (hr == S_OK) *date += (((double)(st->wMilliseconds))/(1000.0*SECINDAY));

	// Convert to internal reference
	*date = varToRef(*date);

	return hr;
	}	// fromSystemTime

HRESULT adtDate :: toSystemTime ( double date, SYSTEMTIME *st )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Converts the date to a SYSTEM time structure.  We cannot use
	//			Variant functions only because they ignore milliseconds.
	//
	//	PARAMETERS
	//		-	date is the date to use
	//		-	st will receive the system time
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr = S_OK;
	U32		uTmp;
	DATE		dc;

	// Just use the conversion routine to get the date.  We will perform the
	// time conversion manually to get the full millisecond resolution.
	// Conversion routine rounds the # of seconds.

	// Convert from internal reference
	dc = refToVar(date);

	// Whole # of days
	uTmp = (U32)floor(dc);

	// Get date
	if (hr == S_OK)
		hr = (VariantTimeToSystemTime ( uTmp, st ) == TRUE) ? S_OK : S_FALSE;

	// Calculate the time
	if (hr == S_OK)
		{
		// # of ms into day
		uTmp = (U32) ceil((dc - uTmp)*MSECINDAY);

		// Hours
		st->wHour	=	(WORD) (uTmp/3600000);
		uTmp			-=	(st->wHour*3600000);
 
		// Minutes
		st->wMinute = (WORD)(uTmp/60000);
		uTmp			-=	(st->wMinute*60000);

		// Seconds
		st->wSecond	= (WORD)(uTmp/1000);
		uTmp			-=	(st->wSecond*1000);

		// Milliseconds
		st->wMilliseconds	= (WORD)(uTmp);
		}	// if

	return hr;
	}	// toSystemTime

#endif

HRESULT adtDate :: fromString ( const WCHAR *s, ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Converts the string to its value.
	//
	//	PARAMETERS
	//		-	s is the string
	//		-  v will receive the date value 
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr			= S_OK;
	adtString	str(s);
	int			i;

	// Support loading doubles as dates as well.  We will use the colon (:) or slash (/)
	// to determine if the format is a date or a double
	for (i = 0;str.at(i) != WCHAR('\0');++i)
		if (	str.at(i) == WCHAR(':') ||
				str.at(i) == WCHAR('/') ) break;

	// Convert
	v.vtype	= VTYPE_DATE;
	v.vdate	= 0;
	if (str.at(i) != WCHAR('\0'))
		{
		// We support fractions of a second, windows does not, remove before converting.
		bool bfract = false;
		for (i = 0;str.at(i) != WCHAR('\0');++i)
			if (str.at(i) == WCHAR('.'))
				{
				str.at(i)	= WCHAR('\0');
				bfract		= true;
				break;
				}	// if

		// Convert whole second string
		if (hr == S_OK)
			hr = VarDateFromStr ( &str.at(0), LOCALE_USER_DEFAULT, 0, &(v.vdate) );

		// Add fractions of a second
		if (hr == S_OK && bfract == true)
			{
			// To integer
			i = (S32)wcstoul ( &(str[i+1]), NULL, 10 );

			// To days
			v.vdate += (i/1000.0)/SECINDAY;
			}	// if

		// Convert to internal reference
		v.vdate = varToRef(v.vdate);
		}	// if
	else
		SWSCANF ( str, L"%lg", &(v.vdate) );

	return hr;
	}	// fromString

adtDate &adtDate :: now ( bool bLocal )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Initialize object with current timestamp.
	//
	//	PARAMETERS
	//		-	bLocal to get the local time, false for GMT
	//
	//	RETURN VALUE
	//		Reference to the value
	//
	////////////////////////////////////////////////////////////////////////
	#ifdef _WIN32
	SYSTEMTIME	st;

	// Current time
	if (bLocal == false) GetSystemTime ( &st );
	else						GetLocalTime ( &st );

	// Convert the system time into a 'variant'/double time
	SystemTimeToVariantTime ( &st, &vdate );

	// SystemTimeToVariantTime ignores millisconds, add them ourselves.
	// VALT_DATE is # of days since Jan 1, 1900.
	vdate += (((double)(st.wMilliseconds))/(1000.0*SECINDAY));

	#elif __APPLE__ || __unix__
	struct timeval tv;
	
	// Current time
	gettimeofday ( &tv, NULL );
	
	// Variant date
	vdate =	(((double)(tv.tv_sec))/SECINDAY) +
				(((double)(tv.tv_usec)/1000000.0)/SECINDAY) +
				(SEC19701899/SECINDAY);
	#endif
	
	// Convert to internal reference
	vdate = varToRef(vdate);

	return *this;
	}	// now

DATE adtDate :: refToVar ( DATE date )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Convert a reference date to a 'variant' date.
	//
	//	PARAMETERS
	//		-	date is the date to convert
	//
	//	RETURN VALUE
	//		Corrected date
	//
	////////////////////////////////////////////////////////////////////////

	// From internal reference date of Jan 1, 2001.
	date += (SEC20011970/SECINDAY);

	// From mumber of days to Jan 1, 1970.
	date += (SEC19701899/SECINDAY);

	return date;
	}	// refToVar

HRESULT adtDate :: toString ( const ADTVALUE &v, adtString &s )
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

	// Date ?
	if (v.vtype != VTYPE_DATE)
		return E_INVALIDARG;

	#if	_WIN32
	// Format is DD/MM/YYYY HH:MM:SS.FFF
	#define		DATESTRLEN		23
	SYSTEMTIME	st;

	// VarBstrFromDate is not flexible for us due to needing fractions
	// of a second.
	if (hr == S_OK) hr = s.allocate ( DATESTRLEN );
	if (hr == S_OK) hr = adtDate::toSystemTime ( v.vdate, &st );
	if (hr == S_OK)
		{
		swprintf ( &s.at(0), 23,
						L"%d/%d/%d %d:%02d:%02d.%03d",
						st.wMonth, st.wDay, st.wYear,
						st.wHour, st.wMinute, st.wSecond,
						st.wMilliseconds );
		}	// if
	#else
	// TODO
	WCHAR	wNumStr[31];
	swprintf ( SWPF(wNumStr,20), L"%lg", v.vdate );
	s = wNumStr;
	#endif

	return hr;
	}	// toString

DATE adtDate :: varToRef ( DATE date )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Convert a 'variant' date to the internal reference.
	//
	//	PARAMETERS
	//		-	date is the date to convert
	//
	//	RETURN VALUE
	//		Corrected date
	//
	////////////////////////////////////////////////////////////////////////

	// Convert to number of days to Jan 1, 1970.
	date -= (SEC19701899/SECINDAY);

	// Convert to internal reference date of Jan 1, 2001.
	date -= (SEC20011970/SECINDAY);

	return date;
	}	// varToRef

//
// Operators
//

adtDate& adtDate::operator= ( const ADTVALUE &v )
	{
	adtValue::clear(*this);
	vtype	= VTYPE_DATE;
	if			(v.vtype == VTYPE_DATE)					vdate	= v.vdate;
	else if	(adtValue::type(v) == VTYPE_STR)		fromString ( v.pstr, *this );
	else if	(	v.vtype == (VTYPE_VALUE|VTYPE_BYREF) &&
					v.pval != NULL)						*this = *(v.pval);
	else														vdate	= 0;
	return *this;
	}	// operator=

//
// For non-Windows systems
//

#ifndef	_WIN32

HRESULT	VarDateFromStr ( WCHAR *pwStr, LCID lcid, ULONG dwF, DATE *pd )
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
	#ifdef	__LINUX__
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
			tm.tm_mon = wcstoul ( c-2, NULL, 10 );
		else if ((c-1) >= pwStr)
			tm.tm_mon = wcstoul ( c-1, NULL, 10 );
		else
			hr = E_INVALIDARG;
		if (hr == S_OK) tm.tm_mon--;					// 0-11

		// Day
		if (hr == S_OK) tm.tm_mday = wcstoul ( ++c, NULL, 10 );

		// Next '/' (required)
		for (;hr == S_OK && (*c) != WCHAR('\0');++c)
			if ((*c) == WCHAR('/')) break;
		if (hr == S_OK && (*c) == WCHAR('\0')) 	hr = E_INVALIDARG;
		if (hr == S_OK && *(c+1) == WCHAR('\0'))	hr = E_INVALIDARG;

		// Year
		if (hr == S_OK)
			{
			tm.tm_year = wcstoul ( c+1, NULL, 10 );
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
			tm.tm_hour = wcstoul ( c-2, NULL, 10 );
		else if ((c-1) >= pwStr)
			tm.tm_hour = wcstoul ( c-1, NULL, 10 );
		else
			hr = E_INVALIDARG;

		// Minutes
		if (hr == S_OK) tm.tm_min = wcstoul ( ++c, NULL, 10 );

		// Next ':' (required)
		for (;hr == S_OK && (*c) != WCHAR('\0');++c)
			if ((*c) == WCHAR(':')) break;
		if (hr == S_OK && (*c) == WCHAR('\0')) 	hr = E_INVALIDARG;
		if (hr == S_OK && *(c+1) == WCHAR('\0'))	hr = E_INVALIDARG;

		// Seconds
		if (hr == S_OK)	tm.tm_sec = wcstoul ( c+1, NULL, 10 );

		// Fractions of a second (after decimal point) (optional)
		for (;hr == S_OK && (*c) != WCHAR('\0');++c)
			if ((*c) == WCHAR('.')) break;
		if (hr == S_OK && (*c) == WCHAR('.'))
			{
			// TODO
			OutputDebugString ( L"VarDateFromStr::Fractional seconds not implemented\n" );
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

#endif
