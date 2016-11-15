////////////////////////////////////////////////////////////////////////
//
//									TIME.CPP
//
//					Implementation of the time node
//
////////////////////////////////////////////////////////////////////////

#include "miscl_.h"
#include <math.h>

#define	SECINDAY			(60.0*60.0*24.0)					// # of seconds in a day

TimeOp :: TimeOp ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	bLocal = false;
	}	// Time

HRESULT TimeOp :: onAttach ( bool bAttach )
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
		adtValue		v;

		// Defaults
		if (pnDesc->load ( adtString(L"Local"), v ) == S_OK)
			bLocal = adtBool(v);
		}	// if

	return hr;
	}	// onAttach

HRESULT TimeOp :: onReceive ( IReceptor *pr, const ADTVALUE &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	FROM	IBehaviour
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

	// Current time
	if (_RCP(Now))
		{
		adtDate		dNow;

		// Set date to now
		CCLOK ( dNow.now(bLocal); )

		// Result
		CCLOK ( _EMT(Now,dNow); )
		}	// if

	// Break time
	else if (_RCP(Break))
		{
		#ifdef		_WIN32
		adtDate		date(v);
		SYSTEMTIME	st;
		U32			calendar;

		// Convert the current date to its pieces
		CCLTRY ( adtDate::toSystemTime ( date, &st ) );

		// Emit result
		CCLOK ( _EMT(Year,adtInt(st.wYear ) ); )
		CCLOK ( _EMT(Month,adtInt(st.wMonth ) ); )
		CCLOK ( _EMT(Day,adtInt(st.wDay ) ); )
		CCLOK ( _EMT(Hour,adtInt(st.wHour ) ); )
		CCLOK ( _EMT(Minute,adtInt(st.wMinute ) ); )
		CCLOK ( _EMT(Second,adtInt(st.wSecond ) ); )
		CCLOK ( _EMT(Millisecond,adtInt(st.wMilliseconds) ); )

		// Emit date (calendar) portion and time portion of full date
		CCLOK ( calendar = (U32) floor(date); )
		CCLOK ( _EMT(Date, adtDate ( calendar ) ); )
		CCLOK ( _EMT(Time, adtDate ( (date.vdate - calendar) ) ); )

		// Done
		CCLOK ( _EMT(Break,adtInt() ); )
		#endif
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

