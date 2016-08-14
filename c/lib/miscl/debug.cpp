////////////////////////////////////////////////////////////////////////
//
//									DIST.CPP
//
//					Implementation of the distribution node
//
////////////////////////////////////////////////////////////////////////

#include "miscl_.h"
#include <stdio.h>

// Globals
static WCHAR	wDbgBfr[16384];
static WCHAR	wDbgBfr2[sizeof(wDbgBfr)/sizeof(WCHAR)];
static sysCS	csDebug;
#define	DBGSZ	sizeof(wDbgBfr)/sizeof(WCHAR)

Debug :: Debug ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pDctLog	= NULL;

	// Windows specific performance frequency
	#ifdef	_WIN32
	LARGE_INTEGER	li;
	QueryPerformanceFrequency ( &li );
	lFreq = li.QuadPart;
	#endif
	}	// Debug

void Debug :: appendDbg ( const WCHAR *pwStr )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Append a string to the debug output buffer.
	//
	//	PARAMETERS
	//		-	pwStr is the string to append
	//
	////////////////////////////////////////////////////////////////////////

	// Only append if there is room
	if (wcslen(wDbgBfr)+wcslen(pwStr)+1 < DBGSZ)
		WCSCAT ( wDbgBfr, sizeof(wDbgBfr)/sizeof(wDbgBfr[0]), pwStr );
	else
		dbgprintf ( L"WARNING:Debug buffer too small\r\n" );
	}	//	appendDbg

HRESULT Debug :: onAttach ( bool bAttach )
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

		// Default message ?
		if (pnDesc->load ( adtString(L"Message"), v ) == S_OK)
			strMsg = v;
		}	// if

	// Detach
	else
		{
		// Clean up
		_RELEASE(pDctLog);
		}	// else

	return hr;
	}	// onAttach

void Debug :: logCallback	( cLogEntry *e, void *p )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Call back function for logging.
	//
	//	PARAMETERS
	//		-	e is the log entry
	//		-	p is the callback parameters
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	Debug			*pThis	= (Debug *)p;
	adtDate		date;

	// Logging dictionary needed ?
	if (hr == S_OK && pThis->pDctLog == NULL)
		hr = COCREATE(L"Adt.Dictionary",IID_IDictionary,&(pThis->pDctLog));

	// Transfer information to dictionary
	CCLTRY ( pThis->pDctLog->clear() );
//	CCLTRY ( adtDate::fromSystemTime ( &(e->date), &(date.vdate) ) );
//	CCLTRY ( pThis->pDctLog->store ( adtString(L"Date"), date ) );
	CCLTRY ( pThis->pDctLog->store ( adtString(L"Function"), adtString((WCHAR *)(e->func+1)) ) );
	CCLTRY ( pThis->pDctLog->store ( adtString(L"File"), adtString((WCHAR *)(e->file+1)) ) );
	CCLTRY ( pThis->pDctLog->store ( adtString(L"Line"), adtInt(e->line) ) );
	CCLTRY ( pThis->pDctLog->store ( adtString(L"Level"), adtInt(e->level) ) );
	CCLTRY ( pThis->pDctLog->store ( adtString(L"Value"), adtString((WCHAR *)(e->str+1)) ) );

	// Emit entry
//	CCLOK ( pThis->peOnLog->emit(adtIUnknown(pThis->pDctLog) ); )
	}	// logCallback

HRESULT Debug :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Fire
	if (_RCP(Fire))
		{
		adtString	sValue;
		adtValue		vL,vDbg;

		// Thread safety
		if (!csDebug.enter())
			return E_UNEXPECTED;

		// Setup
		WCSCPY ( wDbgBfr, sizeof(wDbgBfr)/sizeof(wDbgBfr[0]), L"" );

		// For easier identification of source of debug, access the definition
		// of the container graph.
		if (strPath.length() == 0)
			{
			IDictionary	*pPar	= NULL;
			IDictionary	*pDsc	= NULL;
			adtIUnknown	unkV;

			// Path to this location
			nspcPathTo ( pnLoc, L"./", strPath );

//			if (!WCASECMP(strPath,L"/apps/auto/default/TreeDirect/Render/list/static/tree/.visual/Debug/"))
//				dbgprintf ( L"Hi\r\n" );

			// Reference location for hosting graph
			if (	pnLoc->load ( strnRefPar, vL ) == S_OK		&&
					(IUnknown *)(NULL) != (unkV=vL)				&&
					_QI(unkV,IID_IDictionary,&pPar) == S_OK	&&
//					pPar->load ( strnRefDesc, vL ) == S_OK		&&
//					(IUnknown *)(NULL) != (unkV=vL)				&&
//					_QI(unkV,IID_IDictionary,&pDsc) == S_OK	&&
//					pDsc->load ( strnRefLocn, vL ) == S_OK		&&
					pPar->load ( strnRefLocn, vL ) == S_OK		&&
					adtValue::toString ( vL, sValue ) == S_OK)
				{
				strPath.append ( L" (" );
				strPath.append ( sValue );
				strPath.append ( L")" );
				}	// if

			// Clean up
			_RELEASE(pDsc);
			_RELEASE(pPar);
			}	// if

		// Path to node
		appendDbg ( strPath );
		appendDbg ( L": " );

		// Message
		if (strMsg.length())
			{
			appendDbg ( strMsg );
			appendDbg ( L": " );
			}	// if

		// Value for debug
		if (hr == S_OK && v.vtype == (VTYPE_VALUE|VTYPE_BYREF) && v.pval != NULL)
			hr = adtValue::copy ( *(v.pval), vDbg );
		else
			hr = adtValue::copy ( v, vDbg );

		// Debug
//		if (!WCASECMP(strPath,L"/apps/auto/default/TestInst/Render/list/static/tree/Debug/ (render/gdi/dict/)"))
//			dbgprintf ( L"Hi\r\n" );

		// Value
		switch (adtValue::type(vDbg))
			{
			// Object ?
			case VTYPE_UNK :
				{
				IUnknown			*punk		= NULL;
				IDictionary		*pLoc		= NULL;
				IReceptor		*pRecep	= NULL;
				IByteStream		*pStm		= NULL;
				IDictionary		*pD		= NULL;
				IList				*pL		= NULL;

				// Unknown ptr.
				punk =	vDbg.punk;

				// Object type
				if (punk == NULL)
					appendDbg ( L"null," );
				else if (_QI(punk,IID_IDictionary,&pD) == S_OK)
					{
					IIt		*pKeys	= NULL;
					adtValue	vKey,vValue;
					swprintf ( SWPF(wDbgBfr2,DBGSZ), L"Dictionary(%p):", punk );
					appendDbg ( wDbgBfr2 );
					CCLTRY ( pD->keys ( &pKeys ) );
					while (pKeys->read ( vKey ) == S_OK)
						{
						// Key
						if (	adtValue::toString ( vKey, sValue ) == S_OK)
							{
							swprintf ( SWPF(wDbgBfr2,DBGSZ), L"%ls(%d)=", (LPCWSTR) sValue, vKey.vtype );
							appendDbg ( wDbgBfr2 );
							}	// if
						if (	pD->load ( vKey, vValue ) == S_OK &&
								adtValue::toString ( vValue, sValue ) == S_OK)
							{
							swprintf ( SWPF(wDbgBfr2,DBGSZ), L"%ls(%d),", (LPCWSTR) sValue, vValue.vtype );
							appendDbg ( wDbgBfr2 );
							}	// if

						// Next key
						pKeys->next();
						}	// while

					// Clean up
					_RELEASE(pKeys);
					}	// else if 
				else if (_QI(punk,IID_IList,&pL) == S_OK)
					{
					IIt		*pIt		= NULL;
					adtValue	vValue;

					// Print out values
					swprintf ( SWPF(wDbgBfr2,DBGSZ), L"Container(%p):", punk );
					appendDbg ( wDbgBfr2 );
					CCLTRY ( pL->iterate ( &pIt ); )
					while (hr == S_OK && pIt->read ( vValue ) == S_OK)
						{
						if ( adtValue::toString ( vValue, sValue ) == S_OK)
							{
							swprintf ( SWPF(wDbgBfr2,DBGSZ), L"%ls,", (LPCWSTR) sValue );
							appendDbg ( wDbgBfr2 );
							}	// if

						// Next key
						pIt->next();
						}	// while

					// Clean up
					_RELEASE(pIt);
					}	// else if
/*				else if (_QI(punk,IID_ILocation,&pLoc) == S_OK)
					{
					IDictionary	*pDesc	= NULL;
					IDictionary	*pEmit	= NULL;
					adtValue		v;
					adtIUnknown	unkV;
					adtString	strName,strBehave;

					// Optional descriptor
					CCLTRY ( pLoc->load ( adtString(STR_NSPC_DESC), v ) );
					CCLTRY ( _QISAFE((unkV=v),IID_IDictionary,&pDesc) );
					CCLTRY ( pDesc->load ( adtString(STR_NSPC_NAME), v ) );
					CCLOK  ( strName = v; )
					CCLTRY ( pDesc->load ( adtString(STR_NSPC_BEHAVE), v ) );
					CCLOK  ( strBehave = v; )

					// Debug
					swprintf ( SWPF(wDbgBfr2,DBGSZ), L"Location:%p", pLoc );
					appendDbg ( wDbgBfr2 );
					if (hr == S_OK)
						{
						swprintf ( SWPF(wDbgBfr2,DBGSZ), L":%ls:%ls", (LPCWSTR)strName, (LPCWSTR)strBehave );
						appendDbg ( wDbgBfr2 );
						}	// if

					// Clean up
					_RELEASE(pDesc);
					_RELEASE(pEmit);
					_RELEASE(pLoc);
					}	// else if
*/
				else if (_QI(punk,IID_IReceptor,&pRecep) == S_OK)
					{
					swprintf ( SWPF(wDbgBfr2,DBGSZ), L"Receptor:%p", punk );
					appendDbg ( wDbgBfr2 );
					}	// else if
				else if (_QI(punk,IID_IByteStream,&pStm) == S_OK)
					{
					U64 pos = 0,len = 0;

					// Current position and length
					CCLTRY ( pStm->seek ( 0, STREAM_SEEK_CUR, &pos ) );
					CCLTRY ( pStm->available ( &len ) );
					swprintf ( SWPF(wDbgBfr2,DBGSZ), L"Stream:Position %d/Length %d", (S32)pos, (S32)(pos+len) );
					appendDbg ( wDbgBfr2 );
					}	// else if
				else
					{
					swprintf ( SWPF(wDbgBfr2,DBGSZ), L"Object (%p)", punk );
					appendDbg ( wDbgBfr2 );
					}	// else

				// Clean up
				_RELEASE(pL);
				_RELEASE(pD);
				_RELEASE(pStm);
				_RELEASE(pRecep);
				_RELEASE(pLoc);
				}	// VT_UNKNOWN
				break;

			// Other
			default :
				adtValue::toString ( vDbg, sValue );
				swprintf ( SWPF(wDbgBfr2,DBGSZ), L"%ls (%d)", (LPCWSTR)sValue, vDbg.vtype );
				appendDbg ( wDbgBfr2 );
			}	// switch

		// Done
//		lprintf ( LOG_DBG, wDbgBfr );
		dbgprintf ( L"%s\r\n", wDbgBfr );
		csDebug.leave();
		}	// if
/*
	// Logging on/off
	else if (_RCP(Log))
		{
		adtBool	bLog(v);

		// Enable/disable logging sink
		// TODO: Multiple callbacks
		logSink ( (bLog == true) ? logCallback : NULL, this );
		}	// else if
*/
	// Debug break
	else if (_RCP(Break))
		{
		dbgprintf ( L"MiscDebug::receive:Break @ %s\r\n", (LPCWSTR)strnName );
		#ifdef	_DEBUG
		DebugBreak();
		#endif
		dbgprintf ( L"MiscDebug::receive:Break @ %s\r\n", (LPCWSTR)strnName );
		}	// else if


	// Timing
	else if (_RCP(Reset))
		{
		#ifdef	_WIN32
		LARGE_INTEGER lCnt;
		QueryPerformanceCounter ( &lCnt );
		lRst = lCnt.QuadPart;
		#endif
//		dbgprintf ( L"MiscDebug::%s:%d\r\n", (LPCWSTR)strMsg, dwT0 );
		}	// else if
	else if (_RCP(Mark))
		{
		#ifdef	_WIN32
		LARGE_INTEGER lCnt;
		QueryPerformanceCounter ( &lCnt );

		// Compute difference
		double
		dt = ((lCnt.QuadPart-lRst) * 1.0) / lFreq;
		dbgprintf ( L"MiscDebug:%s:%s:%g s\r\n", (LPCWSTR)strMsg, (LPCWSTR) strnName, dt );//, dwT1, dwT0 );
		#endif
		}	// else if
	else if (_RCP(Sleep))
		{
		adtInt	iMs(v);
		#ifdef	_WIN32
		Sleep(iMs);
		#else
		usleep(iMs*1000);
		#endif
		}	// else if
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

