////////////////////////////////////////////////////////////////////////
//
//									CONNECTION.CPP
//
//				Implementation of the SQL connection node
//
////////////////////////////////////////////////////////////////////////

#define	INITGUID
#include "sqll_.h"
#include <stdio.h>

Connection :: Connection ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	hSQLEnv	= NULL;
	pConn		= NULL;
	}	// Connection

HRESULT Connection :: onAttach ( bool bAttach )
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
		adtValue		vL;

		// Defaults
		if (pnDesc->load ( adtString(L"Location"), vL ) == S_OK)
			adtValue::toString ( vL, strLoc );
		}	// if

	// Detach
	else
		{
		// Shutdown
		if (hSQLEnv != NULL)
			{
			SQLFreeHandle ( SQL_HANDLE_ENV, hSQLEnv );
			hSQLEnv = NULL;
			}	// if
		}	// else

	return hr;
	}	// onAttach

HRESULT Connection :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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

	// Connect
	if (_RCP(Connect))
		{
		SQLHandle	*pNew	= NULL;
		SQLRETURN	sqlr;
		SQLWCHAR		wOutStr[1024];

		// State check
		CCLTRYE ( strLoc.length() > 0, ERROR_INVALID_STATE );

		// Ensure valid Windows path
		CCLOK ( strLoc.replace ( '/', '\\' ); )

		// Allocate an environment handle for the connection
		if (hr == S_OK && hSQLEnv == NULL)
			{
			CCLTRYE ( (sqlr = SQLAllocHandle ( SQL_HANDLE_ENV, SQL_NULL_HANDLE, 
							&hSQLEnv )) == SQL_SUCCESS, sqlr );

			// Attributes/options
			CCLTRYE ( (sqlr = SQLSetEnvAttr ( hSQLEnv, SQL_ATTR_ODBC_VERSION,
							(SQLPOINTER) SQL_OV_ODBC3, 0 )) == SQL_SUCCESS, sqlr );
			}	// if

		// Allocate a handle for the connection
		CCLTRYE	( (pNew = new SQLHandle ( SQL_HANDLE_DBC, hSQLEnv ))
						!= NULL, E_OUTOFMEMORY );
		CCLTRY	( pNew->construct() );

		// Connection to database
		CCLOK		( dbgprintf ( L"%s\r\n", (PCWSTR)strLoc ); )
		CCLOK    ( sqlr = SQLDriverConnect ( pNew->Handle, GetDesktopWindow(), &strLoc.at(),
						SQL_NTS, wOutStr, 1024, NULL, 0 /*SQL_DRIVER_NOPROMPT*/ ); )
		CCLTRYE ( (sqlr == SQL_SUCCESS) || (sqlr == SQL_SUCCESS_WITH_INFO), sqlr );

		// DEBUG
		if (hr != S_OK)
			{
			SQLWCHAR	wMsg[256];
			SQLWCHAR	wState[256];
			SDWORD	native;
			SWORD		msglen;

			SQLError ( hSQLEnv, pNew->Handle, NULL, wState, &native, wMsg, 256, &msglen );
			lprintf ( LOG_DBG, L"Error : %s\r\n", wMsg );
			}	// if

		// Results
		if (hr == S_OK)
			_EMT(Connect,adtIUnknown((IHaveValue *)pNew));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pNew);
		}	// if

	// Disconnect
	else if (_RCP(Disconnect))
		{
		adtValue		vL;

		// State check
		CCLTRYE ( pConn != NULL, ERROR_INVALID_STATE );

		// Disconnect
		if (hr == S_OK && pConn->getValue ( vL ) == S_OK)
			SQLDisconnect ( (SQLHDBC)(U64)adtLong(vL) );

		// Clean up
		_RELEASE(pConn);
		}	// else if

	// State
	else if (_RCP(Connection))
		{
		_RELEASE(pConn);
		_QISAFE(adtIUnknown(v),IID_IHaveValue,&pConn);
		}	// else if
	else if (_RCP(Location))
		hr = adtValue::toString(v,strLoc);

	return hr;
	}	// receive

#if	0

// SQLite DLL
SQLiteDll	sqliteDll ( L"sqlite3.dll" );

HRESULT Connection :: onReceive ( IReceptor *pr, const ADTVALUE &v )
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
		SQLRef	*pConn	= NULL;

		// State check
		CCLTRYE ( strLoc.length() > 0, ERROR_INVALID_STATE );

		// Prepare a connection reference
		CCLTRYE ( (pConn = new SQLRef()) != NULL, E_OUTOFMEMORY );

		//
		// SQLite
		//
		if (hr == S_OK && bSqlite)
			{
			sqlite3	*pDB		= NULL;
			char		*paConn	= NULL;
			int		ret;

			// Open connection to database
//			CCLTRY ( strLoc.toAscii ( &paConn ) );
//			CCLTRYE ( (ret = sqliteDll.sqlite3_open ( paConn, &pDB ))
//							== SQLITE_OK, ret );
			CCLTRYE ( (ret = sqliteDll.sqlite3_open16 ( &strLoc.at(), &pDB ))
							== SQLITE_OK, ret );

			// Debug
			if (hr != S_OK || pDB == NULL)
				lprintf ( LOG_WARN, L"Unable to open database file : %s\r\n", 
								(LPCWSTR)strLoc );

			// Store in reference counted object
			CCLOK ( pConn->plite_db = pDB; )

			// Clean up
			_FREEMEM(paConn);
			}	// if

		//
		// ODBC
		//
		else if (hr == S_OK && bODBC)
			{
			// Port from old code
			hr = E_NOTIMPL;
			}	// else if

		// Result
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown(pConn));
		else
			_EMT(Error,adtInt(hr));

		// Clean up
		_RELEASE(pConn);

		/*
		SQLHandle	*pConn	= NULL;
		IHaveValue	*phv;
		SQLRETURN	sqlr;

		// State check
		if (hr == S_OK && strLoc.length() == 0)
			hr = pnDesc->load ( adtString ( L"Connection" ), strLoc );

		// Allocate an environment handle for the connection
		if (hr == S_OK && hSQLEnv == NULL)
			{
			CCLTRYE ( (sqlr = SQLAllocHandle ( SQL_HANDLE_ENV, SQL_NULL_HANDLE, 
							&hSQLEnv )) == SQL_SUCCESS, sqlr );

			// Attributes/options
			CCLTRYE ( (sqlr = SQLSetEnvAttr ( hSQLEnv, SQL_ATTR_ODBC_VERSION,
							(SQLPOINTER) SQL_OV_ODBC3, 0 )) == SQL_SUCCESS, sqlr );
			}	// if

		// Allocate a handle for the connection
		CCLTRYE	( (pConn = new SQLHandle ( SQL_HANDLE_DBC, hSQLEnv ))
						!= NULL, E_OUTOFMEMORY );
		CCLTRY	( pConn->construct() );

		// Connection to database
		CCLOK		( dbgprintf ( L"%s\r\n", (PCWSTR)strLoc ); )
		CCLTRYE ( (sqlr = SQLDriverConnect ( pConn->Handle, NULL, &strLoc.at(),
						SQL_NTS, NULL, 0, NULL, SQL_DRIVER_NOPROMPT )) == SQL_SUCCESS, sqlr );

		// Results
		CCLOK ( _EMT(Connect,adtIUnknown((phv = pConn))); ) 

		// Clean up
		_RELEASE(pConn);
		*/
		}	// if

	// State
	else if (_RCP(Location))
		adtValue::toString ( v, strLoc );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive
#endif

#ifdef	USE_OLEDB
HRESULT Connection :: receiveFire ( const adtValue &v )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Callback function for a receptor
	//
	//	PARAMETERS
	//		-	v is the value that was emitted
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr			= S_OK;
	IDBInitialize	*pInit	= NULL;
	IDBProperties	*pProp	= NULL;
	DBPROPSET		set[1];
	DBPROP			prop[1];
	CLSID				clsid;

	// State check
	if (hr == S_OK && strLoc.length() == 0)
		hr = pnAttr->load ( adtString ( L"Connection" ), strLoc );

	// For now default to Microsoft Access/Jet engine for default
	CCLTRY(CLSIDFromProgID ( L"Microsoft.Jet.OLEDB.4.0", &clsid ) );
	CCLTRY(COCREATEINSTANCE(clsid, IID_IDBInitialize, &pInit ));
	CCLTRY(_QI(pInit,IID_IDBProperties,&pProp));

	// Properties for connection
	if (hr == S_OK)
		{
		// Location of data source
		VariantInit ( &(prop[0].vValue) );
		prop[0].dwPropertyID	= DBPROP_INIT_DATASOURCE;
		prop[0].dwOptions		= DBPROPOPTIONS_REQUIRED;
		prop[0].vValue			= strLoc;

		// Property set
		set[0].guidPropertySet	= DBPROPSET_DBINIT;
		set[0].cProperties		= 1;
		set[0].rgProperties		= prop;

		// Set
		hr = pProp->SetProperties ( 1, set );
		}	// if

	// Initialize connection
	CCLTRY(pInit->Initialize());

	// Result
	CCLOK ( peConn->emit ( adtIUnknown(pInit) ); )

	// Clean up
	_RELEASE(pProp);
	_RELEASE(pInit);

	return hr;
	}	// receiveFire
#endif

