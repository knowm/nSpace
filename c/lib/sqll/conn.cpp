////////////////////////////////////////////////////////////////////////
//
//									CONN.CPP
//
//				Implementation of the SQL connection node
//
////////////////////////////////////////////////////////////////////////

/*
   Copyright (c) nSpace, LLC
   All right reserved
*/
#define	INITGUID
#include "sqll_.h"
#include <stdio.h>

// SQLite DLL
SQLiteDll	sqliteDll ( L"sqlite3.dll" );

Connection :: Connection ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	bSqlite	= true;
	bODBC		= false;

	#ifdef	USE_ODBC
	hSQLEnv	= NULL;
	#endif
	}	// Connection

void Connection :: destruct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		CCLObject
	//
	//	PURPOSE
	//		-	Called when the object is being destroyed
	//
	////////////////////////////////////////////////////////////////////////
	#ifdef	USE_ODBC
	if (hSQLEnv != NULL)
		{
		SQLFreeHandle ( SQL_HANDLE_ENV, hSQLEnv );
		hSQLEnv = NULL;
		}	// if
	#endif
	}	// destruct

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
			adtValue::toString ( vL, strConn );
		}	// if

	// Detach
	else
		{
		// Shutdown
		}	// else

	return hr;
	}	// onAttach

HRESULT Connection :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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
		CCLTRYE ( strConn.length() > 0, ERROR_INVALID_STATE );

		//
		// SQLite
		//
		if (hr == S_OK && bSqlite)
			{
			sqlite3	*pDB		= NULL;
			char		*paConn	= NULL;
			int		ret;

			// Open connection to database
			CCLTRY ( strConn.toAscii ( &paConn ) );
			CCLTRYE ( (ret = sqliteDll.sqlite3_open ( paConn, &pDB ))
							== SQLITE_OK, ret );

			// Debug
			if (hr != S_OK || pDB == NULL)
				lprintf ( LOG_WARN, L"Unable to open database file : %s\r\n", 
								(LPCWSTR)strConn );

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

		/*
		SQLHandle	*pConn	= NULL;
		IHaveValue	*phv;
		SQLRETURN	sqlr;

		// State check
		if (hr == S_OK && sConn.length() == 0)
			hr = pnDesc->load ( adtString ( L"Connection" ), sConn );

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
		CCLOK		( dbgprintf ( L"%s\r\n", (PCWSTR)sConn ); )
		CCLTRYE ( (sqlr = SQLDriverConnect ( pConn->Handle, NULL, &sConn.at(),
						SQL_NTS, NULL, 0, NULL, SQL_DRIVER_NOPROMPT )) == SQL_SUCCESS, sqlr );

		// Results
		CCLOK ( _EMT(Connect,adtIUnknown((phv = pConn))); ) 

		// Clean up
		_RELEASE(pConn);
		*/
		}	// if

	// State
	else if (_RCP(Location))
		adtValue::toString ( v, strConn );
	else
		hr = ERROR_NO_MATCH;

	return hr;
	}	// receive

#ifdef	USE_ODBC

HRESULT Connection :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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
		SQLHandle	*pConn	= NULL;
		IHaveValue	*phv;
		SQLRETURN	sqlr;

		// State check
		if (hr == S_OK && sConn.length() == 0)
			hr = pnDesc->load ( adtString ( L"Connection" ), sConn );

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
		CCLOK		( dbgprintf ( L"%s\r\n", (PCWSTR)sConn ); )
		CCLTRYE ( (sqlr = SQLDriverConnect ( pConn->Handle, NULL, &sConn.at(),
						SQL_NTS, NULL, 0, NULL, SQL_DRIVER_NOPROMPT )) == SQL_SUCCESS, sqlr );

		// Results
		CCLOK ( _EMT(Connect,adtIUnknown((phv = pConn))); ) 

		// Clean up
		_RELEASE(pConn);
		}	// if

	// State
	else if (_RCP(Connection))
		sConn = (LPCWSTR)adtString(v);

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
	if (hr == S_OK && sConn.length() == 0)
		hr = pnAttr->load ( adtString ( L"Connection" ), sConn );

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
		prop[0].vValue			= sConn;

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

