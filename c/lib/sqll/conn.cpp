////////////////////////////////////////////////////////////////////////
//
//									CONN.CPP
//
//				Implementation of the SQL connection node
//
////////////////////////////////////////////////////////////////////////

/*
   Copyright (c) 2003, nSpace, LLC
   All right reserved

   Redistribution and use in source and binary forms, with or without 
   modification, are permitted provided that the following conditions
   are met:

      - Redistributions of source code must retain the above copyright 
        notice, this list of conditions and the following disclaimer.
      - nSpace, LLC as the copyright holder reserves the right to 
        maintain, update, and provide future releases of the source code.
      - The copyrights to all improvements and modifications to this 
        distribution shall reside in nSpace, LLC.
      - Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in 
        the documentation and/or other materials provided with the 
        distribution.
      - Neither the name of nSpace, LLC nor the names of its contributors 
        may be used to endorse or promote products derived from this 
        software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
   A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT 
   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSBILITY OF SUCH DAMAGE.
*/

#include "sqln_.h"
#include <stdio.h>

SQLConnection :: SQLConnection ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	#ifdef	USE_ODBC
	hSQLEnv	= NULL;
	#endif
	}	// SQLConnection

void SQLConnection :: destruct ( void )
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

#ifdef	USE_ODBC

HRESULT SQLConnection :: receive ( IReceptor *pr, const WCHAR *pl, const ADTVALUE &v )
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
HRESULT SQLConnection :: receiveFire ( const adtValue &v )
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

