////////////////////////////////////////////////////////////////////////
//
//									TABLE.CPP
//
//					Implementation of the SQL table node
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

#define	SIZE_SQLBFR			1024
#define	STRING_PRIMKEY		L"_Auto"				// Default primary key

// Globals

#ifdef	USE_ODBC

SQL2Table :: SQL2Table ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pFlds			= NULL;
	pConn			= NULL;
	hConn			= NULL;
	pQryBfr		= NULL;
	pwQryBfr		= NULL;
	bRemove		= false;
	}	// SQL2Table

HRESULT SQL2Table :: construct ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	OVERLOAD
	//	FROM		CCLObject
	//
	//	PURPOSE
	//		-	Called when the object is being created.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;

	// SQL query string buffer
	CCLTRY ( COCREATE ( L"Io.MemoryBlock", IID_IMemoryMapped, &pQryBfr ) );
	CCLTRY ( pQryBfr->setSize ( SIZE_SQLBFR ) );
	CCLTRY ( pQryBfr->lock ( 0, 0, (PVOID *) &pwQryBfr, NULL ) );

	return hr;
	}	// construct

void SQL2Table :: destruct ( void )
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
	_RELEASE(pFlds);
	_RELEASE(pConn);
	_UNLOCK(pQryBfr,pwQryBfr);
	_RELEASE(pQryBfr);
	}	// destruct

HRESULT SQL2Table :: fieldsAdd ( SQLHANDLE hConn, adtString &sName,
											IDictionary *pFlds )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Adds fields to a table.
	//
	//	PARAMETERS
	//		-	hConn specifies the connection
	//		-	sName specifies the table name
	//		-	pFlds are the list of fields that should be in the table
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	SQLHANDLE	hStmt		= NULL;
	IIt			*pKeys	= NULL;
	U32			uLen		= 0;
	adtString	sKey;
	adtValue		vValue;
	WCHAR			wType[31];

	// Verify existence of auto-increment/default primary key
	// Indexes are used for queries.  Good practice to not use table fields
	// in primary key.
	CCLTRY ( SQLSTMT(hStmt,(SQLAllocHandle ( SQL_HANDLE_STMT, hConn, &hStmt ))) );
	CCLOK ( swprintf ( SWPF(pwQryBfr,SIZE_SQLBFR), 
								L"ALTER TABLE \"%s\" ADD \"%s\" AUTOINCREMENT",
								(LPCWSTR)(sName), STRING_PRIMKEY ); )
	CCLOK  ( SQLSTMT(hStmt,(SQLExecDirect ( hStmt, pwQryBfr, (SQLINTEGER)wcslen(pwQryBfr) )));)
	SQLFREESTMT(hStmt);

	CCLTRY ( SQLSTMT(hStmt,(SQLAllocHandle ( SQL_HANDLE_STMT, hConn, &hStmt ))) );
	CCLOK ( swprintf ( pwQryBfr, SIZE_SQLBFR,
								L"ALTER TABLE \"%s\" ADD PRIMARY KEY (\"%s\")",
								(LPCWSTR)(sName), STRING_PRIMKEY ); )
	CCLOK  ( SQLSTMT(hStmt,(SQLExecDirect ( hStmt, pwQryBfr, (SQLINTEGER)wcslen(pwQryBfr) )));)
	SQLFREESTMT(hStmt);

	// Add each field
	CCLTRY ( pFlds->keys ( &pKeys ) );
	while (hr == S_OK && pKeys->read ( sKey ) == S_OK)
		{
		// Load the value from the dictionary.  This will be used as the
		// default and to get the data type.
		CCLTRY ( pFlds->load ( sKey, vValue ) );

		// Convert data type
		if (hr == S_OK)
			{
			switch(vValue.vtype)
				{
				case VTYPE_I4		:	WCSCPY ( wType, 20, L"int" );			break;
				case VTYPE_I8		:	WCSCPY ( wType, 20, L"bigint" );		break;
				case VTYPE_R4		:	WCSCPY ( wType, 20, L"real" );			break;
				case VTYPE_R8		:	WCSCPY ( wType, 20, L"double" );		break;
				case VTYPE_STR		:	WCSCPY ( wType, 20, L"text" );			break;
				case VTYPE_DATE	:	WCSCPY ( wType, 20, L"datetime" );	break;
				case VTYPE_UNK		:	WCSCPY ( wType, 20, L"image" );		break;

				// Unhandled
				default :
					hr = E_NOTIMPL;
				}	// switch
			}	// if

		// Prepare command
		CCLTRY ( SQLSTMT(hStmt,(SQLAllocHandle ( SQL_HANDLE_STMT, hConn, &hStmt ))) );
		CCLOK ( swprintf ( SWPF( pwQryBfr,SIZE_SQLBFR), L"ALTER TABLE \"%s\" ADD \"%s\" %s",
									(LPCWSTR)(sName), (LPCWSTR)(sKey), wType ); )

		// Execute.
		CCLOK	 ( OutputDebugString ( pwQryBfr ); )
		CCLOK	 ( OutputDebugString ( L"\n" ); )
		CCLOK  ( SQLSTMT(hStmt,(SQLExecDirect ( hStmt, pwQryBfr, 
						(SQLINTEGER)wcslen(pwQryBfr) )));)

		// Clean up
		SQLFREESTMT(hStmt);
		pKeys->next();
		}	// while

	// Clean up
	_RELEASE(pKeys);

	return hr;
	}	// fieldsAdd

HRESULT SQL2Table :: fieldsRemove ( SQLHANDLE hConn, adtString &sName,
												IDictionary *pFlds )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Removes unused fields from a table.
	//
	//	PARAMETERS
	//		-	hConn specifies the connection
	//		-	sName specifies the table name
	//		-	pFlds are the list of fields that should be in the table
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr				= S_OK;
	SQLHANDLE	hStmtCols	= NULL;
	SQLHANDLE	hStmtAlter	= NULL;
	SQLLEN		sLen;
	adtString	sFieldName;
	adtValue		vValue;

	// We accomplish removal by describing the table and removing any fields
	// that do not show up in the given list

	// Table columns
	CCLTRY ( SQLSTMT(hStmtCols,(SQLAllocHandle ( SQL_HANDLE_STMT, hConn, &hStmtCols ))) );
	CCLTRY ( SQLSTMT(hStmtCols,(SQLColumns ( hStmtCols, NULL, SQL_NTS, NULL, SQL_NTS,
												&sName.at(), sName.length(), NULL, SQL_NTS ))) );

	// Allocate memory for and bind table name
	CCLOK  ( sLen = 0; )
	CCLTRY ( sFieldName.allocate ( 255 ) );
	CCLTRY ( SQLSTMT(hStmtCols,(SQLBindCol ( hStmtCols, 4, SQL_C_WCHAR, 
							&sFieldName.at(), 255, &sLen ))) );

	// Iterate fields
	while (hr == S_OK && SQLFetch ( hStmtCols ) == SQL_SUCCESS)
		{
		// Does this field exist in the definition ?
		if (	pFlds->load ( sFieldName, vValue ) != S_OK &&
				WCASECMP(STRING_PRIMKEY,sFieldName))
			{
			// Drop field
			CCLTRY ( SQLSTMT(hStmtAlter,(SQLAllocHandle ( SQL_HANDLE_STMT, hConn, &hStmtAlter ))) );
			CCLOK ( swprintf ( SWPF(pwQryBfr,SIZE_SQLBFR), 
									L"ALTER TABLE \"%s\" DROP \"%s\"",
									(LPCWSTR)(sName), (LPCWSTR)(sFieldName) ); )
			CCLOK	 ( OutputDebugString ( pwQryBfr ); )
			CCLOK	 ( OutputDebugString ( L"\n" ); )
			CCLOK  ( SQLSTMT(hStmtAlter,(SQLExecDirect ( hStmtAlter, pwQryBfr, 
							(SQLINTEGER)wcslen(pwQryBfr) )));)
			}	// if

		// Clean up
		SQLFREESTMT(hStmtAlter);
		}	// while

	// Clean up
	SQLFREESTMT(hStmtCols);

	return hr;
	}	// fieldsRemove

HRESULT SQL2Table :: receive ( IReceptor *pr, const WCHAR *pl, 
											const ADTVALUE &v )
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
		HRESULT			hr			= S_OK;
		SQLHandle		*pStmt	= NULL;

		// State check
		CCLTRYE ( (hConn != NULL),		ERROR_INVALID_STATE );
		if (hr == S_OK && sTableName.length() == 0)
			hr = pnDesc->load ( strRefTableName, sTableName );
		CCLTRYE ( (sTableName.length() > 0),	ERROR_INVALID_STATE );
		CCLOK	  ( pnDesc->load ( strRefRemFlds, bRemove ); )

		// Our statement handle
		CCLTRYE	( (pStmt = new SQLHandle ( SQL_HANDLE_STMT, hConn ))
						!= NULL, E_OUTOFMEMORY );
		CCLTRY	( pStmt->construct() );

		// Make sure table exists.  Ok if fails, might already exist
		if (hr == S_OK)
			{
			swprintf ( SWPF(pwQryBfr,SIZE_SQLBFR), L"CREATE TABLE \"%s\"", (LPCWSTR) sTableName );
			SQLExecDirect ( pStmt->Handle, pwQryBfr, (SQLINTEGER)wcslen(pwQryBfr) );
			}	// if

		// Add specified fields
		if (hr == S_OK && pFlds != NULL)
			hr = fieldsAdd ( hConn, sTableName, pFlds );

		// Remove fields that are no longer specified in definition
		if (hr == S_OK && pFlds != NULL && bRemove == true)
			hr = fieldsRemove ( hConn, sTableName, pFlds );

		// Result
		CCLOK ( _EMT(Fire,sTableName); )

		// Clean up
		_RELEASE(pStmt);
		}	// if

	// Columns
	else if (_RCP(Columns))
		{
		HRESULT		hr			= S_OK;
		SQLHandle	*pStmt	= NULL;
		IHaveValue	*phv;

		// State check
		CCLTRYE ( (hConn != NULL),		ERROR_INVALID_STATE );
		if (hr == S_OK && sTableName.length() == 0)
			hr = pnDesc->load ( strRefTableName, sTableName );
		CCLTRYE ( (sTableName.length() > 0),	ERROR_INVALID_STATE );

		// Our statement handle
		CCLTRYE	( (pStmt = new SQLHandle ( SQL_HANDLE_STMT, hConn ))
						!= NULL, E_OUTOFMEMORY );
		CCLTRY	( pStmt->construct() );

		// Table columns
		CCLTRY ( SQLSTMT(pStmt->Handle,(SQLColumns ( pStmt->Handle, NULL, SQL_NTS, NULL, SQL_NTS,
													&sTableName.at(), sTableName.length(), NULL, SQL_NTS ))) );

		// Result
		CCLOK ( _EMT(Columns,adtIUnknown((phv = pStmt)) ); )

		// Clean up
		_RELEASE(pStmt);
		}	// else if

	// Connection
	else if (_RCP(Connection))
		{
		adtIUnknown unkV(v);
		adtLong		lTmp;

		// Previous connection
		_RELEASE(pConn);
		hConn = NULL;

		// New connection
		CCLTRY(_QISAFE(unkV,IID_IHaveValue,&pConn));
		CCLTRY(pConn->getValue ( lTmp ));
		CCLOK (hConn = (SQLHANDLE)(U64)lTmp;)
		}	// else if

	// Fields
	else if (_RCP(Fields))
		{
		adtIUnknown		unkV(v);

		// Previous object
		_RELEASE(pFlds);

		// New object
		CCLTRY(_QISAFE(unkV,IID_IDictionary,&pFlds));
		}	// else if

	// Table name
	else if (_RCP(TableName))
		hr = adtValue::copy ( adtString(v), sTableName );

	return hr;
	}	// receive

#endif

#ifdef	USE_OLEDB

SQL2Table :: SQL2Table ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pFlds			= NULL;
	pConn			= NULL;
	bRemove		= false;

	// Interfaces
	addInterface ( IID_INodeBehaviour, (INodeBehaviour *) this );
	}	// SQL2Table

void SQL2Table :: destruct ( void )
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
	_RELEASE(pFlds);
	_RELEASE(pConn);
	}	// destruct

HRESULT SQL2Table :: fieldsAdd ( IUnknown *pConn, DBID *dbid,
											IDictionary *pFlds )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Adds fields to a table.
	//
	//	PARAMETERS
	//		-	pConn specifies the connection
	//		-	dbid specifies the table
	//		-	pFlds are the list of fields that should be in the table
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT				hr			= S_OK;
	IIt				*pKeys	= NULL;
	IDBCreateSession	*pCreate	= NULL;
	ITableDefinition	*pDef		= NULL;
	adtString			sKey;
	adtValueImpl		vValue;
	DBCOLUMNDESC		cd;
	DBPROPSET			set[1];
	DBPROP				prop[1];

	// Session object
	CCLTRY	( _QI(pConn,IID_IDBCreateSession,&pCreate) );
	CCLTRY	( pCreate->CreateSession ( NULL, IID_ITableDefinition,
													(IUnknown **) &pDef ) );

	// Always add a default, auto-incrementing primary key
	if (hr == S_OK)
		{
		// Column description
		memset ( &cd, 0, sizeof(cd) );

		// Data type
		cd.wType	= DBTYPE_I4;

		// Column ID
		cd.dbcid.eKind				= DBKIND_NAME;
		cd.dbcid.uName.pwszName	= STRING_PRIMKEY;

		// Auto incrementing
		VariantInit ( &(prop[0].vValue) );
		prop[0].dwPropertyID		= DBPROP_COL_AUTOINCREMENT;
		prop[0].dwOptions			= DBPROPOPTIONS_REQUIRED;
		prop[0].vValue.vtype			= VALT_BOOL;
		prop[0].vValue.boolVal	= VARIANT_TRUE;

		// Property set
		set[0].guidPropertySet	= DBPROPSET_COLUMN;
		set[0].cProperties		= sizeof(prop)/sizeof(prop[0]);
		set[0].rgProperties		= prop;

		// Properties for column
		cd.cPropertySets			= 1;
		cd.rgPropertySets			= set;

		// Add field
		hr = pDef->AddColumn ( dbid, &cd, NULL );
		}	// if

	// Add each field
	CCLTRY ( pFlds->keys ( &pKeys ) );
	while (hr == S_OK && pKeys->read ( sKey ) == S_OK)
		{
		// Load the value from the dictionary.  This will be used to get the data type.
		CCLTRY ( pFlds->load ( sKey, vValue ) );

		// Column information
		CCLOK ( memset ( &cd, 0, sizeof(cd) ); )
		if (hr == S_OK)
			{
			// Specify data type
			switch(vValue.vtype)
				{
				case VALT_I4		:	cd.wType = DBTYPE_I4;		break;
				case VALT_R4		:	cd.wType = DBTYPE_R4;		break;
				case VALT_R8		:	cd.wType = DBTYPE_R8;		break;
				case VALT_DATE	:	cd.wType = DBTYPE_DATE;		break;
				case VALT_UNKNOWN:	cd.wType = DBTYPE_BYTES;	break;
				case VALT_BSTR	:
					{
					// Allow 'long' strings
					cd.wType = DBTYPE_WSTR;
					if (adtString::length(vValue.bstrVal))
						cd.pwszTypeName = vValue.bstrVal;
					}	// case VALT_BSTR
					break;

				// Unhandled
				default :
					hr = E_NOTIMPL;
				}	// switch

			// Column ID
			cd.dbcid.eKind				= DBKIND_NAME;
			cd.dbcid.uName.pwszName	= sKey;
			}	// if

		// Add field/column
		if (hr == S_OK)
			{
			hr = pDef->AddColumn ( dbid, &cd, NULL );

			// Duplicates ok
			if (hr == DB_E_DUPLICATECOLUMNID) hr = S_OK;
			}	// if

		// Clean up
		pKeys->next();
		}	// while

	// Clean up
	_RELEASE(pDef);
	_RELEASE(pCreate);
	_RELEASE(pKeys);

	return hr;
	}	// fieldsAdd

HRESULT SQL2Table :: receiveColumns ( const adtValue &v )
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
	HRESULT				hr				= S_OK;
	IDBCreateSession	*pCreate		= NULL;
	ITableCreation		*pTbl			= NULL;
	IContainer		*pCont		= NULL;
	IIt				*pIt			= NULL;
	IOutIt			*pOut			= NULL;
	WCHAR					*pwNames		= NULL;
	DBCOLUMNDESC		*cd			= NULL;
	DBID					dbidIn;
	DBORDINAL			ord,idx;

	// State check
	CCLTRYE	( (pConn != NULL),		ERROR_INVALID_STATE );
	if (hr == S_OK && sTableName.length() == 0)
		hr = pnAttr->load ( strRefTableName, sTableName );
	CCLTRYE ( (sTableName.length() > 0),	ERROR_INVALID_STATE );
	CCLOK	  ( pnAttr->load ( strRefRemFlds, bRemove ); )

	// Session object
	CCLTRY	( _QI(pConn,IID_IDBCreateSession,&pCreate) );
	CCLTRY	( pCreate->CreateSession ( NULL, IID_ITableCreation,
													(IUnknown **) &pTbl ) );

	// Table information
	if (hr == S_OK)
		{
		// Table name
		memset ( &dbidIn, 0, sizeof(dbidIn) );
		dbidIn.eKind				= DBKIND_NAME;
		dbidIn.uName.pwszName	= sTableName;

		// Make sure table exists
		hr = pTbl->GetTableDefinition ( &dbidIn, &ord, &cd, NULL, NULL,
													NULL, NULL, NULL );
		}	// if

	// Put the column names into a container
	CCLTRY(COCREATEINSTANCE(CLSID_ADTList,IID_IContainer,&pCont));
	CCLTRY(pCont->iterate ( &pIt ));
	CCLTRY(_QI(pIt,IID_IOutIt,&pOut));
	for (idx = 0;hr == S_OK && idx < ord;++idx)
		{
		// Write next column name
		CCLTRY(pOut->end());
		CCLTRY(pOut->write ( adtString(cd[idx].dbcid.uName.pwszName) ));
		}	// for
	_RELEASE(pOut);
	_RELEASE(pIt);

	// Result
	CCLOK ( peCols->emit ( adtIUnknown ( pCont ) ); )

	// Clean up
	_RELEASE(pCont);
	_RELEASE(pTbl);
	_RELEASE(pCreate);

	return hr;
	}	// receiveColumns

HRESULT SQL2Table :: receiveFire ( const adtValue &v )
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
	HRESULT				hr				= S_OK;
	IDBCreateSession	*pCreate		= NULL;
	ITableDefinition	*pDef			= NULL;
	DBID					dbidIn;

	// State check
	CCLTRYE	( (pConn != NULL),		ERROR_INVALID_STATE );
	if (hr == S_OK && sTableName.length() == 0)
		hr = pnAttr->load ( strRefTableName, sTableName );
	CCLTRYE ( (sTableName.length() > 0),	ERROR_INVALID_STATE );
	CCLOK	  ( pnAttr->load ( strRefRemFlds, bRemove ); )

	// Session object
	CCLTRY	( _QI(pConn,IID_IDBCreateSession,&pCreate) );
	CCLTRY	( pCreate->CreateSession ( NULL, IID_ITableDefinition,
													(IUnknown **) &pDef ) );

	// Table existence
	if (hr == S_OK)
		{
		// Table name
		memset ( &dbidIn, 0, sizeof(dbidIn) );
		dbidIn.eKind				= DBKIND_NAME;
		dbidIn.uName.pwszName	= sTableName;

		// Make sure table exists
		hr = pDef->CreateTable ( NULL, &dbidIn, 0, NULL, IID_NULL, 0, NULL,
											NULL, NULL );

		// Ok if it already exists
		if (hr == DB_E_DUPLICATETABLEID) hr = S_OK;
		}	// if

	// Add specified fields
	if (hr == S_OK)
		hr = fieldsAdd ( pConn, &dbidIn, pFlds );

	// TODO: Wait until this is needed to see what form it takes.
	// Remove fields that are no longer specified in definition

	// Result
	CCLOK ( peFire->emit ( sTableName ); )

	// Clean up
	_RELEASE(pDef);
	_RELEASE(pCreate);

	return hr;
	}	// receiveFire

//
// Context
//

HRESULT SQL2Table :: receiveConnection ( const adtValue &v )
	{
	adtIUnknown	unkV(v);
	_RELEASE(pConn);
	_QISAFE(unkV,IID_IUnknown,&pConn);
	return S_OK;
	}	// receiveConnection

#endif

