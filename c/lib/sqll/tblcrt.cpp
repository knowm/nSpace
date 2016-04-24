////////////////////////////////////////////////////////////////////////
//
//									TBLCRT.CPP
//
//				Implementation of the SQL create table node
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

// Globals
static WCHAR	wAddField[]	= L"ALTER TABLE %s ADD \"%s\" %s";
static WCHAR	wCreateTbl[]= L"CREATE TABLE \"%s\"";
static WCHAR	wSelectAll[]= L"SELECT * FROM \"%s\"";
static WCHAR	wSelectOne[]= L"SELECT %s FROM \"%s\"";
static WCHAR	wAlter[]		= L"ALTER TABLE  ADD PRIMARY KEY ";

#ifdef	USE_ODBC

SQLTableCreate :: SQLTableCreate ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pBfr	= NULL;
	pwBfr	= NULL;
	pConn	= NULL;
	hConn	= SQL_NULL_HANDLE;
	pDef	= NULL;
	}	// SQLTableCreate

HRESULT SQLTableCreate :: construct ( void )
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

	// Create buffer
	CCLTRY ( COCREATE ( L"Io.MemoryBlock", IID_IMemoryMapped, &pBfr ) );

	return hr;
	}	// construct

void SQLTableCreate :: destruct ( void )
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
	_RELEASE(pDef);
	_RELEASE(pConn);
	_RELEASE(pBfr);
	}	// destruct

HRESULT SQLTableCreate :: fieldsAdd ( SQLHANDLE hConn, adtString &sName,
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
				case VTYPE_R4		:	WCSCPY ( wType, 20, L"real" );		break;
				case VTYPE_R8		:	WCSCPY ( wType, 20, L"double" );		break;
				case VTYPE_STR		:	WCSCPY ( wType, 20, L"text" );		break;
				case VTYPE_DATE	:	WCSCPY ( wType, 20, L"datetime" );	break;
				case VTYPE_UNK		:	WCSCPY ( wType, 20, L"image" );		break;

				// Unhandled
				default :
					hr = E_NOTIMPL;
				}	// switch
			}	// if

		// Prepare command
		CCLTRY ( SQLSTMT(hStmt,(SQLAllocHandle ( SQL_HANDLE_STMT, hConn, &hStmt ))) );
		CCLOK  ( uLen = ((U32)wcslen(wAddField)+sName.length()+
								sKey.length()+(U32)wcslen(wType)+1)*sizeof(WCHAR); )
		CCLTRY ( pBfr->setSize ( uLen ) );
		CCLTRY ( pBfr->lock ( 0, 0, (PVOID *) &pwBfr, NULL ) );
		CCLOK  ( swprintf ( SWPF(pwBfr,uLen), wAddField, (LPCWSTR) sName, (LPCWSTR) sKey, wType ); )

		// Execute.
		WCHAR wBfr[100];
		swprintf ( SWPF(wBfr,uLen), L"(%d/%d):", (U32)wcslen(pwBfr), uLen );
		CCLOK	 ( OutputDebugString ( wBfr ); )
		CCLOK	 ( OutputDebugString ( pwBfr ); )
		CCLOK	 ( OutputDebugString ( L"\n" ); )
		CCLOK (SQLSTMT(hStmt,(SQLExecDirect ( hStmt, pwBfr, (SQLINTEGER)wcslen(pwBfr) )));)

		// Clean up
		_UNLOCK(pBfr,pwBfr);
		SQLFREESTMT(hStmt);
		pKeys->next();
		}	// while

	// Clean up
	_RELEASE(pKeys);

	return hr;
	}	// fieldsAdd

HRESULT SQLTableCreate :: primaryKey ( SQLHANDLE hConn, adtString &sName,
													IIt *pKeys )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Creates the specified primary key on the table
	//
	//	PARAMETERS
	//		-	hConn specifies the connection
	//		-	sName specifies the table name
	//		-	pKeys is the list of key names
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr			= S_OK;
	SQLHANDLE		hStmt		= NULL;
	U32				klen,nkeys,sz;
	adtString		sKey;
	adtValue			vValue;

	// Compute the full string length of all the keys
	CCLTRY ( SQLStringItLen ( pKeys, &nkeys, &klen ) );

	// Computer amount of string space needed, include space for the commas,
	// quotes and parens
	CCLOK	( sz = (U32)wcslen(wAlter) + sName.length() + klen + 3*nkeys + 2; )
	CCLTRY( pBfr->setSize ( sz*sizeof(WCHAR) ) );
	CCLTRY( pBfr->lock ( 0, 0, (PVOID *) &pwBfr, NULL ) );

	// Generate query string
	CCLOK ( WCSCPY ( pwBfr, sz, L"ALTER TABLE " ); )
	CCLOK ( WCSCAT ( pwBfr, sz, sName ); )
	CCLOK ( WCSCAT ( pwBfr, sz, L" ADD PRIMARY KEY (" ); )

	// Add key fields
	CCLTRY(pKeys->begin());
	while (hr == S_OK && pKeys->read ( sKey ) == S_OK)
		{
		CCLTRYE	( (adtValue::type(sKey) == VTYPE_STR), E_UNEXPECTED );
		CCLOK		( WCSCAT ( pwBfr, sz, L"\"" ); )
		CCLOK		( WCSCAT ( pwBfr, sz, sKey ); )
		CCLOK		( WCSCAT ( pwBfr, sz, L"\"," ); )

		// Next key
		pKeys->next();
		}	// while

	// Last comma unneeded, replace with ending ')'
	CCLOK ( pwBfr[wcslen(pwBfr)-1] = WCHAR(')'); )

	// Execute
	WCHAR wBfr[100];
	CCLTRY(SQLSTMT(hStmt,(SQLAllocHandle ( SQL_HANDLE_STMT, hConn, &hStmt ))) );
	CCLOK (swprintf ( SWPF(wBfr,100), L"(%d/%d):", (U32)wcslen(pwBfr), (U32)(sz*sizeof(WCHAR)) ); )
	CCLOK	(OutputDebugString ( wBfr ); )
	CCLOK (OutputDebugString ( pwBfr ); )
	CCLOK (OutputDebugString ( L"\n" ); )
	CCLOK (SQLSTMT(hStmt,(SQLExecDirect ( hStmt, pwBfr, (SQLINTEGER)wcslen(pwBfr) )));)

	// Clean up
	_UNLOCK(pBfr,pwBfr);
	SQLFREESTMT(hStmt);

	return hr;
	}	// primaryKey

HRESULT SQLTableCreate :: receive ( IReceptor *pr, const WCHAR *pl, 
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
		IDictionary	*pFlds	= NULL;
		IContainer	*pKeys	= NULL;
		IIt			*pIt		= NULL;
		IIt			*pKeysIn	= NULL;
		adtIUnknown		unkV;
		adtString		sTable;

		// State check
		CCLTRYE	( (hConn != SQL_NULL_HANDLE), ERROR_INVALID_STATE );
		CCLTRYE	( (pDef != NULL),					ERROR_INVALID_STATE );

		// Table properties
		CCLTRY	( pDef->load ( strRefFields,		unkV ) );
		CCLTRY	( _QISAFE(unkV,IID_IDictionary,&pFlds) );
		CCLTRY	( pDef->load ( strRefTableName,	sTable ); )
		CCLTRY	( pDef->load ( strRefPrimKey,		unkV ); )
		CCLTRY	( _QISAFE(unkV,IID_IContainer,&pKeys) );
		CCLTRY	( pKeys->iterate ( &pIt ) );
		CCLTRY	( _QI(pIt,IID_IIt,&pKeysIn) );

		// Verify table exists.  If not add fields.
		if (tableCreate ( hConn, sTable ) == S_OK)
			{
			// Add fields from dictionary 
			CCLTRY ( fieldsAdd ( hConn, sTable, pFlds ) );

			// TODO: Remove unused fields
	//		CCLTRY ( fieldsRemove ( hConn, sTable, pFlds ) );

			// Add the primary key
			CCLTRY ( primaryKey ( hConn, sTable, pKeysIn ) );
			}	// if

		// Clean up
		_RELEASE(pIt);
		_RELEASE(pKeysIn);
		_RELEASE(pKeys);
		_RELEASE(pFlds);

		// Result
		_EMT(Fire,v);
		}	// if

	// Connection
	else if (_RCP(Connection))
		{
		HRESULT		hr = S_OK;
		adtIUnknown unkV(v);
		adtLong		lTmp;
		_RELEASE(pConn);
		CCLTRY(_QISAFE(unkV,IID_IHaveValue,&pConn));
		CCLTRY(pConn->getValue ( lTmp ));
		CCLOK (hConn = (SQLHANDLE)(U64)lTmp;)
		}	// else if

	// Definition
	else if (_RCP(Definition))
		{
		HRESULT		hr = S_OK;
		adtIUnknown unkV(v);
		_RELEASE(pDef);
		CCLTRY(_QISAFE(unkV,IID_IDictionary,&pDef));
		}	// else if

	return hr;
	}	// receive

HRESULT SQLTableCreate :: tableCreate ( SQLHANDLE hConn, adtString &sName )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Verifies the existence of a table.
	//
	//	PARAMETERS
	//		-	hConn specifies the connection
	//		-	sName specifies the table name
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr			= S_OK;
	SQLHANDLE		hStmt		= NULL;
	WCHAR				*pwBfr	= NULL;
	U32				sz			= 0;

	// Create the table no matter, it will return an error if it exists

	// Prepare command
	CCLTRY ( SQLSTMT(hStmt,(SQLAllocHandle ( SQL_HANDLE_STMT, hConn, &hStmt ))) );
	CCLTRY ( SQLSTMT(hStmt,(SQLSetStmtAttr ( hStmt, SQL_ATTR_MAX_ROWS,
															(SQLPOINTER) 1, NULL ))) );
	CCLOK	 ( sz = ((U32)wcslen(wCreateTbl)+sName.length()+1)*sizeof(WCHAR); )
	CCLTRY ( pBfr->setSize ( sz ) );
	CCLTRY ( pBfr->lock ( 0, 0, (PVOID *) &pwBfr, NULL ) );
	CCLOK  ( swprintf ( SWPF(pwBfr,sz), wCreateTbl, (LPCWSTR) sName ); )

	// Execute
	WCHAR wBfr[100];
	CCLOK (swprintf ( SWPF(wBfr,100), L"(%d/%d):", (U32)wcslen(pwBfr), sz ); )
	CCLOK	( OutputDebugString ( wBfr ); )
	CCLOK	( OutputDebugString ( pwBfr ); )
	CCLOK	( OutputDebugString ( L"\n" ); )
	CCLTRY( SQLSTMT(hStmt,(SQLExecDirect ( hStmt, pwBfr, (SQLINTEGER)wcslen(pwBfr) ))));

	// Clean up
	_UNLOCK(pBfr,pwBfr);
	SQLFREESTMT(hStmt);

	return hr;
	}	// tableCreate

#endif
