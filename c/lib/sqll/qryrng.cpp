////////////////////////////////////////////////////////////////////////
//
//									QRYRNG.CPP
//
//				Implementation of the SQL range query node
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

#ifdef	USE_ODBC

// Globals
static WCHAR	wSelect[]	=	L"SELECT ";
static WCHAR	wFrom[]		=	L" FROM ";
static WCHAR	wWhere[]		=	L" WHERE ";
static WCHAR	wOrder[]		=	L" ORDER BY ";
static WCHAR	wKeyWords[]	=	L"SELECT FROM  WHERE  >=  AND  <=  ORDER BY  ASC";
static WCHAR	wCount[]		=	L"COUNT(*)";

SQLQueryRange :: SQLQueryRange ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pConn		= NULL;
	hConn		= SQL_NULL_HANDLE;
	pFlds		= NULL;
	sQuery	= L"";
	bSort		= true;
	bCount	= false;
	}	// SQLQueryRange

void SQLQueryRange :: destruct ( void )
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

HRESULT SQLQueryRange :: genQuery ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Generates the query string for the current state.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr	= S_OK;
	U32			nf,flen,qlen;
	adtString	sFld;

	// State check
	CCLTRY(validate());

	// Setup
	CCLOK ( qlen = 0; )

	// Get total length of fields names and add it padding for the punctuation
	if (hr == S_OK)
		{
		if (bCount == false)
			{
			CCLTRY( SQLStringItLen ( pFlds, &nf, &flen ) );
			CCLOK	( qlen += (flen + (2*nf+nf)); )	// For quotes and commas
			}	// if
		else
			qlen += (U32)wcslen(wCount);
		}	// if

	// Table name
	CCLOK	( qlen += sTableName.length(); )

	// Keys are bound directly so just add space for ?
	CCLOK  ( qlen += 2; )								// '?' for values

	// Key name length (shows up 3 times in query string)
	CCLOK ( qlen += (sKey.length()+2)*3; )

	// Add in our length of the keywords (plus trailing spaces)
	// SELECT FROM WHERE >= AND <=
	CCLOK ( qlen += (U32)wcslen(wKeyWords); )

	// Allocate enough space for entire string, it will be of the form
	// SELECT <fields> FROM <table name> WHERE <key> >= <left> AND <key> <= <right>
	CCLTRY ( sQuery.allocate ( qlen ) );

	// SELECT
	CCLOK	 ( WCSCPY ( &sQuery.at(), qlen, wSelect ); )

	// Count queyr
	if (hr == S_OK && bCount == true)
		WCSCAT ( &sQuery.at(), qlen, wCount );

	else
		{
		// Field names
		CCLTRY(pFlds->begin());
		while (hr == S_OK && pFlds->read ( sFld ) == S_OK)
			{
			// Append
			CCLOK ( WCSCAT ( &sQuery.at(), qlen, L"\"" ); )
			CCLOK ( WCSCAT ( &sQuery.at(), qlen, sFld ); )
			CCLOK ( WCSCAT ( &sQuery.at(), qlen, L"\"," ); )

			// Next field
			pFlds->next();
			}	// while

		// One comma too many
		CCLOK ( sQuery.at(sQuery.length()-1) = WCHAR('\0'); )
		}	// else

	// FROM
	CCLOK ( WCSCAT ( &sQuery.at(), qlen, wFrom ); )

	// Table name
	CCLOK ( WCSCAT ( &sQuery.at(), qlen, sTableName ); )

	// WHERE
	CCLOK ( WCSCAT ( &sQuery.at(), qlen, wWhere ); )

	// Left range
	CCLOK ( WCSCAT ( &sQuery.at(), qlen, L"\"" ); )
	CCLOK ( WCSCAT ( &sQuery.at(), qlen, sKey ); )
	CCLOK ( WCSCAT ( &sQuery.at(), qlen, L"\" >= ? AND \"" ); )

	// Right range
	CCLOK ( WCSCAT ( &sQuery.at(), qlen, sKey ); )
	CCLOK ( WCSCAT ( &sQuery.at(), qlen, L"\" <= ?" ); )

	// Order ?
	if	(	hr == S_OK && bCount == false && bSort == true )
		{
		CCLOK ( WCSCAT ( &sQuery.at(), qlen, wOrder ); )
		CCLOK ( WCSCAT ( &sQuery.at(), qlen, L"\"" ); )
		CCLOK ( WCSCAT ( &sQuery.at(), qlen, sKey ); )
		CCLOK ( WCSCAT ( &sQuery.at(), qlen, L"\"" ); )
		CCLOK ( WCSCAT ( &sQuery.at(), qlen, L" ASC" ); )
		}	// if

	// Debug
	if (hr == S_OK)
		{
		WCHAR wBfr[100];
		swprintf ( SWPF(wBfr,100), L"(%d/%d):", (U32)sQuery.length(), qlen );
		CCLOK	 ( OutputDebugString ( wBfr ); )
		CCLOK	 ( OutputDebugString ( sQuery ); )
		CCLOK	 ( OutputDebugString ( L"\n" ); )
		}	// if

	return hr;
	}	// genQuery

HRESULT SQLQueryRange :: receive ( IReceptor *pr, const WCHAR *pl, 
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
		SQLHandle	*pStmt	= NULL;
		IHaveValue	*phv;

		// Our statement handle
		CCLTRYE	( (pStmt = new SQLHandle ( SQL_HANDLE_STMT, hConn ))
						!= NULL, E_OUTOFMEMORY );
		CCLTRY	( pStmt->construct() );

		// Static query (allows SQLFetchScroll)
		CCLTRY ( SQLSTMT(pStmt->Handle,SQLSetStmtAttr ( pStmt->Handle, SQL_ATTR_CURSOR_TYPE,
						(SQLPOINTER) SQL_CURSOR_STATIC, SQL_IS_INTEGER )));

		// Query need generated ?
		if (hr == S_OK && sQuery.length() == 0)
			hr = genQuery();

		// We need to bind our key values directly to avoid converting to strings
		CCLTRY ( SQLBindVariantParam ( pStmt->Handle, 1, &vLeft, &uLeftSz ) );
		CCLTRY ( SQLBindVariantParam ( pStmt->Handle, 2, &vRight, &uRightSz ) );

		// Execute.  We should not get SQL_NEED_DATA here because keys are not blobs
		// and other data types are known size
		CCLTRY ( SQLSTMT(pStmt->Handle,SQLExecDirect ( pStmt->Handle, &sQuery.at(), sQuery.length() )));

		// Result
		CCLOK ( _EMT(Fire,adtIUnknown((phv = pStmt)) ); )

		// Clean up
		_RELEASE(pStmt);
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

		// Query will need to be regenerated
		sQuery.at(0) = WCHAR('\0');
		}	// else if

	// State
	else if (_RCP(Left))
		{
		adtValue::copy ( v, vLeft );

		// Query will need to be regenerated
		sQuery.at(0) = WCHAR('\0');
		}	// else if
	else if (_RCP(Right))
		{
		adtValue::copy ( v, vRight );

		// Query will need to be regenerated
		sQuery.at(0) = WCHAR('\0');
		}	// else if

	return hr;
	}	// receive

HRESULT SQLQueryRange :: validate ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Validates internal state for performing queries
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT	hr	= S_OK;

	// State check
	CCLTRYE	( (hConn != SQL_NULL_HANDLE), ERROR_INVALID_STATE );

	// Table name
	if (hr == S_OK && sTableName.length() == 0)
		CCLTRY(pnDesc->load ( strRefTableName, sTableName ));
	CCLTRYE	( (sTableName.length() > 0), ERROR_INVALID_STATE );

	// Key name
	if (hr == S_OK && sKey.length() == 0)
		CCLTRY(pnDesc->load ( strRefKey, sKey ));
	CCLTRYE	( (sKey.length() > 0), ERROR_INVALID_STATE );

	// Count query ?
	if (hr == S_OK) pnDesc->load ( strRefCount, bCount );

	// Sort ?
	CCLOK ( pnDesc->load ( strRefSort, bSort ); )

	// Fields specified ?
	if (hr == S_OK && bCount == false && pFlds == NULL)
		{
		IContainer		*pCont	= NULL;
		IIt				*pIt		= NULL;
		adtIUnknown		unkV;

		// Obtain field list, expecting list of strings
		CCLTRY(pnDesc->load ( strRefFields, unkV ));
		CCLTRY(_QISAFE(unkV,IID_IContainer,&pCont));
		CCLTRY(pCont->iterate ( &pIt ));
		CCLTRY(_QI(pIt,IID_IIt,&pFlds));

		// Clean up
		_RELEASE(pIt);
		_RELEASE(pCont);

		CCLTRYE	( (pFlds != SQL_NULL_HANDLE), ERROR_INVALID_STATE );
		}	// if

	// Ranges specified ?
	if (hr == S_OK && adtValue::empty ( vLeft ) == true)
		hr = pnDesc->load ( strRefLeft, vLeft );
	CCLTRYE	( adtValue::empty ( vLeft ) == false,		ERROR_INVALID_STATE );
	if (hr == S_OK && adtValue::empty ( vRight ) == true)
		hr = pnDesc->load ( strRefRight, vRight );
	CCLTRYE	( adtValue::empty ( vRight ) == false,	ERROR_INVALID_STATE );
	CCLTRYE	( vLeft.vtype == vRight.vtype,			E_INVALIDARG );

	// Valid range types
	CCLTRYE	(	vLeft.vtype == VTYPE_I4		||		vLeft.vtype == VTYPE_I8 ||
					vLeft.vtype == VTYPE_R4		||		vLeft.vtype == VTYPE_R8 ||
					vLeft.vtype == VTYPE_STR	||		vLeft.vtype == VTYPE_DATE,
					E_INVALIDARG );

	return hr;
	}	// validate

#endif
