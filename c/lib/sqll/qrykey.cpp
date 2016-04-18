////////////////////////////////////////////////////////////////////////
//
//									QRYKEY.CPP
//
//				Implementation of the SQL query key node
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
static WCHAR	wKeyWords[]	=	L"SELECT  FROM  WHERE ";

SQLQueryKey :: SQLQueryKey ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pConn			= NULL;
	hConn			= SQL_NULL_HANDLE;
	pKeys			= NULL;
	pKeyBinds	= NULL;
	uNumKeys		= 0;
	pFlds			= NULL;
	uNumFlds		= 0;
	}	// SQLQueryKey

void SQLQueryKey :: destruct ( void )
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
	if (pKeys != NULL)		delete[] pKeys;
	if (pKeyBinds != NULL)	delete[] pKeyBinds;
	if (pFlds != NULL)		delete[] pFlds;
	_RELEASE(pConn);
	}	// destruct

HRESULT SQLQueryKey :: initializeFields ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Loads/generates the field names from the node properties.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr			= S_OK;
	IADTContainer	*pCont	= NULL;
	IADTInIt			*pIn		= NULL;
	IADTIt			*pIt		= NULL;
	adtIUnknown		unkV;
	adtString		sFld;
	U32				idx;

	// Fields
	CCLTRY(pnAttr->load ( strRefFields, unkV ));
	CCLTRY(_QISAFE(unkV,IID_IADTContainer,&pCont));
	CCLTRY(pCont->iterate ( &pIt ));
	CCLTRY(_QI(pIt,IID_IADTInIt,&pIn));

	// Allocate name objects
	CCLTRY	(pCont->size ( &uNumFlds ));
	CCLTRYE	((pFlds = new adtString[uNumFlds]) != NULL,E_OUTOFMEMORY);

	// Copy names
	CCLOK		(idx = 0;)
	CCLTRY	(pIn->begin());
	while (hr == S_OK && idx < uNumFlds && pIn->read ( sFld ) == S_OK)
		{
		// Copy
		CCLTRYE	( (sFld.vtype == VALT_STR), E_UNEXPECTED );
		CCLOK		( pFlds[idx++] = sFld; )

		// Next key
		pIn->next();
		}	// while

	// Clean up
	_RELEASE(pIn);
	_RELEASE(pIt);
	_RELEASE(pCont);

	return hr;
	}	// initializeFields

HRESULT SQLQueryKey :: initializeKeys ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Loads/generates the key names/columns from the node properties.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT			hr			= S_OK;
	IADTContainer	*pCont	= NULL;
	IADTInIt			*pIn		= NULL;
	IADTIt			*pIt		= NULL;
	adtIUnknown		unkV;
	adtString		sKey;
	U32				idx;

	// Keys
	CCLTRY(pnAttr->load ( strRefKey, unkV ));
	CCLTRY(_QISAFE(unkV,IID_IADTContainer,&pCont));
	CCLTRY(pCont->iterate ( &pIt ));
	CCLTRY(_QI(pIt,IID_IADTInIt,&pIn));

	// Allocate key name objects
	CCLTRY	(pCont->size ( &uNumKeys ));
	CCLTRYE	((pKeys = new adtString[uNumKeys]) != NULL,E_OUTOFMEMORY);

	// Copy key names
	CCLOK		(idx = 0;)
	CCLTRY	(pIn->begin());
	while (hr == S_OK && idx < uNumKeys && pIn->read ( sKey ) == S_OK)
		{
		// Copy
		CCLTRYE	( (sKey.vtype == VALT_STR), E_UNEXPECTED );
		CCLOK		( pKeys[idx++] = sKey; )

		// Next key
		pIn->next();
		}	// while

	// Allocate binding array to match size of keys
	CCLTRYE	((pKeyBinds = new SQLCol[uNumKeys]) != NULL, E_OUTOFMEMORY);

	// Clean up
	_RELEASE(pIn);
	_RELEASE(pIt);
	_RELEASE(pCont);

	return hr;
	}	// initializeKeys

HRESULT SQLQueryKey :: initializeQuery ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Prepares for executing the necessary query.
	//
	//	RETURN VALUE
	//		S_OK if successful
	//
	////////////////////////////////////////////////////////////////////////
	HRESULT		hr			= S_OK;
	U32			uQuerySz,idx;

	// State check
	CCLTRYE	( (hConn != SQL_NULL_HANDLE), ERROR_INVALID_STATE );
	CCLTRY	( pnAttr->load ( strRefTableName, sTableName ) );
	CCLTRYE	( sTableName.length() > 0, ERROR_INVALID_STATE );

	// Initialize keys and fields
	CCLTRY	( initializeKeys() );
	CCLTRY	( initializeFields() );

	// Generate query string of the form :
	// SELECT (x,y,z) FROM table WHERE fd = 2 AND fdd = 3 AND ...
	CCLOK		( uQuerySz = sTableName.length(); )

	// Size of field and key strings
	for (idx = 0;hr == S_OK && idx < uNumKeys;++idx)
		uQuerySz += (pKeys[idx].length());
	for (idx = 0;hr == S_OK && idx < uNumFlds;++idx)
		uQuerySz += (pFlds[idx].length());

	// Add space for quotes around the field names and other puncuation
	CCLOK ( uQuerySz +=	(3*uNumFlds) +				// Commas
								2 +							// Parenthesis
								11*uNumKeys; )				// '"" = ? AND '

	// Keywords
	CCLOK ( uQuerySz += (U32)wcslen(wKeyWords); )

	// Allocate space
	CCLTRY( sQuery.allocate ( uQuerySz ); )

	// Prepare string
	CCLOK	( wcscpy ( &sQuery.at(), L"SELECT " ); )
	for (idx = 0;hr == S_OK && idx < uNumFlds;++idx)
		{
		CCLOK ( wcscat ( &sQuery.at(), L"\"" ); )
		CCLOK ( wcscat ( &sQuery.at(), pFlds[idx] ); )
		CCLOK ( wcscat ( &sQuery.at(), L"\"," ); )
		}	// for
	CCLOK ( sQuery.at(sQuery.length()-1) = WCHAR('\0'); )
	CCLOK ( wcscat ( &sQuery.at(), L" FROM " ); )
	CCLOK ( wcscat ( &sQuery.at(), sTableName ); )
	CCLOK ( wcscat ( &sQuery.at(), L" WHERE " ); )
	for (idx = 0;hr == S_OK && idx < uNumKeys;++idx)
		{
		CCLOK ( wcscat ( &sQuery.at(), L"\"" ); )
		CCLOK ( wcscat ( &sQuery.at(), pKeys[idx] ); )
		CCLOK ( wcscat ( &sQuery.at(), L"\" = ?" ); )
		if (hr == S_OK && idx+1 < uNumKeys)
			wcscat ( &sQuery.at(), L" AND " );
		}	// for
	// Debug
	WCHAR wBfr[100];
	swprintf ( wBfr, L"(%d/%d):", (U32)sQuery.length(), uQuerySz );
	CCLOK	 ( OutputDebugString ( wBfr ); )
	CCLOK	 ( OutputDebugString ( sQuery ); )
	CCLOK	 ( OutputDebugString ( L"\n" ); )

	return hr;
	}	// initializeQuery

HRESULT SQLQueryKey :: receive ( IReceptor *pr, const WCHAR *pl, 
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
	if (prFire == pR)
		{
		SQLHandle	*pStmt	= NULL;
		IHaveValue	*phv;
		U32			idx;

		// State check
		CCLTRYE ( (sQuery.length() > 0),	ERROR_INVALID_STATE );

		// Our statement handle
		CCLTRYE	( (pStmt = new SQLHandle ( SQL_HANDLE_STMT, hConn ))
						!= NULL, E_OUTOFMEMORY );
		CCLTRY	( pStmt->construct() );

		// Close previous results
	//	CCLOK	 ( SQLCloseCursor ( pStmt->Handle ); )
	//	CCLOK	 ( SQLCancel ( pStmt->Handle ); )

		// Bind our keys
		for (idx = 0;hr == S_OK && idx < uNumKeys;++idx)
			hr = SQLBindVariantParam ( pStmt->Handle, idx+1,
					&(pKeyBinds[idx].sData), &(pKeyBinds[idx].uSz) );

		// Execute prepare SQL query
	//	CCLTRY ( SQLSTMT(pStmt->Handle,SQLExecute ( pStmt->Handle )));
		CCLTRY ( SQLSTMT(pStmt->Handle,SQLExecDirect ( pStmt->Handle,
								&sQuery.at(), sQuery.length() )));

		// Result
		CCLOK  ( peFire->emit ( adtIUnknown((phv = pStmt)) ); )

		// Clean up
		_RELEASE(pStmt);

		}	// if

	// Key
	else if (prKey == pR)
		{
		IADTDictionary	*pDict	= NULL;
		IADTInIt			*pKeysIn	= NULL;
		IADTIt			*pIt		= NULL;
		adtIUnknown		unkV(v);
		U32				idx;

		// State check
		CCLTRY(_QISAFE(unkV,IID_IADTDictionary,&pDict));

		// Node properties initialized ?
		if (hr == S_OK && pKeys == NULL)
			hr = initializeQuery();

		// More state check
		CCLTRYE(pKeys		!= NULL,ERROR_INVALID_STATE);
		CCLTRYE(pKeyBinds	!= NULL,ERROR_INVALID_STATE);
		CCLTRYE(pFlds		!= NULL,ERROR_INVALID_STATE);

		// Iterate dictionary and pull out needed keys.  Load key values
		// directly into our columuns
		for (idx = 0;hr == S_OK && idx < uNumKeys;++idx)
			CCLTRY(pDict->load ( pKeys[idx], pKeyBinds[idx].sData ));

		// Clean up
		_RELEASE(pDict);
		}	// else if

	// Connection
	else if (prConn == pR)
		{
		HRESULT		hr = S_OK;
		adtIUnknown unkV(v);
		adtLong		lTmp;
		_RELEASE(pConn);
		CCLTRY(_QISAFE(unkV,IID_IHaveValue,&pConn));
		CCLTRY(pConn->getValue ( lTmp ));
		CCLOK (hConn = (SQLHANDLE)(U64)lTmp;)
		}	// else if

	return hr;
	}	// receive

#endif