////////////////////////////////////////////////////////////////////////
//
//									UPDATE.CPP
//
//					Implementation of the SQL update node
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

#define	SIZE_SQLBFR		1024

// Globals

#ifdef	USE_ODBC

SQLUpdate :: SQLUpdate ( void )
	{
	////////////////////////////////////////////////////////////////////////
	//
	//	PURPOSE
	//		-	Constructor for the node
	//
	////////////////////////////////////////////////////////////////////////
	pConn		= NULL;
	hConn		= NULL;
	pConsIn	= NULL;
	pFlds		= NULL;
	pvCons	= NULL;
	pCons		= NULL;
	szCons	= 0;
	pQryBfr	= NULL;
	pwQryBfr	= NULL;
	}	// SQLUpdate

HRESULT SQLUpdate :: construct ( void )
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

void SQLUpdate :: destruct ( void )
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
	if (pvCons != NULL) delete[] pvCons;
	_RELEASE(pCons);
	_RELEASE(pConsIn);
	_RELEASE(pFlds);
	_RELEASE(pConn);
	_UNLOCK(pQryBfr,pwQryBfr);
	_RELEASE(pQryBfr);
	}	// destruct

HRESULT SQLUpdate :: receive ( IReceptor *pr, const WCHAR *pl, 
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
		IDictionary		*pDict		= NULL;
		IIt				*pKeys		= NULL;
		SQLHandle		*pStmt		= NULL;
		SQLCol			*pvValues	= NULL;
		IHaveValue		*phv;
		adtValue			vVal,vKey;
		adtIUnknown		unkV;
		adtString		strV;
		U32				idx,cidx,nkeys;
		bool				bLike;

		// State check
		CCLTRYE ( (hConn != NULL),					ERROR_INVALID_STATE );
		if (hr == S_OK && sTableName.length() == 0)
			hr = pnDesc->load ( strRefTableName, sTableName );
		CCLTRYE ( (sTableName.length() > 0),	ERROR_INVALID_STATE );
		CCLTRYE ( pFlds != NULL,					ERROR_INVALID_STATE );

		// Our statement handle
		CCLTRYE	( (pStmt = new SQLHandle ( SQL_HANDLE_STMT, hConn ))
						!= NULL, E_OUTOFMEMORY );
		CCLTRY	( pStmt->construct() );

		// Field information
		CCLTRY ( pFlds->keys ( &pKeys ) );
		CCLTRY ( pFlds->size ( &nkeys ) );

		// Allocate memory for our copied value table
		CCLTRYE ( (pvValues = new SQLCol[nkeys]) != NULL, E_OUTOFMEMORY );

		// Time to generate query string.
		// Form is : UPDATE (Table) SET (Fields) WHERE (Constraints)

		// UPDATE
		CCLOK		( WCSCPY ( pwQryBfr, SIZE_SQLBFR, L"UPDATE " ); )

		// Table name
		CCLOK		( WCSCAT ( pwQryBfr, SIZE_SQLBFR, sTableName ); )

		// SET
		CCLOK		( WCSCAT ( pwQryBfr, SIZE_SQLBFR, L" SET " ); )

		// Fields
		CCLOK  ( cidx = 0; )
		CCLTRY ( pKeys->begin() );
		while (hr == S_OK && pKeys->read ( vKey ) == S_OK)
			{
			adtString sKey(vKey);

			// State check
			CCLTRYE ( adtValue::type(vKey) == VTYPE_STR, E_UNEXPECTED );

			// Field name
			CCLOK ( WCSCAT ( pwQryBfr, SIZE_SQLBFR, L"\"" ); )
			CCLOK ( WCSCAT ( pwQryBfr, SIZE_SQLBFR, sKey ); )
			CCLOK ( WCSCAT ( pwQryBfr, SIZE_SQLBFR, L"\" = ?," ); )

			// Value for key
			CCLTRY( pFlds->load ( sKey, pvValues[cidx].sData ) );

			// Bind value
			CCLTRY( SQLBindVariantParam ( pStmt->Handle, cidx+1,
						&(pvValues[cidx].sData), &(pvValues[cidx].uSz) ) );

			// Next
			CCLOK ( cidx++; )
			pKeys->next();
			}	// while
		CCLOK ( pwQryBfr[wcslen(pwQryBfr)-1] = WCHAR('\0'); )

		// Optional constraints
		if (hr == S_OK && szCons == 0 && pCons != NULL)
			{
			// Remove previous constraints
			if (pvCons != NULL) { delete[] pvCons; pvCons = NULL; }

			// Constraint size
			CCLTRY(pCons->size ( &szCons ));

			// Allocate new value list for binding
			if (hr == S_OK && szCons > 0)
				hr = ((pvCons = new SQLCol[szCons]) != NULL) ? S_OK : E_OUTOFMEMORY;
			}	// if
		if (hr == S_OK && szCons > 0)
			{
			// WHERE
			CCLOK		( WCSCAT ( pwQryBfr, SIZE_SQLBFR, L" WHERE " ); )

			// Names, values
			CCLOK		( idx = 0; )
			CCLTRYE	( pConsIn != NULL, E_UNEXPECTED );
			CCLTRY	( pConsIn->begin() );
			while (hr == S_OK && pConsIn->read ( unkV ) == S_OK)
				{
				// Properties
				CCLTRY( _QISAFE(unkV,IID_IDictionary,&pDict) );

				// Name
				CCLTRY( pDict->load ( strRefKey, strV ) );
				CCLOK ( WCSCAT ( pwQryBfr, SIZE_SQLBFR, L"\"" ); )
				CCLOK	( WCSCAT ( pwQryBfr, SIZE_SQLBFR, strV ); )
				CCLOK ( WCSCAT ( pwQryBfr, SIZE_SQLBFR, L"\" " ); )

				// Constraint operator ( <, >, LIKE, etc. )
				CCLTRY( pDict->load ( strRefOp, strV ) );
				CCLOK	( WCSCAT ( pwQryBfr, SIZE_SQLBFR, strV ); )
				CCLOK ( bLike = (WCASECMP(strV,L"LIKE") == 0); )

				// Value, bind
				CCLOK ( WCSCAT ( pwQryBfr, SIZE_SQLBFR, L" ? " ); )
				CCLTRY( pDict->load ( strRefValue, pvCons[idx].sData ) );

				// If the operator is 'like' and the value is a string, add the wildcards
				if (	hr == S_OK && 
						bLike && 
						adtValue::type(pvCons[idx].sData) == VTYPE_STR)
					{
					CCLTRY ( strV.allocate ( (U32)wcslen(adtString(pvCons[idx].sData)) + 2 ) );
					CCLOK  ( WCSCPY ( &strV.at(), SIZE_SQLBFR, L"%" ); )
					CCLOK  ( WCSCAT ( &strV.at(), SIZE_SQLBFR, adtString(pvCons[idx].sData) ); )
					CCLOK  ( WCSCAT ( &strV.at(), SIZE_SQLBFR, L"%" ); )
					CCLTRY ( adtValue::copy ( strV, pvCons[idx].sData ) );
					}	// if
				CCLTRY( SQLBindVariantParam ( pStmt->Handle, cidx+1,
							&(pvCons[idx].sData), &(pvCons[idx].uSz) ) );
				CCLOK ( ++idx; )
				CCLOK ( ++cidx; )

				// Next constraint
				if (hr == S_OK && idx < szCons)
					WCSCAT ( pwQryBfr, SIZE_SQLBFR, L"AND " );
				_RELEASE(pDict);
				pConsIn->next();
				}	// while
			CCLOK ( pwQryBfr[wcslen(pwQryBfr)-1] = WCHAR('\0'); )
			}	// if

		// Execute.  We should not get SQL_NEED_DATA here because keys are not blobs
		// and other data types are known size
		CCLOK ( OutputDebugString ( pwQryBfr ); )
		CCLOK ( OutputDebugString ( L"\n" ); )
		CCLTRY( SQLSTMT(pStmt->Handle,SQLExecDirect ( pStmt->Handle, pwQryBfr, 
																		(SQLINTEGER)wcslen(pwQryBfr) )));

		// Result
		if (hr == S_OK)
			_EMT(Fire,adtIUnknown((phv = pStmt)));
		else					
			_EMT(Fail,adtInt(hr));

		// Clean up
		if (pvValues != NULL) delete[] pvValues;
		_RELEASE(pStmt);
		_RELEASE(pKeys);
		}	// if

	// Connection
	else if (_RCP(Connection))
		{
		HRESULT		hr = S_OK;
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

	// Constraints
	else if (_RCP(Constraints))
		{
		IIt			*pIt		= NULL;
		adtIUnknown		unkV(v);

		// Previous object
		_RELEASE(pCons);
		_RELEASE(pConsIn);
		szCons = 0;

		// New object
		CCLTRY(_QISAFE(unkV,IID_IContainer,&pCons));
		CCLTRY(pCons->iterate ( &pIt ));
		CCLTRY(_QI(pIt,IID_IIt,&pConsIn));
		_RELEASE(pIt);
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
	else if (_RCP(Table))
		hr = adtValue::copy ( adtString(v), sTableName );

	return hr;
	}	// receive

#endif
