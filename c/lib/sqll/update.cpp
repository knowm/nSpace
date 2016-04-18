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
	if (prFire == pR)
		{
		IADTDictionary	*pDict		= NULL;
		IADTInIt			*pKeys		= NULL;
		SQLHandle		*pStmt		= NULL;
		SQLCol			*pvValues	= NULL;
		IHaveValue		*phv;
		adtValueImpl	vVal,vKey;
		adtIUnknown		unkV;
		adtString		strV;
		U32				idx,cidx,nkeys;
		bool				bLike;

		// State check
		CCLTRYE ( (hConn != NULL),					ERROR_INVALID_STATE );
		if (hr == S_OK && sTableName.length() == 0)
			hr = pnAttr->load ( strRefTableName, sTableName );
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
		CCLOK		( wcscpy ( pwQryBfr, L"UPDATE " ); )

		// Table name
		CCLOK		( wcscat ( pwQryBfr, sTableName ); )

		// SET
		CCLOK		( wcscat ( pwQryBfr, L" SET " ); )

		// Fields
		CCLOK  ( cidx = 0; )
		CCLTRY ( pKeys->begin() );
		while (hr == S_OK && pKeys->read ( vKey ) == S_OK)
			{
			adtString sKey(vKey);

			// State check
			CCLTRYE ( vKey.vtype == VALT_STR, E_UNEXPECTED );

			// Field name
			CCLOK ( wcscat ( pwQryBfr, L"\"" ); )
			CCLOK ( wcscat ( pwQryBfr, sKey ); )
			CCLOK ( wcscat ( pwQryBfr, L"\" = ?," ); )

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
			CCLOK		( wcscat ( pwQryBfr, L" WHERE " ); )

			// Names, values
			CCLOK		( idx = 0; )
			CCLTRYE	( pConsIn != NULL, E_UNEXPECTED );
			CCLTRY	( pConsIn->begin() );
			while (hr == S_OK && pConsIn->read ( unkV ) == S_OK)
				{
				// Properties
				CCLTRY( _QISAFE(unkV,IID_IADTDictionary,&pDict) );

				// Name
				CCLTRY( pDict->load ( strRefKey, strV ) );
				CCLOK ( wcscat ( pwQryBfr, L"\"" ); )
				CCLOK	( wcscat ( pwQryBfr, strV ); )
				CCLOK ( wcscat ( pwQryBfr, L"\" " ); )

				// Constraint operator ( <, >, LIKE, etc. )
				CCLTRY( pDict->load ( strRefOp, strV ) );
				CCLOK	( wcscat ( pwQryBfr, strV ); )
				CCLOK ( bLike = (WCASECMP(strV,L"LIKE") == 0); )

				// Value, bind
				CCLOK ( wcscat ( pwQryBfr, L" ? " ); )
				CCLTRY( pDict->load ( strRefValue, pvCons[idx].sData ) );

				// If the operator is 'like' and the value is a string, add the wildcards
				if (hr == S_OK && bLike && pvCons[idx].sData.vtype == VALT_STR)
					{
					CCLTRY ( strV.allocate ( (U32)wcslen(adtString(pvCons[idx].sData)) + 2 ) );
					CCLOK  ( wcscpy ( &strV.at(), L"%" ); )
					CCLOK  ( wcscat ( &strV.at(), adtString(pvCons[idx].sData) ); )
					CCLOK  ( wcscat ( &strV.at(), L"%" ); )
					CCLTRY ( adtValueImpl::copy ( pvCons[idx].sData, strV ) );
					}	// if
				CCLTRY( SQLBindVariantParam ( pStmt->Handle, cidx+1,
							&(pvCons[idx].sData), &(pvCons[idx].uSz) ) );
				CCLOK ( ++idx; )
				CCLOK ( ++cidx; )

				// Next constraint
				if (hr == S_OK && idx < szCons)
					wcscat ( pwQryBfr, L"AND " );
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
		if (hr == S_OK)	peFire->emit ( adtIUnknown((phv = pStmt)) );
		else					peFail->emit ( adtInt(hr) );

		// Clean up
		if (pvValues != NULL) delete[] pvValues;
		_RELEASE(pStmt);
		_RELEASE(pKeys);
		}	// if

	// Connection
	else if (prConn == pR)
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
	else if (prCons == pR)
		{
		IADTIt			*pIt		= NULL;
		adtIUnknown		unkV(v);

		// Previous object
		_RELEASE(pCons);
		_RELEASE(pConsIn);
		szCons = 0;

		// New object
		CCLTRY(_QISAFE(unkV,IID_IADTContainer,&pCons));
		CCLTRY(pCons->iterate ( &pIt ));
		CCLTRY(_QI(pIt,IID_IADTInIt,&pConsIn));
		_RELEASE(pIt);
		}	// else if

	// Fields
	else if (prFlds == pR)
		{
		adtIUnknown		unkV(v);

		// Previous object
		_RELEASE(pFlds);

		// New object
		CCLTRY(_QISAFE(unkV,IID_IADTDictionary,&pFlds));
		}	// else if

	// Table name
	else if (prTbl == pR)
		hr = adtValueImpl::copy ( sTableName, adtString(v) );

	return hr;
	}	// receive

#endif
